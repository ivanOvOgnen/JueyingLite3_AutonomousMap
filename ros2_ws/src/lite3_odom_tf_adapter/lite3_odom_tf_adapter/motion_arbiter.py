"""Fail-closed autonomy arbiter for a future vendor command bridge.

This node intentionally publishes only to an isolated ROS 2 topic by default.
It never publishes to the vendor ROS 1 ``/cmd_vel`` topic and does not open a
UDP socket. Enabling requires an explicit service call, fresh project
readiness, a fresh authenticated Retroid STOP relay record, a fresh command
heartbeat, and a launch-time physical-stop verification flag. Manual/vendor
control remains outside this node.
"""

import math
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class MotionArbiter(Node):
    def __init__(self):
        super().__init__('lite3_motion_arbiter')
        self.declare_parameter('input_topic', '/autonomy/cmd_vel')
        self.declare_parameter('output_topic', '/autonomy/cmd_vel_arbited')
        self.declare_parameter('readiness_topic', '/autonomy/ready')
        self.declare_parameter('manual_override_topic', '/autonomy/manual_override')
        self.declare_parameter('status_topic', '/autonomy/motion_status')
        self.declare_parameter('enable_service', '/autonomy/arbiter/set_enabled')
        self.declare_parameter('physical_stop_verified', False)
        self.declare_parameter(
            'safety_shared_memory_path',
            '/dev/shm/emotion_bot_lite3_safety_state')
        self.declare_parameter('safety_record_timeout', 1.0)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('readiness_timeout', 1.0)
        self.declare_parameter('max_linear_x', 0.10)
        self.declare_parameter('max_angular_z', 0.20)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        readiness_topic = str(self.get_parameter('readiness_topic').value)
        manual_override_topic = str(self.get_parameter('manual_override_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        enable_service = str(self.get_parameter('enable_service').value)
        self._physical_stop_verified = bool(
            self.get_parameter('physical_stop_verified').value)
        self._safety_shared_memory_path = str(
            self.get_parameter('safety_shared_memory_path').value)
        self._safety_record_timeout = float(
            self.get_parameter('safety_record_timeout').value)
        self._command_timeout = float(self.get_parameter('command_timeout').value)
        self._readiness_timeout = float(self.get_parameter('readiness_timeout').value)
        self._max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self._max_angular_z = abs(float(self.get_parameter('max_angular_z').value))

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Twist, output_topic, qos)
        self._status_publisher = self.create_publisher(String, status_topic, qos)
        self._command_sub = self.create_subscription(Twist, input_topic, self._on_command, qos)
        self._ready_sub = self.create_subscription(Bool, readiness_topic, self._on_readiness, qos)
        self._manual_sub = self.create_subscription(
            Bool, manual_override_topic, self._on_manual_override, qos)
        self._enable_service = self.create_service(SetBool, enable_service, self._set_enabled)
        self._watchdog = self.create_timer(0.05, self._check_watchdog)
        self._enabled = False
        self._ready = False
        self._manual_override = False
        self._ready_at = 0.0
        self._last_command_at = 0.0
        self._zero_sent = False
        self._safety_ok = False
        self._safety_detail = 'safety relay record unavailable'
        self._publish_status('disabled: output isolated; physical stop verification required')
        self.get_logger().warning(
            f'Fail-closed arbiter: {input_topic} -> {output_topic}; '
            f'vendor /cmd_vel bridge is intentionally absent')

    @staticmethod
    def _zero():
        return Twist()

    def _publish_status(self, text):
        message = String()
        message.data = text
        self._status_publisher.publish(message)

    def _read_safety_record(self):
        """Read the authenticated ROS 1 STOP relay's seqlock record.

        The relay receiver owns this file. A missing, stale, torn, malformed,
        stopped, or non-zero-axis record is treated as unsafe. The format is
        deliberately duplicated here so this ROS 2 node does not depend on a
        ROS 1 Python package being importable in the Foxy environment.
        """
        header = struct.Struct('<4sIQQI')
        wire = struct.Struct('<I???5xQ')
        try:
            with open(self._safety_shared_memory_path, 'rb') as stream:
                raw = stream.read(header.size + wire.size)
                if len(raw) != header.size + wire.size:
                    raise ValueError('short safety record')
                first = header.unpack(raw[:header.size])
                payload = raw[header.size:]
                stream.seek(0)
                second = header.unpack(stream.read(header.size))
        except (OSError, ValueError, struct.error) as exc:
            return False, f'safety relay unavailable: {exc}'

        magic, version, sequence, receive_ns, length = first
        if magic != b'EBL3' or version != 1 or sequence % 2:
            return False, 'safety relay record is invalid or being updated'
        if first != second or length != wire.size:
            return False, 'safety relay record changed during read'
        try:
            record_version, stop, observed_fresh, axes_zero, _ = wire.unpack(payload)
        except struct.error as exc:
            return False, f'safety relay payload is invalid: {exc}'
        if record_version != 0x00010000:
            return False, 'safety relay record version is unsupported'
        age = (time.monotonic_ns() - receive_ns) / 1e9
        if age < 0 or age > self._safety_record_timeout:
            return False, f'safety relay record is stale ({age:.3f}s)'
        if not observed_fresh:
            return False, 'Retroid STOP observation is stale'
        if stop:
            return False, 'Retroid STOP is active'
        if not axes_zero:
            return False, 'Retroid axes are not zero'
        return True, 'fresh Retroid STOP relay record is clear'

    def _update_safety(self):
        self._safety_ok, self._safety_detail = self._read_safety_record()

    def _disable(self, reason):
        was_enabled = self._enabled
        self._enabled = False
        self._zero_sent = False
        self._publisher.publish(self._zero())
        if was_enabled:
            self.get_logger().error(reason)
        self._publish_status(f'disabled: {reason}')

    def _on_readiness(self, message):
        self._ready = bool(message.data)
        self._ready_at = time.monotonic()
        if not self._ready and self._enabled:
            self._disable('project readiness is false')

    def _on_manual_override(self, message):
        self._manual_override = bool(message.data)
        if self._manual_override and self._enabled:
            self._disable('manual override requested')

    def _set_enabled(self, request, response):
        if not request.data:
            self._disable('disabled by service request')
            response.success = True
            response.message = 'Motion arbiter disabled; isolated zero command sent.'
            return response
        if not self._physical_stop_verified:
            response.success = False
            response.message = 'Refusing autonomy: independent physical stop is not verified.'
            self._publish_status('disabled: independent physical stop is not verified')
            return response
        self._update_safety()
        if not self._safety_ok:
            response.success = False
            response.message = f'Refusing autonomy: {self._safety_detail}.'
            self._publish_status(f'disabled: {self._safety_detail}')
            return response
        if self._manual_override:
            response.success = False
            response.message = 'Refusing autonomy: manual override is active.'
            return response
        ready_fresh = self._ready and (
            time.monotonic() - self._ready_at <= self._readiness_timeout)
        if not ready_fresh:
            response.success = False
            response.message = 'Refusing autonomy: project readiness is false or stale.'
            return response
        self._enabled = True
        self._last_command_at = time.monotonic()
        self._zero_sent = False
        self._publish_status('enabled: isolated arbiter output; vendor bridge absent')
        response.success = True
        response.message = 'Arbiter enabled on isolated output; vendor bridge remains disconnected.'
        return response

    def _on_command(self, message):
        if not self._enabled:
            return
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self._disable('non-finite command received')
            return
        bounded = Twist()
        bounded.linear.x = max(-self._max_linear_x, min(self._max_linear_x, message.linear.x))
        bounded.angular.z = max(-self._max_angular_z, min(self._max_angular_z, message.angular.z))
        self._publisher.publish(bounded)
        self._last_command_at = time.monotonic()
        self._zero_sent = False

    def _check_watchdog(self):
        self._update_safety()
        if not self._enabled:
            return
        now = time.monotonic()
        if not self._safety_ok:
            self._disable(self._safety_detail)
        elif self._manual_override:
            self._disable('manual override is active')
        elif not self._ready or now - self._ready_at > self._readiness_timeout:
            self._disable('project readiness became stale')
        elif now - self._last_command_at > self._command_timeout:
            self._publisher.publish(self._zero())
            if not self._zero_sent:
                self._publish_status('enabled: command heartbeat stale; zero command sent')
                self._zero_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._disable('process exiting')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
