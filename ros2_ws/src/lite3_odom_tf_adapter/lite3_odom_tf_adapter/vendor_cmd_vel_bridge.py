"""Fail-closed bridge from the project arbiter to the vendor ROS command name.

This node is deliberately not part of the normal navigation launch.  It is a
commissioning component: it publishes only while explicitly enabled, while
the project readiness heartbeat is fresh, the manual override is clear, and
the authenticated stop relay record is fresh and clear.  It does not open a
UDP socket; a separately launched ``ros1_bridge`` parameter bridge carries
``/cmd_vel`` to the vendor ROS 1 ``ros2qnx`` subscriber. The vendor's
``/cmd_vel_corrected`` topic is already published by ``/safety_node``.
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
from lite3_odom_tf_adapter.command_permission import CommandPermission


class VendorCmdVelBridge(CommandPermission, Node):
    def __init__(self):
        super().__init__('lite3_vendor_cmd_vel_bridge')
        self.declare_parameter('input_topic', '/autonomy/cmd_vel_arbited')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('readiness_topic', '/autonomy/ready')
        self.declare_parameter('manual_override_topic', '/autonomy/manual_override')
        self.declare_parameter('enable_service', '/autonomy/vendor_bridge/set_enabled')
        self.declare_parameter('physical_stop_verified', False)
        self.declare_parameter('safety_shared_memory_path', '/dev/shm/emotion_bot_lite3_safety_state')
        self.declare_parameter('safety_record_timeout', 1.0)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('readiness_timeout', 1.0)
        self.declare_parameter('manual_override_timeout', 1.0)
        self.declare_parameter('max_linear_x', 0.10)
        self.declare_parameter('max_angular_z', 0.20)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        readiness_topic = str(self.get_parameter('readiness_topic').value)
        manual_topic = str(self.get_parameter('manual_override_topic').value)
        enable_service = str(self.get_parameter('enable_service').value)
        self._physical_stop_verified = bool(self.get_parameter('physical_stop_verified').value)
        self._safety_path = str(self.get_parameter('safety_shared_memory_path').value)
        self._safety_timeout = float(self.get_parameter('safety_record_timeout').value)
        self._command_timeout = float(self.get_parameter('command_timeout').value)
        self._readiness_timeout = float(self.get_parameter('readiness_timeout').value)
        self._manual_override_timeout = float(self.get_parameter('manual_override_timeout').value)
        self._max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self._max_angular_z = abs(float(self.get_parameter('max_angular_z').value))

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self._publisher = self.create_publisher(Twist, output_topic, qos)
        self._status_pub = self.create_publisher(String, '/autonomy/vendor_bridge/status', qos)
        self._command_generation = 0
        self._command_topic, self._command_qos = input_topic, qos
        self._command_sub = self.create_subscription(Twist, input_topic,
                                                     lambda msg: self._guard_command(msg, 0), qos)
        self._ready_sub = self.create_subscription(Bool, readiness_topic, self._on_ready, qos)
        self._manual_sub = self.create_subscription(Bool, manual_topic, self._on_manual, qos)
        self._enable_srv = self.create_service(SetBool, enable_service, self._set_enabled)
        self._watchdog = self.create_timer(0.05, self._watchdog_tick)

        self._enabled = False
        self._ready = False
        self._manual_override = True
        self._manual_override_at = None
        self._ready_at = 0.0
        self._last_command_at = 0.0
        self._init_permission('relay', self._status_pub)
        self._publish_status('disabled: vendor command bridge is off')
        self.get_logger().warning(
            f'Fail-closed vendor bridge: {input_topic} -> {output_topic}; disabled by default')

    @staticmethod
    def _zero():
        return Twist()

    def _publish_status(self, text):
        self._emit_status(text)

    def _safety_ok(self):
        header = struct.Struct('<4sIQQI')
        wire = struct.Struct('<I???5xQ')
        try:
            with open(self._safety_path, 'rb') as stream:
                raw = stream.read(header.size + wire.size)
                if len(raw) != header.size + wire.size:
                    return False, 'safety relay record is short'
                first = header.unpack(raw[:header.size])
                payload = raw[header.size:]
                stream.seek(0)
                second = header.unpack(stream.read(header.size))
        except (OSError, ValueError, struct.error) as exc:
            return False, f'safety relay unavailable: {exc}'
        magic, version, sequence, receive_ns, length = first
        if magic != b'EBL3' or version != 1 or sequence % 2 or first != second or length != wire.size:
            return False, 'safety relay record is invalid or changing'
        try:
            record_version, stop, observed_fresh, axes_zero, _ = wire.unpack(payload)
        except struct.error as exc:
            return False, f'safety relay payload is invalid: {exc}'
        age = (time.monotonic_ns() - receive_ns) / 1e9
        if record_version != 0x00010000 or age < 0 or age > self._safety_timeout:
            return False, 'safety relay record is stale or unsupported'
        if not observed_fresh:
            return False, 'Retroid STOP observation is stale'
        if stop:
            return False, 'Retroid STOP is active'
        if not axes_zero:
            return False, 'Retroid axes are not zero'
        return True, 'fresh stop relay record is clear'

    def _disable(self, reason):
        self._command_generation += 1
        was_enabled = self._enabled
        self._enabled = False
        self._publisher.publish(self._zero())
        if was_enabled:
            self.get_logger().error(reason)
        self._publish_status(f'disabled: {reason}')

    def _on_ready(self, message):
        self._ready = bool(message.data)
        self._ready_at = time.monotonic()
        if not self._ready and self._enabled:
            self._disable('project readiness is false')

    def _on_manual(self, message):
        self._manual_override = bool(message.data)
        self._manual_override_at = time.monotonic()
        if self._manual_override and self._enabled:
            self._disable('manual override is active')

    def _manual_state_fresh(self, now):
        return self._manual_override_at is not None and (
            0.0 <= now - self._manual_override_at <= self._manual_override_timeout)

    def _set_enabled(self, request, response):
        if not request.data:
            self._disable('disabled by service request')
            response.success = True
            response.message = 'Vendor bridge disabled; zero command sent.'
            return response
        if not self._ownership_valid():
            response.success = False
            response.message = 'Exclusive ownership permission is missing or stale.'
            return response
        if not self._physical_stop_verified:
            response.success = False
            response.message = 'Refusing vendor bridge: independent physical stop is not verified.'
            return response
        safety_ok, detail = self._safety_ok()
        if not safety_ok:
            response.success = False
            response.message = f'Refusing vendor bridge: {detail}.'
            return response
        if self._manual_override or not self._manual_state_fresh(time.monotonic()):
            response.success = False
            response.message = 'Refusing vendor bridge: manual state is active, missing, or stale.'
            return response
        if not self._ready or time.monotonic() - self._ready_at > self._readiness_timeout:
            response.success = False
            response.message = 'Refusing vendor bridge: project readiness is false or stale.'
            return response
        self._renew_command_generation()
        self._enabled = True
        self._last_command_at = time.monotonic()
        self._publish_status('enabled: guarded vendor command bridge')
        response.success = True
        response.message = 'Vendor command bridge enabled.'
        return response

    def _on_command(self, message):
        self._watchdog_tick()
        if not self._enabled:
            return
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self._disable('non-finite command received')
            return
        bounded = Twist()
        bounded.linear.x = max(-self._max_linear_x, min(self._max_linear_x, message.linear.x))
        bounded.angular.z = max(-self._max_angular_z, min(self._max_angular_z, message.angular.z))
        self._publisher.publish(bounded)
        self._last_command_at = time.monotonic()

    def _watchdog_tick(self):
        if not self._enabled:
            return
        safety_ok, detail = self._safety_ok()
        if not self._ownership_valid():
            self._disable('Exclusive ownership permission expired')
            return
        now = time.monotonic()
        if not safety_ok:
            self._disable(detail)
        elif self._manual_override or not self._manual_state_fresh(now):
            self._disable('manual state is active, missing, or stale')
        elif not self._ready or now - self._ready_at > self._readiness_timeout:
            self._disable('project readiness became stale')
        elif now - self._last_command_at > self._command_timeout:
            self._disable('command heartbeat stale; explicit re-arm required')


def main(args=None):
    rclpy.init(args=args)
    node = VendorCmdVelBridge()
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
