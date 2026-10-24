"""Explicitly enabled, velocity-limited gate for Nav2 Twist commands.

This node publishes only to /autonomy/cmd_vel. It does not bridge to the dog.
The ROS 1 command path must remain separately disabled until its RC interaction
and stop behavior have been verified.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from lite3_odom_tf_adapter.command_permission import CommandPermission


class CmdVelGate(CommandPermission, Node):
    def __init__(self):
        super().__init__('lite3_cmd_vel_gate')
        self.declare_parameter('input_topic', '/nav2/cmd_vel')
        self.declare_parameter('output_topic', '/autonomy/cmd_vel')
        self.declare_parameter('readiness_topic', '/autonomy/ready')
        self.declare_parameter('command_timeout', 0.30)
        self.declare_parameter('readiness_timeout', 1.0)
        self.declare_parameter('max_linear_x', 0.10)
        self.declare_parameter('max_angular_z', 0.20)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        readiness_topic = str(self.get_parameter('readiness_topic').value)
        self._command_timeout = float(self.get_parameter('command_timeout').value)
        self._readiness_timeout = float(self.get_parameter('readiness_timeout').value)
        self._max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self._max_angular_z = abs(float(self.get_parameter('max_angular_z').value))

        self._enabled = False
        self._ready = False
        self._ready_at = 0.0
        self._last_command_at = 0.0
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Twist, output_topic, qos)
        self._command_generation = 0
        self._command_topic, self._command_qos = input_topic, qos
        self._command_sub = self.create_subscription(Twist, input_topic,
                                                     lambda msg: self._guard_command(msg, 0), qos)
        self._ready_sub = self.create_subscription(
            Bool, readiness_topic, self._on_readiness, qos)
        self._enable_service = self.create_service(
            SetBool, '/autonomy/set_enabled', self._set_enabled)
        self._watchdog = self.create_timer(0.05, self._check_watchdog)
        self._init_permission('gate')
        self.get_logger().warning(
            f'Disabled at startup. Gating {input_topic} -> {output_topic}; '
            f'limits |x| <= {self._max_linear_x:.2f} m/s, '
            f'|yaw| <= {self._max_angular_z:.2f} rad/s')

    @staticmethod
    def _zero():
        return Twist()

    def _on_readiness(self, message):
        self._ready = bool(message.data)
        self._ready_at = time.monotonic()
        if not self._ready and self._enabled:
            self._disable('Readiness lost')

    def _disable(self, reason):
        self._command_generation += 1
        self._enabled = False
        self._publisher.publish(self._zero())
        self.get_logger().error(f'{reason}; autonomy disabled; explicit re-arm required')
        self._emit_status(reason)

    def _set_enabled(self, request, response):
        if request.data:
            if not self._ownership_valid():
                response.success = False
                response.message = 'Exclusive ownership permission is missing or stale.'
                return response
            ready_fresh = (
                self._ready and
                (time.monotonic() - self._ready_at) <= self._readiness_timeout
            )
            if not ready_fresh:
                response.success = False
                response.message = 'Refusing autonomy: readiness is false or stale.'
                return response
            self._renew_command_generation()
            self._enabled = True
            self._emit_status('Gate explicitly enabled')
            self._last_command_at = time.monotonic()
            response.success = True
            response.message = 'Autonomy gate enabled with conservative velocity limits.'
            self.get_logger().warning(response.message)
            return response

        was_enabled = self._enabled
        self._enabled = False
        if was_enabled:
            self._publisher.publish(self._zero())
        response.success = True
        response.message = 'Autonomy gate disabled.'
        self._emit_status(response.message)
        self.get_logger().warning(response.message)
        return response

    def _on_command(self, message):
        # Reject a command that arrives after a deadline even when the timer
        # callback has not run yet. Returning traffic cannot clear a fault.
        self._check_watchdog()
        if not self._enabled:
            return
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self._disable('Non-finite velocity command')
            return
        bounded = Twist()
        bounded.linear.x = max(-self._max_linear_x, min(self._max_linear_x, message.linear.x))
        bounded.angular.z = max(-self._max_angular_z, min(self._max_angular_z, message.angular.z))
        self._publisher.publish(bounded)
        self._last_command_at = time.monotonic()

    def _check_watchdog(self):
        if not self._enabled:
            return
        if not self._ownership_valid():
            self._disable('Exclusive ownership permission expired')
            return
        command_stale = (time.monotonic() - self._last_command_at) > self._command_timeout
        readiness_stale = (time.monotonic() - self._ready_at) > self._readiness_timeout
        if readiness_stale or not self._ready:
            self._disable('Readiness stale/lost')
            return
        if command_stale:
            self._disable('Nav2 command timed out')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._enabled:
            node._publisher.publish(node._zero())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
