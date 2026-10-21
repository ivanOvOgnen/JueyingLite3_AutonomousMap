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


class CmdVelGate(Node):
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
        self._zero_sent_for_timeout = False
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Twist, output_topic, qos)
        self._command_sub = self.create_subscription(Twist, input_topic, self._on_command, qos)
        self._ready_sub = self.create_subscription(
            Bool, readiness_topic, self._on_readiness, qos)
        self._enable_service = self.create_service(
            SetBool, '/autonomy/set_enabled', self._set_enabled)
        self._watchdog = self.create_timer(0.05, self._check_watchdog)
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
            self._enabled = False
            self._publisher.publish(self._zero())
            self.get_logger().error('Readiness lost; autonomy disabled and zero Twist sent')

    def _set_enabled(self, request, response):
        if request.data:
            ready_fresh = (
                self._ready and
                (time.monotonic() - self._ready_at) <= self._readiness_timeout
            )
            if not ready_fresh:
                response.success = False
                response.message = 'Refusing autonomy: readiness is false or stale.'
                return response
            self._enabled = True
            self._last_command_at = time.monotonic()
            self._zero_sent_for_timeout = False
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
        self.get_logger().warning(response.message)
        return response

    def _on_command(self, message):
        if not self._enabled:
            return
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Dropped non-finite velocity command')
            return
        bounded = Twist()
        bounded.linear.x = max(-self._max_linear_x, min(self._max_linear_x, message.linear.x))
        bounded.angular.z = max(-self._max_angular_z, min(self._max_angular_z, message.angular.z))
        self._publisher.publish(bounded)
        self._last_command_at = time.monotonic()
        self._zero_sent_for_timeout = False

    def _check_watchdog(self):
        if not self._enabled:
            return
        command_stale = (time.monotonic() - self._last_command_at) > self._command_timeout
        readiness_stale = (time.monotonic() - self._ready_at) > self._readiness_timeout
        if readiness_stale or not self._ready:
            self._enabled = False
            self._publisher.publish(self._zero())
            self.get_logger().error('Readiness stale/lost; autonomy disabled and zero Twist sent')
            return
        if command_stale:
            self._publisher.publish(self._zero())
            if not self._zero_sent_for_timeout:
                self.get_logger().error('Nav2 command timed out; zero Twist sent')
                self._zero_sent_for_timeout = True


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
