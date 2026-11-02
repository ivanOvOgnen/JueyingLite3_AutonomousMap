"""Short, explicitly armed Nav2-to-vendor trial; never a permanent autonomy mode."""

import json
import math
from pathlib import Path
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


SAFETY_PATH = Path('/dev/shm/emotion_bot_lite3_safety_state')
HEADER = struct.Struct('<4sIQQI')
WIRE = struct.Struct('<I???5xQ')
MAX_LINEAR = 0.05
MAX_ANGULAR = 0.10
MAX_ARM_SECONDS = 10.0
COMMAND_TIMEOUT = 0.25
CONTEXT_TIMEOUT = 0.5


def safety_clear(path=SAFETY_PATH):
    try:
        with path.open('rb') as stream:
            first = stream.read(HEADER.size)
            payload = stream.read(WIRE.size)
            stream.seek(0)
            second = stream.read(HEADER.size)
        magic, version, sequence, received_ns, length = HEADER.unpack(first)
        record_version, stop, fresh, axes_zero, _ = WIRE.unpack(payload)
        age = (time.monotonic_ns() - received_ns) / 1e9
        return (first == second and magic == b'EBL3' and version == 1 and
                sequence % 2 == 0 and length == WIRE.size and
                record_version == 0x00010000 and 0 <= age <= 0.25 and
                not stop and fresh and axes_zero)
    except (OSError, ValueError, struct.error):
        return False


def eligible(context, context_age, ready, safety, goal_id):
    return (context_age <= CONTEXT_TIMEOUT and ready and safety and
            context.get('ready') is True and
            context.get('operator_lease_valid') is True and
            context.get('goal_active') is True and
            context.get('active_goal_id') == goal_id and bool(goal_id))


class SupervisedNavRelay(Node):
    def __init__(self):
        super().__init__('lite3_supervised_nav_relay')
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self._status = self.create_publisher(String, '/autonomy/supervised_trial/status', 1)
        # Do not use Node._context: rclpy owns that field and its Context
        # handle is required when timers/services are constructed.
        self._readiness_context = {}
        self._readiness_context_at = 0.0
        self._ready = False
        self._ready_at = 0.0
        self._command = Twist()
        self._command_at = 0.0
        self._armed_until = 0.0
        self._output_active = False
        self._goal_id = None
        self._reason = 'Not armed'
        self.create_subscription(String, '/autonomy/context', self._on_context, 1)
        self.create_subscription(Bool, '/autonomy/ready', self._on_ready, 1)
        self.create_subscription(Twist, '/nav2/cmd_vel', self._on_command, 1)
        self.create_service(SetBool, '/autonomy/supervised_trial/set_enabled', self._set_enabled)
        self.create_timer(0.05, self._tick)

    def _on_context(self, message):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict):
                self._readiness_context = data
                self._readiness_context_at = time.monotonic()
        except ValueError:
            self._readiness_context = {}

    def _on_ready(self, message):
        self._ready = message.data
        self._ready_at = time.monotonic()

    def _on_command(self, message):
        self._command = message
        self._command_at = time.monotonic()

    def _set_enabled(self, request, response):
        if not request.data:
            self._disarm('Disabled by operator/API')
            response.success = True
            response.message = self._reason
            return response
        now = time.monotonic()
        goal_id = self._readiness_context.get('active_goal_id')
        if self._armed_until > now:
            response.message = 'Trial is already armed; it cannot be extended while active'
        elif not eligible(self._readiness_context, now-self._readiness_context_at,
                          self._ready and now-self._ready_at <= CONTEXT_TIMEOUT,
                          safety_clear(), goal_id):
            response.message = 'Goal, lease, readiness, or vendor STOP observation is not current and clear'
        elif self._publisher.get_subscription_count() != 1:
            response.message = 'Expected exactly one ROS 2 /cmd_vel bridge subscriber'
        else:
            self._goal_id = goal_id
            self._armed_until = now + MAX_ARM_SECONDS
            self._output_active = True
            self._reason = 'Armed for at most 10 seconds'
            response.success = True
            response.message = self._reason
        return response

    def _disarm(self, reason):
        was_active = self._output_active
        self._armed_until = 0.0
        self._output_active = False
        self._goal_id = None
        self._reason = reason
        # A relay which has never been armed must be completely silent.  Once
        # it has owned the output, one zero command is required on disarm.
        if was_active:
            self._publisher.publish(Twist())

    def _tick(self):
        now = time.monotonic()
        if self._armed_until:
            if now >= self._armed_until:
                self._disarm('Ten-second trial limit reached; re-arm explicitly if needed')
            elif not eligible(self._readiness_context, now-self._readiness_context_at,
                              self._ready and now-self._ready_at <= CONTEXT_TIMEOUT,
                              safety_clear(), self._goal_id):
                self._disarm('Goal, lease, readiness, or vendor STOP observation changed')
            elif self._publisher.get_subscription_count() != 1:
                self._disarm('ROS 2 vendor bridge subscriber changed')
            elif now-self._command_at > COMMAND_TIMEOUT:
                self._publisher.publish(Twist())
                self._reason = 'Waiting for fresh Nav2 velocity'
            else:
                source = self._command
                values = (source.linear.x, source.linear.y, source.linear.z,
                          source.angular.x, source.angular.y, source.angular.z)
                if not all(math.isfinite(value) for value in values):
                    self._disarm('Non-finite Nav2 command')
                elif any(abs(value) > 1e-6 for value in
                         (source.linear.y, source.linear.z, source.angular.x, source.angular.y)):
                    self._disarm('Nav2 requested unsupported velocity axes')
                else:
                    output = Twist()
                    output.linear.x = max(-MAX_LINEAR, min(MAX_LINEAR, source.linear.x))
                    output.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, source.angular.z))
                    self._publisher.publish(output)
        self._status.publish(String(data=json.dumps({
            'enabled': self._armed_until > now,
            'goal_id': self._goal_id,
            'remaining_seconds': round(max(0.0, self._armed_until-now), 1),
            'reason': self._reason,
        })))

    def destroy_node(self):
        self._disarm('Process exiting')
        return super().destroy_node()


def main():
    rclpy.init()
    node = SupervisedNavRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
