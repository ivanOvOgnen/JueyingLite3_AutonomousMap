"""Shared fresh ownership permit and structured stage status for command nodes."""
import json
import time
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist


class CommandPermission:
    def _init_permission(self, stage, publisher=None):
        self._permission_stage = stage
        self._ownership = False
        self._ownership_at = None
        self._permission_reason = 'Disabled at startup; ownership unknown'
        self._permission_status = publisher or self.create_publisher(String, '/autonomy/gate/status', 1)
        self.create_subscription(Bool, '/autonomy/ownership_permit', self._on_ownership, 1)
        self.create_timer(0.1, self._emit_status)

    def _on_ownership(self, message):
        self._ownership = bool(message.data)
        self._ownership_at = time.monotonic()
        if not self._ownership and self._enabled:
            self._disable('Exclusive ownership permission revoked')

    def _ownership_valid(self):
        return bool(self._ownership and self._ownership_at is not None and
                    0 <= time.monotonic()-self._ownership_at <= 0.3)

    def _emit_status(self, reason=None):
        if reason is not None:
            self._permission_reason = reason
        message = String()
        message.data = json.dumps({'stage': self._permission_stage, 'enabled': self._enabled,
                                   'state': 'enabled' if self._enabled else 'disabled',
                                   'ownership_fresh': self._ownership_valid(),
                                   'reason': self._permission_reason}, allow_nan=False)
        self._permission_status.publish(message)

    def _guard_command(self, message, generation):
        if generation == self._command_generation:
            self._on_command(message)

    def _renew_command_generation(self):
        self._command_generation += 1
        generation = self._command_generation
        self.destroy_subscription(self._command_sub)
        self._command_sub = self.create_subscription(
            Twist, self._command_topic, lambda msg: self._guard_command(msg, generation), self._command_qos)
