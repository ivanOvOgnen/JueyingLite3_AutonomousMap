"""Delay scans briefly so the corresponding delayed odometry TF is available."""

from collections import deque
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanDelay(Node):
    def __init__(self):
        super().__init__('lite3_scan_delay')
        self.declare_parameter('delay_seconds', 0.5)
        self.declare_parameter('queue_size', 60)
        self.delay_seconds = float(self.get_parameter('delay_seconds').value)
        self.queue_size = int(self.get_parameter('queue_size').value)
        self._pending = deque()
        self._publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan, qos_profile_sensor_data)
        self._timer = self.create_timer(0.01, self._publish_ready)
        self.get_logger().info(
            f'Delaying /scan_raw by {self.delay_seconds:.3f}s; preserving scan timestamps')

    def _on_scan(self, message):
        if len(self._pending) >= self.queue_size:
            self._pending.popleft()
            self.get_logger().warning('Scan delay queue full; dropping oldest scan')
        self._pending.append((time.monotonic() + self.delay_seconds, message))

    def _publish_ready(self):
        now = time.monotonic()
        while self._pending and self._pending[0][0] <= now:
            _, message = self._pending.popleft()
            self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ScanDelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
