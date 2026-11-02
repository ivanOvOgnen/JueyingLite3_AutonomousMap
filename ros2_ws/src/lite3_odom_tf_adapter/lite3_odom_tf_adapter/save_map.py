#!/usr/bin/env python3
"""Save one retained ROS OccupancyGrid as a Nav2-compatible PGM/YAML pair."""

import argparse
import hashlib
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from lite3_odom_tf_adapter.map_bundle import build_manifest, manifest_path, write_manifest


class OneShotMapSubscriber(Node):
    def __init__(self):
        super().__init__('lite3_one_shot_map_saver')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.message = None
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, qos)

    def _on_map(self, message):
        if self.message is None:
            self.message = message


from lite3_odom_tf_adapter.map_persistence import save_grid


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_prefix', help='Output path without .pgm/.yaml suffix')
    parser.add_argument('--timeout', type=float, default=15.0,
                        help='Seconds to wait for a retained /map (default: 15)')
    options, ros_args = parser.parse_known_args(args=args)
    if not math.isfinite(options.timeout) or options.timeout <= 0.0:
        parser.error('--timeout must be finite and positive')

    rclpy.init(args=ros_args)
    node = OneShotMapSubscriber()
    deadline = time.monotonic() + options.timeout
    try:
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if node.message is None:
            node.get_logger().error(
                'No /map received before timeout. Check the publisher and its QoS.')
            return 2
        image_path, yaml_path, manifest_path = save_grid(node.message, options.output_prefix)
        node.get_logger().info(
            f"Saved {node.message.info.width}x{node.message.info.height} map: "
            f'{image_path}, {yaml_path}, and {manifest_path}')
        return 0
    except (OSError, ValueError) as error:
        node.get_logger().error(f'Could not save map: {error}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
