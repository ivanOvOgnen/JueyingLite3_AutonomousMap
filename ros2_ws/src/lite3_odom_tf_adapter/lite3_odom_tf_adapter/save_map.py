#!/usr/bin/env python3
"""Save one retained ROS OccupancyGrid as a Nav2-compatible PGM/YAML pair."""

import argparse
import math
from pathlib import Path
import sys
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

from lite3_odom_tf_adapter.map_bundle import build_manifest, write_manifest


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


def _quaternion_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def save_grid(message, prefix):
    width = int(message.info.width)
    height = int(message.info.height)
    if width <= 0 or height <= 0 or len(message.data) != width * height:
        raise ValueError('OccupancyGrid dimensions do not match its cell data')

    prefix = Path(prefix).expanduser()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    image_path = prefix.with_suffix('.pgm')
    yaml_path = prefix.with_suffix('.yaml')

    free_limit = round(0.196 * 100)
    occupied_limit = round(0.65 * 100)

    def gray(value):
        if value < 0:
            return 205
        if value <= free_limit:
            return 254
        if value >= occupied_limit:
            return 0
        return 205

    with image_path.open('wb') as image:
        image.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        for row in reversed(range(height)):
            start = row * width
            image.write(bytes(gray(v) for v in message.data[start:start + width]))

    origin = message.info.origin
    yaw = _quaternion_yaw(origin.orientation)
    yaml_path.write_text(
        f'image: {image_path.name}\n'
        'mode: trinary\n'
        f'resolution: {message.info.resolution:.12g}\n'
        f'origin: [{origin.position.x:.12g}, {origin.position.y:.12g}, {yaw:.12g}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n',
        encoding='utf-8',
    )
    manifest_path = write_manifest(
        prefix,
        build_manifest(
            prefix, width=width, height=height,
            resolution=message.info.resolution,
            origin=(origin.position.x, origin.position.y, yaw),
            frame=message.header.frame_id or 'map'))
    return image_path, yaml_path, manifest_path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_prefix', help='Output path without .pgm/.yaml suffix')
    parser.add_argument('--timeout', type=float, default=15.0,
                        help='Seconds to wait for a retained /map (default: 15)')
    options, ros_args = parser.parse_known_args(args=args)
    if options.timeout <= 0.0:
        parser.error('--timeout must be positive')

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
