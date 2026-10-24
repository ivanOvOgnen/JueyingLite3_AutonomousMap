#!/usr/bin/env python3
"""Read-only Nav2 map/AMCL probe run on the perception computer."""

import json
import math
from pathlib import Path
import re
import subprocess
import time
import traceback
from collections import Counter
from urllib.request import build_opener, ProxyHandler


def bounded(argv, seconds=8):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=seconds)
        return {'returncode': result.returncode, 'stdout': result.stdout[-10000:],
                'stderr': result.stderr[-3000:]}
    except subprocess.TimeoutExpired:
        return {'error': 'timed out after {} seconds'.format(seconds)}


def main():
    report = {'format': 'lite3_localization_probe/v1', 'read_only': True}
    try:
        opener = build_opener(ProxyHandler({}))
        with opener.open('http://127.0.0.1:8080/api/v1/status', timeout=5) as response:
            status = json.load(response)
        report['api'] = {
            'ok': status.get('ok'),
            'api_contract_version': status.get('api_contract_version'),
            'components': {key: status.get('components', {}).get(key) for key in
                           ('map', 'scan', 'odometry', 'localization', 'navigation_stack',
                            'transforms', 'costmaps', 'obstacle_observations',
                            'motion_output', 'safety_stop')},
            'navigation': status.get('components', {}).get('navigation'),
            'map': status.get('map'),
        }
    except Exception as error:
        report['api_error'] = str(error)

    log = Path.home() / 'lite3_autonomy_ws/logs/robot_api_navigation.log'
    try:
        with log.open('rb') as source:
            size = source.seek(0, 2)
            source.seek(max(0, size - 750000))
            history = source.read().decode('utf-8', 'replace')
        report['navigation_log_tail'] = history[-16000:]
        map_events = re.compile(
            r'\[map_server|\[lifecycle_manager_localization|yaml_filename|'
            r'Failed to (?:load|configure|bring up).*map', re.IGNORECASE)
        errors = re.compile(r'\[ERROR\]|\[FATAL\]|Traceback|Exception', re.IGNORECASE)
        lines = history.splitlines()
        report['map_server_log_events'] = [line[-700:] for line in lines if map_events.search(line)][-100:]
        report['navigation_error_events'] = [line[-700:] for line in lines if errors.search(line)][-100:]
        report['costmap_log_events'] = [line[-700:] for line in lines
                                        if 'costmap' in line.lower() and
                                        any(word in line.lower() for word in
                                            ('warn', 'error', 'activating', 'initializing', 'timeout'))][-100:]
    except OSError as error:
        report['navigation_log_error'] = str(error)

    report['lifecycle_map_server'] = bounded(['ros2', 'lifecycle', 'get', '/map_server'])
    report['lifecycle_amcl'] = bounded(['ros2', 'lifecycle', 'get', '/amcl'])
    report['lifecycle_controller_server'] = bounded(['ros2', 'lifecycle', 'get', '/controller_server'])
    report['lifecycle_planner_server'] = bounded(['ros2', 'lifecycle', 'get', '/planner_server'])
    report['map_server_yaml'] = bounded(['ros2', 'param', 'get', '/map_server', 'yaml_filename'])
    report['processes'] = bounded(['ps', '-eo', 'pid,args'])
    if 'stdout' in report['processes']:
        report['processes']['stdout'] = '\n'.join(
            line for line in report['processes']['stdout'].splitlines()
            if any(marker in line for marker in ('navigation.launch.py', 'map_server',
                                                '/amcl', 'lifecycle_manager_localization',
                                                '/robot_api')))

    try:
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
        from lite3_odom_tf_adapter.map_identity import grid_signature, read_bundle

        rclpy.init()
        node = Node('lite3_readonly_localization_probe')
        received = []
        costmap_samples = {'local': [], 'global': []}
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = node.create_subscription(OccupancyGrid, '/map', received.append, qos)
        costmap_subscriptions = [
            node.create_subscription(OccupancyGrid, '/{}_costmap/costmap'.format(name),
                                     costmap_samples[name].append, 10)
            for name in ('local', 'global')]
        try:
            deadline = time.monotonic() + 9.0
            while time.monotonic() < deadline and (
                    not received or not costmap_samples['local'] or not costmap_samples['global']):
                rclpy.spin_once(node, timeout_sec=0.1)
            report['ros_graph'] = {
                'map_publishers': node.count_publishers('/map'),
                'map_subscribers': node.count_subscribers('/map'),
                'amcl_pose_publishers': node.count_publishers('/amcl_pose'),
                'initialpose_subscribers': node.count_subscribers('/initialpose'),
                'local_costmap_publishers': node.count_publishers('/local_costmap/costmap'),
                'local_costmap_subscribers': node.count_subscribers('/local_costmap/costmap'),
                'global_costmap_publishers': node.count_publishers('/global_costmap/costmap'),
                'global_costmap_subscribers': node.count_subscribers('/global_costmap/costmap'),
                'depth_obstacle_publishers': node.count_publishers('/perception/depth_obstacles'),
                'nodes': sorted(name for name, _namespace in node.get_node_names_and_namespaces()
                                if name in ('map_server', 'amcl', 'lifecycle_manager_localization')),
            }
            report['costmap_samples'] = {
                name: ({'frame': samples[-1].header.frame_id,
                        'stamp_ns': int(samples[-1].header.stamp.sec) * 1000000000 +
                                    int(samples[-1].header.stamp.nanosec),
                        'width': int(samples[-1].info.width),
                        'height': int(samples[-1].info.height),
                        'resolution': float(samples[-1].info.resolution),
                        'cell_count': len(samples[-1].data)} if samples else None)
                for name, samples in costmap_samples.items()}
            if received:
                grid = received[0]
                q = grid.info.origin.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                report['map_sample'] = {
                    'width': grid.info.width, 'height': grid.info.height,
                    'resolution': grid.info.resolution,
                    'frame': grid.header.frame_id,
                    'origin': [grid.info.origin.position.x, grid.info.origin.position.y, yaw],
                    'raw_cell_counts': dict(Counter(str(value) for value in grid.data)),
                    'signature': grid_signature(grid),
                }
            else:
                report['map_sample'] = {'error': 'No retained /map received in 9 seconds'}
        finally:
            node.destroy_subscription(subscription)
            for item in costmap_subscriptions:
                node.destroy_subscription(item)
            node.destroy_node()
            rclpy.shutdown()

        map_name = report.get('api', {}).get('navigation', {}).get('active_map_name')
        if map_name:
            prefix = Path.home() / 'lite3_autonomy_ws/webapp/maps' / map_name
            bundle = read_bundle(prefix, allow_legacy=True)
            report['saved_map'] = {
                'name': map_name, 'width': bundle['width'], 'height': bundle['height'],
                'grid_signature': bundle['grid_signature'],
                'origin': bundle['origin'],
                'cell_counts': dict(Counter(str(value) for value in bundle['cells'])),
                'yaml': prefix.with_suffix('.yaml').read_text(errors='replace'),
            }
            if report.get('map_sample', {}).get('signature'):
                report['signature_matches'] = (
                    report['map_sample']['signature'] == bundle['grid_signature'])
                actual = [-1 if value < 0 or 19 < value < 65 else
                          (0 if value <= 19 else 100) for value in grid.data]
                expected = bundle['cells']
                if len(actual) == len(expected):
                    differing = [index for index, (a, b) in enumerate(zip(actual, expected)) if a != b]
                    width, height = bundle['width'], bundle['height']
                    flipped = [expected[(height - 1 - row) * width + col]
                               for row in range(height) for col in range(width)]
                    report['map_comparison'] = {
                        'different_cells': len(differing),
                        'vertical_flip_different_cells': sum(a != b for a, b in zip(actual, flipped)),
                        'actual_canonical_counts': dict(Counter(str(value) for value in actual)),
                        'examples': [{'index': index, 'actual_raw': int(grid.data[index]),
                                      'actual_canonical': actual[index], 'saved': expected[index]}
                                     for index in differing[:20]],
                    }
    except Exception as error:
        report['ros_probe_error'] = str(error)
        report['ros_probe_traceback'] = traceback.format_exc()[-4000:]

    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
