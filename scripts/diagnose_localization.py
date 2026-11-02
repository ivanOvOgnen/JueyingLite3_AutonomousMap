#!/usr/bin/env python3
"""Collect one read-only localization report while on the robot Wi-Fi."""

import argparse
from datetime import datetime
import ipaddress
import json
from pathlib import Path
import subprocess
import sys
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway
from deploy_supervised_nav_trial import cached_perception_address


ROOT = Path(__file__).resolve().parents[1]


def summarize(report):
    graph = report.get('ros_graph', {})
    components = report.get('api', {}).get('components', {})
    navigation_stack = components.get('navigation_stack', {}) or {}
    map_state = components.get('map', {}) or {}
    navigation = report.get('api', {}).get('navigation', {}) or {}
    if report.get('api_error'):
        return 'Robot API is unavailable; inspect API error in the report.'
    if not navigation_stack.get('process_running'):
        return ('Localization/Nav2 was stopped when this report was captured. '
                'Start it and rerun this probe while Stop localization/Nav2 is available.')
    if report.get('ros_probe_error'):
        return 'ROS probe failed; inspect ros_probe_error and Nav2 log in the report.'
    if graph.get('map_publishers') == 0:
        return 'No /map publisher: inspect map_server lifecycle and Nav2 log in the report.'
    if report.get('map_sample', {}).get('error'):
        return '/map has a publisher but no retained map arrived: inspect lifecycle and QoS.'
    if report.get('signature_matches') is False:
        return 'Published /map differs from the selected saved map; do not set a pose on it.'
    if report.get('signature_matches') is True and map_state.get('state') != 'ready':
        return 'The correct /map is published, but the API has not accepted it; inspect the API subscription/session.'
    if map_state.get('state') == 'ready' and graph.get('initialpose_subscribers') == 0:
        return 'Map is ready, but AMCL has not subscribed to /initialpose.'
    if map_state.get('state') == 'ready':
        if components.get('localization', {}).get('state') == 'ready':
            blockers = []
            if navigation.get('active_map_reviewed') is False:
                blockers.append('saved map is not reviewed for walking')
            if components.get('costmaps', {}).get('state') != 'ready':
                blockers.append('Nav2 costmaps are not ready')
            if components.get('obstacle_observations', {}).get('state') != 'ready':
                blockers.append('fresh depth obstacle observations are unavailable')
            if blockers:
                return 'Localization is ready; target walking is blocked: ' + '; '.join(blockers) + '.'
            return 'Localization and map are ready; inspect navigation and motor-output details before a target trial.'
        return 'Map is ready; the clicked robot pose can now be sent to AMCL.'
    return 'Map is not ready; inspect Nav2 log and ROS graph in the report.'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-host', help='verified motion-host IPv4')
    parser.add_argument('--perception-host', help='verified perception-host IPv4')
    options = parser.parse_args()
    if options.motion_host:
        motion = str(ipaddress.IPv4Address(options.motion_host))
    else:
        ssid, motion = robot_gateway()
        print('Robot Wi-Fi:', ssid, flush=True)
    perception = options.perception_host or cached_perception_address(motion)
    if not perception:
        config = command('ssh', '-o', 'ConnectTimeout=10', f'ysc@{motion}',
                         "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
        perception = perception_address(config)
    perception = str(ipaddress.IPv4Address(perception))

    stage = '/tmp/lite3_localization_probe_' + uuid.uuid4().hex[:12]
    control = '/tmp/lite3_localization_ssh_' + uuid.uuid4().hex[:12]
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
              '-o', 'ControlPersist=120', '-o', 'ControlPath=' + control,
              '-J', f'ysc@{motion}']
    target = f'ysc@{perception}'
    ssh = ['ssh', *common, target]
    subprocess.run([*ssh, 'mkdir -p ' + stage], check=True)
    source = Path(__file__).with_name('robot_localization_probe.py')
    subprocess.run(['scp', *common, str(source), target + ':' + stage + '/'], check=True)
    remote = ('source /opt/ros/foxy/setup.bash >/dev/null 2>&1; '
              'source ~/lite3_autonomy_ws/install/setup.bash >/dev/null 2>&1; '
              'python3 ' + stage + '/robot_localization_probe.py')
    result = subprocess.run([*ssh, 'bash -lc ' + repr(remote)], capture_output=True,
                            text=True, timeout=100)
    if result.returncode != 0:
        raise RuntimeError('Robot probe failed: ' + result.stderr[-3000:])
    report = json.loads(result.stdout)
    report['diagnosis'] = summarize(report)
    destination = ROOT / 'log' / ('localization_probe_' +
                  datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + '\n')
    print('Saved localization report:', destination, flush=True)
    print('Diagnosis:', report['diagnosis'], flush=True)
    components = report.get('api', {}).get('components', {})
    navigation = report.get('api', {}).get('navigation', {}) or {}
    if navigation.get('active_map_name'):
        print('Active map:', navigation['active_map_name'],
              'reviewed:', navigation.get('active_map_reviewed'), flush=True)
    if components.get('costmaps'):
        print('Costmaps:', json.dumps(components['costmaps'], sort_keys=True), flush=True)
    if components.get('obstacle_observations'):
        print('Depth obstacles:', json.dumps(components['obstacle_observations'], sort_keys=True), flush=True)
    if components.get('transforms'):
        print('Transforms:', json.dumps(components['transforms'], sort_keys=True), flush=True)
    if components.get('motion_output'):
        print('Motor output:', json.dumps(components['motion_output'], sort_keys=True), flush=True)
    for key in ('map_sample', 'saved_map', 'signature_matches', 'ros_graph',
                'costmap_samples', 'lifecycle_controller_server',
                'lifecycle_planner_server', 'ros_probe_error'):
        if key in report:
            print(key + ':', json.dumps(report[key]), flush=True)
    if 'navigation_log_tail' in report:
        print('Nav2 log tail saved in report.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError, subprocess.TimeoutExpired,
            RuntimeError, json.JSONDecodeError) as error:
        print('Localization diagnostic stopped:', error, file=sys.stderr)
        raise SystemExit(1)
