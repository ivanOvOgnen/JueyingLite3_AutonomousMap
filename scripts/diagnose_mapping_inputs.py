#!/usr/bin/env python3
"""Collect current Lite3 camera/odometry/mapping evidence without changing the robot.

Run on the laptop while connected to the robot network. This script performs
read-only SSH queries and writes one report under the local log directory.
"""

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
QUERIES = {
    'api_status': "curl --noproxy '*' -fsS --max-time 6 http://127.0.0.1:8080/api/v1/status",
    'preflight': 'bash ~/lite3_autonomy_ws/scripts/robot_preflight.sh',
    'mapping_log': 'tail -n 100 ~/lite3_autonomy_ws/logs/robot_api_mapping.log',
    'bridge_log': 'tail -n 100 ~/lite3_autonomy_ws/logs/robot_api_bridge.log',
    'api_log': 'tail -n 100 ~/lite3_autonomy_ws/logs/supervised_nav_deploy.log',
    'usb_devices': 'lsusb',
}


def redacted(value):
    return '\n'.join('[redacted]' if any(word in line.lower() for word in
                     ('password', 'api_token', 'secret', 'pairing key')) else line
                     for line in value.splitlines())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-host', help='verified motion-host IPv4')
    parser.add_argument('--perception-host', help='verified perception-host IPv4')
    options = parser.parse_args()
    if options.motion_host:
        motion = options.motion_host
    else:
        ssid, motion = robot_gateway()
        print('Robot Wi-Fi:', ssid, flush=True)
    motion = str(ipaddress.IPv4Address(motion))
    perception = options.perception_host or cached_perception_address(motion)
    if not perception:
        config = command('ssh', '-o', 'ConnectTimeout=10', f'ysc@{motion}',
                         "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
        perception = perception_address(config)
    perception = str(ipaddress.IPv4Address(perception))
    control_path = '/tmp/lite3_mapping_probe_ssh_'+uuid.uuid4().hex[:12]
    ssh = ['ssh', '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
           '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
           '-o', 'ControlPersist=120', '-o', 'ControlPath='+control_path,
           '-J', f'ysc@{motion}', f'ysc@{perception}']
    report = {'format': 'lite3_mapping_inputs/v1', 'motion_host': motion,
              'perception_host': perception, 'queries': {}}
    for name, remote in QUERIES.items():
        print('Collecting', name, flush=True)
        try:
            result = subprocess.run([*ssh, remote], text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    timeout=75 if name == 'preflight' else 20)
            report['queries'][name] = {'returncode': result.returncode,
                                       'stdout': redacted(result.stdout),
                                       'stderr': redacted(result.stderr)}
        except subprocess.TimeoutExpired:
            report['queries'][name] = {'error': 'Remote query timed out'}
    destination = ROOT/'log'/('mapping_inputs_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2)+'\n')
    print('Saved mapping diagnostic:', destination, flush=True)
    status = report['queries'].get('api_status', {})
    if status.get('returncode') == 0:
        try:
            components = json.loads(status['stdout']).get('components', {})
            for name in ('camera', 'vendor_odometry', 'mapping', 'motion_output'):
                print(name+':', json.dumps(components.get(name, {}), sort_keys=True), flush=True)
        except ValueError:
            print('API status was not JSON; see report.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('Mapping diagnostic stopped:', error, file=sys.stderr)
        raise SystemExit(1)
