#!/usr/bin/env python3
"""Stage and run the bounded vendor /cmd_vel trial without switching Wi-Fi.

Connect the laptop to the robot yourself. Default is read-only preflight.
--apply requires an interactive Auto Mode/app STOP confirmation on the laptop.
"""

import argparse
from datetime import datetime
import ipaddress
import json
from pathlib import Path
import shlex
import subprocess
import sys
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / 'scripts/robot_supervised_cmd_vel_trial.py'


def cached_perception_address(motion):
    """Reuse the last read-only audit's verified pair, avoiding another login."""
    manifests = sorted((ROOT / 'log').glob('vendor_interface_audit_*/manifest.json'),
                       key=lambda path: path.stat().st_mtime, reverse=True)
    for path in manifests:
        try:
            value = json.loads(path.read_text())
            if value.get('motion_host') == motion:
                return str(ipaddress.IPv4Address(value['perception_host']))
        except (OSError, ValueError, KeyError):
            continue
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Run the short physical vendor STOP trial')
    parser.add_argument('--motion-host', help='verified motion-host IPv4, otherwise robot Wi-Fi gateway')
    parser.add_argument('--perception-host', help='verified perception-host IPv4, otherwise network.toml')
    options = parser.parse_args()
    if not HELPER.is_file():
        parser.error('robot trial helper is missing')
    if options.motion_host:
        motion = options.motion_host
    else:
        ssid, motion = robot_gateway()
        print('Robot Wi-Fi:', ssid, flush=True)
    motion = str(ipaddress.IPv4Address(motion))
    if options.perception_host:
        perception = options.perception_host
    else:
        perception = cached_perception_address(motion)
        if perception:
            print('Perception host from latest robot audit:', perception, flush=True)
        else:
            config = command('ssh', '-o', 'ConnectTimeout=10', f'ysc@{motion}',
                             "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
            perception = perception_address(config)
    perception = str(ipaddress.IPv4Address(perception))
    stage = '/tmp/lite3_supervised_trial_' + uuid.uuid4().hex[:12]
    control_path = '/tmp/lite3_trial_ssh_' + uuid.uuid4().hex[:12]
    target = f'ysc@{perception}'
    jump = f'ysc@{motion}'
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
              '-o', 'ControlPersist=120', '-o', 'ControlPath=' + control_path]
    ssh = ['ssh', *common, '-J', jump]
    scp = ['scp', *common, '-o', 'ProxyJump=' + jump]
    subprocess.run([*ssh, target, 'mkdir -m 700 ' + shlex.quote(stage)], check=True)
    subprocess.run([*scp, str(HELPER), target + ':' + stage + '/'], check=True)
    remote_report = stage + '/report.json'
    remote_cmd = ('source /opt/ros/noetic/setup.bash >/dev/null 2>&1; '
                  'python3 ' +
                  shlex.quote(stage + '/robot_supervised_cmd_vel_trial.py') +
                  (' --apply' if options.apply else '') + ' --output ' + shlex.quote(remote_report))
    print('Robot command:', 'bounded physical trial' if options.apply else 'read-only preflight', flush=True)
    result = subprocess.run([*ssh[:1], '-tt', *ssh[1:], target,
                             '/bin/bash -lc ' + shlex.quote(remote_cmd)], check=False)
    destination = ROOT / 'log' / ('supervised_vendor_trial_' +
                                  datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
    copy = subprocess.run([*scp, target + ':' + remote_report, str(destination)], check=False)
    if copy.returncode == 0:
        print('Trial report:', destination, flush=True)
    else:
        print('Could not retrieve robot trial report; inspect remote ' + remote_report,
              file=sys.stderr)
    return result.returncode if result.returncode else copy.returncode


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('Supervised trial stopped:', error, file=sys.stderr)
        raise SystemExit(1)
