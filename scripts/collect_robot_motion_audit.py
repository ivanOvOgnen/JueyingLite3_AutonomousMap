#!/usr/bin/env python3
"""Collect a read-only robot motion audit over SSH from this laptop.

Join the robot Wi-Fi, then run this file. SSH prompts for credentials normally.
Only a temporary audit helper and report are written under the robot's /tmp.
"""

import argparse
from datetime import datetime
import ipaddress
from pathlib import Path
import shlex
import subprocess
import sys
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / 'scripts/robot_motion_audit.py'
PREFLIGHT = ROOT / 'scripts/robot_preflight.sh'


def run(args):
    print('+', ' '.join(shlex.quote(str(arg)) for arg in args), flush=True)
    subprocess.run(args, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-host', help='verified motion-host IPv4; otherwise use the robot Wi-Fi gateway')
    parser.add_argument('--perception-host', help='verified perception-host IPv4; otherwise read network.toml')
    options = parser.parse_args()
    if not HELPER.is_file() or not PREFLIGHT.is_file():
        parser.error('robot audit helper or preflight script is missing')
    if options.motion_host:
        motion = options.motion_host
    else:
        ssid, motion = robot_gateway()
        print('Robot Wi-Fi:', ssid, 'motion host:', motion, flush=True)
    motion = str(ipaddress.IPv4Address(motion))
    if options.perception_host:
        perception = options.perception_host
    else:
        config = command('ssh', '-o', 'ConnectTimeout=10', f'ysc@{motion}',
                         "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
        perception = perception_address(config)
    perception = str(ipaddress.IPv4Address(perception))
    jump = f'ysc@{motion}'
    target = f'ysc@{perception}'
    stage = '/tmp/lite3_motion_audit_' + uuid.uuid4().hex[:12]
    ssh_options = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
                   '-o', 'ServerAliveCountMax=3', '-J', jump]
    scp_options = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
                   '-o', 'ServerAliveCountMax=3', '-o', 'ProxyJump=' + jump]
    destination = ROOT / 'log' / ('robot_motion_audit_' + datetime.now().strftime('%Y%m%d_%H%M%S')
                                  + '_' + uuid.uuid4().hex[:6] + '.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    print('Perception host:', perception, flush=True)
    run(['ssh', *ssh_options, target, 'mkdir -m 700 ' + shlex.quote(stage)])
    run(['scp', *scp_options, str(HELPER), str(PREFLIGHT), target + ':' + stage + '/'])
    remote = ('python3 ' + shlex.quote(stage + '/robot_motion_audit.py')
              + ' --preflight ' + shlex.quote(stage + '/robot_preflight.sh')
              + ' --output ' + shlex.quote(stage + '/report.json'))
    run(['ssh', *ssh_options, target, remote])
    run(['scp', *scp_options, target + ':' + stage + '/report.json', str(destination)])
    print('Audit report:', destination, flush=True)
    print('Share this report after reconnecting the laptop to the internet.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('Motion audit stopped:', error, file=sys.stderr)
        sys.exit(1)
