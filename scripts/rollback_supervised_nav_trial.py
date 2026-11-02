#!/usr/bin/env python3
"""Restore the project to its robot-side backup from before the Nav2 trial.

Run from this repository while the laptop is manually connected to robot Wi-Fi.
The default is a dry run; --apply restores the saved project files and API.
"""

import argparse
import ipaddress
from pathlib import Path
import shlex
import subprocess
import sys
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway
from deploy_supervised_nav_trial import cached_perception_address


ROOT = Path(__file__).resolve().parents[1]
HELPERS = (ROOT/'scripts/robot_rollback_supervised_nav_trial.py',
           ROOT/'scripts/robot_install_supervised_nav_trial.py')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='restore after dry-run inspection')
    parser.add_argument('--motion-host', help='verified motion-host IPv4')
    parser.add_argument('--perception-host', help='verified perception-host IPv4')
    parser.add_argument('--backup', type=Path,
                        help='exact robot backup path created by the installation')
    options = parser.parse_args()
    if options.apply and not options.backup:
        parser.error('--apply requires --backup from the install report')
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
    print('Perception host:', perception, flush=True)
    stage = '/tmp/lite3_nav_rollback_'+uuid.uuid4().hex[:12]
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3']
    jump, target = f'ysc@{motion}', f'ysc@{perception}'
    ssh = ['ssh', *common, '-J', jump, target]
    scp = ['scp', *common, '-o', 'ProxyJump='+jump]
    subprocess.run([*ssh, 'mkdir -m 700 '+shlex.quote(stage)], check=True)
    subprocess.run([*scp, *(str(path) for path in HELPERS), target+':'+stage+'/'], check=True)
    remote = 'python3 '+shlex.quote(stage+'/robot_rollback_supervised_nav_trial.py')
    if options.backup:
        remote += ' --backup '+shlex.quote(str(options.backup))
    subprocess.run([*ssh, remote + (' --apply' if options.apply else '')], check=True)
    if not options.apply:
        print('Dry run complete. Run this same command with --apply to restore.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('Supervised Nav2 rollback stopped:', error, file=sys.stderr)
        raise SystemExit(1)
