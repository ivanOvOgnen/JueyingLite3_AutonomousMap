#!/usr/bin/env python3
"""Deploy only the AMCL active-goal refresh fix to the Lite3 perception host.

Run from the laptop after joining the robot Wi-Fi. The default checks source,
API idle state, install path, and patch compatibility without updating code.
Pass --apply to back up, patch, build, restart the project API, and verify it.
OpenSSH prompts for credentials normally; this script stores no passwords.
"""

import argparse
import ipaddress
from pathlib import Path
import shlex
import subprocess
import sys
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'patches/amcl_refresh_during_goal.patch'
HELPER = ROOT / 'scripts/robot_apply_amcl_hotfix.py'


def run(args):
    print('+', ' '.join(shlex.quote(str(arg)) for arg in args), flush=True)
    subprocess.run(args, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='perform the update after preflight')
    parser.add_argument('--motion-host', help='verified motion-host IPv4; omit to discover the robot Wi-Fi gateway')
    parser.add_argument('--perception-host', help='verified perception IPv4; omit to read motion-host network.toml')
    options = parser.parse_args()
    if not PATCH.is_file() or not HELPER.is_file():
        parser.error('hotfix patch or robot-side helper is missing')
    if options.motion_host:
        motion = options.motion_host
    else:
        ssid, motion = robot_gateway()
        print('Robot Wi-Fi:', ssid, 'motion host:', motion, flush=True)
    if options.perception_host:
        perception = options.perception_host
    else:
        config = command('ssh', '-o', 'ConnectTimeout=10', f'ysc@{motion}',
                         "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
        perception = perception_address(config)
    motion = str(ipaddress.IPv4Address(motion))
    perception = str(ipaddress.IPv4Address(perception))
    print('Perception host:', perception, flush=True)
    jump = f'ysc@{motion}'
    target = f'ysc@{perception}'
    stage = '/tmp/lite3_amcl_hotfix_' + uuid.uuid4().hex[:12]
    ssh_options = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
                   '-o', 'ServerAliveCountMax=3', '-J', jump]
    run(['ssh', *ssh_options, target, 'mkdir -m 700 ' + shlex.quote(stage)])
    run(['scp', '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
         '-o', 'ServerAliveCountMax=3', '-o', 'ProxyJump=' + jump,
         str(PATCH), str(HELPER), target + ':' + stage + '/'])
    remote = 'python3 ' + shlex.quote(stage + '/robot_apply_amcl_hotfix.py') + \
             ' --patch ' + shlex.quote(stage + '/amcl_refresh_during_goal.patch')
    run(['ssh', *ssh_options, target, remote])
    if options.apply:
        run(['ssh', *ssh_options, target, remote + ' --apply'])
    else:
        print('Dry run only. Re-run with --apply to install this hotfix.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('AMCL deployment stopped:', error, file=sys.stderr)
        sys.exit(1)
