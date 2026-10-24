#!/usr/bin/env python3
"""Stage and install the bounded map-click Nav2 trial without changing laptop Wi-Fi.

Connect to the robot network yourself. The default is a read-only robot
preflight; --apply verifies all project stacks are idle, latches and verifies
project STOP, then backs up, builds, and
restarts only the project API. It never starts Nav2 or arms a motor command.
"""

import argparse
import ipaddress
import json
from pathlib import Path
import shlex
import subprocess
import sys
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway
from robot_install_supervised_nav_trial import FILES as ROBOT_FILES


ROOT = Path(__file__).resolve().parents[1]
FILES = tuple(ROOT/('ros2_ws/'+relative if relative.startswith('src/') else relative)
              for relative in ROBOT_FILES.values()) + (
    ROOT/'scripts/robot_install_supervised_nav_trial.py',
)


def cached_perception_address(motion):
    manifests = sorted((ROOT/'log').glob('vendor_interface_audit_*/manifest.json'),
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
    parser.add_argument('--apply', action='store_true', help='Deploy after backed-up robot preflight')
    parser.add_argument('--motion-host', help='verified motion-host IPv4')
    parser.add_argument('--perception-host', help='verified perception-host IPv4')
    options = parser.parse_args()
    for path in FILES:
        if not path.is_file():
            parser.error('Missing bundle file: '+str(path))
    if len({path.name for path in FILES}) != len(FILES):
        parser.error('Bundle basenames must be distinct')
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
    stage = '/tmp/lite3_supervised_nav_'+uuid.uuid4().hex[:12]
    control_path = '/tmp/lite3_nav_ssh_'+uuid.uuid4().hex[:12]
    jump, target = f'ysc@{motion}', f'ysc@{perception}'
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
              '-o', 'ControlPersist=120', '-o', 'ControlPath='+control_path]
    ssh = ['ssh', *common, '-J', jump]
    scp = ['scp', *common, '-o', 'ProxyJump='+jump]
    subprocess.run([*ssh, target, 'mkdir -m 700 '+shlex.quote(stage)], check=True)
    subprocess.run([*scp, *(str(path) for path in FILES), target+':'+stage+'/'], check=True)
    remote = ('python3 '+shlex.quote(stage+'/robot_install_supervised_nav_trial.py')+
              ' --stage '+shlex.quote(stage))
    subprocess.run([*ssh, target, remote], check=True)
    if options.apply:
        subprocess.run([*ssh, target, remote+' --apply'], check=True)
    else:
        print('Read-only preflight passed. Re-run with --apply to install.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('Supervised Nav2 deployment stopped:', error, file=sys.stderr)
        raise SystemExit(1)
