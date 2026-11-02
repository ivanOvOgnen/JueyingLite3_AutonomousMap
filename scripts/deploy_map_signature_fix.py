#!/usr/bin/env python3
"""Install the backed-up zero-yaw map signature fix; run on robot Wi-Fi."""

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
from deploy_supervised_nav_trial import cached_perception_address


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='perform the backed-up update')
    parser.add_argument('--motion-host', help='verified motion-host IPv4')
    parser.add_argument('--perception-host', help='verified perception-host IPv4')
    args = parser.parse_args()
    if args.motion_host:
        motion = str(ipaddress.IPv4Address(args.motion_host))
    else:
        ssid, motion = robot_gateway()
        print('Robot Wi-Fi:', ssid, flush=True)
    perception = args.perception_host or cached_perception_address(motion)
    if not perception:
        config = command('ssh', '-o', 'ConnectTimeout=10', f'ysc@{motion}',
                         "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
        perception = perception_address(config)
    perception = str(ipaddress.IPv4Address(perception))
    stage = '/tmp/lite3_map_signature_' + uuid.uuid4().hex[:12]
    control = '/tmp/lite3_map_signature_ssh_' + uuid.uuid4().hex[:12]
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
              '-o', 'ControlPersist=120', '-o', 'ControlPath=' + control,
              '-J', f'ysc@{motion}']
    target = f'ysc@{perception}'
    ssh = ['ssh', *common, target]
    subprocess.run([*ssh, 'mkdir -m 700 ' + shlex.quote(stage)], check=True)
    helper = Path(__file__).with_name('robot_apply_map_signature_fix.py')
    subprocess.run(['scp', *common, str(helper), target + ':' + stage + '/'], check=True)
    remote = 'python3 ' + shlex.quote(stage + '/' + helper.name)
    subprocess.run([*ssh, remote], check=True)
    if not args.apply:
        print('Dry run passed. Re-run with --apply to install.', flush=True)
        return
    result = subprocess.run([*ssh, remote + ' --apply'], check=True,
                            stdout=subprocess.PIPE, text=True)
    print(result.stdout, end='', flush=True)
    lines = [line for line in result.stdout.splitlines() if line.startswith('{')]
    deployed = next((json.loads(line) for line in reversed(lines)
                     if json.loads(line).get('deployed') is True), None)
    if deployed is None:
        raise RuntimeError('Robot did not confirm deployment')
    destination = ROOT / 'log' / ('map_signature_deploy_' +
                  datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(deployed, indent=2) + '\n')
    print('Saved deployment report:', destination, flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Map signature deployment stopped:', error, file=sys.stderr)
        raise SystemExit(1)
