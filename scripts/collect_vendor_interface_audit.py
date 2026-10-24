#!/usr/bin/env python3
"""Collect read-only vendor mode and STOP evidence from both Lite3 hosts.

Run on the laptop while it is connected to robot Wi-Fi. No password is stored.
This script does not switch networks or send any robot-control command.
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
HELPER = ROOT / 'scripts/robot_vendor_interface_audit.py'


def run(args):
    print('+', ' '.join(shlex.quote(str(arg)) for arg in args), flush=True)
    subprocess.run(args, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-host', help='verified motion-host IPv4; otherwise robot Wi-Fi gateway')
    parser.add_argument('--perception-host', help='verified perception-host IPv4; otherwise network.toml')
    options = parser.parse_args()
    if not HELPER.is_file():
        parser.error('robot-side vendor audit helper is missing')
    if options.motion_host:
        motion = options.motion_host
    else:
        ssid, motion = robot_gateway()
        print('Robot Wi-Fi:', ssid, flush=True)
    motion = str(ipaddress.IPv4Address(motion))
    if options.perception_host:
        perception = options.perception_host
    else:
        config = command('ssh', '-o', 'ConnectTimeout=10', f'ysc@{motion}',
                         "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
        perception = perception_address(config)
    perception = str(ipaddress.IPv4Address(perception))
    stage = '/tmp/lite3_vendor_audit_' + uuid.uuid4().hex[:12]
    root = ROOT / 'log' / ('vendor_interface_audit_' + datetime.now().strftime('%Y%m%d_%H%M%S')
                            + '_' + uuid.uuid4().hex[:6])
    root.mkdir(parents=True, exist_ok=False)
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3']
    targets = (
        ('motion', f'ysc@{motion}', []),
        ('perception', f'ysc@{perception}', ['-J', f'ysc@{motion}']),
    )
    for role, target, jump in targets:
        ssh = ['ssh', *common, *jump]
        scp = ['scp', *common]
        if jump:
            scp += ['-o', 'ProxyJump=' + jump[1]]
        run([*ssh, target, 'mkdir -m 700 ' + shlex.quote(stage)])
        run([*scp, str(HELPER), target + ':' + stage + '/'])
        remote = ('python3 ' + shlex.quote(stage + '/robot_vendor_interface_audit.py')
                  + ' --role ' + role + ' --output ' + shlex.quote(stage + '/' + role + '.json'))
        run([*ssh, target, remote])
        run([*scp, target + ':' + stage + '/' + role + '.json', str(root / (role + '.json'))])
    manifest = {'format': 'lite3_vendor_interface_bundle/v1', 'motion_host': motion,
                'perception_host': perception,
                'reports': ['motion.json', 'perception.json']}
    (root / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Vendor interface audit:', root, flush=True)
    print('Reconnect the laptop to the internet and share motion.json and perception.json.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('Vendor interface audit stopped:', error, file=sys.stderr)
        sys.exit(1)
