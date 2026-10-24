#!/usr/bin/env python3
"""Collect robot API startup evidence without switching laptop Wi-Fi.

Optional --recover starts only the project API if neither its process nor
port 8080 is active; it never starts Nav2 or sends motion commands.
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
HELPER = ROOT/'scripts/robot_api_restart_probe.py'


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
    parser.add_argument('--recover', action='store_true', help='Retry project API startup if safe')
    parser.add_argument('--motion-host', help='verified motion-host IPv4')
    parser.add_argument('--perception-host', help='verified perception-host IPv4')
    options = parser.parse_args()
    if not HELPER.is_file():
        parser.error('Robot-side probe is missing')
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
    stage = '/tmp/lite3_api_probe_'+uuid.uuid4().hex[:12]
    control = '/tmp/lite3_api_probe_ssh_'+uuid.uuid4().hex[:12]
    jump, target = f'ysc@{motion}', f'ysc@{perception}'
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
              '-o', 'ControlPersist=120', '-o', 'ControlPath='+control]
    ssh = ['ssh', *common, '-J', jump]
    scp = ['scp', *common, '-o', 'ProxyJump='+jump]
    subprocess.run([*ssh, target, 'mkdir -m 700 '+shlex.quote(stage)], check=True)
    subprocess.run([*scp, str(HELPER), target+':'+stage+'/'], check=True)
    remote_report = stage+'/report.json'
    remote = ('python3 '+shlex.quote(stage+'/robot_api_restart_probe.py')+
              ' --output '+shlex.quote(remote_report)+(' --recover' if options.recover else ''))
    subprocess.run([*ssh, target, remote], check=True)
    local_report = ROOT/'log'/('api_restart_probe_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    subprocess.run([*scp, target+':'+remote_report, str(local_report)], check=True)
    print('Saved report:', local_report, flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('API probe stopped:', error, file=sys.stderr)
        raise SystemExit(1)
