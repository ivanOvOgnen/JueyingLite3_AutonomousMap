#!/usr/bin/env python3
"""Deploy the Room Mapper UI without changing vendor services or maps.

Connect the laptop to the robot network yourself. A dry run is the default;
--apply backs up and replaces the three web assets after API compatibility checks.
--recover-api first runs the guarded project-API recovery probe and saves its
report locally. It starts the project API only if no API process or listener exists.
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
from deploy_supervised_nav_trial import cached_perception_address


ROOT = Path(__file__).resolve().parents[1]
FILES = tuple(ROOT/'webapp'/name for name in ('app.js', 'styles.css', 'index.html')) + (
    ROOT/'scripts/deploy_web_ui.py',
    ROOT/'scripts/robot_api_restart_probe.py',
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--recover-api', action='store_true',
                        help='Safely start only the project API if absent before the UI deployment')
    parser.add_argument('--motion-host', help='verified motion-host IPv4')
    parser.add_argument('--perception-host', help='verified perception-host IPv4')
    options = parser.parse_args()
    for path in FILES:
        if not path.is_file():
            parser.error('Missing UI bundle file: '+str(path))
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
    stage = '/tmp/lite3_mapping_ui_'+uuid.uuid4().hex[:12]
    control_path = '/tmp/lite3_mapping_ui_ssh_'+uuid.uuid4().hex[:12]
    jump, target = f'ysc@{motion}', f'ysc@{perception}'
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
              '-o', 'ControlPersist=120', '-o', 'ControlPath='+control_path]
    ssh = ['ssh', *common, '-J', jump]
    scp = ['scp', *common, '-o', 'ProxyJump='+jump]
    subprocess.run([*ssh, target, 'mkdir -m 700 '+shlex.quote(stage)], check=True)
    subprocess.run([*scp, *(str(path) for path in FILES), target+':'+stage+'/'], check=True)
    if options.recover_api:
        remote_report = stage+'/api_probe.json'
        probe = ('python3 '+shlex.quote(stage+'/robot_api_restart_probe.py')+
                 ' --output '+shlex.quote(remote_report)+' --recover')
        subprocess.run([*ssh, target, probe], check=True, stdout=subprocess.PIPE, text=True)
        report_dir = ROOT/'log'
        report_dir.mkdir(exist_ok=True)
        local_report = report_dir/('ui_api_recovery_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
        subprocess.run([*scp, target+':'+remote_report, str(local_report)], check=True)
        report = json.loads(local_report.read_text())
        health = (report.get('after') or report.get('before') or {}).get('api') or {}
        print('Project API recovery report:', local_report, flush=True)
        print('Recovery result:', report.get('recovery', 'no action'), flush=True)
        if health.get('reachable') is not True or health.get('ok') is not True:
            raise RuntimeError('Project API still does not answer on robot port 8080; no UI files changed. '
                               'Inspect the saved recovery report and robot API logs.')
        if health.get('api_contract_version') != 2:
            raise RuntimeError('Project API answers, but its contract is not version 2; '
                               'no UI files changed. Inspect the saved report before updating the backend.')
    remote = ('python3 '+shlex.quote(stage+'/deploy_web_ui.py')+
              ' --source '+shlex.quote(stage))
    subprocess.run([*ssh, target, remote+(' --apply' if options.apply else '')], check=True)
    if not options.apply:
        print('Read-only UI preflight passed. Re-run with --apply to install.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Mapping-mode UI deployment stopped:', error, file=sys.stderr)
        raise SystemExit(1)
