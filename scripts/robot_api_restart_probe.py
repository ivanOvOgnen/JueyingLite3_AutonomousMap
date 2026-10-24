#!/usr/bin/env python3
"""Inspect a failed project API restart and optionally retry API startup only."""

import argparse
from collections import deque
import json
from pathlib import Path
import subprocess
import time
import urllib.request


def api_status():
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open('http://127.0.0.1:8080/api/v1/status', timeout=3) as response:
            data = json.load(response)
        return {'reachable': True, 'ok': data.get('ok'),
                'api_contract_version': data.get('api_contract_version'),
                'capabilities': data.get('capabilities'),
                'stop_state': ((data.get('components') or {}).get('safety_stop') or {}).get('state')}
    except Exception as error:
        return {'reachable': False, 'error': type(error).__name__+': '+str(error)}


def processes():
    result = []
    needles = ('robot_api', 'start_robot_api.sh', 'vendor_cmd_vel_bridge',
               'supervised_nav_relay', 'motion_arbiter', 'cmd_vel_gate',
               'ros2 run lite3_odom_tf_adapter')
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            args = [part.decode('utf-8', 'replace') for part in
                    (item/'cmdline').read_bytes().split(b'\0') if part]
            if not args or not any(needle in ' '.join(args) for needle in needles):
                continue
            stat = (item/'stat').read_text().rsplit(')', 1)[1].split()
            result.append({'pid': int(item.name), 'state': stat[0],
                           'ppid': int(stat[1]), 'argv': args})
        except (OSError, ValueError, IndexError):
            continue
    return sorted(result, key=lambda item: item['pid'])


def listening_port():
    try:
        result = subprocess.run(['ss', '-ltnp', '( sport = :8080 )'], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=4, check=False)
        return result.stdout.strip()[:2500]
    except (OSError, subprocess.TimeoutExpired) as error:
        return type(error).__name__+': '+str(error)


def tail(path, count=80):
    if not path.is_file():
        return '<missing>'
    with path.open('r', errors='replace') as stream:
        lines = list(deque(stream, maxlen=count))
    return '\n'.join('[redacted]' if any(word in line.lower()
                                      for word in ('password', 'api_token', 'secret', 'pairing key'))
                     else line for line in lines)


def snapshot(workspace):
    logs = workspace/'logs'
    return {'api': api_status(), 'processes': processes(), 'port_8080': listening_port(),
            'logs': {name: tail(logs/name) for name in
                     ('supervised_nav_deploy.log', 'supervised_nav_rollback.log',
                      'robot_api_autostart.log', 'robot_api_amcl_hotfix.log',
                      'robot_api.log', 'robot_api_bridge.log',
                      'api_restart_probe.log')},
            'launcher': str(workspace/'scripts/start_robot_api.sh'),
            'launcher_exists': (workspace/'scripts/start_robot_api.sh').is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home()/'lite3_autonomy_ws')
    parser.add_argument('--recover', action='store_true',
                        help='If no API process or listener exists, start only the project API')
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    workspace = options.workspace.resolve()
    report = {'format': 'lite3_api_restart_probe/v1', 'before': snapshot(workspace)}
    if options.recover:
        before = report['before']
        active_api = [entry for entry in before['processes']
                      if any(arg.endswith('/robot_api') or arg == 'robot_api' or
                             arg.endswith('/start_robot_api.sh') for arg in entry['argv'])]
        active_command = [entry for entry in before['processes']
                          if any(arg.endswith(('/vendor_cmd_vel_bridge', '/supervised_nav_relay',
                                               '/motion_arbiter', '/cmd_vel_gate'))
                                 for arg in entry['argv'])]
        if before['api']['reachable']:
            report['recovery'] = 'API already answers; no process started'
        elif active_command:
            report['recovery'] = 'Project command process exists; no API recovery started'
        elif active_api or 'LISTEN' in before['port_8080']:
            report['recovery'] = 'API process or listener exists; no duplicate started'
        elif not before['launcher_exists']:
            report['recovery'] = 'Project API launcher is missing; no process started'
        else:
            log = workspace/'logs/api_restart_probe.log'
            with log.open('ab') as stream:
                process = subprocess.Popen(['/bin/bash', before['launcher']],
                                           cwd=workspace, stdin=subprocess.DEVNULL,
                                           stdout=stream, stderr=subprocess.STDOUT,
                                           start_new_session=True)
            report['recovery'] = 'Started project API launcher PID '+str(process.pid)
            for _ in range(15):
                if api_status()['reachable'] or process.poll() is not None:
                    break
                time.sleep(1)
            report['after'] = snapshot(workspace)
            report['api_restart_probe_log'] = tail(log)
    options.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
