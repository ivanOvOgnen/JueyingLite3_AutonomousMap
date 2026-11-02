#!/usr/bin/env python3
"""Reassert Lite3 Manual ownership and Move joystick mode, without velocity.

This only confirms UDP transmission. The operator must test the DEEP joystick.
The command is refused while a project navigation or ROS 1 velocity publisher
is active. This script does not change laptop Wi-Fi or vendor files/services.
"""

import argparse
from datetime import datetime, timezone
import inspect
import json
from pathlib import Path
import shlex
import socket
import struct
import subprocess
import sys

from connect_lite3_linux import command, perception_address, robot_gateway


ROOT = Path(__file__).resolve().parents[1]
MODE_CODES = ((0x21010C02, 'Manual ownership'),
              (0x21010D06, 'Move joystick interpretation'))
SSH_OPTIONS = ('-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
               '-o', 'ServerAliveCountMax=3')


def is_project_navigation_process(args):
    """Match executable arguments, never text inside a shell's -c program."""
    if not args:
        return False
    executable = args[0].rsplit('/', 1)[-1]
    if (executable in ('supervised_nav_relay', 'motion_arbiter') and
            '/lite3_odom_tf_adapter/' in args[0]):
        return True
    return ('launch' in args and 'lite3_odom_tf_adapter' in args and
            'navigation.launch.py' in args and
            (executable == 'ros2' or
             (executable.startswith('python') and len(args) > 1 and
              args[1].rsplit('/', 1)[-1] == 'ros2')))


def preflight(motion, perception):
    probe = ('import json,os,socket\nfrom pathlib import Path\nimport rosgraph\n'
             + inspect.getsource(is_project_navigation_process) + r'''
socket.setdefaulttimeout(3)
master=rosgraph.Master('/lite3_mode_restore_preflight')
publishers=dict(master.getSystemState()[0]).get('/cmd_vel',[])
project_nav=[]
for entry in Path('/proc').iterdir():
 if not entry.name.isdigit() or int(entry.name) in (os.getpid(),os.getppid()): continue
 try: args=[p.decode('utf-8','replace') for p in (entry/'cmdline').read_bytes().split(b'\0') if p]
 except (OSError,PermissionError): continue
 if is_project_navigation_process(args):
  project_nav.append({'pid':int(entry.name),'command':args[:5]})
print(json.dumps({'cmd_vel_publishers':publishers,'project_navigation_processes':project_nav}))''')
    remote = ('source /opt/ros/noetic/setup.bash >/dev/null 2>&1; python3 -c '
              + shlex.quote(probe))
    result = subprocess.run(
        ['ssh', *SSH_OPTIONS, '-J', 'ysc@'+motion, 'ysc@'+perception,
         '/bin/bash -lc '+shlex.quote(remote)],
        check=True, text=True, stdout=subprocess.PIPE, timeout=25)
    state = json.loads(result.stdout.strip().splitlines()[-1])
    if state['cmd_vel_publishers'] or state['project_navigation_processes']:
        raise RuntimeError('Cannot return modes while navigation or velocity '
                           'is active: '+json.dumps(state))
    return state


def restore(*, assume_neutral=False):
    ssid, motion = robot_gateway()
    config = command('ssh', *SSH_OPTIONS, 'ysc@'+motion,
                     "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
    perception = perception_address(config)
    state = preflight(motion, perception)
    if not assume_neutral:
        print('Keep the DEEP joysticks neutral; Manual + Move may respond immediately.')
        if input('Type YES to restore Manual + Move: ').strip() != 'YES':
            raise RuntimeError('No mode command sent')
    sent = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(2)
        for code, purpose in MODE_CODES:
            payload = struct.pack('<III', code, 0, 0)
            count = sock.sendto(payload, (motion, 43893))
            if count != len(payload):
                raise RuntimeError('Incomplete UDP transmission: '+purpose)
            sent.append({'code': hex(code), 'purpose': purpose, 'bytes': count})
    report = {'format': 'lite3_manual_mode_restore/v1',
              'at_utc': datetime.now(timezone.utc).isoformat(),
              'ssid': ssid, 'motion_host': motion, 'preflight': state,
              'sent': sent, 'acknowledgement_available': False}
    destination = ROOT/'log'/('manual_mode_restore_'+
                              datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    destination.parent.mkdir(exist_ok=True)
    destination.write_text(json.dumps(report, indent=2)+'\n')
    print('Manual + Move packets sent. Verify the DEEP joystick now. Report:',
          destination, flush=True)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--yes', action='store_true',
                        help='skip the neutral-stick prompt (for one-shot caller)')
    args = parser.parse_args()
    if not args.apply:
        print('Prepared:', ', '.join(hex(code) for code, _ in MODE_CODES))
        print('No packet sent. Use --apply after joining robot Wi-Fi.')
        return 0
    restore(assume_neutral=args.yes)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Manual mode restore stopped:', error, file=sys.stderr)
        raise SystemExit(1)
