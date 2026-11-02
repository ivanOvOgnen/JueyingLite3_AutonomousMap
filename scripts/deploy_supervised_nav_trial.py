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
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway
from robot_install_supervised_nav_trial import FILES as ROBOT_FILES


ROOT = Path(__file__).resolve().parents[1]
FILES = tuple(ROOT/('ros2_ws/'+relative if relative.startswith('src/') else relative)
              for relative in ROBOT_FILES.values()) + (
    ROOT/'scripts/robot_install_supervised_nav_trial.py',
)


def ssh_common_options(control_path=None):
    """Return portable OpenSSH options for the deployment subprocesses.

    Windows OpenSSH does not support the Unix-domain ControlPath used for
    connection multiplexing and fails with ``getsockname: Not a socket``.
    Reusing a master connection is only an optimization, so keep it on POSIX
    and use ordinary SSH/SCP sessions on Windows.
    """
    # Windows opens a fresh two-hop connection for every transfer because its
    # OpenSSH build cannot use the Unix ControlPath below. The perception host
    # can take more than ten seconds to present its banner while Nav2/colcon
    # has recently been active, so allow setup margin and one TCP retry.
    connect_timeout = '30' if os.name == 'nt' else '10'
    options = ['-o', 'ConnectTimeout='+connect_timeout,
               '-o', 'ConnectionAttempts=2',
               '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3']
    if os.name != 'nt' and control_path:
        options.extend(['-o', 'ControlMaster=auto', '-o', 'ControlPersist=120',
                        '-o', 'ControlPath='+control_path])
    return options


def prepare_transfer_files(paths, directory):
    """Copy the bundle to a basename-only staging area with robot-safe EOLs."""
    directory = Path(directory)
    prepared = []
    for source in paths:
        target = directory/source.name
        shutil.copy2(source, target)
        if source.suffix == '.sh':
            data = target.read_bytes().replace(b'\r\n', b'\n').replace(b'\r', b'\n')
            target.write_bytes(data)
        prepared.append(target)
    return tuple(prepared)


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
    common = ssh_common_options(control_path)
    ssh = ['ssh', *common, '-J', jump]
    scp = ['scp', *common, '-o', 'ProxyJump='+jump]
    subprocess.run([*ssh, target, 'mkdir -m 700 '+shlex.quote(stage)], check=True)
    with tempfile.TemporaryDirectory(prefix='lite3-deploy-') as transfer_dir:
        transfer_files = prepare_transfer_files(FILES, transfer_dir)
        subprocess.run([*scp, *(str(path) for path in transfer_files), target+':'+stage+'/'], check=True)
    remote = ('python3 '+shlex.quote(stage+'/robot_install_supervised_nav_trial.py')+
              ' --stage '+shlex.quote(stage))
    subprocess.run([*ssh, target, remote+(' --recover-api' if options.apply else '')], check=True)
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
