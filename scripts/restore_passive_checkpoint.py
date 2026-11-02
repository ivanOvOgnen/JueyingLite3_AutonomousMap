#!/usr/bin/env python3
"""Restore the Lite3 project to the accepted 2026-10-21 passive checkpoint.

Run on the laptop after manually joining robot Wi-Fi. The script never changes
Wi-Fi. Default performs robot preflight; --apply backs up, restores and builds
only the project package, then leaves its API stopped for a manual-control test.
"""

import argparse
import hashlib
import ipaddress
import io
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tarfile
import tempfile
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway
from deploy_supervised_nav_trial import cached_perception_address


ROOT = Path(__file__).resolve().parents[1]
COMMIT = '51cb1f39b9f99d3f432a5726cd5c00be04e34762'
HELPER = ROOT/'scripts/robot_restore_passive_checkpoint.py'


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, check=True,
                          stdout=subprocess.PIPE).stdout


def checkpoint_files():
    names = git('ls-tree', '-r', '--name-only', COMMIT).decode().splitlines()
    return [name for name in names if
            (name.startswith('ros2_ws/src/lite3_odom_tf_adapter/') and
             not name.startswith('ros2_ws/src/lite3_odom_tf_adapter/maps/')) or
            name in ('webapp/app.js', 'webapp/index.html', 'webapp/styles.css',
                     'scripts/start_robot_api.sh')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='restore after idle robot preflight')
    parser.add_argument('--motion-host', help='verified motion-host IPv4')
    parser.add_argument('--perception-host', help='verified perception-host IPv4')
    options = parser.parse_args()
    if git('rev-parse', COMMIT).decode().strip() != COMMIT:
        raise RuntimeError('Accepted checkpoint commit is unavailable')
    names = checkpoint_files()
    if len(names) < 25 or 'ros2_ws/src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/robot_api.py' not in names:
        raise RuntimeError('Incomplete accepted checkpoint file list')
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
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3']
    jump, target = f'ysc@{motion}', f'ysc@{perception}'
    ssh = ['ssh', *common, '-J', jump, target]
    scp = ['scp', *common, '-o', 'ProxyJump='+jump]
    stage = '/tmp/lite3_checkpoint_'+uuid.uuid4().hex[:12]
    with tempfile.TemporaryDirectory(prefix='lite3_checkpoint_') as folder:
        archive = Path(folder)/'checkpoint.tar'
        manifest_path = Path(folder)/'manifest.json'
        manifest = {'format': 'lite3_passive_checkpoint_restore/v1',
                    'commit': COMMIT, 'files': {}}
        with tarfile.open(archive, 'w:') as tar:
            for name in names:
                data = git('show', COMMIT+':'+name)
                manifest['files'][name] = hashlib.sha256(data).hexdigest()
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = 0o644
                tar.addfile(info, io.BytesIO(data))
        manifest['archive_sha256'] = hashlib.sha256(archive.read_bytes()).hexdigest()
        manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
        subprocess.run([*ssh, 'mkdir -m 700 '+shlex.quote(stage)], check=True)
        subprocess.run([*scp, str(archive), str(manifest_path), str(HELPER),
                        target+':'+stage+'/'], check=True)
        remote = 'python3 '+shlex.quote(stage+'/robot_restore_passive_checkpoint.py')+\
                 ' --stage '+shlex.quote(stage)
        subprocess.run([*ssh, remote+(' --apply' if options.apply else '')], check=True)
    if options.apply:
        print('Restore finished. The project API is off; test the DEEP manual controller now.', flush=True)
    else:
        print('Dry run passed. Re-run with --apply to restore the checkpoint.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Checkpoint restore stopped:', error, file=sys.stderr)
        raise SystemExit(1)
