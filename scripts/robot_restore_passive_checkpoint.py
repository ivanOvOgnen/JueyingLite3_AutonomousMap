#!/usr/bin/env python3
"""Restore the committed 2026-10-21 Lite3 project files on the robot.

Only ~/lite3_autonomy_ws is changed. Vendor services, the DEEP Android app,
firmware, maps, and recordings are never changed. The project API is left off.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tarfile
import time
from urllib.request import ProxyHandler, build_opener


WORKSPACE = Path.home()/'lite3_autonomy_ws'
PACKAGE = WORKSPACE/'src/lite3_odom_tf_adapter'
SOURCE_GROUPS = ('config', 'launch', 'lite3_odom_tf_adapter')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inside(path, root):
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def project_pids():
    prefix = str(WORKSPACE/'install/lite3_odom_tf_adapter/lib/lite3_odom_tf_adapter/').encode()
    found = {}
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            args = [part for part in (item/'cmdline').read_bytes().split(b'\0') if part]
        except (OSError, PermissionError):
            continue
        executable = next((part for part in args[:2] if part.startswith(prefix)), None)
        if executable:
            found[int(item.name)] = executable.decode('utf-8', 'replace')
    return found


def api_status():
    try:
        with build_opener(ProxyHandler({})).open(
                'http://127.0.0.1:8080/api/v1/status', timeout=5) as response:
            result = json.load(response)
        if result.get('simulation') is True or not isinstance(result.get('components'), dict):
            raise RuntimeError('Unexpected project API response')
        return result
    except OSError:
        return None


def preflight():
    pids = project_pids()
    status = api_status()
    api = [pid for pid, path in pids.items() if path.endswith('/robot_api')]
    if len(api) > 1:
        raise RuntimeError('Multiple project APIs are running: '+str(api))
    if status is None and api:
        raise RuntimeError('Project API process is running but status is unavailable; stop it first')
    if status is not None and not api:
        raise RuntimeError('An unrecognized API responds on port 8080')
    if status is not None:
        parts = status['components']
        reasons = []
        if (parts.get('mapping') or {}).get('process_running'):
            reasons.append('mapping running')
        if (parts.get('navigation_stack') or {}).get('process_running'):
            reasons.append('Nav2 running')
        if (parts.get('navigation') or {}).get('unresolved_goal_count'):
            reasons.append('unresolved goal')
        if (parts.get('safety_stop') or {}).get('state') != 'latched':
            reasons.append('project STOP not latched')
        if (parts.get('safety_stop') or {}).get('cleanup_pending'):
            reasons.append('STOP cleanup pending')
        if reasons:
            raise RuntimeError('Project must be idle before restore: '+', '.join(reasons))
    other = {pid: path for pid, path in pids.items() if pid not in api}
    allowed_relays = {'vendor_cmd_vel_bridge', 'supervised_nav_relay'}
    unknown = {pid: path for pid, path in other.items()
               if Path(path).name not in allowed_relays}
    if unknown:
        raise RuntimeError('Unexpected project node still running; inspect before restore: '+str(unknown))
    # These are exact executables under this project's install tree. Stopping
    # them removes project velocity output and does not signal vendor services.
    return api, other


def stop_project_pid(pid, expected):
    if project_pids().get(pid) != expected:
        return
    for sig, seconds in ((signal.SIGINT, 6), (signal.SIGTERM, 6)):
        if project_pids().get(pid) != expected:
            return
        print('Stopping project process:', pid, Path(expected).name, sig.name, flush=True)
        os.kill(pid, sig)
        deadline = time.monotonic()+seconds
        while project_pids().get(pid) == expected and time.monotonic() < deadline:
            time.sleep(0.25)
    if project_pids().get(pid) == expected:
        raise RuntimeError('Project process did not stop: '+str(pid)+' '+expected)


def target_path(entry):
    if entry.startswith('ros2_ws/'):
        return WORKSPACE/entry[len('ros2_ws/'):]
    return WORKSPACE/entry


def package_extras(expected):
    extras = []
    for group in SOURCE_GROUPS:
        root = PACKAGE/group
        if not root.is_dir():
            continue
        for path in root.rglob('*'):
            if path.is_file() and path.suffix != '.pyc' and '__pycache__' not in path.parts:
                if not inside(path, WORKSPACE):
                    raise RuntimeError('Project source link escapes workspace: '+str(path))
                rel = path.relative_to(WORKSPACE).as_posix()
                if rel not in expected:
                    extras.append(path)
    return extras


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    options = parser.parse_args()
    stage = options.stage.resolve()
    manifest = json.loads((stage/'manifest.json').read_text())
    if manifest.get('format') != 'lite3_passive_checkpoint_restore/v1':
        raise RuntimeError('Unexpected checkpoint manifest')
    files = manifest['files']
    if manifest.get('commit') != '51cb1f39b9f99d3f432a5726cd5c00be04e34762':
        raise RuntimeError('This is not the verified 2026-10-21 commit')
    archive = stage/'checkpoint.tar'
    if digest(archive) != manifest['archive_sha256']:
        raise RuntimeError('Checkpoint archive hash mismatch')
    extracted = stage/'checkpoint'
    extracted.mkdir(exist_ok=True)
    with tarfile.open(archive, 'r:') as tar:
        members = tar.getmembers()
        if any(not member.isfile() or member.name not in files or
               Path(member.name).is_absolute() or '..' in Path(member.name).parts
               for member in members):
            raise RuntimeError('Archive has an unexpected member')
        if {member.name for member in members} != set(files):
            raise RuntimeError('Archive file list differs from manifest')
        tar.extractall(extracted)
    for name, expected in files.items():
        if digest(extracted/name) != expected:
            raise RuntimeError('Checkpoint file hash mismatch: '+name)
        target = target_path(name)
        if not inside(target.parent, WORKSPACE) or not target.parent.is_dir() or target.is_symlink():
            raise RuntimeError('Invalid robot project path: '+str(target))
    api, relays = preflight()
    extras = package_extras({target_path(name).relative_to(WORKSPACE).as_posix() for name in files})
    print(json.dumps({'checkpoint': manifest['commit'], 'project_files': len(files),
                      'additional_project_files_to_archive': [str(p) for p in extras],
                      'api_pids': api, 'project_relays_to_stop': relays,
                      'dry_run': not options.apply}, indent=2), flush=True)
    if not options.apply:
        return
    backup = WORKSPACE/'backups'/datetime.now(timezone.utc).strftime('%Y%m%d')/(
        'before_passive_checkpoint_restore_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    backup.mkdir(parents=True, exist_ok=False)
    current = {}
    for name in files:
        target = target_path(name)
        current[name] = digest(target) if target.is_file() else None
        if current[name] is not None:
            saved = backup/name
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, saved)
            if digest(saved) != current[name]:
                raise RuntimeError('Could not verify current backup: '+name)
    for target in extras:
        saved = backup/'extra'/target.relative_to(WORKSPACE)
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(target, saved)
        if digest(saved) != digest(target):
            raise RuntimeError('Could not verify added project file: '+str(target))
    (backup/'manifest.json').write_text(json.dumps({'checkpoint': manifest['commit'],
                                                     'current_sha256': current,
                                                     'extras': [str(p.relative_to(WORKSPACE)) for p in extras]}, indent=2)+'\n')
    print('Current project backup:', backup, flush=True)
    installed = WORKSPACE/'install/lite3_odom_tf_adapter'
    if installed.is_dir():
        if not inside(installed, WORKSPACE):
            raise RuntimeError('Installed package escapes project workspace')
        shutil.copytree(installed, backup/'installed_package', symlinks=True)
    if preflight() != (api, relays):
        raise RuntimeError('Project process state changed before restore')
    if api:
        api_path = str(WORKSPACE/'install/lite3_odom_tf_adapter/lib/lite3_odom_tf_adapter/robot_api')
        stop_project_pid(api[0], api_path)
    for pid, path in relays.items():
        stop_project_pid(pid, path)
        if Path(path).name == 'vendor_cmd_vel_bridge' and pid not in project_pids():
            pid_file = WORKSPACE/'logs/vendor_cmd_vel_bridge.pid'
            try:
                if int(pid_file.read_text().strip()) == pid:
                    pid_file.unlink()
            except (OSError, ValueError):
                pass
    if project_pids():
        raise RuntimeError('A project process remained; no files changed')
    for name in files:
        shutil.copy2(extracted/name, target_path(name))
        if digest(target_path(name)) != files[name]:
            raise RuntimeError('Source restore mismatch: '+name)
    for target in extras:
        target.unlink()
    command = ('source /opt/ros/foxy/setup.bash && cd '+str(WORKSPACE)+
               ' && colcon build --packages-select lite3_odom_tf_adapter')
    subprocess.run(['/bin/bash', '-c', command], check=True)
    # Foxy colcon can retain old installed copies even after a successful
    # source rebuild. Align project modules/resources with the checkpoint.
    module_dirs = list((installed/'lib').glob('python*/site-packages/lite3_odom_tf_adapter'))
    if len(module_dirs) != 1:
        raise RuntimeError('Expected one installed Python module directory; found '+str(module_dirs))
    module_dir = module_dirs[0]
    for group, installed_dir in (
            ('lite3_odom_tf_adapter', module_dir),
            ('launch', installed/'share/lite3_odom_tf_adapter/launch'),
            ('config', installed/'share/lite3_odom_tf_adapter/config')):
        source_dir = PACKAGE/group
        if not installed_dir.is_dir() or not inside(installed_dir, WORKSPACE):
            raise RuntimeError('Invalid installed project directory: '+str(installed_dir))
        allowed = {p.name for p in source_dir.iterdir() if p.is_file()}
        for source in source_dir.iterdir():
            if source.is_file():
                target = installed_dir/source.name
                if target.is_symlink() and not inside(target, WORKSPACE):
                    raise RuntimeError('Installed resource escapes workspace: '+str(target))
                if not target.is_file() or digest(target) != digest(source):
                    shutil.copy2(source, target)
                if digest(target) != digest(source):
                    raise RuntimeError('Installed project file mismatch: '+str(target))
        for target in installed_dir.iterdir():
            if target.is_file() and target.name not in allowed and target.suffix in ('.py', '.yaml', '.xml'):
                target.unlink()
    allowed_executables = {'odom_tf_adapter', 'scan_delay', 'scan_info_adjuster',
                           'cmd_vel_gate', 'motion_arbiter', 'vendor_cmd_vel_bridge',
                           'save_live_map', 'mapping_recorder', 'robot_api'}
    executable_dir = installed/'lib/lite3_odom_tf_adapter'
    if executable_dir.is_dir() and inside(executable_dir, WORKSPACE):
        for target in executable_dir.iterdir():
            if target.is_file() and target.name not in allowed_executables:
                target.unlink()
    print(json.dumps({'restored': True, 'checkpoint': manifest['commit'],
                      'backup': str(backup), 'project_api_running': False,
                      'vendor_services_changed': False}, indent=2), flush=True)


if __name__ == '__main__':
    main()
