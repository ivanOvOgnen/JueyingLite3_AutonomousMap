#!/usr/bin/env python3
"""Restore the robot project's pre-supervised-nav deployment snapshot."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

from robot_install_supervised_nav_trial import (
    FILES, PACKAGE_MODULES, api_pids, build, check_idle, digest, inside,
    installed_python_module, installed_resources, status, wait_api,
)


def candidates(workspace):
    for path in sorted((workspace/'backups').glob('*/supervised_nav_*'), reverse=True):
        if not path.is_dir():
            continue
        required = ('app.js', 'index.html', 'styles.css', 'robot_api.py',
                    'setup.py', 'setup.cfg', 'package.xml', 'start_robot_api.sh',
                    'installed_python_robot_api.py')
        if not all((path/name).is_file() for name in required):
            continue
        # A later redeployment may back up the trial itself. Select the latest
        # snapshot that predates the supervised-trial API and UI.
        if (b'/api/v1/motion/arm-supervised' in (path/'robot_api.py').read_bytes() or
                b'UI_DEPLOY_VERSION = 30' in (path/'app.js').read_bytes()):
            continue
        yield path


def verified_snapshot(workspace, requested):
    """Only restore the exact install backup, with every saved file verified."""
    backup_root = (workspace/'backups').resolve()
    if requested.is_symlink():
        raise RuntimeError('Requested backup directory must not be a symlink')
    snapshot = requested.resolve()
    if snapshot.parent.parent != backup_root or not snapshot.name.startswith('supervised_nav_'):
        raise RuntimeError('Backup is not a supervised Nav2 snapshot in this workspace')
    if not snapshot.is_dir() or snapshot.is_symlink():
        raise RuntimeError('Requested backup directory is missing or is a symlink')
    manifest_path = snapshot/'manifest.json'
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise RuntimeError('Requested backup has no regular manifest')
    manifest = json.loads(manifest_path.read_text())
    if (manifest.get('format') != 'lite3_supervised_nav_backup/v2' or
            manifest.get('workspace') != str(workspace)):
        raise RuntimeError('Backup manifest does not match this robot workspace')
    expected_sections = {
        'source': set(FILES),
        'installed_resources': set(installed_resources(workspace)),
        'installed_python': set(PACKAGE_MODULES),
    }
    for section, expected_names in expected_sections.items():
        entries = manifest.get(section)
        if not isinstance(entries, dict) or set(entries) != expected_names:
            raise RuntimeError('Backup manifest has incomplete '+section+' entries')
        prefix = {'source': '', 'installed_resources': 'installed_',
                  'installed_python': 'installed_python_'}[section]
        for name, expected_hash in entries.items():
            saved = snapshot/(prefix+name)
            if expected_hash is None:
                if saved.exists():
                    raise RuntimeError('Unexpected saved file: '+name)
            elif (not isinstance(expected_hash, str) or len(expected_hash) != 64 or
                    not saved.is_file() or saved.is_symlink() or digest(saved) != expected_hash):
                raise RuntimeError('Backup file missing or hash mismatch: '+name)
    return snapshot


def project_children(workspace):
    """Flag project navigation/mapping executables if the API is unavailable."""
    prefix = str(workspace/'install').encode() + b'/'
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = (entry/'cmdline').read_bytes().split(b'\0')
        except (OSError, PermissionError):
            continue
        if any(part.startswith(prefix) and any(tag in part for tag in
               (b'/nav2_', b'/slam_toolbox', b'/mapping_recorder',
                b'/supervised_nav_relay', b'/vendor_cmd_vel_bridge')) for part in args):
            found.append(int(entry.name))
    return sorted(found)


def safe_to_restore(workspace):
    pids = api_pids(workspace)
    if len(pids) > 1:
        raise RuntimeError('Multiple project APIs are running: '+str(pids))
    children = project_children(workspace)
    if children:
        raise RuntimeError('Project mapping/Nav2/relay processes still run: '+str(children))
    if pids:
        check_idle(status())
    else:
        try:
            status()
        except (OSError, ValueError):
            pass
        else:
            raise RuntimeError('API responds but its project process identity is unknown')
    return pids


def backup_current(workspace, paths, resources, module_dir):
    root = workspace/'backups'/datetime.now(timezone.utc).strftime('%Y%m%d')
    destination = root/('before_supervised_nav_rollback_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    destination.mkdir(parents=True, exist_ok=False)
    present = {}
    for name, target in paths.items():
        present[name] = target.is_file()
        if present[name]:
            shutil.copy2(target, destination/name)
            if digest(target) != digest(destination/name):
                raise RuntimeError('Could not verify current file backup: '+name)
    for name, target in resources.items():
        key = 'installed_'+name
        present[key] = target.is_file()
        if present[key]:
            shutil.copy2(target, destination/key)
            if digest(target) != digest(destination/key):
                raise RuntimeError('Could not verify installed resource backup: '+name)
    for name in PACKAGE_MODULES:
        target = module_dir/name
        key = 'installed_python_'+name
        present[key] = target.is_file()
        if present[key]:
            shutil.copy2(target, destination/key)
            if digest(target) != digest(destination/key):
                raise RuntimeError('Could not verify installed module backup: '+name)
    (destination/'manifest.json').write_text(json.dumps({'present': present}, indent=2)+'\n')
    return destination, present


def restore_files(snapshot, paths, resources, module_dir, workspace):
    all_targets = dict(paths)
    all_targets.update({'installed_'+name: target for name, target in resources.items()})
    all_targets.update({'installed_python_'+name: module_dir/name for name in PACKAGE_MODULES})
    for name, target in all_targets.items():
        if not inside(target, workspace) or target.is_symlink() and not inside(target.resolve(), workspace):
            raise RuntimeError('Restore target escapes project workspace: '+str(target))
        source = snapshot/name
        if source.is_file():
            shutil.copy2(source, target)
            if digest(source) != digest(target):
                raise RuntimeError('Restored file hash mismatch: '+name)
        elif target.is_file():
            target.unlink()


def launch_api(workspace):
    launcher = workspace/'scripts/start_robot_api.sh'
    log = workspace/'logs/supervised_nav_rollback.log'
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('ab') as output:
        subprocess.Popen(['/bin/bash', str(launcher)], cwd=workspace,
                         stdin=subprocess.DEVNULL, stdout=output,
                         stderr=subprocess.STDOUT, start_new_session=True)
    return wait_api(workspace, expected_contract=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--backup', type=Path,
                        help='exact verified backup created by this installation')
    parser.add_argument('--workspace', type=Path, default=Path.home()/'lite3_autonomy_ws')
    options = parser.parse_args()
    workspace = options.workspace.resolve()
    if options.apply and not options.backup:
        raise RuntimeError('An exact --backup path is required for apply; refusing to choose an older snapshot')
    if options.backup:
        selected = verified_snapshot(workspace, options.backup)
    else:
        snapshots = list(candidates(workspace))
        if not snapshots:
            raise RuntimeError('No complete pre-supervised-nav deployment snapshot found')
        selected = snapshots[0]
    paths = {name: workspace/relative for name, relative in FILES.items()}
    resources = installed_resources(workspace)
    try:
        installed_api = installed_python_module(workspace, 'robot_api.py')
    except (OSError, RuntimeError, subprocess.CalledProcessError):
        matches = list((workspace/'install/lite3_odom_tf_adapter/lib').glob(
            'python*/site-packages/lite3_odom_tf_adapter/robot_api.py'))
        installed_api = matches[0] if len(matches) == 1 else None
    if installed_api is None:
        raise RuntimeError('Installed API module could not be located')
    if not inside(installed_api, workspace):
        raise RuntimeError('Installed API module escapes project workspace')
    module_dir = installed_api.parent
    pids = safe_to_restore(workspace)
    report = {'selected_backup': str(selected), 'project_api_pids': pids,
              'restore_files': len(paths), 'dry_run': not options.apply}
    print(json.dumps(report, indent=2), flush=True)
    if not options.apply:
        return
    rollback_backup, _ = backup_current(workspace, paths, resources, module_dir)
    print('Current project saved for recovery:', rollback_backup, flush=True)
    if safe_to_restore(workspace) != pids:
        raise RuntimeError('Project state changed before rollback')
    if pids:
        os.kill(pids[0], signal.SIGINT)
        deadline = time.monotonic()+20
        while api_pids(workspace) and time.monotonic() < deadline:
            time.sleep(0.25)
        if api_pids(workspace):
            raise RuntimeError('Project API did not stop; no files restored')
    try:
        restore_files(selected, paths, resources, module_dir, workspace)
        build(workspace)
        # Colcon can leave previously installed copies in place, so put back
        # the exact saved installed resources and Python modules after build.
        restore_files(selected, {}, resources, module_dir, workspace)
        if project_children(workspace):
            raise RuntimeError('Project command/mapping process appeared during rollback')
        fresh = launch_api(workspace)
        print(json.dumps({'rolled_back': True, 'from_backup': str(selected),
                          'undo_backup': str(rollback_backup), 'api_running': True,
                          'api_contract_version': fresh.get('api_contract_version')}, indent=2), flush=True)
    except Exception:
        print('Rollback failed; attempting to restore the pre-rollback project copy.', flush=True)
        try:
            live = api_pids(workspace)
            if len(live) == 1:
                os.kill(live[0], signal.SIGINT)
                deadline = time.monotonic()+20
                while api_pids(workspace) and time.monotonic() < deadline:
                    time.sleep(0.25)
            if api_pids(workspace):
                raise RuntimeError('Project API did not stop for recovery')
            restore_files(rollback_backup, paths, resources, module_dir, workspace)
            build(workspace)
            restore_files(rollback_backup, {}, resources, module_dir, workspace)
            launch_api(workspace)
            print('Pre-rollback project copy restored and API restarted.', flush=True)
        except Exception as recovery_error:
            print('Recovery needs inspection:', recovery_error, flush=True)
        print('Pre-rollback project copy is saved at:', rollback_backup, flush=True)
        raise


if __name__ == '__main__':
    main()
