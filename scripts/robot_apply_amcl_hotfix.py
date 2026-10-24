#!/usr/bin/env python3
"""Robot-side half of deploy_amcl_hotfix.py; run on the perception computer.

Applies only the AMCL no-motion refresh patch. Never installs the laptop's
other candidate files or changes the vendor command path.
"""

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time
import urllib.request


PACKAGE = 'lite3_odom_tf_adapter'
SOURCE_RELATIVE = Path('src') / PACKAGE / PACKAGE / 'robot_api.py'
PATCH_MARKER = b'Keep AMCL scan updates alive for a stationary planner-only goal.'
OLD_MARKER = b'if goal_active or not self._localization_refresh_client.service_is_ready():'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def is_within(path, directory):
    try:
        path.relative_to(directory)
        return True
    except ValueError:
        return False


def status():
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open('http://127.0.0.1:8080/api/v1/status', timeout=3) as response:
        result = json.load(response)
    if result.get('ok') is not True or result.get('simulation') is True:
        raise RuntimeError('Port 8080 did not return a real robot API status')
    return result


def check_idle(result):
    components = result.get('components') or {}
    nav = components.get('navigation') or {}
    stop = components.get('safety_stop') or {}
    if (components.get('mapping') or {}).get('process_running'):
        raise RuntimeError('Mapping is still running. Finish/save the scan and stop mapping first.')
    if stop.get('cleanup_pending') or stop.get('cleanup_error'):
        raise RuntimeError('Software STOP cleanup is unfinished: ' +
                           str(stop.get('cleanup_error') or 'cleanup_pending=true'))
    goal_state = nav.get('active_goal_state')
    unresolved = nav.get('unresolved_goal_count', 0)
    terminal = ('idle', 'stopped', 'rejected', 'canceled', 'aborted',
                'finished_2', 'finished_4', 'finished_5', 'finished_6')
    if unresolved or goal_state not in terminal:
        raise RuntimeError('Navigation goal is not settled: state=' + str(goal_state) +
                           ', unresolved=' + str(unresolved) +
                           '. Cancel/STOP it and wait for a terminal result before retrying.')
    motion = components.get('motion_output') or {}
    if motion.get('state') != 'not_ready':
        raise RuntimeError('Motor output is not reported isolated; refusing this planner-only hotfix')


def api_pids(workspace):
    found = []
    install = str((workspace / 'install').resolve())
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            arguments = (entry / 'cmdline').read_bytes().split(b'\0')
            arguments = [part.decode('utf-8', 'replace') for part in arguments if part]
        except (OSError, PermissionError):
            continue
        if any(arg.startswith(install + '/') and arg.endswith('/robot_api')
               for arg in arguments):
            found.append(int(entry.name))
    return found


def installed_module(workspace):
    py = 'from lite3_odom_tf_adapter import robot_api; print(robot_api.__file__)'
    command = ('source /opt/ros/foxy/setup.bash && source ' +
               shlex.quote(str(workspace / 'install/setup.bash')) +
               ' && python3 -c ' + shlex.quote(py))
    result = subprocess.run(['/bin/bash', '-c', command], check=True,
                            text=True, stdout=subprocess.PIPE)
    lines = result.stdout.strip().splitlines()
    if not lines:
        raise RuntimeError('Could not locate the installed robot_api Python module')
    path = Path(lines[-1]).resolve()
    if not path.is_file() or path.suffix != '.py' or path.name != 'robot_api.py':
        raise RuntimeError('Installed robot_api is not a regular Python source file')
    if path != (workspace / SOURCE_RELATIVE).resolve() and not is_within(path, (workspace / 'install').resolve()):
        raise RuntimeError('Installed robot_api resolves outside the project workspace')
    return path


def build(workspace):
    command = ('source /opt/ros/foxy/setup.bash && cd ' + shlex.quote(str(workspace)) +
               ' && colcon build --packages-select ' + PACKAGE)
    subprocess.run(['/bin/bash', '-c', command], check=True)


def apply_patch(source, patch):
    subprocess.run(['patch', '--batch', '--fuzz=0', '-p1'], cwd=str(source.parent),
                   input=patch.read_bytes(), check=True)


def dry_run(source, patch):
    subprocess.run(['patch', '--dry-run', '--batch', '--fuzz=0', '-p1'],
                   cwd=str(source.parent), input=patch.read_bytes(), check=True)


def wait_for_exit(pid, seconds=20):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        process = Path('/proc', str(pid))
        if not process.exists():
            return
        try:
            if (process / 'stat').read_text().split()[2] == 'Z':
                return
        except OSError:
            return
        time.sleep(0.25)
    raise RuntimeError('Old API did not exit after SIGINT; no replacement was started')


def start_api(workspace):
    launcher = workspace / 'scripts/start_robot_api.sh'
    if not launcher.is_file():
        raise RuntimeError('Project API launcher is missing')
    log = workspace / 'logs/robot_api_amcl_hotfix.log'
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('ab') as output:
        subprocess.Popen(['/bin/bash', str(launcher)], cwd=str(workspace),
                         stdin=subprocess.DEVNULL, stdout=output,
                         stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        try:
            result = status()
            if api_pids(workspace):
                return result
        except (OSError, ValueError, RuntimeError):
            pass
        time.sleep(1)
    raise RuntimeError('Updated API did not answer within 45 seconds; inspect ' + str(log))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default=str(Path.home() / 'lite3_autonomy_ws'))
    parser.add_argument('--patch', required=True)
    parser.add_argument('--apply', action='store_true', help='perform the backed-up update and API restart')
    args = parser.parse_args()
    workspace = Path(args.workspace).resolve()
    source_entry = workspace / SOURCE_RELATIVE
    source = source_entry.resolve()
    patch = Path(args.patch).resolve()
    if (not source.is_file() or source_entry.is_symlink() or
            not is_within(source, workspace) or not patch.is_file()):
        raise RuntimeError('Expected a regular project source file and staged patch')
    if PATCH_MARKER in source.read_bytes():
        raise RuntimeError('This AMCL hotfix is already present in the robot source')
    if OLD_MARKER not in source.read_bytes():
        raise RuntimeError('Robot source differs from the expected deployed code; inspect it before updating')
    dry_run(source, patch)
    live = status()
    check_idle(live)
    pids = api_pids(workspace)
    if len(pids) != 1:
        raise RuntimeError('Expected exactly one project robot_api process; found ' + str(pids))
    installed = installed_module(workspace)
    print('Source:', source, sha256(source), flush=True)
    print('Installed:', installed, sha256(installed), flush=True)
    print('Robot API PID:', pids[0], flush=True)
    if (live.get('components', {}).get('navigation_stack') or {}).get('process_running'):
        print('Nav2 is running without an active goal; API restart will stop it.', flush=True)
    if not args.apply:
        print('DRY RUN passed. Run the laptop script with --apply to deploy.', flush=True)
        return

    lock_path = workspace / '.amcl-hotfix-deploy.lock'
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        check_idle(status())
        if api_pids(workspace) != pids:
            raise RuntimeError('API process changed after preflight; retry')
        dry_run(source, patch)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        backup = workspace / 'backups' / datetime.now(timezone.utc).strftime('%Y%m%d') / ('amcl_goal_predeploy_' + stamp)
        backup.mkdir(parents=True, exist_ok=False)
        shutil.copy2(source, backup / 'robot_api_source.py')
        shutil.copy2(installed, backup / 'robot_api_installed.py')
        shutil.copy2(patch, backup / 'amcl_refresh_during_goal.patch')
        print('Backup:', backup, flush=True)
        old_source_hash = sha256(source)
        old_installed_hash = sha256(installed)
        if (sha256(backup / 'robot_api_source.py') != old_source_hash or
                sha256(backup / 'robot_api_installed.py') != old_installed_hash):
            raise RuntimeError('Backup hash verification failed; no code changed')
        restarted = False
        after_install = installed
        try:
            apply_patch(source, patch)
            if PATCH_MARKER not in source.read_bytes() or sha256(source) == old_source_hash:
                raise RuntimeError('Patch did not change the expected source')
            subprocess.run([sys.executable, '-m', 'py_compile', str(source)], check=True)
            build(workspace)
            after_install = installed_module(workspace)
            if sha256(after_install) != sha256(source):
                if after_install == source or not is_within(after_install, (workspace / 'install').resolve()):
                    raise RuntimeError('Build did not install the patched API')
                shutil.copy2(source, after_install)
            if sha256(after_install) != sha256(source):
                raise RuntimeError('Installed API hash does not match the patched source')
            # Repeat the state check immediately before stopping the API.
            check_idle(status())
            if api_pids(workspace) != pids:
                raise RuntimeError('API process changed before restart')
            os.kill(pids[0], signal.SIGINT)
            wait_for_exit(pids[0])
            restarted = True
            fresh = start_api(workspace)
            nav = (fresh.get('components') or {}).get('navigation') or {}
            if nav.get('goal_requests_enabled') is not False:
                raise RuntimeError('Restarted API unexpectedly enabled planner goal requests')
            print('DEPLOYED. Installed SHA-256:', sha256(after_install), flush=True)
            print('API answers on 127.0.0.1:8080; planner goals disabled.', flush=True)
            print('Maps were not copied or changed. Motion output remains isolated.', flush=True)
        except Exception:
            # Restore the two files this hotfix can change. A failed build or
            # restart still needs a human to inspect the robot before goals.
            shutil.copy2(backup / 'robot_api_source.py', source)
            if installed != source and installed.exists():
                shutil.copy2(backup / 'robot_api_installed.py', installed)
            if after_install != installed and after_install != source and after_install.exists():
                shutil.copy2(backup / 'robot_api_installed.py', after_install)
            if restarted:
                try:
                    for pid in api_pids(workspace):
                        os.kill(pid, signal.SIGINT)
                        wait_for_exit(pid)
                    start_api(workspace)
                except Exception as error:
                    print('ROLLBACK API restart failed:', error, file=sys.stderr)
            print('Hotfix failed; original source/installed API files restored from ' + str(backup),
                  file=sys.stderr)
            raise


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print('AMCL deploy error:', error, file=sys.stderr)
        sys.exit(1)
