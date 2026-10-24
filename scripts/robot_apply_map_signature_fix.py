#!/usr/bin/env python3
"""Back up and fix only map_identity.py on the Lite3 perception computer."""

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
from urllib.request import build_opener, ProxyHandler


OLD = b"    geometry = json.dumps([width, height, round(resolution, 6), [round(x, 6) for x in origin]], separators=(',', ':'))"
NEW = (b"    # YAML commonly parses a zero map yaw as int 0, while OccupancyGrid\n"
       b"    # reports the same angle as float 0.0. Normalize both representations\n"
       b"    # before hashing so an identical saved and published map keeps its ID.\n"
       b"    geometry = json.dumps([int(width), int(height), round(float(resolution), 6),\n"
       b"                           [round(float(x), 6) for x in origin]], separators=(',', ':'))")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def api_status():
    with build_opener(ProxyHandler({})).open(
            'http://127.0.0.1:8080/api/v1/status', timeout=4) as response:
        result = json.load(response)
    if result.get('ok') is not True or result.get('simulation') is True:
        raise RuntimeError('Port 8080 did not return the real robot API')
    return result


def check_idle(status):
    parts = status.get('components') or {}
    nav = parts.get('navigation') or {}
    motion = parts.get('motion_output') or {}
    if (parts.get('mapping') or {}).get('process_running'):
        raise RuntimeError('Finish and stop mapping before this update')
    if nav.get('unresolved_goal_count') or nav.get('goal_requests_enabled'):
        raise RuntimeError('Resolve the Nav2 goal and disable planner goal requests before this update')
    if motion.get('state') == 'ready' or motion.get('trial_remaining_seconds', 0) > 0:
        raise RuntimeError('Motion output is active; stop it before this update')
    if (parts.get('safety_stop') or {}).get('cleanup_pending'):
        raise RuntimeError('Wait for software STOP cleanup before this update')


def api_pids(workspace):
    install = str((workspace / 'install').resolve()) + '/'
    found = []
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            args = [part.decode('utf-8', 'replace') for part in
                    (item / 'cmdline').read_bytes().split(b'\0') if part]
        except (OSError, PermissionError):
            continue
        if any(arg.startswith(install) and arg.endswith('/robot_api') for arg in args):
            found.append(int(item.name))
    return found


def installed_module(workspace):
    probe = 'from lite3_odom_tf_adapter import map_identity; print(map_identity.__file__)'
    command = ('source /opt/ros/foxy/setup.bash && source ' +
               shlex.quote(str(workspace / 'install/setup.bash')) +
               ' && python3 -c ' + shlex.quote(probe))
    result = subprocess.run(['/bin/bash', '-c', command], check=True,
                            capture_output=True, text=True)
    path = Path(result.stdout.strip().splitlines()[-1]).resolve()
    try:
        path.relative_to(workspace)
        inside_workspace = True
    except ValueError:
        inside_workspace = False
    if path.name != 'map_identity.py' or not path.is_file() or not inside_workspace:
        raise RuntimeError('Installed map_identity.py does not resolve inside the project workspace')
    return path


def patched(data):
    if data.count(OLD) != 1 or NEW in data:
        raise RuntimeError('Robot map_identity.py differs from the expected version; no file changed')
    return data.replace(OLD, NEW, 1)


def write_atomic(path, data):
    staged = path.with_name(path.name + '.map-signature-stage')
    try:
        staged.write_bytes(data)
        staged.chmod(path.stat().st_mode & 0o777)
        os.replace(staged, path)
    finally:
        staged.unlink(missing_ok=True)


def verify_installed_signature(workspace):
    probe = ('from lite3_odom_tf_adapter.map_identity import _signature; '
             'assert _signature(1,1,0.05,[0,0,0],[-1]) == '
             '_signature(1,1,0.05,[0.0,0.0,0.0],[-1])')
    command = ('source /opt/ros/foxy/setup.bash && source ' +
               shlex.quote(str(workspace / 'install/setup.bash')) +
               ' && python3 -c ' + shlex.quote(probe))
    subprocess.run(['/bin/bash', '-c', command], check=True)


def wait_exit(pid, seconds=35):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not (Path('/proc') / str(pid)).exists():
            return
        try:
            if (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()[0] == 'Z':
                return
        except OSError:
            return
        time.sleep(0.25)
    raise RuntimeError('Old API did not exit after SIGINT')


def start_api(workspace, old_pid):
    launcher = workspace / 'scripts/start_robot_api.sh'
    if not launcher.is_file():
        raise RuntimeError('Project API launcher is missing')
    log = workspace / 'logs/robot_api_map_signature_fix.log'
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('ab') as output:
        subprocess.Popen(['/bin/bash', str(launcher)], cwd=workspace,
                         stdin=subprocess.DEVNULL, stdout=output,
                         stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        pids = api_pids(workspace)
        if len(pids) == 1 and pids[0] != old_pid:
            try:
                status = api_status()
                if status.get('api_contract_version') == 2:
                    return status
            except (OSError, ValueError, RuntimeError):
                pass
        time.sleep(1)
    raise RuntimeError('Updated API did not restart within 90 seconds; inspect ' + str(log))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home() / 'lite3_autonomy_ws')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    source = workspace / 'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/map_identity.py'
    if source.is_symlink() or not source.is_file():
        raise RuntimeError('Project map_identity source is missing or a symlink')
    installed = installed_module(workspace)
    targets = list(dict.fromkeys((source, installed)))
    originals = {path: path.read_bytes() for path in targets}
    replacements = {path: patched(data) for path, data in originals.items()}
    current = api_status()
    check_idle(current)
    pids = api_pids(workspace)
    if len(pids) != 1:
        raise RuntimeError('Expected one project API process; found ' + str(pids))
    before = {'ready': True, 'api_pid': pids[0],
              'navigation_running': bool((current['components'].get('navigation_stack') or {}).get('process_running')),
              'source_sha256': sha(source), 'installed_sha256': sha(installed)}
    print(json.dumps({'preflight': before}), flush=True)
    if not args.apply:
        print('Dry run passed. Run the laptop script with --apply to install.', flush=True)
        return

    lock_path = workspace / '.map-signature-fix.lock'
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        check_idle(api_status())
        if api_pids(workspace) != pids or any(path.read_bytes() != originals[path] for path in targets):
            raise RuntimeError('API process or map_identity.py changed during preflight')
        stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        backup = workspace / 'backups' / datetime.now(timezone.utc).strftime('%Y%m%d') / ('map_signature_' + stamp)
        backup.mkdir(parents=True, exist_ok=False)
        for index, path in enumerate(targets):
            saved = backup / ('map_identity_' + str(index) + '.py')
            shutil.copy2(path, saved)
            if saved.read_bytes() != originals[path]:
                raise RuntimeError('Backup verification failed for ' + str(path))
        (backup / 'manifest.json').write_text(json.dumps({
            'format': 'lite3_map_signature_backup/v1',
            'paths': [str(path) for path in targets],
            'sha256': [hashlib.sha256(originals[path]).hexdigest() for path in targets],
        }, indent=2) + '\n')
        old_pid = pids[0]
        api_stopped = False
        try:
            for path in targets:
                write_atomic(path, replacements[path])
                subprocess.run([sys.executable, '-m', 'py_compile', str(path)], check=True)
            if any(path.read_bytes() != replacements[path] for path in targets):
                raise RuntimeError('Updated files did not match expected contents')
            verify_installed_signature(workspace)
            check_idle(api_status())
            if api_pids(workspace) != pids:
                raise RuntimeError('API process changed before restart')
            os.kill(old_pid, signal.SIGINT)
            wait_exit(old_pid)
            api_stopped = True
            fresh = start_api(workspace, old_pid)
            if ((fresh.get('components') or {}).get('navigation') or {}).get('goal_requests_enabled'):
                raise RuntimeError('Restarted API unexpectedly enabled planner goals')
            result = {'deployed': True, 'backup': str(backup),
                      'source_sha256': sha(source), 'installed_sha256': sha(installed),
                      'api_contract_version': fresh.get('api_contract_version'),
                      'navigation_running': (fresh.get('components', {}).get('navigation_stack') or {}).get('process_running')}
            print(json.dumps(result), flush=True)
        except Exception:
            for index, path in enumerate(targets):
                write_atomic(path, (backup / ('map_identity_' + str(index) + '.py')).read_bytes())
            if api_stopped:
                try:
                    for pid in api_pids(workspace):
                        os.kill(pid, signal.SIGINT)
                        wait_exit(pid)
                    start_api(workspace, old_pid)
                except Exception as error:
                    print('Rollback API restart needs inspection: ' + str(error), file=sys.stderr)
            print('Original map_identity.py restored from ' + str(backup), file=sys.stderr)
            raise


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Map signature update stopped: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
