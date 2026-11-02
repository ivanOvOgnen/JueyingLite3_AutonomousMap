#!/usr/bin/env python3
"""Robot-side backed-up installer for the supervised map-goal trial bundle."""

import argparse
from http.cookiejar import CookieJar
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import time
import urllib.request


PACKAGE_MODULES = (
    '__init__.py', 'capture_tf.py', 'cmd_vel_gate.py', 'command_permission.py',
    'depth_obstacle_cloud.py', 'depth_projection.py', 'map_bundle.py',
    'map_identity.py', 'map_persistence.py', 'mapping_recorder.py',
    'motion_arbiter.py', 'motion_supervisor.py', 'navigation_config.py',
    'odom_tf_adapter.py', 'robot_api.py', 'runtime_state.py', 'save_map.py',
    'scan_delay.py', 'scan_info_adjuster.py', 'supervised_nav_relay.py',
    'vendor_cmd_vel_bridge.py',
)
LAUNCH_FILES = (
    'command_gate.launch.py', 'localization.launch.py', 'mapping.launch.py',
    'motion_arbiter.launch.py', 'navigation.launch.py', 'perception.launch.py',
    'vendor_cmd_vel_bridge.launch.py',
)
CONFIG_FILES = (
    'mapping_ros1_bridge.yaml', 'mapping.yaml', 'nav2_camera_overrides.yaml',
    'navigate_observed_forward.xml', 'ros1_bridge.yaml',
    'vendor_cmd_vel_bridge.yaml',
)
FILES = {
    **{name: 'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/'+name
       for name in PACKAGE_MODULES},
    **{name: 'src/lite3_odom_tf_adapter/launch/'+name for name in LAUNCH_FILES},
    **{name: 'src/lite3_odom_tf_adapter/config/'+name for name in CONFIG_FILES},
    'setup.py': 'src/lite3_odom_tf_adapter/setup.py',
    'setup.cfg': 'src/lite3_odom_tf_adapter/setup.cfg',
    'package.xml': 'src/lite3_odom_tf_adapter/package.xml',
    'app.js': 'webapp/app.js',
    'index.html': 'webapp/index.html',
    'styles.css': 'webapp/styles.css',
    'start_robot_api.sh': 'scripts/start_robot_api.sh',
    'robot_preflight.sh': 'scripts/robot_preflight.sh',
}
INSTALLED_RESOURCES = {
    **{name: 'share/lite3_odom_tf_adapter/launch/'+name for name in LAUNCH_FILES},
    **{name: 'share/lite3_odom_tf_adapter/config/'+name for name in CONFIG_FILES},
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inside(path, workspace):
    try:
        path.resolve().relative_to(workspace.resolve())
        return True
    except ValueError:
        return False


def installed_resources(workspace):
    root = workspace/'install/lite3_odom_tf_adapter'
    return {name: root/relative for name, relative in INSTALLED_RESOURCES.items()}


def installed_python_module(workspace, name):
    module_name = 'lite3_odom_tf_adapter.'+name[:-3]
    probe = ('import importlib.util; s=importlib.util.find_spec('+repr(module_name)+
             '); print(s.origin if s else "")')
    command = ('source /opt/ros/foxy/setup.bash && source '+
               shlex.quote(str(workspace/'install/setup.bash'))+
               ' && python3 -c '+shlex.quote(probe))
    result = subprocess.run(['/bin/bash', '-c', command], check=True, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    value = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ''
    path = Path(value).resolve() if value else None
    if path is not None and (not path.is_file() or not inside(path, workspace)):
        raise RuntimeError('Installed Python module resolves outside project workspace: '+str(path))
    return path


def sync_installed_resource(source, installed, workspace):
    """Repair a stale colcon data-file copy, never a link outside the workspace."""
    if not inside(installed, workspace) or not installed.parent.is_dir():
        raise RuntimeError('Installed resource is outside the project workspace: '+str(installed))
    if installed.exists() and not installed.is_file():
        raise RuntimeError('Installed resource is not a regular file: '+str(installed))
    if installed.is_symlink() and not inside(installed, workspace):
        raise RuntimeError('Installed resource symlink escapes workspace: '+str(installed))
    if not installed.is_file() or digest(installed) != digest(source):
        print('Repairing stale Foxy resource copy:', installed,
              'resolved to', installed.resolve(), flush=True)
        shutil.copy2(source, installed)
    if digest(installed) != digest(source):
        raise RuntimeError('Installed resource still differs: '+str(installed))


def status():
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open('http://127.0.0.1:8080/api/v1/status', timeout=4) as response:
        data = json.load(response)
    if data.get('ok') is not True or data.get('simulation') is True:
        raise RuntimeError('Expected a real robot API at 127.0.0.1:8080')
    return data


def check_idle(data, *, require_stop=True):
    parts = data.get('components', {})
    if (parts.get('mapping') or {}).get('process_running'):
        raise RuntimeError('Stop mapping before deployment')
    if (parts.get('navigation_stack') or {}).get('process_running'):
        raise RuntimeError('Press project STOP and wait for Nav2 to stop before deployment')
    stop = parts.get('safety_stop') or {}
    if 'state' not in stop:
        raise RuntimeError('Project STOP status is unavailable')
    if require_stop and stop['state'] != 'latched':
        raise RuntimeError('Project STOP must be latched before deployment')
    if stop.get('cleanup_pending'):
        raise RuntimeError('Project STOP cleanup is still pending')
    if stop.get('cleanup_error'):
        raise RuntimeError('Project STOP cleanup failed: '+str(stop['cleanup_error']))
    if (parts.get('navigation') or {}).get('unresolved_goal_count'):
        raise RuntimeError('A Nav2 goal remains unresolved')


def ensure_idle_project_stop():
    """Latch only the project software STOP after proving its stacks are idle."""
    before = status()
    check_idle(before, require_stop=False)
    if before['components']['safety_stop']['state'] == 'latched':
        return before
    if before.get('api_paired') is not True:
        raise RuntimeError('Robot API is not paired; refusing to create a session during deployment')
    origin = 'http://127.0.0.1:8080'
    opener = urllib.request.build_opener(
        urllib.request.ProxyHandler({}), urllib.request.HTTPCookieProcessor(CookieJar()))
    with opener.open(origin+'/api/v1/session', timeout=4) as response:
        session = json.load(response)
    if session.get('ok') is not True or session.get('authorized') is not True:
        raise RuntimeError('Could not establish a local project API session')
    # Recheck immediately before the POST: never stop a mapping/goal session
    # that appeared after the first read-only inspection.
    check_idle(status(), require_stop=False)
    request = urllib.request.Request(origin+'/api/v1/safety/stop', data=b'',
        headers={'Origin': origin}, method='POST')
    with opener.open(request, timeout=15) as response:
        result = json.load(response)
    if result.get('ok') is not True or result.get('stop_latched') is not True:
        raise RuntimeError('Project STOP did not confirm it latched')
    deadline = time.monotonic()+15
    while time.monotonic() < deadline:
        current = status()
        if not (current.get('components', {}).get('safety_stop') or {}).get('cleanup_pending'):
            check_idle(current)
            print('Project software STOP latched and idle for deployment.', flush=True)
            return current
        time.sleep(0.5)
    raise RuntimeError('Project STOP cleanup did not finish within 15 seconds')


def api_pids(workspace):
    install = str((workspace/'install').resolve())
    result = []
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            args = (item/'cmdline').read_bytes().split(b'\0')
        except (OSError, PermissionError):
            continue
        if any(value.decode('utf-8', 'replace').startswith(install+'/') and
               value.endswith(b'/robot_api') for value in args):
            result.append(int(item.name))
    return result


def build(workspace):
    command = ('source /opt/ros/foxy/setup.bash && cd ' + shlex.quote(str(workspace)) +
               ' && colcon build --packages-select lite3_odom_tf_adapter')
    subprocess.run(['/bin/bash', '-c', command], check=True)


def wait_api(workspace, old_pid=None, expected_contract=2):
    deadline = time.monotonic()+90
    while time.monotonic() < deadline:
        pids = api_pids(workspace)
        if len(pids) == 1 and pids[0] != old_pid:
            try:
                current = status()
                if expected_contract is None or current.get('api_contract_version') == expected_contract:
                    return current
            except (OSError, ValueError, RuntimeError):
                pass
        time.sleep(1)
    log = workspace/'logs/supervised_nav_deploy.log'
    recent = log.read_text(errors='replace')[-4000:] if log.is_file() else '(no startup log)'
    raise RuntimeError('Robot API did not restart within 90 seconds; '
                       'current API PIDs='+str(api_pids(workspace))+'; startup log:\n'+recent)


def normalize_shell_launcher(path):
    """Repair CRLF introduced by a Windows staging checkout."""
    data = path.read_bytes()
    normalized = data.replace(b'\r\n', b'\n').replace(b'\r', b'\n')
    if normalized != data:
        path.write_bytes(normalized)
    return normalized != data


def recover_missing_api(workspace):
    """Restart only the fail-closed project API after a launcher-EOL failure."""
    if api_pids(workspace):
        return status()
    launcher = workspace/'scripts/start_robot_api.sh'
    if not launcher.is_file() or launcher.is_symlink() or not inside(launcher, workspace):
        raise RuntimeError('Cannot recover API: project launcher is missing or unsafe')
    repaired = normalize_shell_launcher(launcher)
    output_path = workspace/'logs/supervised_nav_recovery.log'
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('ab') as output:
        subprocess.Popen(['/bin/bash', str(launcher)], cwd=workspace,
                         stdin=subprocess.DEVNULL, stdout=output,
                         stderr=subprocess.STDOUT, start_new_session=True)
    fresh = wait_api(workspace, expected_contract=None)
    print('Recovered stopped project API after normalizing launcher LF endings='
          +str(repaired)+'. Goal requests remain disabled.', flush=True)
    return fresh


def restart_api(workspace, pid, expected_contract=2):
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic()+20
    while pid in api_pids(workspace) and time.monotonic() < deadline:
        time.sleep(0.25)
    if pid in api_pids(workspace):
        raise RuntimeError('Old API did not exit after SIGINT')
    launcher = workspace/'scripts/start_robot_api.sh'
    if not launcher.is_file():
        raise RuntimeError('Project API launcher is missing')
    output_path = workspace/'logs/supervised_nav_deploy.log'
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('ab') as output:
        subprocess.Popen(['/bin/bash', str(launcher)], cwd=workspace,
                         stdin=subprocess.DEVNULL, stdout=output,
                         stderr=subprocess.STDOUT, start_new_session=True)
    return wait_api(workspace, pid, expected_contract)


def check_installed_api_import(workspace):
    """Catch missing package dependencies while the previous API is still running."""
    probe = ('from lite3_odom_tf_adapter import robot_api, supervised_nav_relay; '
             'print(robot_api.__file__)')
    command = ('source /opt/ros/foxy/setup.bash && source '+
               shlex.quote(str(workspace/'install/setup.bash'))+
               ' && python3 -c '+shlex.quote(probe))
    result = subprocess.run(['/bin/bash', '-c', command], text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        raise RuntimeError('Installed API import failed before restart:\n'+result.stderr)
    print('Installed API import passed:', result.stdout.strip(), flush=True)


LEGACY_CHECKPOINT_RELAY_SHA256 = 'bbffae50993f5c73e057e886b8525493a9d67a5a80ad8c2f54b28a2be559393a'


def check_disabled_legacy_relay(data, command=None, expected=None,
                                installed_digest=None):
    check_idle(data)
    relay = ((data.get('components') or {}).get('command_stages') or {}).get('stages', {}).get('relay') or {}
    if relay.get('fresh') is True:
        if relay.get('enabled') is not False:
            raise RuntimeError('Legacy relay reports enabled; refusing to stop it: '+str(relay))
        return 'fresh disabled relay status'
    # Older boot relays may not expose a status the current API can observe.
    # Their startup command fixes physical_stop_verified=false; this value is
    # copied into the node at initialization and prevents service arming.
    args = [part for part in (command or b'').split(b'\0') if part]
    if (expected is not None and is_project_relay_process(args, expected) and
            b'physical_stop_verified:=false' in args):
        return 'project executable with fixed disabled startup parameter'
    # The accepted passive checkpoint starts this exact module with a False
    # default. Some ROS launchers omit the redundant false parameter in proc
    # argv; the installed module hash plus absence of an override is a second
    # way to prove the legacy relay cannot arm before retiring it.
    if (expected is not None and is_project_relay_process(args, expected) and
            installed_digest == LEGACY_CHECKPOINT_RELAY_SHA256 and
            not any(b'physical_stop_verified:=' in part for part in args)):
        return 'accepted checkpoint relay with compiled disabled default'
    raise RuntimeError('Legacy relay status is unavailable and its disabled startup was not verified: '+str(relay))


def is_project_relay_process(args, expected):
    """Accept direct exec or a Python shebang exposing the script as argv[1]."""
    return bool(args and (args[0] == expected or
                         (len(args) > 1 and args[0].rsplit(b'/', 1)[-1].startswith(b'python3')
                          and args[1] == expected)))


def stop_legacy_bridge(workspace):
    """Retire the project's disabled boot relay before trial launch.

    The trial launch owns its isolated relay. A second project relay with the
    same ROS node name prevents Nav2 startup and may publish stray zeroes.
    """
    pid_file = workspace/'logs/vendor_cmd_vel_bridge.pid'
    expected = str(workspace/'install/lite3_odom_tf_adapter/lib/lite3_odom_tf_adapter/vendor_cmd_vel_bridge').encode()
    matches = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            candidate = (entry/'cmdline').read_bytes()
        except (OSError, PermissionError):
            continue
        parts = [part for part in candidate.split(b'\0') if part]
        if is_project_relay_process(parts, expected):
            matches.append((int(entry.name), candidate))
    if len(matches) > 1:
        raise RuntimeError('Multiple project legacy relays exist: '+str([pid for pid, _ in matches]))
    if pid_file.is_file():
        try:
            recorded = int(pid_file.read_text().strip())
            if (Path('/proc')/str(recorded)).is_dir() and (
                    not matches or recorded != matches[0][0]):
                raise RuntimeError('Legacy relay PID file names another live process')
        except ValueError:
            raise RuntimeError('Legacy relay PID file is malformed')
    if not matches:
        pid_file.unlink(missing_ok=True)
        return False
    pid, command = matches[0]
    args = [part for part in command.split(b'\0') if part]
    installed_relay = installed_python_module(workspace, 'vendor_cmd_vel_bridge.py')
    installed_digest = digest(installed_relay) if installed_relay else None
    evidence = check_disabled_legacy_relay(status(), command, expected,
                                           installed_digest)
    print('Legacy relay cleanup evidence:', evidence, flush=True)

    def same_process():
        try:
            current = (Path('/proc')/str(pid)/'cmdline').read_bytes()
            parts = [part for part in current.split(b'\0') if part]
            return is_project_relay_process(parts, expected)
        except OSError:
            return False

    for sig, seconds in ((signal.SIGINT, 3), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        if not same_process():
            break
        print('Stopping disabled legacy project relay PID', pid, 'with', sig.name, flush=True)
        os.kill(pid, sig)
        deadline = time.monotonic()+seconds
        while same_process() and time.monotonic() < deadline:
            time.sleep(0.2)
    if same_process():
        raise RuntimeError('Disabled legacy relay PID '+str(pid)+' survived SIGINT/SIGTERM/SIGKILL; inspect process state')
    pid_file.unlink(missing_ok=True)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, default=Path.home()/'lite3_autonomy_ws')
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--recover-api', action='store_true',
                        help='Repair LF endings and restart a missing fail-closed API before preflight')
    args = parser.parse_args()
    stage, workspace = args.stage.resolve(), args.workspace.resolve()
    if not stage.is_dir() or not workspace.is_dir():
        raise RuntimeError('Staging directory or robot workspace is missing')
    paths = {name: workspace/relative for name, relative in FILES.items()}
    resources = installed_resources(workspace)
    for name, target in paths.items():
        if not (stage/name).is_file() or (stage/name).is_symlink() or target.is_symlink():
            raise RuntimeError('Missing staged file or unexpected symlink: '+name)
        if not target.parent.is_dir():
            raise RuntimeError('Robot target directory missing: '+str(target.parent))
        if not target.is_file() and not name in PACKAGE_MODULES+LAUNCH_FILES+CONFIG_FILES:
            raise RuntimeError('Robot target file missing: '+str(target))
    if b'UI_DEPLOY_VERSION = 30' not in (stage/'app.js').read_bytes():
        raise RuntimeError('Staged UI is not version 30')
    if b'/api/v1/motion/arm-supervised' not in (stage/'robot_api.py').read_bytes():
        raise RuntimeError('Staged API lacks the supervised trial endpoint')
    if args.recover_api and not api_pids(workspace):
        recover_missing_api(workspace)
    initial = status()
    check_idle(initial, require_stop=False)
    pids = api_pids(workspace)
    if len(pids) != 1:
        raise RuntimeError('Expected one project API process; found '+str(pids))
    print(json.dumps({'ready': True, 'api_pid': pids[0],
                      'project_stop_state': initial['components']['safety_stop']['state'],
                      'project_stop_will_be_latched_on_apply': initial['components']['safety_stop']['state'] != 'latched',
                      'files': {name: {'robot': digest(target) if target.is_file() else None,
                                       'staged': digest(stage/name)}
                                for name, target in paths.items()},
                      'installed_resources': {
                          name: {'path': str(target), 'resolved': str(target.resolve()),
                                 'sha256': digest(target) if target.is_file() else None}
                          for name, target in resources.items()}}, indent=2), flush=True)
    if not args.apply:
        print('Dry run only. Re-run laptop deploy script with --apply.', flush=True)
        return
    ensure_idle_project_stop()
    check_idle(status())
    if api_pids(workspace) != pids:
        raise RuntimeError('API process changed since preflight')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    backup = workspace/'backups'/datetime.now(timezone.utc).strftime('%Y%m%d')/('supervised_nav_'+stamp)
    backup.mkdir(parents=True, exist_ok=False)
    prior = {}
    for name, target in paths.items():
        if target.is_file():
            shutil.copy2(target, backup/name)
            prior[name] = digest(target)
            if digest(backup/name) != prior[name]:
                raise RuntimeError('Backup verification failed: '+name)
        else:
            prior[name] = None
    prior_resources = {}
    for name, target in resources.items():
        if not inside(target, workspace):
            raise RuntimeError('Installed resource resolves outside workspace: '+str(target))
        backup_name = 'installed_'+name
        if target.is_file():
            shutil.copy2(target, backup/backup_name)
            prior_resources[name] = digest(target)
            if digest(backup/backup_name) != prior_resources[name]:
                raise RuntimeError('Installed resource backup verification failed: '+name)
        else:
            prior_resources[name] = None
    installed_api = installed_python_module(workspace, 'robot_api.py')
    if installed_api is None:
        raise RuntimeError('Installed robot_api.py is missing')
    installed_module_dir = installed_api.parent
    prior_modules = {}
    for name in PACKAGE_MODULES:
        target = installed_module_dir/name
        if not inside(target, workspace):
            raise RuntimeError('Installed Python target escapes workspace: '+str(target))
        prior_modules[name] = digest(target) if target.is_file() else None
        if prior_modules[name] is not None:
            shutil.copy2(target, backup/('installed_python_'+name))
            if digest(backup/('installed_python_'+name)) != prior_modules[name]:
                raise RuntimeError('Installed Python backup verification failed: '+name)
    print('Backup:', backup, flush=True)
    (backup/'manifest.json').write_text(json.dumps({
        'format': 'lite3_supervised_nav_backup/v2',
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'workspace': str(workspace),
        'source': prior,
        'installed_resources': prior_resources,
        'installed_python': prior_modules,
    }, indent=2, sort_keys=True)+'\n')
    changed = []
    restarted = False
    legacy_relay_stopped = False
    try:
        # Retire the old disabled relay while its original installed module
        # hash still proves the fail-closed default. Building first would
        # overwrite that evidence and caused the previous deployment failure.
        legacy_relay_stopped = stop_legacy_bridge(workspace)
        for name, target in paths.items():
            if name in ('app.js', 'styles.css', 'index.html'):
                continue
            shutil.copy2(stage/name, target)
            changed.append(name)
        build(workspace)
        installed = workspace/'install/lite3_odom_tf_adapter'
        executable = installed/'lib/lite3_odom_tf_adapter/supervised_nav_relay'
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError('Foxy build did not install the supervised relay executable')
        for name, installed_file in resources.items():
            sync_installed_resource(stage/name, installed_file, workspace)
        for name in PACKAGE_MODULES:
            sync_installed_resource(stage/name, installed_module_dir/name, workspace)
        check_installed_api_import(workspace)
        fresh = restart_api(workspace, pids[0])
        restarted = True
        if 'supervised_goal_trial' not in fresh.get('capabilities', []):
            raise RuntimeError('Restarted API did not load the supervised trial code')
        if (fresh.get('components', {}).get('navigation') or {}).get('goal_requests_enabled'):
            raise RuntimeError('Restarted API unexpectedly enabled goal requests')
        if (fresh.get('components', {}).get('motion_output') or {}).get('bridge_profile') != 'telemetry':
            raise RuntimeError('Restarted API did not reset to telemetry bridge profile')
        for name in ('app.js', 'styles.css', 'index.html'):
            shutil.copy2(stage/name, paths[name])
            changed.append(name)
        for name in paths:
            if digest(paths[name]) != digest(stage/name):
                raise RuntimeError('Post-deployment source hash mismatch: '+name)
        print(json.dumps({'deployed': True, 'backup': str(backup),
                          'api_pid': api_pids(workspace)[0],
                          'navigation_started': False}, indent=2), flush=True)
    except Exception:
        for name in reversed(changed):
            target = paths[name]
            if prior[name] is None:
                target.unlink(missing_ok=True)
            else:
                shutil.copy2(backup/name, target)
        # Rebuild the saved source. If the API was already restarted, it must
        # also be restarted to load the restored module.
        try:
            build(workspace)
            for name, target in resources.items():
                if prior_resources[name] is None:
                    if target.exists() and inside(target, workspace):
                        target.unlink()
                else:
                    if not inside(target, workspace):
                        raise RuntimeError('Rollback target escaped workspace: '+str(target))
                    shutil.copy2(backup/('installed_'+name), target)
            for name, old_digest in prior_modules.items():
                target = installed_module_dir/name
                if old_digest is not None:
                    shutil.copy2(backup/('installed_python_'+name), target)
                elif target.exists() and inside(target, workspace):
                    target.unlink()
            live_pids = api_pids(workspace)
            if restarted and len(live_pids) == 1:
                restart_api(workspace, live_pids[0], expected_contract=None)
            elif not live_pids:
                launcher = workspace/'scripts/start_robot_api.sh'
                with (workspace/'logs/supervised_nav_rollback.log').open('ab') as output:
                    subprocess.Popen(['/bin/bash', str(launcher)], cwd=workspace,
                                     stdin=subprocess.DEVNULL, stdout=output,
                                     stderr=subprocess.STDOUT, start_new_session=True)
                wait_api(workspace, expected_contract=None)
            if legacy_relay_stopped:
                relay_launcher = workspace/'scripts/start_vendor_cmd_vel_bridge.sh'
                if relay_launcher.is_file():
                    subprocess.run(['/bin/bash', str(relay_launcher)],
                                   cwd=workspace, check=True)
        except Exception as recovery_error:
            print('Rollback needs inspection:', recovery_error, flush=True)
        raise


if __name__ == '__main__':
    main()
