"""Check the stale Foxy resource repair that failed on the robot."""

import importlib.util
import io
import json
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[1]/'scripts/robot_install_supervised_nav_trial.py'
SPEC = importlib.util.spec_from_file_location('robot_install_supervised_nav_trial_test', SCRIPT)
INSTALL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INSTALL)
sys.path.insert(0, str(SCRIPT.parent))
from deploy_supervised_nav_trial import FILES as STAGED_FILES


def test_stale_installed_resource_is_repaired_within_workspace(tmp_path):
    staged = tmp_path/'stage/navigation.launch.py'
    staged.parent.mkdir()
    staged.write_text('new launch')
    installed = tmp_path/'install/share/navigation.launch.py'
    installed.parent.mkdir(parents=True)
    installed.write_text('old launch')
    INSTALL.sync_installed_resource(staged, installed, tmp_path)
    assert installed.read_text() == 'new launch'


def test_installed_resource_link_outside_workspace_is_rejected(tmp_path):
    staged = tmp_path/'new.yaml'
    staged.write_text('new')
    outside = tmp_path.parent/(tmp_path.name+'_outside_nav_resource.yaml')
    outside.write_text('untouched')
    installed = tmp_path/'install/config/nav.yaml'
    installed.parent.mkdir(parents=True)
    installed.symlink_to(outside)
    with pytest.raises(RuntimeError, match='outside'):
        INSTALL.sync_installed_resource(staged, installed, tmp_path)
    assert outside.read_text() == 'untouched'


def test_legacy_relay_requires_fresh_disabled_status_before_cleanup():
    report = {'components': {
        'mapping': {'process_running': False},
        'navigation_stack': {'process_running': False},
        'safety_stop': {'state': 'latched', 'cleanup_pending': False},
        'navigation': {'unresolved_goal_count': 0},
        'command_stages': {'stages': {'relay': {'fresh': True, 'enabled': False}}},
    }}
    INSTALL.check_disabled_legacy_relay(report)
    report['components']['command_stages']['stages']['relay']['enabled'] = True
    with pytest.raises(RuntimeError, match='enabled'):
        INSTALL.check_disabled_legacy_relay(report)


def test_legacy_relay_can_use_fixed_disabled_boot_argument_when_status_missing():
    report = {'components': {
        'mapping': {'process_running': False},
        'navigation_stack': {'process_running': False},
        'safety_stop': {'state': 'latched', 'cleanup_pending': False},
        'navigation': {'unresolved_goal_count': 0},
        'command_stages': {'stages': {}},
    }}
    executable = b'/home/ysc/lite3_autonomy_ws/install/lite3_odom_tf_adapter/lib/lite3_odom_tf_adapter/vendor_cmd_vel_bridge'
    command = executable+b'\0--ros-args\0-p\0physical_stop_verified:=false\0'
    assert 'fixed disabled' in INSTALL.check_disabled_legacy_relay(report, command, executable)
    shebang_command = b'/usr/bin/python3\0'+command
    assert 'fixed disabled' in INSTALL.check_disabled_legacy_relay(report, shebang_command, executable)
    with pytest.raises(RuntimeError, match='not verified'):
        INSTALL.check_disabled_legacy_relay(report, executable+b'\0', executable)
    report['components']['command_stages']['stages']['relay'] = {'fresh': True, 'enabled': True}
    with pytest.raises(RuntimeError, match='enabled'):
        INSTALL.check_disabled_legacy_relay(report, command, executable)


def test_accepted_checkpoint_relay_without_explicit_false_arg_is_retired():
    report = {'components': {
        'mapping': {'process_running': False},
        'navigation_stack': {'process_running': False},
        'safety_stop': {'state': 'latched', 'cleanup_pending': False},
        'navigation': {'unresolved_goal_count': 0},
        'command_stages': {'stages': {}},
    }}
    executable = b'/home/ysc/lite3_autonomy_ws/install/lite3_odom_tf_adapter/lib/lite3_odom_tf_adapter/vendor_cmd_vel_bridge'
    expected = INSTALL.LEGACY_CHECKPOINT_RELAY_SHA256
    assert 'accepted checkpoint relay' in INSTALL.check_disabled_legacy_relay(
        report, executable+b'\0', executable, expected)
    with pytest.raises(RuntimeError, match='not verified'):
        INSTALL.check_disabled_legacy_relay(report, executable+b'\0', executable,
                                            '0'*64)
    with pytest.raises(RuntimeError, match='not verified'):
        INSTALL.check_disabled_legacy_relay(
            report, executable+b'\0--ros-args\0-p\0physical_stop_verified:=true\0',
            executable, expected)


def test_deployment_stages_complete_package_without_map_data():
    root = SCRIPT.parents[1]
    package = root/'ros2_ws/src/lite3_odom_tf_adapter'
    expected_modules = {p.name for p in (package/'lite3_odom_tf_adapter').glob('*.py')}
    expected_launch = {p.name for p in (package/'launch').glob('*.launch.py')}
    expected_config = {p.name for p in (package/'config').iterdir() if p.is_file()}
    assert set(INSTALL.PACKAGE_MODULES) == expected_modules
    assert set(INSTALL.LAUNCH_FILES) == expected_launch
    assert set(INSTALL.CONFIG_FILES) == expected_config
    assert {path.name for path in STAGED_FILES} == set(INSTALL.FILES) | {SCRIPT.name}
    assert all(path.is_file() for path in STAGED_FILES)
    assert not any('maps/' in relative for relative in INSTALL.FILES.values())


def test_rollback_accepts_old_api_contract(monkeypatch, tmp_path):
    monkeypatch.setattr(INSTALL, 'api_pids', lambda _workspace: [123])
    monkeypatch.setattr(INSTALL, 'status', lambda: {'ok': True})
    assert INSTALL.wait_api(tmp_path, expected_contract=None) == {'ok': True}


def test_idle_preflight_can_latch_project_stop_without_touching_active_stack():
    report = {'components': {
        'mapping': {'process_running': False},
        'navigation_stack': {'process_running': False},
        'navigation': {'unresolved_goal_count': 0},
        'safety_stop': {'state': 'not_latched'},
    }}
    INSTALL.check_idle(report, require_stop=False)
    with pytest.raises(RuntimeError, match='must be latched'):
        INSTALL.check_idle(report)
    report['components']['mapping']['process_running'] = True
    with pytest.raises(RuntimeError, match='Stop mapping'):
        INSTALL.check_idle(report, require_stop=False)


def test_one_shot_latches_and_verifies_only_project_stop(monkeypatch):
    def report(state):
        return {'ok': True, 'api_paired': True, 'components': {
            'mapping': {'process_running': False},
            'navigation_stack': {'process_running': False},
            'navigation': {'unresolved_goal_count': 0},
            'safety_stop': {'state': state, 'cleanup_pending': False},
        }}
    states = iter([report('not_latched'), report('not_latched'), report('latched')])
    monkeypatch.setattr(INSTALL, 'status', lambda: next(states))
    requested = []

    class Opener:
        def open(self, request, timeout):
            requested.append(request)
            if isinstance(request, str):
                return io.BytesIO(json.dumps({'ok': True, 'authorized': True}).encode())
            return io.BytesIO(json.dumps({'ok': True, 'stop_latched': True}).encode())

    monkeypatch.setattr(INSTALL.urllib.request, 'build_opener', lambda *_args: Opener())
    result = INSTALL.ensure_idle_project_stop()
    assert result['components']['safety_stop']['state'] == 'latched'
    assert requested[0].endswith('/api/v1/session')
    assert requested[1].full_url.endswith('/api/v1/safety/stop')
    assert requested[1].get_header('Origin') == 'http://127.0.0.1:8080'


def test_project_stop_is_not_requested_when_mapping_active(monkeypatch):
    active = {'ok': True, 'api_paired': True, 'components': {
        'mapping': {'process_running': True},
        'navigation_stack': {'process_running': False},
        'navigation': {'unresolved_goal_count': 0},
        'safety_stop': {'state': 'not_latched'},
    }}
    monkeypatch.setattr(INSTALL, 'status', lambda: active)
    monkeypatch.setattr(INSTALL.urllib.request, 'build_opener',
                        lambda *_args: (_ for _ in ()).throw(AssertionError('unexpected HTTP session')))
    with pytest.raises(RuntimeError, match='Stop mapping'):
        INSTALL.ensure_idle_project_stop()
