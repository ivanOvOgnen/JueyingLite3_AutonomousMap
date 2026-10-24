from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml


def bridge_graph(api, api_module, monkeypatch, pids='42', topics=None):
    if topics is None:
        topics = yaml.safe_load(api._bridge_config.read_text())['topics']
    monkeypatch.setattr(api_module.subprocess, 'run', Mock(return_value=SimpleNamespace(
        returncode=0 if pids else 1, stdout=pids)))
    api._run_noetic_command = Mock(return_value=SimpleNamespace(
        returncode=0, stdout=yaml.safe_dump(topics)))
    api._bridge_process_identity = Mock(return_value=(42, 12345))


def test_adopted_bridge_starts_mapping_without_owned_process(api, api_module, monkeypatch):
    bridge_graph(api, api_module, monkeypatch)
    mapping = Mock()
    mapping.poll.return_value = None
    spawn = Mock(return_value=mapping)
    monkeypatch.setattr(api_module.subprocess, 'Popen', spawn)
    assert api.start_mapping()['ok']
    assert api._bridge_external
    assert api._bridge_process is None
    assert api._mapping_process is mapping
    assert spawn.call_args[0][0] == [
        'ros2', 'launch', 'lite3_odom_tf_adapter', 'mapping.launch.py']


@pytest.mark.parametrize('pids,topics', [
    ('42\n43', None),
    ('', None),
    ('42', [{'topic': '/cmd_vel_corrected'}]),
])
def test_refuse_conflicting_bridge_without_mutating_topics(api, api_module, monkeypatch, pids, topics):
    bridge_graph(api, api_module, monkeypatch, pids, topics)
    assert not api._adopt_or_start_bridge()
    assert not api._bridge_external
    assert all(call.args[0] == 'rosparam get /topics'
               for call in api._run_noetic_command.call_args_list)


def test_adopted_bridge_requires_fresh_odometry(api, api_module, monkeypatch):
    bridge_graph(api, api_module, monkeypatch)
    api._last_rx['odometry'] = 1.0
    assert api._adopt_or_start_bridge()
    assert 'odometry' not in api._last_rx


@pytest.mark.parametrize('identity', [None, (42, 99999)])
def test_dead_or_reused_external_pid_is_not_alive(api, identity):
    api._bridge_external = True
    api._bridge_external_identity = (42, 12345)
    api._bridge_process_identity = Mock(return_value=identity)
    assert not api._bridge_alive()


def test_adopted_bridge_shutdown_does_not_signal_or_delete(api):
    api._bridge_external = True
    api._bridge_external_identity = (42, 12345)
    api._run_noetic_command = Mock(side_effect=AssertionError('external parameters mutated'))
    api.stop_bridge()
    assert not api._bridge_external
    assert api._bridge_external_identity is None
    api._run_noetic_command.assert_not_called()


def test_dead_external_bridge_is_reported_during_navigation_start(api, api_module):
    api._adopt_or_start_bridge = Mock(return_value=True)
    api._bridge_alive = Mock(side_effect=[False, False])
    api.stop_bridge = Mock()
    with pytest.raises(api_module.ApiError, match='bridge exited'):
        api._start_bridge()
    api.stop_bridge.assert_called_once()


def test_failed_mapping_launch_detaches_external_bridge(api, api_module, monkeypatch):
    bridge_graph(api, api_module, monkeypatch)
    monkeypatch.setattr(api_module.subprocess, 'Popen', Mock(side_effect=OSError('launch failed')))
    with pytest.raises(OSError, match='launch failed'):
        api.start_mapping()
    assert not api._bridge_external
    assert api._mapping_process is None
    assert api._mapping_log is None


@pytest.mark.parametrize('matching', [True, False])
def test_owned_parameter_cleanup_only_deletes_unchanged_topics(api, matching):
    api._bridge_param_owned = True
    topics = yaml.safe_load(api._bridge_config.read_text())['topics'] if matching else []
    api._run_noetic_command = Mock(return_value=SimpleNamespace(
        returncode=0, stdout=yaml.safe_dump(topics)))
    api.stop_bridge()
    commands = [call.args[0] for call in api._run_noetic_command.call_args_list]
    assert ('rosparam delete /topics' in commands) is matching
    assert not api._bridge_param_owned


def test_proc_identity_rejects_unrelated_process_and_missing_pid(api_module):
    # The test process is Python, never the target Foxy bridge executable.
    assert api_module.RobotApi._bridge_process_identity(api_module.os.getpid()) is None
    assert api_module.RobotApi._bridge_process_identity('not-a-pid') is None


def test_proc_identity_uses_start_time_and_rejects_zombie(api_module, tmp_path, monkeypatch):
    process = tmp_path / '42'
    process.mkdir()
    (process / 'cmdline').write_bytes(b'/opt/ros/foxy/lib/ros1_bridge/parameter_bridge\0')
    fields = ['S'] + ['0'] * 18 + ['12345']
    (process / 'stat').write_text('42 (parameter_bridg) ' + ' '.join(fields))
    monkeypatch.setattr(api_module, 'Path', lambda _path: tmp_path)
    assert api_module.RobotApi._bridge_process_identity(42) == (42, 12345)
    fields[0] = 'Z'
    (process / 'stat').write_text('42 (parameter_bridg) ' + ' '.join(fields))
    assert api_module.RobotApi._bridge_process_identity(42) is None


def test_spawn_failure_removes_only_owned_unchanged_parameters(api, api_module, monkeypatch):
    missing = SimpleNamespace(returncode=1, stdout='')
    success = SimpleNamespace(returncode=0, stdout='')
    expected = SimpleNamespace(returncode=0, stdout=yaml.safe_dump(
        yaml.safe_load(api._bridge_config.read_text())['topics']))
    monkeypatch.setattr(api_module.subprocess, 'run', Mock(return_value=missing))
    api._run_noetic_command = Mock(side_effect=[missing, success, expected, success])
    monkeypatch.setattr(api_module.subprocess, 'Popen', Mock(side_effect=OSError('spawn failed')))
    with pytest.raises(OSError, match='spawn failed'):
        api._adopt_or_start_bridge()
    assert api._bridge_log is None
    assert not api._bridge_param_owned
    assert api._run_noetic_command.call_args_list[-1].args[0] == 'rosparam delete /topics'
