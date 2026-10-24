from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def test_attended_trial_accepts_intact_unreviewed_map_but_not_legacy_or_synthetic(
        api, api_module, saver, grid):
    from lite3_odom_tf_adapter.map_bundle import load_manifest, manifest_path, write_manifest
    from lite3_odom_tf_adapter.map_identity import read_bundle

    prefix = api.maps_dir / 'trial_room'
    saver.save_grid(grid, prefix)
    identity = read_bundle(prefix)['map_id']
    api._navigation_map_name = 'trial_room'
    api._navigation_identity = identity
    api._bridge_profile = 'supervised_trial'
    assert api._assert_target_map('trial_room', identity)['map_id'] == identity

    manifest = load_manifest(prefix)
    manifest['synthetic'] = True
    write_manifest(prefix, manifest)
    with pytest.raises(api_module.ApiError, match='non-synthetic'):
        api._assert_target_map('trial_room', identity)

    manifest_path(prefix).unlink()
    with pytest.raises(api_module.ApiError, match='verified'):
        api._assert_target_map('trial_room', identity)

    api._bridge_profile = 'command'
    with pytest.raises(api_module.ApiError, match='integrity manifest'):
        api._assert_target_map('trial_room', identity)


def test_supervised_goal_arm_requires_current_goal_and_operator_confirmation(api, api_module):
    api._bridge_profile = 'supervised_trial'
    api._goal_id = 'tracked'
    api._goal_state = 'accepted'
    api._goal_requests = {'tracked': {}}
    api.status = Mock(return_value={'components': {'navigation': {'state': 'ready'}}})
    future = SimpleNamespace(add_done_callback=lambda callback: callback(None),
                             result=lambda: SimpleNamespace(success=True, message='Armed'))
    service = SimpleNamespace(service_is_ready=lambda: True,
                              call_async=Mock(return_value=future))
    api._autonomy_disable_clients = {'supervised_trial': service}
    with pytest.raises(api_module.ApiError):
        api.arm_supervised_trial(True, False, True)
    assert service.call_async.call_count == 0
    response = api.arm_supervised_trial(True, True, True)
    assert response['arming_requested'] is True
    assert service.call_async.call_count == 1
    api._goal_state = 'succeeded'
    with pytest.raises(api_module.ApiError):
        api.arm_supervised_trial(True, True, True)
    assert service.call_async.call_count == 1
