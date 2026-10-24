from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock
import threading

import pytest


def request(api_module, api, key='inert-unit-test-key'):
    handler_type = api_module.make_handler(api)
    handler = handler_type.__new__(handler_type)
    handler.path = '/api/v1/safety/stop'
    handler.headers = {'X-Lite3-Token': key}
    handler._json = lambda status, body: (status, body)
    return handler.do_POST()


def test_stop_bypasses_busy_action_lock_and_disables_all_outputs(api, api_module):
    api.stop_navigation = Mock()
    clients = {}
    for name in ('gate', 'arbiter', 'vendor_bridge'):
        future = Future()
        future.set_result(SimpleNamespace(success=True))
        clients[name] = Mock()
        clients[name].service_is_ready.return_value = True
        clients[name].call_async.return_value = future
    api._autonomy_disable_clients = clients
    api._api_action_lock.acquire()
    status, payload = request(api_module, api)
    assert status == 202
    assert payload['stop_latched'] and payload['cleanup_pending']
    assert api._autonomy_stop_latched
    assert not api.allow_nav2_goal_requests
    assert api._readiness_pub.publish.call_args[0][0].data is False
    assert payload['disable_results'] == {name: 'confirmed' for name in clients}
    for client in clients.values():
        assert client.call_async.call_args[0][0].data is False
    api.stop_navigation.assert_not_called()
    api._api_action_lock.release()
    api._stop_cleanup_thread.join(timeout=1)
    api.stop_navigation.assert_called_once()
    assert not api._stop_cleanup_status()['cleanup_pending']
    assert api._goal_state == 'stopped'


def test_unauthenticated_stop_does_not_bypass_authentication(api, api_module):
    api.valid_browser_session = Mock(return_value=False)
    status, _ = request(api_module, api, key='wrong-key')
    assert status == 401
    assert not api._autonomy_stop_latched
    api._readiness_pub.publish.assert_not_called()


def test_repeated_stop_uses_one_pending_worker(api):
    api.stop_navigation = Mock()
    api._api_action_lock.acquire()
    api.stop_autonomy()
    worker = api._stop_cleanup_thread
    api.stop_autonomy()
    assert api._stop_cleanup_thread is worker
    assert api._stop_generation == 1
    assert api._autonomy_stop_latched


def test_cleanup_failure_is_visible_and_stop_remains_latched(api):
    api.stop_navigation = Mock(side_effect=RuntimeError('Nav2 shutdown failed'))
    api.stop_autonomy()
    api._stop_cleanup_thread.join(timeout=1)
    status = api._stop_cleanup_status()
    assert status['cleanup_error'] == 'Nav2 shutdown failed'
    assert not status['cleanup_pending']
    assert api._autonomy_stop_latched
    assert not api.allow_nav2_goal_requests
    assert api._goal_state != 'stopped'


def test_unavailable_disable_does_not_claim_confirmation(api):
    api.stop_navigation = Mock()
    client = Mock()
    client.service_is_ready.return_value = False
    api._autonomy_disable_clients = {'gate': client}
    payload = api.stop_autonomy()
    assert not payload['gate_disable_confirmed']
    assert payload['disable_results']['gate'] == 'unavailable'
    client.call_async.assert_not_called()


def test_late_disable_result_cannot_overwrite_new_stop_session(api):
    api._stop_generation = 2
    api._stop_disable_results['gate'] = 'requested'
    future = Future()
    future.set_result(SimpleNamespace(success=True))
    api._record_stop_disable('gate', 1, future)
    assert api._stop_disable_results['gate'] == 'requested'


def test_stop_during_navigation_start_cannot_clear_latch(api, api_module, monkeypatch):
    for suffix in ('.pgm', '.yaml'):
        (api.maps_dir / ('room' + suffix)).write_text('inert')
    monkeypatch.setattr(api_module, 'read_bundle', lambda *args, **kwargs: {
        'map_id': 'test', 'grid_signature': 'test', 'approval_state': 'experimental',
        'manifest': {'review': {}},
    })
    api._start_bridge = api.stop_autonomy
    api.stop_bridge = Mock()
    api.stop_navigation = Mock()
    spawn = Mock(side_effect=AssertionError('Navigation launched after STOP'))
    monkeypatch.setattr(api_module.subprocess, 'Popen', spawn)
    api._api_action_lock.acquire()
    with pytest.raises(api_module.ApiError, match='STOP interrupted'):
        api.start_navigation('room')
    spawn.assert_not_called()
    assert api._autonomy_stop_latched


def test_readiness_snapshot_cannot_publish_true_after_stop(api):
    api._api_action_lock.acquire()
    api.stop_navigation = Mock()

    def racing_status():
        api.stop_autonomy()
        return {'components': {'navigation': {'planner_ready': True}}}

    api.status = racing_status
    api._publish_readiness()
    assert all(call.args[0].data is False for call in api._readiness_pub.publish.call_args_list)


def test_stop_is_still_available_during_api_shutdown(api, api_module):
    api._shutdown_requested = True
    api.stop_navigation = Mock()
    status, payload = request(api_module, api)
    assert status in (200, 202)
    assert payload['stop_latched']


def test_failed_readiness_publish_does_not_skip_disable_and_cleanup(api):
    api._readiness_pub.publish.side_effect = RuntimeError('publisher unavailable')
    api.stop_navigation = Mock()
    client = Mock()
    client.service_is_ready.return_value = False
    api._autonomy_disable_clients = {'gate': client}
    payload = api.stop_autonomy()
    api._stop_cleanup_thread.join(timeout=1)
    assert payload['stop_latched']
    assert payload['disable_results']['readiness'] == 'publish_failed'
    client.service_is_ready.assert_called_once()
    api.stop_navigation.assert_called_once()


def test_failed_cleanup_blocks_navigation_restart(api, api_module):
    api._stop_cleanup_error = 'could not stop previous process'
    with pytest.raises(api_module.ApiError, match='cleanup failed'):
        api.start_navigation('room')


@pytest.mark.parametrize('mode', ['mapping', 'navigation'])
def test_slow_bridge_discovery_does_not_hold_up_stop_latch(api, api_module, mode, monkeypatch):
    monkeypatch.setattr(api_module, 'read_bundle', lambda *args, **kwargs: {'map_id': 'test', 'grid_signature': 'test'})
    discovery_started = threading.Event()
    release_discovery = threading.Event()
    stop_returned = threading.Event()
    startup_errors = []
    for suffix in ('.pgm', '.yaml'):
        (api.maps_dir / ('room' + suffix)).write_text('inert')

    def discover():
        discovery_started.set()
        assert release_discovery.wait(2), 'test did not release bridge discovery'
        return False

    def start():
        with api._api_action_lock:
            try:
                api.start_mapping() if mode == 'mapping' else api.start_navigation('room')
            except api_module.ApiError as error:
                startup_errors.append(error)

    def stop():
        api.stop_autonomy()
        stop_returned.set()

    api._adopt_or_start_bridge = discover
    api.stop_navigation = Mock()
    startup = threading.Thread(target=start)
    stopper = threading.Thread(target=stop)
    startup.start()
    try:
        assert discovery_started.wait(1)
        stopper.start()
        assert stop_returned.wait(0.5), 'STOP waited for slow bridge discovery'
        assert api._autonomy_stop_latched
        assert not release_discovery.is_set()
    finally:
        release_discovery.set()
        startup.join(timeout=2)
        if stopper.ident is not None:
            stopper.join(timeout=2)
    assert len(startup_errors) == 1
