"""Action transport races with real futures; no ROS or hardware activity."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


def ready(value=None, error=None):
    future = Future()
    if error:
        future.set_exception(error)
    else:
        future.set_result(value)
    return future


@pytest.fixture
def planner(api, api_module, monkeypatch):
    api._assert_active_map = Mock()
    api._validate_preview = Mock()
    api._validate_live_clearance = Mock()
    api._navigation_map_name = 'room'
    api._map = object()
    api._amcl_pose = object()
    api.status = Mock(return_value={'components': {'navigation': {'state': 'ready'}}})
    api._validate_goal_cell = Mock()
    api.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'inert-stamp'))
    monkeypatch.setattr(api_module, 'NavigateToPose', NS(Goal=lambda: NS(
        pose=NS(header=NS(), pose=NS(position=NS(), orientation=NS())))))
    api._nav_action = Mock()
    api._nav_action.server_is_ready.return_value = True

    def handle(number=1):
        goal = Mock()
        goal.accepted = True
        goal.goal_id = NS(uuid=[number] * 16)
        goal.result_future = Future()
        goal.get_result_async.return_value = goal.result_future
        goal.cancel_future = Future()
        goal.cancel_goal_async.return_value = goal.cancel_future
        return goal

    def submit():
        return api.submit_nav_goal(1.0, 2.0, map_name='room')

    return NS(api=api, handle=handle, submit=submit, module=api_module)


def test_late_acceptance_after_timeout_is_canceled_and_retained(planner):
    p = planner
    acceptance = Future()

    def send(_, **kwargs):
        next(iter(p.api._goal_requests.values()))['accepted'] = Mock(wait=lambda _: False)
        return acceptance

    p.api._nav_action.send_goal_async.side_effect = send
    with pytest.raises(p.module.ApiError, match='remains tracked'):
        p.submit()
    assert p.api._goal_state == 'acceptance_pending'
    with pytest.raises(p.module.ApiError, match='unresolved'):
        p.submit()
    handle = p.handle()
    acceptance.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    assert p.api._goal_handle is handle
    assert len(p.api._goal_requests) == 1
    handle.cancel_future.set_result(NS(goals_canceling=[NS(goal_id=handle.goal_id)]))
    assert p.api._goal_state == 'canceling'
    assert len(p.api._goal_requests) == 1
    handle.result_future.set_result(NS(status=5))
    assert p.api._goal_state == 'canceled'
    assert not p.api._goal_requests


def test_cancel_acknowledgment_does_not_clear_goal(planner):
    p = planner
    handle = p.handle()
    p.api._nav_action.send_goal_async.return_value = ready(handle)
    response = p.submit()
    assert len(response['goal_id']) == 32
    response = p.api.cancel_nav_goal()
    assert not response['terminal_confirmed']
    handle.cancel_future.set_result(NS(goals_canceling=[NS(goal_id=handle.goal_id)]))
    assert p.api._goal_handle is handle
    with pytest.raises(p.module.ApiError, match='unresolved'):
        p.submit()
    handle.result_future.set_result(NS(status=5))
    assert p.api._goal_handle is None


def test_cancel_requires_matching_ros_goal_id_and_can_retry(planner):
    p = planner
    handle = p.handle()
    p.api._nav_action.send_goal_async.return_value = ready(handle)
    p.submit()
    p.api.cancel_nav_goal()
    handle.cancel_future.set_result(NS(goals_canceling=[NS(goal_id=NS(uuid=[9] * 16))]))
    assert p.api._goal_state == 'cancel_unconfirmed'
    assert p.api._goal_requests
    handle.cancel_goal_async.return_value = ready(error=RuntimeError('transport'))
    p.api.cancel_nav_goal()
    assert handle.cancel_goal_async.call_count == 2
    assert p.api._goal_state == 'cancel_unconfirmed'


def test_result_transport_failure_keeps_handle_and_inhibits_replacement(planner):
    p = planner
    handle = p.handle()
    p.api._nav_action.send_goal_async.return_value = ready(handle)
    p.submit()
    handle.result_future.set_exception(RuntimeError('result lost'))
    assert p.api._goal_state == 'unresolved'
    assert p.api._goal_handle is handle
    with pytest.raises(p.module.ApiError, match='unresolved'):
        p.submit()


def test_old_result_cannot_overwrite_new_goal(planner):
    p = planner
    old = p.handle(1)
    p.api._nav_action.send_goal_async.return_value = ready(old)
    old_id = p.submit()['goal_id']
    old.result_future.set_result(NS(status=4))
    new = p.handle(2)
    p.api._nav_action.send_goal_async.return_value = ready(new)
    new_id = p.submit()['goal_id']
    p.api._on_goal_result(old_id, ready(NS(status=6)))
    assert p.api._goal_id == new_id
    assert p.api._goal_state == 'accepted'
    assert p.api._goal_handle is new


def test_stop_during_acceptance_cancels_without_rearming(planner):
    p = planner
    handle = p.handle()
    p.api.stop_navigation = Mock()
    p.api._api_action_lock.acquire()

    def send(_, **kwargs):
        p.api.stop_autonomy()
        return ready(handle)

    p.api._nav_action.send_goal_async.side_effect = send
    with pytest.raises(p.module.ApiError, match='invalidated'):
        p.submit()
    assert p.api._autonomy_stop_latched
    assert not p.api.allow_nav2_goal_requests
    handle.cancel_goal_async.assert_called_once()
    p.api._api_action_lock.release()
    p.api._stop_cleanup_thread.join(1)
    assert p.api._goal_requests  # Stopping local processes is not a terminal result.
    assert p.api._goal_handle is handle


@pytest.mark.parametrize('failure', ['request_exception', 'acceptance_exception', 'result_subscription'])
def test_action_exception_is_visible_and_request_is_retained(planner, failure):
    p = planner
    if failure == 'request_exception':
        p.api._nav_action.send_goal_async.side_effect = RuntimeError('transport')
    elif failure == 'acceptance_exception':
        p.api._nav_action.send_goal_async.return_value = ready(error=RuntimeError('transport'))
    else:
        handle = p.handle()
        handle.get_result_async.side_effect = RuntimeError('transport')
        p.api._nav_action.send_goal_async.return_value = ready(handle)
    with pytest.raises(p.module.ApiError):
        p.submit()
    assert p.api._goal_state == 'unresolved'
    assert p.api._goal_requests


def test_rejection_releases_request(planner):
    p = planner
    p.api._nav_action.send_goal_async.return_value = ready(NS(accepted=False))
    with pytest.raises(p.module.ApiError, match='rejected'):
        p.submit()
    assert p.api._goal_state == 'rejected'
    assert not p.api._goal_requests


@pytest.mark.parametrize('status,state', [(4, 'succeeded'), (5, 'canceled'), (6, 'aborted'), (2, 'unresolved')])
def test_terminal_results_are_distinguished(planner, status, state):
    p = planner
    handle = p.handle()
    p.api._nav_action.send_goal_async.return_value = ready(handle)
    p.submit()
    handle.result_future.set_result(NS(status=status))
    assert p.api._goal_state == state
    assert bool(p.api._goal_requests) == (state == 'unresolved')


def test_stop_between_validation_and_send_prevents_dispatch(planner):
    p = planner
    p.api._validate_goal_cell.side_effect = lambda *_: setattr(p.api, '_autonomy_stop_latched', True)
    with pytest.raises(p.module.ApiError, match='interrupted'):
        p.submit()
    p.api._nav_action.send_goal_async.assert_not_called()


def test_initial_pose_refused_until_goal_terminates(planner):
    p = planner
    p.api._nav_action.send_goal_async.return_value = ready(p.handle())
    p.submit()
    with pytest.raises(p.module.ApiError, match='terminate'):
        p.api.set_initial_pose(1, 2)
