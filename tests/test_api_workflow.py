from concurrent.futures import Future
import io
import json
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import threading

import pytest


def done(value):
    future=Future();future.set_result(value);return future


def test_active_planner_goal_keeps_stationary_amcl_refresh_running(api, api_module, monkeypatch):
    now=time.monotonic()
    api._localization_refresh_active=True
    api._localization_refresh_started_at=now-10
    api._localization_refresh_requested_at=now-4
    api._localization_refresh_inflight=False
    api._localization_converged=True
    api._goal_requests={'active-goal': {}}
    api._localization_refresh_client=Mock()
    api._localization_refresh_client.service_is_ready.return_value=True
    api._localization_refresh_client.call_async.return_value=done(None)
    monkeypatch.setattr(api_module, 'Empty', NS(Request=lambda: object()))
    api.status=Mock(return_value={'components': {
        name: {'state': 'ready'} for name in ('map', 'depth_sensor', 'scan', 'odometry')}})

    api._localization_refresh_tick()

    api._localization_refresh_client.call_async.assert_called_once()
    assert api._localization_refresh_active is True
    assert api._localization_refresh_inflight is False


def test_stale_scan_still_stops_amcl_refresh_with_specific_reason(api):
    now=time.monotonic()
    api._localization_refresh_active=True
    api._localization_refresh_started_at=now-10
    api._localization_refresh_requested_at=now-4
    api._localization_refresh_inflight=False
    api._localization_converged=True
    api._localization_refresh_client=Mock()
    api.status=Mock(return_value={'components': {
        name: {'state': 'not_ready' if name == 'scan' else 'ready'}
        for name in ('map', 'depth_sensor', 'scan', 'odometry')}})

    api._localization_refresh_tick()

    api._localization_refresh_client.call_async.assert_not_called()
    assert api._localization_refresh_active is False
    assert api._localization_reason == 'Localization refresh stopped because scan became stale.'


def test_clicked_robot_pose_is_a_tight_prior_and_waits_for_amcl(api, api_module, monkeypatch):
    api.status = Mock(return_value={'components': {
        'navigation_stack': {'process_running': True}, 'map': {'state': 'ready'}}})
    api._map = object()
    api._validate_goal_cell = Mock()
    api._initial_pose_pub = Mock()
    api._initial_pose_pub.get_subscription_count.return_value = 1
    api.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100, to_msg=lambda: 'now'))

    def pose_message():
        return NS(header=NS(), pose=NS(
            pose=NS(position=NS(), orientation=NS(z=0.0, w=1.0)),
            covariance=[0.0]*36))

    monkeypatch.setattr(api_module, 'PoseWithCovarianceStamped', pose_message)
    response = api.set_initial_pose(1.5, 2.0, 0.5)
    message = api._initial_pose_pub.publish.call_args.args[0]
    assert message.pose.covariance[0] == message.pose.covariance[7] == 0.04
    assert message.pose.covariance[35] == 0.07
    assert message.pose.pose.position.x == 1.5
    assert api._localization_converged is False
    assert response['planner_only'] is True


def test_clicked_robot_pose_is_not_lost_before_amcl_subscribes(api, api_module):
    api.status = Mock(return_value={'components': {
        'navigation_stack': {'process_running': True}, 'map': {'state': 'ready'}}})
    api._initial_pose_pub = Mock()
    api._initial_pose_pub.get_subscription_count.return_value = 0
    with pytest.raises(api_module.ApiError, match='not subscribed'):
        api.set_initial_pose(1.5, 2.0)
    api._initial_pose_pub.publish.assert_not_called()


@pytest.mark.parametrize('value', [None, 'invalid', True, -1])
def test_malformed_obstacle_acquisition_stamp_inhibits(api, value):
    api._on_observation_health(NS(data=json.dumps({'healthy': True, 'stamp_ns': value, 'frame_id': 'camera'})))
    assert api._observation_health['healthy'] is False
    assert 'stamp_ns' not in api._observation_health


@pytest.fixture
def path_api(api,api_module,monkeypatch):
    grid=NS(header=NS(frame_id='map',stamp=NS(sec=100,nanosec=0)),
            info=NS(width=40,height=40,resolution=.1,origin=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*1600)
    api._map=grid
    api._amcl_pose=NS(pose=NS(pose=NS(position=NS(x=1.05,y=1.05))))
    api._costmaps={'global_costmap':grid,'local_costmap':grid}
    api._navigation_map_name='room';api._navigation_identity='map'
    api._assert_active_map=Mock()
    api.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_100_000_000,to_msg=lambda:'inert'))
    api.status=Mock(return_value={'components':{'navigation':{'planner_ready':True,'state':'ready','reason':'ready'}}})
    monkeypatch.setattr(api_module,'ComputePathToPose',NS(Goal=lambda:NS(pose=NS(header=NS(),pose=NS(position=NS(),orientation=NS())))))
    api._path_action=Mock();api._path_action.server_is_ready.return_value=True
    def handle(points):
        h=Mock();h.accepted=True
        path=NS(header=NS(frame_id='map'),poses=[NS(pose=NS(position=NS(x=x,y=y))) for x,y in points])
        h.get_result_async.return_value=done(NS(status=4,result=NS(path=path)))
        return h
    h=handle([(1.05,1.05),(2.05,1.05)])
    api._path_action.send_goal_async.return_value=done(h)
    return NS(api=api,module=api_module,grid=grid,handle=handle,
              preview=lambda:api.preview_nav_path(2.05,1.05,map_name='room',map_id='map'))


def test_actual_foxy_pose_field_preview_is_bound_to_map_epoch_and_target(path_api):
    p=path_api;response=p.preview();preview=response['path_preview']
    request=p.api._path_action.send_goal_async.call_args.args[0]
    assert request.pose.pose.position.x==2.05
    assert request.planner_id=='GridBased'
    assert preview['map_id']=='map' and preview['session_epoch']==p.api._runtime.epoch
    p.api._validate_preview(2.05,1.05,0,preview['preview_id'])
    p.api._runtime.reset('map changed')
    with pytest.raises(p.module.ApiError,match='fresh route'):p.api._validate_preview(2.05,1.05,0,preview['preview_id'])


def test_path_crossing_wall_between_sparse_waypoints_is_rejected(path_api):
    p=path_api;p.grid.data[10*40+15]=100
    with pytest.raises(p.module.ApiError,match='known-free'):p.preview()
    assert p.api._path_preview is None


def test_supervised_trial_rejects_distant_target_before_planning(path_api):
    p = path_api
    p.api._bridge_profile = 'supervised_trial'
    p.api._assert_target_map = Mock()
    with pytest.raises(p.module.ApiError, match='within 0.50 m'):
        p.preview()
    p.api._path_action.send_goal_async.assert_not_called()


def test_costmap_obstacle_rejects_otherwise_free_map_target(path_api):
    p=path_api
    obstacle=NS(header=p.grid.header,info=p.grid.info,data=list(p.grid.data))
    obstacle.data[10*40+20]=100
    p.api._costmaps['local_costmap']=obstacle
    with pytest.raises(p.module.ApiError,match='known-free'):p.preview()
    p.api._path_action.send_goal_async.assert_not_called()


def test_inflated_costmap_clearance_is_not_applied_twice(path_api):
    p = path_api
    costmap = NS(header=p.grid.header, info=p.grid.info, data=list(p.grid.data))
    # One cell beside, but not at, the target is already marked by Nav2's
    # inflation layer. The saved map independently enforces the 0.30 m radius.
    costmap.data[10*40+21] = 100
    p.api._costmaps['local_costmap'] = costmap
    response = p.preview()
    assert response['path_preview']['goal'][:2] == [2.05, 1.05]


def test_traversable_inflation_cost_at_target_is_allowed(path_api):
    p = path_api
    costmap = NS(header=p.grid.header, info=p.grid.info, data=list(p.grid.data))
    costmap.data[10*40+20] = 50
    p.api._costmaps['global_costmap'] = costmap
    p.api._costmaps['local_costmap'] = costmap
    response = p.preview()
    assert response['path_preview']['goal'][:2] == [2.05, 1.05]


def test_path_must_start_near_robot_and_end_at_target(path_api):
    p=path_api;p.api._path_action.send_goal_async.return_value=done(p.handle([(3.,3.),(2.05,1.05)]))
    with pytest.raises(p.module.ApiError,match='start near'):p.preview()


def test_stale_map_and_costmap_callbacks_cannot_update_new_epoch(api):
    previous=api._runtime.epoch;api._runtime.reset('new session')
    api._on_map(object(),epoch=previous)
    api._on_costmap('local_costmap',object(),epoch=previous)
    assert api._map is None and not api._costmaps


def test_zero_stamped_foxy_costmap_is_validated_by_receipt(api):
    grid = NS(header=NS(stamp=NS(sec=0, nanosec=0)),
              info=NS(width=2, height=2, resolution=.05), data=[0, 0, 0, 0])
    api._on_costmap('local_costmap', grid, epoch=api._runtime.epoch)
    assert api._costmap_valid['local_costmap'] is True
    assert api._costmaps['local_costmap'] is grid
    assert api._last_rx['local_costmap'] > 0
    assert 'local_costmap' not in api._runtime.streams


def test_depth_pointcloud_can_bootstrap_without_optional_depth_image(api):
    now = time.monotonic()
    api._last_rx['pointcloud'] = now
    depth = api._depth_sensor_component(now)
    assert depth['state'] == 'ready'
    assert depth['source'] == 'pointcloud'


def test_finish_scan_requires_new_map_from_same_session(api,api_module):
    api._mapping_process=NS(poll=lambda:None)
    api._stationary_since=time.monotonic()-3
    with pytest.raises(api_module.ApiError,match='No final map'):api.finish_scan('fresh',timeout=.001)
    assert not list(api.maps_dir.iterdir())


def test_direct_map_save_refuses_static_or_stale_grid(api,api_module,grid):
    api._map=grid
    with pytest.raises(api_module.ApiError,match='fresh live SLAM map'):
        api.save_latest_map('new_scan')
    api._mapping_process=NS(poll=lambda:None)
    api._last_rx['map']=time.monotonic()-20
    with pytest.raises(api_module.ApiError,match='fresh live SLAM map'):
        api.save_latest_map('new_scan')
    assert not list(api.maps_dir.iterdir())


def test_finish_scan_freezes_saves_then_stops(api,api_module,grid):
    api._mapping_process=NS(poll=lambda:None);api._stationary_since=time.monotonic()-3
    api.software_identity=Mock(return_value='a'*64);api.stop_mapping=Mock()
    result={}
    def finish():result.update(api.finish_scan('final',timeout=1))
    worker=threading.Thread(target=finish);worker.start()
    time.sleep(.02)
    api._on_map(grid,epoch=api._runtime.epoch)
    worker.join(1)
    assert not worker.is_alive() and result['name']=='final'
    assert result['map_id'] and result['integrity']=='verified'
    assert (api.maps_dir/'final.yaml').is_file()
    api.stop_mapping.assert_called_once()


def test_authenticated_lease_renewal_bypasses_busy_action_lock(api,api_module):
    api._runtime.release();api._api_action_lock.acquire()
    handler_type=api_module.make_handler(api)
    def request(path,body):
        h=handler_type.__new__(handler_type);h.path=path
        payload=json.dumps(body).encode()
        h.headers={'X-Lite3-Token':api.api_token,'Content-Length':str(len(payload))}
        h.rfile=io.BytesIO(payload);h._json=lambda status,body:(status,body)
        return h.do_POST()
    status,lease=request('/api/v1/control/acquire',{})
    assert status==200
    status,_=request('/api/v1/control/heartbeat',lease)
    assert status==200
    status,_=request('/api/v1/control/heartbeat',{'session_epoch':api._runtime.epoch})
    assert status==409


def test_missing_or_stale_ownership_permit_latches_each_command_stage(command_nodes):
    from test_command_faults import arm,nonzero
    for kind in ('gate','arbiter','relay'):
        node=command_nodes.build(kind);arm(node,kind)
        command_nodes.clock.now+=.31
        node._last_command_at=command_nodes.clock.now
        node._on_command(nonzero(command_nodes))
        assert not node._enabled
        node._on_ownership(NS(data=True))
        node._on_command(nonzero(command_nodes))
        assert not node._enabled


def test_commands_queued_before_rearm_are_rejected_by_generation(command_nodes):
    from test_command_faults import arm,nonzero
    for kind in ('gate','arbiter','relay'):
        node=command_nodes.build(kind)
        previous=node._command_generation
        arm(node,kind)
        node._guard_command(nonzero(command_nodes),previous)
        node._publisher.publish.assert_not_called()
        node._guard_command(nonzero(command_nodes),node._command_generation)
        assert node._publisher.publish.call_args.args[0].linear.x==.08


def test_supervised_trial_relay_preserves_rclpy_context(command_nodes):
    node = command_nodes.build('trial')
    assert hasattr(node._context, 'handle')
    assert node._readiness_context == {}
