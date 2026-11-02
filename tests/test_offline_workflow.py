"""Robot-free functional flow using the real map saver, identities and leases."""
import importlib.util
from pathlib import Path
import pytest

SPEC=importlib.util.spec_from_file_location('offline_demo_tests',Path(__file__).resolve().parents[1]/'scripts/offline_navigation_demo.py')
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


def ready_demo(tmp_path):
    demo=M.Demo(tmp_path)
    demo.action('/api/v1/navigation/start',{'name':'demo_room'},'operator')
    lease=demo.action('/api/v1/control/acquire',{},'operator')
    context={'map_id':demo.active_id,'session_epoch':demo.runtime.epoch,'lease_token':lease['lease_token']}
    demo.action('/api/v1/navigation/goal-requests',{**context,'enabled':True},'operator')
    return demo,context


def submit(demo,context,x=4.025,y=3.025):
    target={**context,'x':x,'y':y,'yaw':.5,'map_name':'demo_room'}
    demo.runtime.renew('operator',context['lease_token'],context['session_epoch'])
    preview=demo.action('/api/v1/navigation/preview',target,'operator')['path_preview']
    demo.runtime.renew('operator',context['lease_token'],context['session_epoch'])
    result=demo.action('/api/v1/navigation/goal',{**target,'preview_id':preview['preview_id']},'operator')
    return preview,result


def test_synthetic_map_target_route_and_virtual_arrival(tmp_path):
    demo,context=ready_demo(tmp_path)
    preview,result=submit(demo,context)
    assert len(preview['points'])>2
    assert all(demo.clear(*point) for point in preview['points'])
    for _ in range(600):
        demo.runtime.renew('operator',context['lease_token'],context['session_epoch'])
        demo.tick(.1)
        if demo.goal_state=='succeeded':break
    assert demo.goal_state=='succeeded'
    assert demo.pose==pytest.approx([4.025,3.025,.5])
    assert demo.status()['simulation'] is True
    assert result['planner_only'] is True
    from lite3_odom_tf_adapter.map_identity import read_bundle
    with pytest.raises(ValueError,match='Synthetic'):read_bundle(demo.maps/'demo_room',require_reviewed=True)


@pytest.mark.parametrize('fault',['camera','odometry','tf','obstacle'])
def test_dependency_loss_stops_virtual_motion_and_requires_explicit_restart(tmp_path,fault):
    demo,context=ready_demo(tmp_path);submit(demo,context)
    demo.tick(.1);pose=list(demo.pose)
    demo.action('/api/v1/demo/fault',{'name':fault},'operator')
    demo.tick(.1)
    assert demo.pose==pose
    assert demo.stopped and demo.goal_state=='stopped'
    demo.action('/api/v1/demo/fault',{'name':fault,'active':False},'operator')
    demo.tick(1)
    assert demo.pose==pose
    with pytest.raises(ValueError):demo.runtime.renew('operator',context['lease_token'],context['session_epoch'])


def test_wall_target_wrong_map_and_missing_preview_are_rejected(tmp_path):
    demo,context=ready_demo(tmp_path)
    with pytest.raises(ValueError,match='occupied'):submit(demo,context,2.5,2)
    with pytest.raises(ValueError,match='Map/session'):demo.action('/api/v1/navigation/preview',{'map_id':'wrong'},'operator')
    with pytest.raises(ValueError,match='preview'):demo.action('/api/v1/navigation/goal',{**context,'x':4.,'y':3.,'yaw':0},'operator')
