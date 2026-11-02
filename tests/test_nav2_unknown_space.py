from pathlib import Path

import yaml


def test_unknown_saved_map_stays_blocked_while_local_window_uses_live_obstacles():
    from lite3_odom_tf_adapter.navigation_config import merge_config

    root = Path(__file__).resolve().parents[1]
    overlay = yaml.safe_load((root / 'ros2_ws/src/lite3_odom_tf_adapter/config/nav2_camera_overrides.yaml').read_text())
    vendor = {'controller_server': {'ros__parameters': {
        'FollowPath': {'plugin': 'dwb_core::DWBLocalPlanner'}}}}
    merged = merge_config(vendor, overlay, behavior_tree=root / 'dummy.xml')
    local = merged['local_costmap']['local_costmap']['ros__parameters']
    global_map = merged['global_costmap']['global_costmap']['ros__parameters']
    assert local['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert local['track_unknown_space'] is False
    assert global_map['plugins'] == ['static_layer', 'inflation_layer']
    assert global_map['track_unknown_space'] is True
    assert merged['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
