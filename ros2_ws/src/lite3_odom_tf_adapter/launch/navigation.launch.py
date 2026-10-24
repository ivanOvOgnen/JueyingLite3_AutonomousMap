"""Launch Nav2 with camera overrides and a gated, remapped velocity output.

This is an uncommissioned profile. The gate publishes only to
/autonomy/cmd_vel; no ROS 1 bridge or robot motion path is launched here.
"""

import json
import os
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from lite3_odom_tf_adapter.navigation_config import merge_config


def _merge_dicts(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge_dicts(base[key], value)
        else:
            base[key] = value
    return base


def _setup(context, *args, **kwargs):
    vendor_params = Path(context.perform_substitution(LaunchConfiguration('vendor_params')))
    map_yaml = context.perform_substitution(LaunchConfiguration('map'))
    package_share = Path(get_package_share_directory('lite3_odom_tf_adapter'))
    overlay_path = package_share / 'config' / 'nav2_camera_overrides.yaml'
    if not vendor_params.is_file():
        raise RuntimeError(f'Vendor Nav2 parameters not found: {vendor_params}')
    if not overlay_path.is_file():
        raise RuntimeError(f'Project Nav2 overlay not found: {overlay_path}')

    commissioned = context.perform_substitution(LaunchConfiguration('commissioned_motion')).lower() == 'true'
    record = None
    if commissioned:
        record_path = Path.home()/'.config/lite3_autonomy/commissioning.json'
        record = json.loads(record_path.read_text())
        if any(record.get(key) is not True for key in ('vendor_exclusivity_verified', 'independent_stop_verified',
                                                      'downstream_timeout_verified', 'footprint_measured', 'visibility_reviewed')):
            raise RuntimeError('Motion commissioning record is incomplete')
    merged = merge_config(
        yaml.safe_load(vendor_params.read_text(encoding='utf-8')) or {},
        yaml.safe_load(overlay_path.read_text(encoding='utf-8')) or {},
        behavior_tree=package_share/'config'/'navigate_observed_forward.xml', commissioning=record)
    fd, filename = tempfile.mkstemp(prefix='lite3_nav2_', suffix='.yaml')
    merged_path = Path(filename)
    with os.fdopen(fd, 'w') as output:
        yaml.safe_dump(merged, output, sort_keys=False)

    nav2_launch_dir = Path(get_package_share_directory('nav2_bringup')) / 'launch'
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'localization.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': str(merged_path),
            'use_sim_time': 'False',
            'autostart': 'True',
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_launch_dir / 'navigation_launch.py')),
        launch_arguments={
            'params_file': str(merged_path),
            'use_sim_time': 'False',
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )
    perception = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(package_share / 'launch' / 'perception.launch.py')))
    # Route all Nav2 producers, including recoveries, to the gate input.
    nav2_group = GroupAction([
        SetRemap(src='cmd_vel', dst='/nav2/cmd_vel'),
        SetRemap(src='/cmd_vel', dst='/nav2/cmd_vel'),
        localization,
        navigation,
    ])
    gate = Node(
        package='lite3_odom_tf_adapter',
        executable='cmd_vel_gate',
        name='lite3_cmd_vel_gate',
        output='screen',
        parameters=[{
            'input_topic': '/nav2/cmd_vel',
            'output_topic': '/autonomy/cmd_vel',
            'readiness_topic': '/autonomy/ready',
            'command_timeout': 0.30,
            'readiness_timeout': 1.0,
            'max_linear_x': 0.10,
            'max_angular_z': 0.20,
        }],
    )
    supervisor = Node(package='lite3_odom_tf_adapter', executable='motion_supervisor',
                      name='lite3_motion_supervisor', output='screen')
    arbiter = Node(package='lite3_odom_tf_adapter', executable='motion_arbiter',
                   name='lite3_motion_arbiter', output='screen',
                   parameters=[{'physical_stop_verified': commissioned}])
    # Isolated sink unless the explicit commissioned profile was selected.
    relay = Node(package='lite3_odom_tf_adapter', executable='vendor_cmd_vel_bridge',
                 name='lite3_vendor_cmd_vel_bridge', output='screen',
                 parameters=[{'physical_stop_verified': commissioned,
                              'output_topic': '/cmd_vel' if commissioned else '/autonomy/vendor_sink'}])
    supervised = context.perform_substitution(LaunchConfiguration('supervised_trial')).lower() == 'true'
    if supervised and commissioned:
        raise RuntimeError('Supervised trial and commissioned motion cannot run together')
    trial_relay = [Node(package='lite3_odom_tf_adapter', executable='supervised_nav_relay',
                        name='lite3_supervised_nav_relay', output='screen')] if supervised else []
    return [perception, gate, arbiter, relay, supervisor, nav2_group] + trial_relay


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('commissioned_motion', default_value='false',
                              description='Use reviewed measured footprint and vendor output; never auto-arm'),
        DeclareLaunchArgument('supervised_trial', default_value='false',
                              description='Launch a separate, time-limited and explicitly armed Nav2 relay'),
        DeclareLaunchArgument(
            'map',
            default_value=str(Path(get_package_share_directory('lite3_odom_tf_adapter')) /
                              'maps' / 'lite3_room.yaml'),
            description='Map YAML to localize against'),
        DeclareLaunchArgument(
            'vendor_params',
            default_value='/home/ysc/lite_cog_ros2/nav/src/dr_nav2/config/lite_nav2.yaml',
            description='Base Foxy Nav2 YAML supplied by the vendor'),
        OpaqueFunction(function=_setup),
    ])
