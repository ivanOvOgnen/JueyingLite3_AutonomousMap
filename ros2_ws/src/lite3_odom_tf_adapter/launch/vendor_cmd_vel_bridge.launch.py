"""Explicitly commissioned, fail-closed vendor command bridge.

This launch is never included by navigation.launch.py. It starts only the
guarded relay. The project owns one combined ROS 1 parameter bridge for both
``/leg_odom2`` and ``/cmd_vel_corrected``; starting another parameter_bridge
here would create competing bridge processes and is intentionally avoided.
The relay still refuses its enable service unless the physical-stop parameter
is true and the shared stop record is fresh and clear.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enabled = LaunchConfiguration('enable_vendor_bridge')
    physical_stop = LaunchConfiguration('physical_stop_verified')
    relay = Node(
        package='lite3_odom_tf_adapter',
        executable='vendor_cmd_vel_bridge',
        name='lite3_vendor_cmd_vel_bridge',
        output='screen',
        condition=IfCondition(enabled),
        parameters=[{'physical_stop_verified': physical_stop}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('enable_vendor_bridge', default_value='false'),
        DeclareLaunchArgument('physical_stop_verified', default_value='false'),
        relay,
    ])
