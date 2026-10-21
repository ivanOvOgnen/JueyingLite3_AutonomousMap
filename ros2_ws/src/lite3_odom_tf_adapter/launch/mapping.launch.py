"""Start the shared perception pipeline and SLAM; publishes no velocities."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lite3_odom_tf_adapter')
    perception = os.path.join(package_share, 'launch', 'perception.launch.py')
    slam_params = os.path.join(package_share, 'config', 'mapping.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(perception)),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[slam_params], output='screen'),
    ])
