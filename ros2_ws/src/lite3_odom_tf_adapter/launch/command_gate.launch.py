"""Start the closed-by-default velocity gate. No robot command bridge is started."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
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
        ),
    ])
