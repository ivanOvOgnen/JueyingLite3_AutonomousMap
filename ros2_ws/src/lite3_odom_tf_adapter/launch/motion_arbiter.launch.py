"""Start the fail-closed, vendor-disconnected motion arbiter."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lite3_odom_tf_adapter',
            executable='motion_arbiter',
            name='lite3_motion_arbiter',
            output='screen',
            parameters=[{
                'input_topic': '/autonomy/cmd_vel',
                'output_topic': '/autonomy/cmd_vel_arbited',
                'readiness_topic': '/autonomy/ready',
                'manual_override_topic': '/autonomy/manual_override',
                'status_topic': '/autonomy/motion_status',
                'physical_stop_verified': False,
                'safety_shared_memory_path': '/dev/shm/emotion_bot_lite3_safety_state',
                'safety_record_timeout': 1.0,
                'command_timeout': 0.25,
                'readiness_timeout': 1.0,
                'max_linear_x': 0.10,
                'max_angular_z': 0.20,
            }],
        ),
    ])
