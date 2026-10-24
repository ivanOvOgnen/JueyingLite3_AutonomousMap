"""Start the shared Lite3 odometry and RealSense depth-to-scan pipeline."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='lite3_odom_tf_adapter', executable='depth_obstacle_cloud',
             name='lite3_depth_obstacle_cloud', output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_camera_transform',
            arguments=['0.25489', '0', '0.07249', '0', '0.34907', '0',
                       'base_link', 'camera_link'],
        ),
        # The vendor camera publishes points in RealSense's optical convention
        # but does not publish the camera_link -> depth optical TF on this robot.
        # Match the standard RealSense depth-to-optical rotation so Nav2 can
        # transform the live point cloud into its costmap frames.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='camera_link_to_depth_optical_transform',
            arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679',
                       'camera_link', 'camera_depth_optical_frame'],
        ),
        # The planar synthetic scan is leveled; the scan adjuster compensates
        # for the camera's downward pitch by selecting the matching image row.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_camera_scan_transform',
            arguments=['0.25489', '0', '0.07249', '0', '0', '0',
                       'base_link', 'camera_scan_frame'],
        ),
        Node(package='lite3_odom_tf_adapter', executable='odom_tf_adapter',
             name='lite3_odom_tf_adapter', output='screen'),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth', '/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info_scan'),
                        ('scan', '/scan_raw')],
            parameters=[{'scan_time': 0.033, 'range_min': 0.45,
                         'range_max': 6.0, 'scan_height': 10,
                         'output_frame': 'camera_scan_frame'}],
            output='screen'),
        Node(
            package='lite3_odom_tf_adapter', executable='scan_info_adjuster',
            name='lite3_scan_info_adjuster',
            parameters=[{'camera_pitch_down': 0.34907, 'scan_height': 10}],
            output='screen'),
        Node(
            package='lite3_odom_tf_adapter', executable='scan_delay',
            name='lite3_scan_delay',
            parameters=[{'delay_seconds': 0.5, 'queue_size': 60}],
            output='screen'),
    ])
