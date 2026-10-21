#!/usr/bin/env python3
"""Normalize Lite3 odometry headers and publish odom -> base_link TF."""

import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfAdapter(Node):
    def __init__(self):
        super().__init__('lite3_odom_tf_adapter')

        self.declare_parameter('input_topic', '/leg_odom2')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._on_odometry, 10
        )

        self.get_logger().info(
            f'Remapping {input_topic} to {output_topic}; '
            f'frames {self.odom_frame} -> {self.base_frame}; '
            f'TF broadcast={self.publish_tf}'
        )

    def _on_odometry(self, source_msg):
        # Vendor messages are stamped with steady-clock uptime. Restamp on arrival
        # with this ROS node's system clock so they can synchronize with the camera.
        stamp = self.get_clock().now().to_msg()

        odom = copy.deepcopy(source_msg)
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        self._publisher.publish(odom)

        if self._tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
