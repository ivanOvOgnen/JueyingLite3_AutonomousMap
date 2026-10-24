"""Paired acquisition-stamped obstacle cloud, independent of broken raw cloud."""
from collections import OrderedDict
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from lite3_odom_tf_adapter.depth_projection import project_depth, stamp_ns


class DepthObstacleCloud(Node):
    def __init__(self):
        super().__init__('lite3_depth_obstacle_cloud')
        self.declare_parameter('stride', 4)
        # The vendor depth stream has measured transport latency near 0.75 s.
        # Keep acquisition timestamps and reject observations older than 1 s.
        self.declare_parameter('max_source_age', 1.0)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 3.0)
        self._images, self._infos = OrderedDict(), OrderedDict()
        self._last_stamp = 0
        self._last_valid_at = None
        self._health = {'healthy': False, 'reason': 'No paired depth observation'}
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._cloud = self.create_publisher(PointCloud2, '/perception/depth_obstacles', qos_profile_sensor_data)
        self._status = self.create_publisher(String, '/perception/depth_obstacles/status', 1)
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 lambda msg: self._receive(msg, self._images), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 lambda msg: self._receive(msg, self._infos), qos_profile_sensor_data)
        self.create_timer(0.1, self._publish_health)

    def _receive(self, message, cache):
        key = (stamp_ns(message.header), message.header.frame_id)
        cache[key] = message
        while len(cache) > 4:
            cache.popitem(last=False)
        if key not in self._images or key not in self._infos:
            return
        image, info = self._images.pop(key), self._infos.pop(key)
        try:
            if key[0] <= self._last_stamp:
                raise ValueError('Duplicate/backward depth acquisition stamp')
            points, health = project_depth(
                image, info, now_ns=self.get_clock().now().nanoseconds,
                max_age=float(self.get_parameter('max_source_age').value),
                stride=int(self.get_parameter('stride').value),
                min_range=float(self.get_parameter('min_range').value),
                max_range=float(self.get_parameter('max_range').value))
            acquisition = Time.from_msg(image.header.stamp)
            for target in ('base_link',):
                if not self._tf.can_transform(target, image.header.frame_id, acquisition):
                    raise ValueError('Acquisition-time TF unavailable: ' + target)
            cloud = PointCloud2()
            cloud.header = image.header
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                            for index, name in enumerate(('x', 'y', 'z'))]
            cloud.is_bigendian, cloud.is_dense = False, True
            cloud.point_step, cloud.row_step = 12, len(points)*12
            cloud.data = points.tobytes()
            self._cloud.publish(cloud)
            self._last_stamp = key[0]
            self._last_valid_at = time.monotonic()
            self._health = {'healthy': True, 'reason': 'Paired source depth and acquisition-time TF valid', **health}
        except (ValueError, TransformException) as error:
            self._health = {'healthy': False, 'reason': str(error)}
        self._publish_health()

    def _publish_health(self):
        status = dict(self._health)
        age = None if self._last_valid_at is None else time.monotonic() - self._last_valid_at
        source_age = ((self.get_clock().now().nanoseconds - self._last_stamp) / 1e9
                      if self._last_stamp else None)
        max_source_age = float(self.get_parameter('max_source_age').value)
        if age is None or age > 0.5:
            detail = (self._health.get('reason') if self._health.get('healthy') is False
                      else 'last valid cloud timed out')
            status.update(healthy=False, reason='No recent valid depth observation: ' + str(detail))
        elif source_age is None or not -0.05 <= source_age <= max_source_age:
            status.update(healthy=False, reason='Depth acquisition is future or stale')
        status['receipt_age_seconds'] = age
        status['source_age_seconds'] = source_age
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self._status.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
