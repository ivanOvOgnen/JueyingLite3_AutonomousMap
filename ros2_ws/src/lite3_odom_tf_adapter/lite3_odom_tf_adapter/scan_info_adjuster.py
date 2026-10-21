"""Move the depth-to-scan sampling row to the robot's horizontal plane.

Foxy's depthimage_to_laserscan 2.3.x has no scan_offset parameter. It chooses
rows around CameraInfo.cy, so this node adjusts cy on a private CameraInfo
topic by the calibrated camera pitch. The generated LaserScan is labeled with
a separate level frame whose origin is the depth camera optical center.
"""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo


class ScanInfoAdjuster(Node):
    def __init__(self):
        super().__init__('lite3_scan_info_adjuster')
        self.declare_parameter('camera_pitch_down', 0.34907)
        self.declare_parameter('scan_height', 10)
        self._pitch = float(self.get_parameter('camera_pitch_down').value)
        self._scan_height = int(self.get_parameter('scan_height').value)
        self._logged = False
        self._invalid_logged = False
        # Foxy's depthimage_to_laserscan CameraInfo subscription requests
        # RELIABLE QoS. Keep the incoming RealSense subscription sensor-data
        # compatible, then publish the adjusted copy reliably for the converter.
        camera_info_output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            CameraInfo, '/camera/depth/camera_info_scan', camera_info_output_qos)
        self._subscription = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self._on_info,
            qos_profile_sensor_data)

    def _on_info(self, message):
        # Rectified pinhole projection: P[5] is fy and P[6] is cy.
        fy = float(message.p[5]) if message.p[5] > 0.0 else float(message.k[4])
        cy = float(message.p[6]) if message.p[5] > 0.0 else float(message.k[5])
        if fy <= 0.0 or message.height <= 0:
            if not self._invalid_logged:
                self.get_logger().error('CameraInfo has no valid vertical focal length/height')
                self._invalid_logged = True
            return

        # The camera optical axis is pitched down from base_link. Sample the
        # image row that points level with base_link instead of the image center.
        adjusted_cy = cy - fy * math.tan(self._pitch)
        half_band = self._scan_height / 2.0
        if adjusted_cy < half_band or adjusted_cy + half_band >= message.height:
            if not self._invalid_logged:
                self.get_logger().error(
                    f'Horizontal scan row {adjusted_cy:.1f} is outside the '
                    f'{message.height}-row depth image; scan info will not be published')
                self._invalid_logged = True
            return

        adjusted = copy.deepcopy(message)
        adjusted.k[5] = adjusted_cy
        adjusted.p[6] = adjusted_cy
        self._publisher.publish(adjusted)
        if not self._logged:
            self.get_logger().info(
                f'CameraInfo cy {cy:.1f} -> {adjusted_cy:.1f}; '
                f'pitch-down compensation {math.degrees(self._pitch):.2f} deg')
            self._logged = True


def main(args=None):
    rclpy.init(args=args)
    node = ScanInfoAdjuster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
