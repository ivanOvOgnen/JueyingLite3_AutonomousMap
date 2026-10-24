#!/usr/bin/env python3
"""Passive, loss-accounting recorder for Lite3 mapping validation sessions.

It publishes no topics and sends no service/action requests.  Full depth image
bytes are retained separately from JSONL event metadata so an offline 3D
reconstructor can use the source image timestamp and camera intrinsics rather
than the known-invalid raw PointCloud2 clock.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


FORMAT = 'lite3_mapping_capture/v1'


def json_safe(value):
    """Preserve non-finite sensor values as explicit strings in strict JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('Infinity' if value > 0 else '-Infinity')
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_dict(pose):
    return {'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
            'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y,
                            'z': pose.orientation.z, 'w': pose.orientation.w}}


def vector_dict(vector):
    return {'x': vector.x, 'y': vector.y, 'z': vector.z}


def quaternion_dict(quaternion):
    return {'x': quaternion.x, 'y': quaternion.y, 'z': quaternion.z, 'w': quaternion.w}


def transform_dict(transform):
    return {'parent': transform.header.frame_id, 'child': transform.child_frame_id,
            'stamp_ns': stamp_ns(transform.header.stamp),
            'translation': {'x': transform.transform.translation.x, 'y': transform.transform.translation.y,
                            'z': transform.transform.translation.z},
            'rotation': {'x': transform.transform.rotation.x, 'y': transform.transform.rotation.y,
                         'z': transform.transform.rotation.z, 'w': transform.transform.rotation.w}}


class MappingRecorder(Node):
    def __init__(self, output_dir, max_bytes):
        super().__init__('lite3_mapping_recorder')
        self.output = Path(output_dir).expanduser().resolve()
        if self.output.exists():
            raise RuntimeError(f'Refusing to overwrite existing recording: {self.output}')
        self.output.mkdir(parents=True)
        self.depth_dir = self.output / 'depth'
        self.depth_dir.mkdir()
        self.events = (self.output / 'events.jsonl').open('x', encoding='utf-8', buffering=1)
        self.max_bytes = max_bytes
        self.saved_bytes = 0
        self.reserved_bytes = 0
        self.state_lock = threading.Lock()
        self.writer_error = None
        self.depth_index = 0
        self.stopping = False
        self.counts = {}
        self.dropped = {}
        self.event_lock = threading.Lock()
        self.depth_queue = queue.Queue(maxsize=64)
        self.writer = threading.Thread(target=self._write_depth, daemon=True)
        self.writer.start()
        self._write_metadata()
        sensor = qos_profile_sensor_data
        retained = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self._depth, sensor)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self._camera_info, sensor)
        self.create_subscription(LaserScan, '/scan', self._scan, sensor)
        self.create_subscription(Odometry, '/leg_odom2', lambda msg: self._odom(msg, 'vendor_odom'), 10)
        self.create_subscription(Odometry, '/odom', lambda msg: self._odom(msg, 'normalized_odom'), 10)
        self.create_subscription(Imu, '/imu/data', self._imu, sensor)
        self.create_subscription(OccupancyGrid, '/map', self._map, retained)
        self.create_subscription(TFMessage, '/tf', lambda msg: self._tf(msg, 'tf'), 100)
        self.create_subscription(TFMessage, '/tf_static', lambda msg: self._tf(msg, 'tf_static'), retained)
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self._cloud_header, sensor)
        self.timer = self.create_timer(1.0, self._heartbeat)
        self._event('session_start', output=str(self.output), max_bytes=max_bytes)

    def _write_metadata(self):
        data = {'format': FORMAT, 'created_unix_ns': time.time_ns(), 'topics': {
            'depth': '/camera/depth/image_rect_raw', 'camera_info': '/camera/depth/camera_info',
            'scan': '/scan', 'vendor_odom': '/leg_odom2', 'normalized_odom': '/odom',
            'imu': '/imu/data', 'map': '/map', 'tf': '/tf', 'tf_static': '/tf_static',
            'raw_cloud_header': '/camera/depth/color/points'},
            'notes': ['Reception timestamps are recorder host wall-clock timestamps.',
                      'Message header stamps are preserved verbatim.',
                      'Non-finite numeric values use JSON strings NaN, Infinity and -Infinity.',
                      'Raw PointCloud2 payload is intentionally not trusted for map registration.']}
        (self.output / 'metadata.json').write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')

    def _event(self, kind, **payload):
        with self.event_lock:
            self.counts[kind] = self.counts.get(kind, 0) + 1
            row = {'kind': kind, 'received_unix_ns': time.time_ns(), **payload}
            self.events.write(json.dumps(json_safe(row), separators=(',', ':'), allow_nan=False) + '\n')

    def _header(self, header):
        return {'frame_id': header.frame_id, 'stamp_ns': stamp_ns(header.stamp)}

    def _depth(self, message):
        received = time.time_ns()
        if self.stopping:
            return
        payload = bytes(message.data)
        with self.state_lock:
            if self.writer_error:
                self.dropped['depth_writer_failed'] = self.dropped.get('depth_writer_failed', 0) + 1
                return
            if self.reserved_bytes + len(payload) > self.max_bytes:
                self.dropped['depth_disk_limit'] = self.dropped.get('depth_disk_limit', 0) + 1
                return
            # Reserve queued and in-progress bytes, not just completed writes.
            self.reserved_bytes += len(payload)
        index = self.depth_index
        self.depth_index += 1
        item = (index, payload, self._header(message.header), message.width, message.height,
                message.encoding, message.is_bigendian, message.step, received)
        try:
            self.depth_queue.put_nowait(item)
        except queue.Full:
            with self.state_lock:
                self.reserved_bytes -= len(payload)
                self.dropped['depth_writer_queue'] = self.dropped.get('depth_writer_queue', 0) + 1

    def _write_depth(self):
        while True:
            item = self.depth_queue.get()
            try:
                if item is None:
                    return
                index, payload, header, width, height, encoding, big_endian, step, received = item
                filename = f'{index:08d}.bin'
                path = self.depth_dir / filename
                with path.open('xb') as output:
                    output.write(payload)
                with self.state_lock:
                    self.saved_bytes += len(payload)
                self._event('depth', index=index, file=f'depth/{filename}', bytes=len(payload),
                            sha256=hashlib.sha256(payload).hexdigest(), header=header, width=width,
                            height=height, encoding=encoding, is_bigendian=bool(big_endian), step=step,
                            received_unix_ns=received)
            except Exception as error:
                with self.state_lock:
                    self.writer_error = f'{type(error).__name__}: {error}'
                    self.dropped['depth_write_error'] = self.dropped.get('depth_write_error', 0) + 1
                return
            finally:
                self.depth_queue.task_done()

    def _camera_info(self, msg):
        self._event('camera_info', header=self._header(msg.header), width=msg.width, height=msg.height,
                    distortion_model=msg.distortion_model, d=list(msg.d), k=list(msg.k), r=list(msg.r), p=list(msg.p))

    def _scan(self, msg):
        self._event('scan', header=self._header(msg.header), angle_min=msg.angle_min, angle_max=msg.angle_max,
                    angle_increment=msg.angle_increment, time_increment=msg.time_increment, scan_time=msg.scan_time,
                    range_min=msg.range_min, range_max=msg.range_max, ranges=list(msg.ranges), intensities=list(msg.intensities))

    def _odom(self, msg, kind):
        self._event(kind, header=self._header(msg.header), child_frame_id=msg.child_frame_id,
                    pose=pose_dict(msg.pose.pose), twist={'linear': vector_dict(msg.twist.twist.linear), 'angular': vector_dict(msg.twist.twist.angular)},
                    pose_covariance=list(msg.pose.covariance), twist_covariance=list(msg.twist.covariance))

    def _imu(self, msg):
        self._event('imu', header=self._header(msg.header), orientation=quaternion_dict(msg.orientation),
                    angular_velocity=vector_dict(msg.angular_velocity), linear_acceleration=vector_dict(msg.linear_acceleration),
                    orientation_covariance=list(msg.orientation_covariance), angular_velocity_covariance=list(msg.angular_velocity_covariance),
                    linear_acceleration_covariance=list(msg.linear_acceleration_covariance))

    def _map(self, msg):
        self._event('map', header=self._header(msg.header), width=msg.info.width, height=msg.info.height,
                    resolution=msg.info.resolution, origin=pose_dict(msg.info.origin), data=list(msg.data))

    def _tf(self, msg, kind):
        self._event(kind, transforms=[transform_dict(item) for item in msg.transforms])

    def _cloud_header(self, msg):
        self._event('raw_cloud_header', header=self._header(msg.header), width=msg.width, height=msg.height,
                    point_step=msg.point_step, row_step=msg.row_step, fields=[{'name': field.name, 'offset': field.offset, 'datatype': field.datatype, 'count': field.count} for field in msg.fields])

    def _heartbeat(self):
        free = shutil.disk_usage(self.output).free
        with self.state_lock:
            dropped, saved, reserved, error = (dict(self.dropped), self.saved_bytes,
                                                self.reserved_bytes, self.writer_error)
        with self.event_lock:
            counts = dict(self.counts)
        self._event('heartbeat', counts=counts, dropped=dropped, queued_depth=self.depth_queue.qsize(),
                    saved_depth_bytes=saved, reserved_depth_bytes=reserved,
                    writer_error=error, free_bytes=free)

    def close(self):
        if self.stopping:
            return
        self.stopping = True
        deadline = time.monotonic() + 15.0
        # A full queue plus a dead writer used to block close indefinitely.
        while self.writer.is_alive() and time.monotonic() < deadline:
            try:
                self.depth_queue.put(None, timeout=min(0.1, max(0.001, deadline - time.monotonic())))
                break
            except queue.Full:
                continue
        self.writer.join(timeout=max(0.0, deadline - time.monotonic()))
        finished = not self.writer.is_alive()
        with self.state_lock:
            summary = {'format': FORMAT, 'closed_unix_ns': time.time_ns(),
                       'dropped': dict(self.dropped), 'saved_depth_bytes': self.saved_bytes,
                       'reserved_depth_bytes': self.reserved_bytes, 'writer_error': self.writer_error,
                       'queued_depth': self.depth_queue.qsize(), 'writer_finished': finished}
        with self.event_lock:
            summary['counts'] = dict(self.counts)
        (self.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        if finished:
            self.events.close()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_dir', help='new session directory; it must not already exist')
    parser.add_argument('--max-gib', type=float, default=20.0, help='hard cap for depth payloads (default: 20 GiB)')
    options, ros_args = parser.parse_known_args(args=args)
    if not math.isfinite(options.max_gib) or options.max_gib <= 0:
        parser.error('--max-gib must be finite and positive')
    rclpy.init(args=ros_args)
    recorder = MappingRecorder(options.output_dir, int(options.max_gib * 1024 ** 3))
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
