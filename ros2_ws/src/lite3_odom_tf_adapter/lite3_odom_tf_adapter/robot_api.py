#!/usr/bin/env python3
"""HTTP gateway for Lite3 mapping and gated Nav2 planner requests."""

import json
import base64
import hashlib
import hmac
from http.cookies import SimpleCookie
import math
import os
from pathlib import Path
import re
import secrets
import signal
import struct
import subprocess
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse
import yaml

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from std_msgs.msg import Bool, String
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.type_support import check_for_type_support
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_srvs.srv import Empty, SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from lite3_odom_tf_adapter.save_map import save_grid
from lite3_odom_tf_adapter.map_bundle import load_manifest
from lite3_odom_tf_adapter.map_identity import grid_signature, read_bundle, review_bundle
from lite3_odom_tf_adapter.runtime_state import RuntimeState


STALE_AFTER = 2.0
MAP_STALE_AFTER = 10.0
LOCALIZATION_STALE_AFTER = 6.0
LOCALIZATION_MAX_SEARCH_SECONDS = 90.0
MAP_NAME = re.compile(r'^[A-Za-z0-9_-]{1,64}$')


class RobotApi(Node):
    def __init__(self):
        super().__init__('lite3_robot_api')
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('allow_nav2_goal_requests', False)
        self.declare_parameter('web_root', '/home/ysc/lite3_autonomy_ws/webapp')
        self.declare_parameter('maps_dir', '/home/ysc/lite3_autonomy_ws/webapp/maps')
        self.declare_parameter(
            'token_file', '/home/ysc/.config/lite3_robot_api/token')
        self.bind_address = str(self.get_parameter('bind_address').value)
        self.port = int(self.get_parameter('port').value)
        self.allow_nav2_goal_requests = bool(
            self.get_parameter('allow_nav2_goal_requests').value)
        self.web_root = Path(str(self.get_parameter('web_root').value)).expanduser().resolve()
        self.maps_dir = Path(str(self.get_parameter('maps_dir').value)).expanduser().resolve()
        self.token_file = Path(str(self.get_parameter('token_file').value)).expanduser()
        self._token_lock = threading.Lock()
        self.api_token = self._load_token()
        self._session_lock = threading.Lock()
        self._browser_sessions = {}
        self._session_ttl_seconds = 12 * 60 * 60
        self._lock = threading.RLock()
        self._api_action_lock = threading.Lock()
        self._shutdown_requested = False
        self._http_server_shutdown = None
        self._runtime = RuntimeState()
        self._session_subscriptions = []
        self._navigation_identity = None
        self._navigation_map_reviewed = False
        self._navigation_map_trial_eligible = False
        self._expected_map_signature = None
        self._map_condition = threading.Condition(self._lock)
        self._localization_generation = 0
        self._localization_not_before_ns = 0
        self._stationary_since = None
        self._operating_region = None
        self._map_sequence = 0
        self._path_preview = None
        self._path_request_id = None
        self._goal_feedback = None
        self._observation_health = {}
        self._stage_health = {}
        self._sensor_frames = {}
        self._last_rx = {}
        self._map = None
        self._costmaps = {}
        self._goal_clearance_radius = 0.30
        self._costmap_valid = {'local_costmap': False, 'global_costmap': False}
        self._pointcloud = None
        self._pointcloud_frame = ''
        self._pointcloud_count = 0
        self._stitched_pointcloud = bytearray()
        self._stitched_point_count = 0
        self._last_stitch_time = 0.0
        self._stitch_limit_points = 200_000
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._amcl_pose = None
        self._localization_lock = threading.Lock()
        self._localization_refresh_client = self.create_client(
            Empty, '/request_nomotion_update')
        self._localization_refresh_active = False
        self._localization_refresh_inflight = False
        self._localization_refresh_started_at = 0.0
        self._localization_refresh_requested_at = 0.0
        self._localization_converged = False
        self._localization_stable_samples = 0
        self._localization_previous_candidate = None
        self._localization_reason = 'No AMCL pose has been received.'
        self._localization_refresh_timer = self.create_timer(
            0.5, self._localization_refresh_tick)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._goal_state = 'idle'
        self._goal_detail = 'No navigation goal submitted.'
        self._goal_id = None
        # Retain requests until rejection or a terminal action result. HTTP
        # timeout, cancellation acknowledgment and process cleanup are not
        # terminal action results.
        self._goal_requests = {}
        # Fail closed on process startup. Starting localization/Nav2 is the
        # explicit software re-arm; pressing STOP latches goals off again.
        self._autonomy_stop_latched = True
        self._stop_generation = 0
        self._stop_cleanup_pending = False
        self._stop_cleanup_error = None
        self._stop_cleanup_thread = None
        self._stop_disable_results = {}
        self._nav_action = None
        self._nav_action_error = None
        self._global_localization_client = self.create_client(
            Empty, '/reinitialize_global_localization')
        self._autonomy_enable_client = self.create_client(
            SetBool, '/autonomy/set_enabled')
        self._autonomy_disable_clients = {
            'supervisor': self.create_client(SetBool, '/autonomy/supervisor/set_enabled'),
            'gate': self._autonomy_enable_client,
            'arbiter': self.create_client(SetBool, '/autonomy/arbiter/set_enabled'),
            'vendor_bridge': self.create_client(SetBool, '/autonomy/vendor_bridge/set_enabled'),
            'supervised_trial': self.create_client(SetBool, '/autonomy/supervised_trial/set_enabled'),
        }
        self._context_pub = self.create_publisher(String, '/autonomy/context', 1)
        self._readiness_pub = self.create_publisher(Bool, '/autonomy/ready', 10)
        self._readiness_timer = self.create_timer(0.2, self._publish_readiness)
        try:
            # Validate generated action support before constructing ActionClient.
            # On mismatched ROS Python environments ActionClient can fail partway
            # through initialization, leaving a partially constructed object
            # whose destructor raises another exception. Keep the HTTP API
            # available for non-navigation diagnostics while failing navigation
            # closed if the action type cannot be loaded.
            check_for_type_support(NavigateToPose)
            self._nav_action = ActionClient(
                self, NavigateToPose, '/navigate_to_pose')
        except Exception as error:
            self._nav_action_error = f'{type(error).__name__}: {error}'
            self.get_logger().error(
                'Nav2 NavigateToPose action type is unavailable; '
                f'navigation requests will remain disabled: {self._nav_action_error}')
        self._path_action = None
        try:
            check_for_type_support(ComputePathToPose)
            self._path_action = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        except Exception as error:
            self.get_logger().error(f'Path preview type support unavailable: {error}')
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._mapping_process = None
        self._mapping_paused = False
        self._navigation_process = None
        self._navigation_map_name = None
        self._navigation_started_at = 0.0
        self._navigation_log = None
        self._bridge_process = None
        self._bridge_external = False
        self._bridge_external_identity = None
        self._bridge_param_owned = False
        self._mapping_log = None
        self._bridge_log = None
        self._workspace = Path.home() / 'lite3_autonomy_ws'
        self._bridge_profile = 'telemetry'
        self._software_hash_at = 0.0
        self._software_id = None
        self._bridge_config = (
            self._workspace / 'install' / 'lite3_odom_tf_adapter' / 'share' /
            'lite3_odom_tf_adapter' / 'config' / 'mapping_ros1_bridge.yaml')

        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 lambda msg: self._observe('camera', msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/leg_odom2',
                                 lambda msg: self._observe('vendor_odometry', msg), 10)
        self.create_subscription(Odometry, '/odom',
                                 lambda msg: self._observe('odometry', msg), 10)
        self.create_subscription(LaserScan, '/scan',
                                 lambda msg: self._observe('scan', msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/camera/depth/color/points',
                                 self._on_pointcloud, qos_profile_sensor_data)
        self.create_subscription(String, '/perception/depth_obstacles/status', self._on_observation_health, 1)
        for name, topic in (('gate', '/autonomy/gate/status'),
                            ('arbiter', '/autonomy/motion_status'),
                            ('relay', '/autonomy/vendor_bridge/status'),
                            ('supervisor', '/autonomy/supervisor/status'),
                            ('supervised_trial', '/autonomy/supervised_trial/status')):
            self.create_subscription(String, topic,
                                     lambda msg, key=name: self._on_stage_health(key, msg), 1)
        self._bind_session_topics()
        # Foxy's executor builds a wait set from subscription handles. Changing
        # those handles from an HTTP worker can destroy one while spin() is
        # using it, permanently killing the callback thread (InvalidHandle).
        # The spin loop detaches/shuts down its executor before each rebind.
        self._rebind_lock = threading.Lock()
        self._rebind_requested = 0
        self._rebind_completed = 0
        self._rebind_error = None
        self._rebind_event = threading.Event()

    def _bind_session_topics(self):
        for subscription in self._session_subscriptions:
            self.destroy_subscription(subscription)
        self._session_subscriptions.clear()
        epoch = self._runtime.epoch
        localization_generation = self._localization_generation
        retained = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        specs = [
            (OccupancyGrid, '/map', lambda msg: self._on_map(msg, epoch=epoch), retained),
            (PoseWithCovarianceStamped, '/amcl_pose',
             lambda msg: self._on_localization(msg, epoch=epoch, generation=localization_generation), retained),
        ]
        for key in ('local_costmap', 'global_costmap'):
            specs.append((OccupancyGrid, '/' + key + '/costmap',
                          lambda msg, name=key: self._on_costmap(name, msg, epoch=epoch), 10))
        for message_type, topic, callback, qos in specs:
            self._session_subscriptions.append(self.create_subscription(message_type, topic, callback, qos))

    def _service_session_rebind(self):
        with self._rebind_lock:
            requested = self._rebind_requested
            if requested == self._rebind_completed:
                return
        try:
            self._bind_session_topics()
            error = None
        except Exception as exc:
            error = str(exc)
            self.get_logger().error('ROS session subscription rebind failed: '+error)
        with self._rebind_lock:
            self._rebind_completed = requested
            self._rebind_error = error
            self._rebind_event.set()

    def _session_rebind_pending(self):
        with self._rebind_lock:
            return self._rebind_requested != self._rebind_completed

    def _request_session_rebind(self):
        with self._rebind_lock:
            self._rebind_requested += 1
            requested = self._rebind_requested
            self._rebind_event.clear()
        if not self._rebind_event.wait(3.0):
            raise ApiError(503, 'ROS callback thread did not refresh session subscriptions')
        with self._rebind_lock:
            if self._rebind_completed < requested or self._rebind_error:
                raise ApiError(503, 'ROS session subscription refresh failed: '+str(self._rebind_error))

    def _new_pipeline_session(self, reason):
        with self._lock:
            epoch = self._runtime.reset(reason)
            self._map = self._amcl_pose = self._path_preview = None
            self._path_request_id = None
            self._goal_feedback = None
            self._last_rx.clear()
            self._stationary_since = None
            self._costmap_valid = {'local_costmap': False, 'global_costmap': False}
            self._costmaps.clear()
            self._observation_health = {}
            self.allow_nav2_goal_requests = False
            self._map_condition.notify_all()
        self._request_session_rebind()
        return epoch

    def _observe(self, key, message, epoch=None):
        stamp = int(message.header.stamp.sec)*1_000_000_000 + int(message.header.stamp.nanosec)
        now = self.get_clock().now().nanoseconds
        if epoch is not None and epoch != self._runtime.epoch:
            return
        if self._runtime.observe(key, stamp, now, epoch=epoch):
            self._sensor_frames[key] = message.header.frame_id
            self._mark_rx(key)
            if key == 'odometry':
                twist = message.twist.twist
                values = (twist.linear.x, twist.linear.y, twist.linear.z,
                          twist.angular.x, twist.angular.y, twist.angular.z)
                stationary = all(math.isfinite(v) and abs(v) <= 0.02 for v in values)
                with self._lock:
                    if not stationary:
                        self._stationary_since = None
                    elif self._stationary_since is None:
                        self._stationary_since = time.monotonic()

    def _on_observation_health(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or not isinstance(value.get('healthy'), bool):
                raise ValueError('Malformed obstacle health')
            if value['healthy'] and (type(value.get('stamp_ns')) is not int or value['stamp_ns'] <= 0
                                     or not isinstance(value.get('frame_id'), str) or not value['frame_id']):
                raise ValueError('Missing obstacle acquisition identity')
            if not value['healthy']:
                value.pop('stamp_ns', None)
                value.pop('frame_id', None)
        except (ValueError, TypeError):
            value = {'healthy': False, 'reason': 'Malformed obstacle health'}
        with self._lock:
            self._observation_health = {**value, 'rx': time.monotonic()}

    def _on_stage_health(self, name, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or not isinstance(value.get('enabled'), bool):
                raise ValueError('Malformed stage state')
        except (ValueError, TypeError):
            value = {'enabled': False, 'state': 'unknown', 'reason': 'Malformed/missing structured state'}
        with self._lock:
            self._stage_health[name] = {**value, 'rx': time.monotonic()}

    def _mark_rx(self, key):
        with self._lock:
            self._last_rx[key] = time.monotonic()

    def _on_map(self, message, epoch=None):
        with self._lock:
            if epoch is not None and epoch != self._runtime.epoch:
                return
            if self._expected_map_signature and grid_signature(message) != self._expected_map_signature:
                self._last_rx['map'] = None
                self._map = None
                self._path_preview = None
                self.allow_nav2_goal_requests = False
                return
            self._map = message
            self._last_rx['map'] = time.monotonic()
            self._map_sequence += 1
            self._map_condition.notify_all()

    def _on_costmap(self, name, message, epoch=None):
        if epoch is not None and epoch != self._runtime.epoch:
            return
        # Nav2's public `*/costmap` topics are OccupancyGrid. The
        # nav2_msgs/Costmap type is published on the separate `*/costmap_raw`
        # topics; using it here silently prevents a DDS endpoint match.
        self._observe(name, message, epoch=epoch)
        info = message.info
        size_x = int(info.width)
        size_y = int(info.height)
        valid = (
            size_x > 0 and size_y > 0 and
            math.isfinite(float(info.resolution)) and float(info.resolution) > 0.0 and
            len(message.data) == size_x * size_y)
        with self._lock:
            if epoch is not None and epoch != self._runtime.epoch:
                return
            self._costmap_valid[name] = valid
            self._costmaps[name] = message if valid else None
            if valid:
                self._last_rx[name] = time.monotonic()

    def _tf_available(self, target_frame, source_frame, stamp_ns=None):
        try:
            return self._tf_buffer.can_transform(target_frame, source_frame,
                                                 Time(nanoseconds=stamp_ns) if stamp_ns is not None else Time())
        except (TransformException, ValueError):
            return False

    def _on_pointcloud(self, message):
        fields = {field.name: field for field in message.fields}
        xyz = [fields.get(name) for name in ('x', 'y', 'z')]
        if any(field is None or field.datatype != 7 for field in xyz):
            return
        width, height = int(message.width), int(message.height)
        if width <= 0 or height <= 0 or message.point_step < 12:
            return
        total = width * height
        stride = max(1, int(math.ceil(total / 5000.0)))
        packed = bytearray()
        count = 0
        raw = message.data
        endian = '>' if message.is_bigendian else '<'
        for index in range(0, total, stride):
            row, column = divmod(index, width)
            base = row * message.row_step + column * message.point_step
            try:
                x = struct.unpack_from(endian + 'f', raw, base + xyz[0].offset)[0]
                y = struct.unpack_from(endian + 'f', raw, base + xyz[1].offset)[0]
                z = struct.unpack_from(endian + 'f', raw, base + xyz[2].offset)[0]
            except struct.error:
                continue
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if z <= 0.10 or z > 8.0 or abs(x) > 8.0 or abs(y) > 8.0:
                continue
            packed.extend(struct.pack('<fff', x, y, z))
            count += 1
        if count:
            with self._lock:
                self._pointcloud = bytes(packed)
                self._pointcloud_count = count
                self._pointcloud_frame = message.header.frame_id
                self._last_rx['pointcloud'] = time.monotonic()
                mapping_active = (
                    self._mapping_process is not None and
                    self._mapping_process.poll() is None and
                    not self._mapping_paused)
                should_stitch = (
                    mapping_active and
                    time.monotonic() - self._last_stitch_time >= 0.5)
                if should_stitch:
                    self._last_stitch_time = time.monotonic()
            if should_stitch:
                self._append_stitched_cloud(message, fields)

    def _append_stitched_cloud(self, message, fields):
        """Append a sparse camera cloud transformed through current map TF."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', message.header.frame_id, Time.from_msg(message.header.stamp))
        except TransformException:
            return
        q = transform.transform.rotation
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        tz = float(transform.transform.translation.z)
        qx, qy, qz, qw = map(float, (q.x, q.y, q.z, q.w))
        endian = '>' if message.is_bigendian else '<'
        xyz = [fields.get(name) for name in ('x', 'y', 'z')]
        width, height = int(message.width), int(message.height)
        total = width * height
        stride = max(1, int(math.ceil(total / 1000.0)))
        raw = message.data
        packed = bytearray()
        for index in range(0, total, stride):
            row, column = divmod(index, width)
            base = row * message.row_step + column * message.point_step
            try:
                x, y, z = (
                    struct.unpack_from(endian + 'f', raw, base + field.offset)[0]
                    for field in xyz)
            except (struct.error, AttributeError):
                continue
            if not all(math.isfinite(value) for value in (x, y, z)) or z <= 0.10 or z > 8.0:
                continue
            # Rotate by q*v*q^-1 without allocating a ROS message per point.
            ix = qw * x + qy * z - qz * y
            iy = qw * y + qz * x - qx * z
            iz = qw * z + qx * y - qy * x
            iw = -qx * x - qy * y - qz * z
            mx = ix * qw + iw * -qx + iy * -qz - iz * -qy + tx
            my = iy * qw + iw * -qy + iz * -qx - ix * -qz + ty
            mz = iz * qw + iw * -qz + ix * -qy - iy * -qx + tz
            if not all(math.isfinite(value) and abs(value) <= 100.0 for value in (mx, my, mz)):
                continue
            packed.extend(struct.pack('<fff', mx, my, mz))
        if not packed:
            return
        with self._lock:
            self._stitched_pointcloud.extend(packed)
            self._stitched_point_count += len(packed) // 12
            max_bytes = self._stitch_limit_points * 12
            if len(self._stitched_pointcloud) > max_bytes:
                excess = len(self._stitched_pointcloud) - max_bytes
                drop = ((excess + 11) // 12) * 12
                del self._stitched_pointcloud[:drop]
            self._stitched_point_count = len(self._stitched_pointcloud) // 12
            self._last_rx['stitched_pointcloud'] = time.monotonic()

    def _on_localization(self, message, epoch=None, generation=None):
        stamp = int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)
        if ((generation is not None and generation != self._localization_generation) or
                stamp < self._localization_not_before_ns):
            return
        if epoch is not None and epoch != self._runtime.epoch:
            return
        self._observe('localization', message, epoch=epoch)
        if self._runtime.health('localization', max_age=LOCALIZATION_STALE_AFTER)['state'] != 'ready':
            return
        pose = message.pose.pose
        covariance = message.pose.covariance
        x, y = float(pose.position.x), float(pose.position.y)
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cov_x, cov_y, cov_yaw = (
            float(covariance[0]), float(covariance[7]), float(covariance[35]))
        finite = (all(math.isfinite(value) for value in (x, y, yaw, q.x, q.y, q.z, q.w, *covariance)) and
                  abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) <= 0.01 and message.header.frame_id == 'map')
        low_uncertainty = finite and 0.0 <= cov_x <= 0.25 and \
            0.0 <= cov_y <= 0.25 and 0.0 <= cov_yaw <= 0.30
        with self._lock:
            if epoch is not None and epoch != self._runtime.epoch:
                return
            self._amcl_pose = message
            self._last_rx['localization'] = time.monotonic()
        with self._localization_lock:
            if not self._localization_refresh_active:
                return
            previous = self._localization_previous_candidate
            if low_uncertainty and previous is not None:
                position_delta = math.hypot(x - previous[0], y - previous[1])
                yaw_delta = abs(math.atan2(
                    math.sin(yaw - previous[2]), math.cos(yaw - previous[2])))
                if position_delta <= 0.15 and yaw_delta <= 0.20:
                    self._localization_stable_samples += 1
                else:
                    self._localization_stable_samples = 1
            elif low_uncertainty:
                self._localization_stable_samples = 1
            else:
                self._localization_stable_samples = 0
            self._localization_previous_candidate = (x, y, yaw) if low_uncertainty else None
            self._localization_converged = self._localization_stable_samples >= 3
            if self._localization_converged:
                self._localization_reason = 'AMCL estimate is stable with bounded covariance.'
            elif low_uncertainty:
                self._localization_reason = (
                    f'AMCL estimate is becoming stable '
                    f'({self._localization_stable_samples}/3 consistent updates).')
            else:
                self._localization_reason = (
                    'AMCL is searching the saved map; pose uncertainty is still high.')

    def _localization_refresh_tick(self):
        now = time.monotonic()
        with self._localization_lock:
            if not self._localization_refresh_active:
                return
            started_at = self._localization_refresh_started_at
            converged = self._localization_converged
            interval = 3.0 if converged else 1.0
            if now - started_at > LOCALIZATION_MAX_SEARCH_SECONDS and not converged:
                self._localization_refresh_active = False
                self._localization_reason = (
                    'AMCL did not converge within 90 seconds. Check map/scan alignment '
                    'or set a known initial pose.')
                return
            if self._localization_refresh_inflight or \
                    now - self._localization_refresh_requested_at < interval:
                return
        components = self.status()['components']
        stale_inputs = [name for name in ('map', 'camera', 'scan', 'odometry')
                        if components[name]['state'] != 'ready']
        if stale_inputs:
            with self._localization_lock:
                self._localization_refresh_active = False
                self._localization_reason = (
                    'Localization refresh stopped because ' +
                    ', '.join(stale_inputs) + ' became stale.')
            return
        # A planner goal can remain active while motor output is isolated, so
        # AMCL may see no odometry motion and publish no new pose. Keep its
        # no-motion scan updates running; otherwise the last good pose expires
        # after LOCALIZATION_STALE_AFTER and revokes a valid planner session.
        if not self._localization_refresh_client.service_is_ready():
            return
        with self._localization_lock:
            self._localization_refresh_inflight = True
            self._localization_refresh_requested_at = now
        try:
            future = self._localization_refresh_client.call_async(Empty.Request())
        except Exception as error:
            with self._localization_lock:
                self._localization_refresh_inflight = False
                self._localization_reason = f'AMCL refresh request failed: {error}'
            return

        epoch = self._runtime.epoch
        generation = self._localization_generation
        def refresh_done(done_future):
            if epoch != self._runtime.epoch or generation != self._localization_generation:
                return
            with self._localization_lock:
                self._localization_refresh_inflight = False
                try:
                    done_future.result()
                except Exception as error:
                    self._localization_reason = f'AMCL refresh request failed: {error}'

        future.add_done_callback(refresh_done)

    def _component(self, name, now):
        stamp = self._last_rx.get(name)
        age = None if stamp is None else max(0.0, now - stamp)
        fresh_limit = MAP_STALE_AFTER if name == 'map' else STALE_AFTER
        if name in ('camera', 'odometry', 'scan', 'vendor_odometry'):
            health = self._runtime.health(name, acquisition=name != 'vendor_odometry')
            health['age_seconds'] = health['receipt_age_seconds']
            if name == 'odometry':
                health['timestamp_semantics'] = 'normalized receipt-derived odometry; vendor acquisition latency unverified'
            return health
        return {
            'state': 'ready' if age is not None and age <= fresh_limit else 'not_ready',
            'age_seconds': None if age is None else round(age, 2),
        }

    def _publish_readiness(self):
        """Heartbeat consumed by the existing fail-closed command gates.

        Readiness never enables a gate itself.  It is true only while the
        API's existing planner checks pass and an operator has explicitly
        opted in to planner goals.
        """
        if (self._runtime.lease_owner is not None and not self._runtime.valid_lease() and
                not self._autonomy_stop_latched):
            self.stop_autonomy()
        navigation = self.status()['components']['navigation']
        message = Bool()
        with self._lock:
            # Serialize publication with priority STOP. A status snapshot taken
            # before STOP must not publish true after the false stop heartbeat.
            message.data = bool(
                navigation['planner_ready'] and
                self.allow_nav2_goal_requests and
                not self._autonomy_stop_latched and self._runtime.valid_lease())
            self._readiness_pub.publish(message)
            with self._goal_lock:
                active_goal_id = self._goal_id if self._goal_state in ('accepted', 'executing') else None
            context = String()
            context.data = json.dumps({'ready': message.data, 'operator_lease_valid': self._runtime.valid_lease(),
                                       'map_id': self._navigation_identity, 'software_id': self.software_identity(),
                                       'session_epoch': self._runtime.epoch,
                                       'goal_active': active_goal_id is not None,
                                       'active_goal_id': active_goal_id})
            self._context_pub.publish(context)

    def software_identity(self):
        now = time.monotonic()
        if now-self._software_hash_at < 1.0 and self._software_id:
            return self._software_id
        import hashlib
        digest = hashlib.sha256()
        paths = list(Path(__file__).parent.glob('*.py')) + list(self._bridge_config.parent.glob('*.yaml')) + list(self._bridge_config.parent.glob('*.xml'))
        vendor = Path('/home/ysc/lite_cog_ros2/nav/src/dr_nav2/config/lite_nav2.yaml')
        for path in sorted(paths + [vendor]):
            digest.update(path.name.encode())
            digest.update(path.read_bytes() if path.is_file() else b'MISSING')
        self._software_id, self._software_hash_at = digest.hexdigest(), now
        return self._software_id

    def status(self):
        now = time.monotonic()
        with self._lock:
            mapping_running = (
                self._mapping_process is not None and
                self._mapping_process.poll() is None)
            navigation_running = (
                self._navigation_process is not None and
                self._navigation_process.poll() is None)
            components = {k: self._component(k, now) for k in (
                'camera', 'odometry', 'vendor_odometry', 'scan', 'map', 'pointcloud')}
            localization_age = self._last_rx.get('localization')
            localization_age = (None if localization_age is None else
                                max(0.0, now - localization_age))
            grid = self._map
            amcl_pose = self._amcl_pose
            navigation_map_name = self._navigation_map_name
            navigation_started_at = self._navigation_started_at
            map_received_at = self._last_rx.get('map', 0.0)
        # Nav2's map_server publishes a static map with transient-local QoS.
        # It is intentionally sent once, so the ordinary telemetry freshness
        # timeout must not make a successfully loaded active map go stale.
        if (navigation_running and navigation_map_name and grid is not None and
                map_received_at >= navigation_started_at):
            components['map']['state'] = 'ready'
            components['map']['source'] = 'active_static_map_server'
        with self._localization_lock:
            localization_converged = self._localization_converged
            localization_reason = self._localization_reason
            localization_active = self._localization_refresh_active
            stable_samples = self._localization_stable_samples
        localization_ready = (
            localization_converged and localization_age is not None and
            localization_age <= LOCALIZATION_STALE_AFTER)
        components['localization'] = {
            'state': 'ready' if localization_ready else 'not_ready',
            'age_seconds': None if localization_age is None else round(localization_age, 2),
            'converged': localization_converged,
            'stable_samples': stable_samples,
            'refresh_active': localization_active,
            'reason': localization_reason,
        }
        if amcl_pose is not None:
            q = amcl_pose.pose.pose.orientation
            components['localization']['pose'] = {
                'x': float(amcl_pose.pose.pose.position.x),
                'y': float(amcl_pose.pose.pose.position.y),
                'yaw': math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
                'covariance_xy': [
                    float(amcl_pose.pose.covariance[0]),
                    float(amcl_pose.pose.covariance[7])],
                'covariance_yaw': float(amcl_pose.pose.covariance[35]),
            }
        if mapping_running and self._mapping_paused:
            mapping_state = 'paused'
        elif mapping_running and components['scan']['state'] == 'ready':
            mapping_state = 'ready'
        elif mapping_running:
            mapping_state = 'starting'
        else:
            mapping_state = 'not_ready'
        components['mapping'] = {
            'state': mapping_state,
            'process_running': mapping_running,
            'paused': bool(mapping_running and self._mapping_paused),
        }
        components['stitched_pointcloud'] = self._component('stitched_pointcloud', now)
        with self._lock:
            components['stitched_pointcloud']['point_count'] = self._stitched_point_count
        action_ready = (
            self._nav_action is not None and self._nav_action.server_is_ready())
        components['navigation_stack'] = {
            'state': 'starting' if navigation_running and not action_ready
            else ('ready' if navigation_running else 'not_ready'),
            'process_running': navigation_running,
        }
        required_transforms = {
            'odom_to_base': self._tf_available('odom', 'base_link'),
            'base_to_scan': self._tf_available('base_link', 'camera_scan_frame'),
            'base_to_depth_camera': self._tf_available(
                'base_link', 'camera_depth_optical_frame'),
            'map_to_base': self._tf_available('map', 'base_link'),
        }
        missing_transforms = [name for name, available in required_transforms.items()
                              if not available]
        components['transforms'] = {
            'state': 'ready' if not missing_transforms else 'not_ready',
            'required': required_transforms,
            'missing': missing_transforms,
        }
        with self._lock:
            costmap_valid = dict(self._costmap_valid)
            costmap_stamps = {
                key: self._last_rx.get(key) for key in
                ('local_costmap', 'global_costmap')}
        costmaps = {}
        for key in ('local_costmap', 'global_costmap'):
            stamp = costmap_stamps[key]
            age = None if stamp is None else max(0.0, now - stamp)
            source = self._runtime.health(key)
            ready = (costmap_valid[key] and age is not None and age <= STALE_AFTER and
                     source['state'] == 'ready')
            reason = ('No costmap grid received' if stamp is None else
                      'Costmap grid dimensions/data are invalid' if not costmap_valid[key] else
                      'No recent costmap grid received' if age > STALE_AFTER else
                      'Costmap source timestamp is stale, future, or out of order'
                      if source['state'] != 'ready' else 'Fresh grid received')
            costmaps[key] = {
                'state': 'ready' if ready else 'not_ready',
                'age_seconds': None if age is None else round(age, 2),
                'source_age_seconds': source.get('source_age_seconds'),
                'reason': reason,
            }
        components['costmaps'] = {
            'state': 'ready' if all(value['state'] == 'ready'
                                     for value in costmaps.values()) else 'not_ready',
            **costmaps,
        }
        components['global_localization'] = {
            'state': 'ready' if navigation_running and
            self._global_localization_client.service_is_ready() and
            self._localization_refresh_client.service_is_ready() else 'not_ready',
            'method': 'AMCL global relocalization over saved-map free space',
            'refresh_ready': self._localization_refresh_client.service_is_ready(),
            'search_active': localization_active,
            'reason': localization_reason,
        }
        if self._runtime.health('localization', max_age=LOCALIZATION_STALE_AFTER)['state'] != 'ready':
            components['localization']['state'] = 'not_ready'
        localization_ready = components['localization']['state'] == 'ready'
        map_ready = components['map']['state'] == 'ready' and grid is not None
        sensor_ready = all(components[k]['state'] == 'ready' for k in ('camera', 'odometry', 'scan'))
        known_fraction = (
            sum(1 for value in grid.data if value >= 0) / len(grid.data)
            if grid is not None and grid.data else 0.0)
        map_has_observed_cells = known_fraction > 0.0
        trial_map_eligible = (self._bridge_profile == 'supervised_trial' and
                              self._navigation_map_trial_eligible)
        target_map_eligible = self._navigation_map_reviewed or trial_map_eligible
        planner_ready = (
            action_ready and localization_ready and map_ready and sensor_ready and
            map_has_observed_cells and target_map_eligible and not mapping_running and
            components['transforms']['state'] == 'ready' and
            components['costmaps']['state'] == 'ready' and
            not self._autonomy_stop_latched)
        # Readiness loss revokes planner opt-in. The operator must explicitly
        # opt in again after localization, map, TF, or costmap data recovers.
        if not planner_ready:
            self.allow_nav2_goal_requests = False
        if not map_has_observed_cells:
            nav_reason = 'The current map has no observed cells.'
        elif mapping_running:
            nav_reason = 'Stop mapping before starting localization and Nav2.'
        elif not action_ready:
            nav_reason = (
                'Nav2 action type is unavailable: ' + self._nav_action_error
                if self._nav_action is None and self._nav_action_error else
                'Nav2 NavigateToPose action server is not active.')
        elif not map_ready:
            nav_reason = 'The active map is missing or stale.'
        elif not target_map_eligible:
            nav_reason = ('Targets require a verified saved map in supervised trial mode '
                          'or a measured map review in commissioned mode.')
        elif not sensor_ready:
            missing_sensors = [name for name in ('camera', 'odometry', 'scan')
                               if components[name]['state'] != 'ready']
            nav_reason = 'Fresh sensor data required: ' + ', '.join(missing_sensors) + '.'
        elif not localization_ready:
            nav_reason = (components['localization'].get('reason') or
                          'AMCL has not established a stable pose on the active map.')
        elif components['transforms']['state'] != 'ready':
            nav_reason = 'Required map, odometry, scan, or depth-camera TF is missing.'
        elif components['costmaps']['state'] != 'ready':
            missing = [name.replace('_', ' ') + ': ' + costmaps[name]['reason']
                       for name in ('local_costmap', 'global_costmap')
                       if costmaps[name]['state'] != 'ready']
            nav_reason = 'Nav2 costmaps unavailable (' + '; '.join(missing) + ').'
        elif not self.allow_nav2_goal_requests:
            nav_reason = 'Navigation checks pass, but planner goal requests remain disabled by default.'
        else:
            nav_reason = ('Navigation checks pass. Submit a previewed goal, then arm the attended trial.'
                          if self._bridge_profile == 'supervised_trial' else
                          'Navigation checks pass. Motor output still requires separate arming.')
        if self._autonomy_stop_latched:
            nav_reason += ' Software STOP is latched; restart localization/Nav2 explicitly before another planner request.'
        with self._lock:
            observation = dict(self._observation_health)
            stages = {key: {**value, 'fresh': now-value['rx'] <= 0.5}
                      for key, value in self._stage_health.items()}
        source_age = (self.get_clock().now().nanoseconds - observation.get('stamp_ns', 0))/1e9
        observation_ready = (observation.get('healthy') is True and
                             0 <= now-observation.get('rx', 0) <= 0.5 and -0.05 <= source_age <= 1.0)
        scan_stamp = self._runtime.health('scan').get('stamp_ns')
        scan_frame = self._sensor_frames.get('scan')
        acquisition_tf = bool(scan_stamp and scan_frame and all(
            self._tf_available(target, scan_frame, scan_stamp) for target in ('map', 'odom')))
        depth_stamp = observation.get('stamp_ns')
        depth_frame = observation.get('frame_id')
        depth_tf = bool(depth_stamp and depth_frame and all(
            self._tf_available(target, depth_frame, depth_stamp) for target in ('map', 'odom')))
        observation_ready = observation_ready and acquisition_tf and depth_tf
        components['transforms']['acquisition_time_scan'] = acquisition_tf
        components['transforms']['acquisition_time_depth'] = depth_tf
        components['obstacle_observations'] = {
            'state': 'ready' if observation_ready else 'not_ready',
            'source_age_seconds': source_age if observation.get('stamp_ns') else None,
            'reason': observation.get('reason', 'No validated depth observations'),
            'costmap_buffer_acceptance': 'unverified; costmap publication is separate',
        }
        components['command_stages'] = {'state': 'observed' if stages else 'unknown', 'stages': stages}
        lease_valid = self._runtime.valid_lease()
        base_planner_ready = planner_ready
        planner_ready = planner_ready and observation_ready
        if not planner_ready:
            self.allow_nav2_goal_requests = False
        if not observation_ready:
            detail = (str(observation.get('reason') or 'no valid depth cloud') if observation.get('healthy') is not True else
                      'depth acquisition timestamp or map/odom TF is not current')
            observation_reason = 'Depth obstacle observations unavailable: ' + detail + '.'
            nav_reason = (observation_reason if base_planner_ready else
                          nav_reason + ' ' + observation_reason)
        if not lease_valid:
            nav_reason += ' No current operator lease.'
        components['navigation'] = {
            'state': 'ready' if planner_ready and self.allow_nav2_goal_requests and lease_valid else 'not_ready',
            'planner_ready': planner_ready,
            'goal_requests_opt_in_available': planner_ready,
            'goal_requests_enabled': self.allow_nav2_goal_requests,
            'action_server_ready': action_ready,
            'active_map_name': navigation_map_name,
            'active_map_id': self._navigation_identity,
            'active_map_reviewed': self._navigation_map_reviewed,
            'active_map_target_eligible': target_map_eligible,
            'target_mode': ('supervised_trial' if trial_map_eligible and not self._navigation_map_reviewed
                            else 'reviewed' if self._navigation_map_reviewed else 'localization_only'),
            'session_epoch': self._runtime.epoch,
            'goal_clearance_radius': self._goal_clearance_radius,
            'operator_lease_valid': lease_valid,
            'feedback': self._goal_feedback,
            'path_preview': self._path_preview,
            'map_known_fraction': round(known_fraction, 3),
            **self._goal_status(),
            'stop_latched': self._autonomy_stop_latched,
            'reason': nav_reason,
        }
        components['safety_stop'] = {
            'state': 'latched' if self._autonomy_stop_latched else 'not_latched',
            **self._stop_cleanup_status(),
            'physical_estop_verified': False,
            'reason': 'Software stop only; the vendor motor path has no verified independent stop/interlock.',
        }
        supervisor = stages.get('supervisor', {})
        command_path = (self._bridge_profile == 'command' and supervisor.get('fresh') and supervisor.get('enabled') is True and
                        all(stages.get(key, {}).get('fresh') and stages[key].get('enabled') is True
                            for key in ('gate', 'arbiter', 'relay')))
        components['motion_output'] = {
            'state': ('armed' if command_path else
                      'trial_armed' if self._bridge_profile == 'supervised_trial' and
                      stages.get('supervised_trial', {}).get('fresh') and
                      stages['supervised_trial'].get('enabled') else 'not_ready'),
            'bridge_profile': self._bridge_profile,
            'commissioning_id': supervisor.get('commissioning_id'),
            'reason': (stages.get('supervised_trial', {}).get('reason', 'Trial relay not ready')
                       if self._bridge_profile == 'supervised_trial' else
                       supervisor.get('reason', 'No fresh commissioned supervisor state')),
            'trial_remaining_seconds': stages.get('supervised_trial', {}).get('remaining_seconds', 0),
        }
        map_info = None
        if grid is not None:
            map_info = {
                'frame': grid.header.frame_id,
                'width': grid.info.width,
                'height': grid.info.height,
                'resolution': grid.info.resolution,
            }
        return {
            'ok': True,
            'api_contract_version': 2,
            'capabilities': ['priority_software_stop', 'tracked_goal_lifecycle', 'map_identity', 'path_preview', 'operator_lease', 'supervised_goal_trial'],
            'control_mode': 'autonomy' if command_path else 'unknown',
            'control_mode_reason': supervisor.get('reason', 'No authoritative vendor ownership state is connected.'),
            'api_auth_required': True,
            'api_paired': self.api_token is not None,
            'components': components,
            'map': map_info,
            'server_monotonic_seconds': round(now, 2),
        }

    def _commissioning_record(self):
        path = Path.home()/'.config/lite3_autonomy/commissioning.json'
        try:
            record = json.loads(path.read_text())
        except (OSError, ValueError) as error:
            raise ApiError(409, 'Commissioning record unavailable: ' + str(error))
        if (not isinstance(record, dict) or record.get('format') != 'lite3_motion_commissioning/v1'
                or record.get('software_id') != self.software_identity()):
            raise ApiError(409, 'Commissioning record does not match this software/configuration')
        required = ('vendor_exclusivity_verified', 'independent_stop_verified', 'downstream_timeout_verified',
                    'footprint_measured', 'visibility_reviewed')
        if any(record.get(name) is not True for name in required) or not record.get('commissioning_id'):
            raise ApiError(409, 'Physical commissioning evidence is incomplete')
        footprint = record.get('footprint')
        if (not isinstance(footprint, list) or len(footprint) < 3 or
                any(not isinstance(point, list) or len(point) != 2 or
                    any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in point) for point in footprint)):
            raise ApiError(409, 'Measured footprint polygon is missing or invalid')
        return record

    def select_bridge_profile(self, profile):
        if profile not in ('telemetry', 'command', 'supervised_trial'):
            raise ApiError(400, 'Bridge profile must be telemetry, supervised_trial, or command')
        with self._lock:
            if any(process is not None and process.poll() is None
                   for process in (self._mapping_process, self._navigation_process)):
                raise ApiError(409, 'Stop project mapping/navigation before switching bridge profile')
            if self._bridge_external:
                raise ApiError(409, 'An external bridge is attached; its configuration will not be changed')
            if not self._autonomy_stop_latched:
                raise ApiError(409, 'Latch STOP before selecting a bridge profile')
        if profile == 'command':
            self._commissioning_record()
        self.stop_bridge()
        filename = 'mapping_ros1_bridge.yaml' if profile == 'telemetry' else 'vendor_cmd_vel_bridge.yaml'
        self._bridge_config = self._bridge_config.parent / filename
        self._bridge_profile = profile
        self._new_pipeline_session('Bridge profile explicitly selected: ' + profile)
        return {'ok': True, 'profile': profile, 'message': 'Profile selected; bridge/output remain stopped until explicit startup.'}

    def arm_motion(self):
        record = self._commissioning_record()
        if self._bridge_profile != 'command' or self._navigation_identity not in record.get('allowed_map_ids', []):
            raise ApiError(409, 'Current map/bridge profile is not commissioned for motion')
        with self._goal_lock:
            if self._goal_state not in ('accepted', 'executing'):
                raise ApiError(409, 'A tracked, accepted goal is required before arming')
        if self._autonomy_stop_latched or not self._runtime.valid_lease():
            raise ApiError(409, 'STOP/lease inhibits motion arming')
        client = self._autonomy_disable_clients['supervisor']
        if not client.service_is_ready():
            raise ApiError(503, 'Ownership supervisor service unavailable')
        future = client.call_async(SetBool.Request(data=True))
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(1.5):
            self.stop_autonomy()
            raise ApiError(504, 'Supervisor arm response timed out; STOP latched')
        result = future.result()
        if not result.success:
            raise ApiError(409, result.message)
        return {'ok': True, 'message': result.message, 'arming_requested': True, 'armed_confirmed': False}

    def arm_supervised_trial(self, auto_mode_confirmed, app_stop_ready, area_clear):
        if self._bridge_profile != 'supervised_trial':
            raise ApiError(409, 'Select the supervised trial bridge profile before starting Nav2')
        if not (auto_mode_confirmed is True and app_stop_ready is True and area_clear is True):
            raise ApiError(400, 'Standing Auto Mode, reachable app STOP, and clear area must be confirmed')
        with self._goal_lock:
            goal_active = self._goal_state in ('accepted', 'executing') and self._goal_id in self._goal_requests
        if not goal_active or self._autonomy_stop_latched or not self._runtime.valid_lease():
            raise ApiError(409, 'A tracked goal, current browser lease, and clear project STOP are required')
        if self.status()['components']['navigation']['state'] != 'ready':
            raise ApiError(409, 'Localization, costmaps, sensors, or planner readiness was lost')
        client = self._autonomy_disable_clients['supervised_trial']
        if not client.service_is_ready():
            raise ApiError(503, 'Supervised trial relay is unavailable')
        future = client.call_async(SetBool.Request(data=True))
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(1.5):
            self.stop_autonomy()
            raise ApiError(504, 'Trial arm response timed out; STOP latched')
        result = future.result()
        if result is None or not result.success:
            raise ApiError(409, result.message if result is not None else 'Trial arm failed')
        return {'ok': True, 'message': result.message, 'arming_requested': True}

    def _assert_active_map(self, map_name, map_id, *, require_reviewed=True):
        if map_name != self._navigation_map_name or not map_id or map_id != self._navigation_identity:
            raise ApiError(409, 'Map name/checksum does not match the active navigation session.')
        try:
            bundle = read_bundle(self.maps_dir / map_name,
                                 require_reviewed=require_reviewed,
                                 allow_legacy=not require_reviewed)
        except (OSError, ValueError, KeyError) as error:
            raise ApiError(409, 'Map integrity/review no longer valid: ' + str(error))
        if bundle['map_id'] != map_id:
            self._path_preview = None
            self.allow_nav2_goal_requests = False
            raise ApiError(409, 'Map changed; restart localization and select a new target.')
        return bundle

    def _assert_target_map(self, map_name, map_id):
        if self._bridge_profile != 'supervised_trial':
            return self._assert_active_map(map_name, map_id)
        # The attended, bounded trial can use an unreviewed scan only when it
        # has an intact integrity manifest. A legacy or synthetic map remains
        # localization-only; map cells and the complete path are checked later.
        bundle = self._assert_active_map(map_name, map_id, require_reviewed=False)
        if bundle['manifest'].get('legacy_unverified') or bundle['manifest'].get('synthetic') is True:
            raise ApiError(409, 'Supervised targets need a verified, non-synthetic map bundle.')
        return bundle

    def acquire_operator_lease(self, owner):
        with self._lock:
            if self._autonomy_stop_latched or self._stop_cleanup_pending:
                raise ApiError(409, 'Start localization explicitly before acquiring a control lease.')
            try:
                token = self._runtime.acquire(owner)
            except ValueError as error:
                raise ApiError(409, str(error))
            return {'ok': True, 'lease_token': token, 'session_epoch': self._runtime.epoch,
                    'expires_in_seconds': self._runtime.lease_seconds}

    def renew_operator_lease(self, owner, token, epoch):
        try:
            self._runtime.renew(owner, token, epoch)
        except ValueError as error:
            raise ApiError(409, str(error))
        return {'ok': True, 'session_epoch': self._runtime.epoch,
                'expires_in_seconds': self._runtime.lease_seconds}

    def finish_scan(self, name, timeout=5.0):
        if not isinstance(name, str) or not MAP_NAME.fullmatch(name):
            raise ApiError(400, 'Choose a unique map name')
        # Wait for a new map from this generation; do not save a cache and do
        # not freeze SLAM with SIGSTOP. The operator must hold the robot still.
        with self._map_condition:
            if self._mapping_process is None or self._mapping_process.poll() is not None or self._mapping_paused:
                raise ApiError(409, 'Finish scan requires a running, unpaused mapping session.')
            if self._stationary_since is None or time.monotonic()-self._stationary_since < 2.0:
                raise ApiError(409, 'Hold still until fresh odometry has been stationary for two seconds')
            epoch, sequence = self._runtime.epoch, self._map_sequence
            ready = self._map_condition.wait_for(
                lambda: self._runtime.epoch != epoch or self._map_sequence > sequence,
                timeout=timeout)
            if not ready or self._runtime.epoch != epoch or self._map is None:
                raise ApiError(409, 'No final map arrived from the current scan session; nothing saved.')
            frozen = self._map
        payload = self.save_latest_map(name, message=frozen)
        self.stop_mapping()
        return {**payload, 'message': 'Final map snapshot saved; mapping stopped.',
                'frozen_session_epoch': epoch}

    def review_map(self, name, review):
        if not isinstance(name, str) or not MAP_NAME.fullmatch(name):
            raise ApiError(400, 'Select a map bundle name.')
        if self._navigation_map_name == name:
            raise ApiError(409, 'Stop navigation before changing the active map review.')
        try:
            bundle = review_bundle(self.maps_dir / name, review)
        except (OSError, ValueError, KeyError) as error:
            raise ApiError(400, str(error))
        return {'ok': True, 'map_id': bundle['map_id'], 'approval_state': bundle['approval_state']}

    def preview_nav_path(self, x, y, yaw=0.0, map_name=None, map_id=None):
        self._assert_target_map(map_name, map_id)
        if not all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in (x,y,yaw)):
            raise ApiError(400, 'Path target must contain finite coordinates/heading.')
        if self._path_action is None or not self._path_action.server_is_ready():
            raise ApiError(503, 'Nav2 ComputePathToPose is unavailable.')
        components = self.status()['components']
        if not components['navigation']['planner_ready']:
            raise ApiError(409, components['navigation']['reason'])
        with self._goal_lock:
            if self._goal_requests:
                raise ApiError(409, 'Wait for the current goal to terminate before previewing another route')
        with self._lock:
            grid, pose = self._map, self._amcl_pose
            epoch = self._runtime.epoch
            if self._bridge_profile == 'supervised_trial':
                if pose is None or math.hypot(float(x)-pose.pose.pose.position.x,
                                               float(y)-pose.pose.pose.position.y) > 0.50:
                    raise ApiError(409, 'Supervised trial targets must be within 0.50 m of the localized robot.')
            self._validate_goal_cell(grid, x, y)
            self._validate_live_clearance(x, y)
            if self._path_request_id is not None:
                raise ApiError(409, 'A path request is still unresolved.')
            request_id = secrets.token_hex(16)
            self._path_request_id = request_id
            self._path_preview = None
        request = ComputePathToPose.Goal()
        destination = request.pose if hasattr(request, 'pose') else request.goal
        destination.header.frame_id = 'map'
        destination.header.stamp = self.get_clock().now().to_msg()
        destination.pose.position.x, destination.pose.position.y = float(x), float(y)
        destination.pose.orientation.z, destination.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        request.planner_id = 'GridBased'
        completed = threading.Event()
        record = {'expired': False, 'error': None, 'handle': None, 'path': None}

        def cancel_late(handle):
            try:
                handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().warning(f'Path cancellation unconfirmed: {error}')

        def terminal(done):
            try:
                wrapped = done.result()
                if wrapped.status != 4:
                    raise ValueError('Path planner did not succeed')
                path = wrapped.result.path
                if path.header.frame_id != 'map' or not 1 <= len(path.poses) <= 5000:
                    raise ValueError('Path is empty, oversized, or not in map frame')
                points = [[item.pose.position.x, item.pose.position.y] for item in path.poses]
                if any(not all(math.isfinite(v) for v in point) for point in points):
                    raise ValueError('Non-finite path point')
                if math.hypot(points[-1][0]-x, points[-1][1]-y) > 0.10:
                    raise ValueError('Path does not reach the selected target')
                start = pose.pose.pose.position
                if math.hypot(points[0][0]-start.x, points[0][1]-start.y) > 0.25:
                    raise ValueError('Path does not start near the localized robot')
                checks = 0
                previous = points[0]
                for point in points:
                    intervals = max(1, math.ceil(math.hypot(point[0]-previous[0], point[1]-previous[1]) / (grid.info.resolution/2)))
                    checks += intervals
                    if checks > 100000:
                        raise ValueError('Path validation budget exceeded')
                    for index in range(intervals+1):
                        self._validate_goal_cell(grid, previous[0]+(point[0]-previous[0])*index/intervals,
                                                 previous[1]+(point[1]-previous[1])*index/intervals)
                    previous = point
                with self._lock:
                    if record['expired'] or epoch != self._runtime.epoch or self._path_request_id != request_id:
                        raise ValueError('Path session expired or changed')
                    self._path_preview = {'preview_id': request_id, 'map_id': map_id, 'session_epoch': epoch,
                                          'goal': [x,y,yaw], 'start': [start.x,start.y], 'points': points,
                                          'created_monotonic': time.monotonic()}
                    record['path'] = self._path_preview
            except Exception as error:
                record['error'] = str(error)
            finally:
                with self._lock:
                    if self._path_request_id == request_id:
                        self._path_request_id = None
                completed.set()

        def accepted(done):
            try:
                handle = done.result()
                record['handle'] = handle
                if not handle.accepted:
                    raise ValueError('Path request rejected')
                handle.get_result_async().add_done_callback(terminal)
                if record['expired'] or epoch != self._runtime.epoch:
                    cancel_late(handle)
            except Exception as error:
                record['error'] = str(error)
                with self._lock:
                    if self._path_request_id == request_id:
                        self._path_request_id = None
                completed.set()
        try:
            self._path_action.send_goal_async(request).add_done_callback(accepted)
        except Exception as error:
            # Keep unresolved identity on transport uncertainty; STOP/restart
            # clears this non-motion planner request.
            raise ApiError(503, 'Path transport failed: ' + str(error))
        if not completed.wait(5.0):
            record['expired'] = True
            if record['handle'] and record['handle'].accepted:
                cancel_late(record['handle'])
            raise ApiError(504, 'Path preview timed out; late acceptance will be canceled.')
        if record['error']:
            raise ApiError(409, record['error'])
        return {'ok': True, 'path_preview': record['path'], 'motion_command_sent': False}

    def _validate_preview(self, x, y, yaw, preview_id):
        with self._lock:
            preview = self._path_preview
            if (not preview or preview['preview_id'] != preview_id or
                    preview['map_id'] != self._navigation_identity or preview['session_epoch'] != self._runtime.epoch or
                    time.monotonic()-preview['created_monotonic'] > 10.0 or
                    any(abs(a-b)>1e-6 for a,b in zip(preview['goal'], (x,y,yaw)))):
                raise ApiError(409, 'Preview a fresh route for this target/map/session first.')
            position = self._amcl_pose.pose.pose.position
            if math.hypot(position.x-preview['start'][0],position.y-preview['start'][1]) > 0.10:
                self._path_preview = None
                raise ApiError(409, 'Robot pose changed; preview a new route.')

    def _on_goal_feedback(self, goal_id, message):
        feedback = message.feedback
        values = (feedback.current_pose.pose.position.x, feedback.current_pose.pose.position.y,
                  feedback.distance_remaining)
        if not all(math.isfinite(v) for v in values) or feedback.distance_remaining < 0:
            return
        with self._goal_lock:
            if goal_id != self._goal_id or goal_id not in self._goal_requests:
                return
            if self._goal_requests[goal_id]['abandoned']:
                return
            self._goal_feedback = {'goal_id': goal_id, 'x': values[0], 'y': values[1],
                                   'distance_remaining': values[2],
                                   'navigation_seconds': feedback.navigation_time.sec + feedback.navigation_time.nanosec/1e9,
                                   'recoveries': feedback.number_of_recoveries,
                                   'received_monotonic': time.monotonic()}
            self._goal_state = 'executing'

    def submit_nav_goal(self, x, y, yaw=0.0, map_name=None, map_id=None, preview_id=None):
        if not self.allow_nav2_goal_requests:
            raise ApiError(503, 'Nav2 goal requests are disabled; API default is planner-off.')
        if self._autonomy_stop_latched:
            raise ApiError(409, 'Autonomy is stop-latched. Explicitly restart localization/Nav2 to re-arm planner requests.')
        if not isinstance(map_name, str) or map_name != self._navigation_map_name:
            raise ApiError(409, 'Load and click the same saved robot map that localization/Nav2 is using.')
        self._assert_target_map(map_name, map_id)
        if not self._runtime.valid_lease():
            raise ApiError(409, 'A fresh operator lease is required.')
        values = (x, y, yaw)
        if not all(not isinstance(value, bool) and isinstance(value, (int, float)) and
                   math.isfinite(value) for value in values):
            raise ApiError(400, 'Goal x, y, and yaw must be finite numbers.')
        status = self.status()['components']
        if status['navigation']['state'] != 'ready':
            raise ApiError(503, status['navigation']['reason'])
        with self._lock:
            grid = self._map
            pose = self._amcl_pose
        if grid is None or pose is None:
            raise ApiError(503, 'A fresh map and AMCL pose are required.')
        if self._bridge_profile == 'supervised_trial':
            position = pose.pose.pose.position
            if math.hypot(float(x)-position.x, float(y)-position.y) > 0.50:
                raise ApiError(409, 'Supervised trial goals must be within 0.50 m of the current verified pose')
        self._validate_goal_cell(grid, float(x), float(y))
        self._validate_live_clearance(float(x), float(y))
        self._validate_preview(x, y, yaw, preview_id)
        if self._nav_action is None:
            raise ApiError(503, 'Nav2 NavigateToPose action type is unavailable.')
        if not self._nav_action.server_is_ready():
            raise ApiError(503, 'Nav2 goal action server is not available.')

        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = float(x)
        request.pose.pose.position.y = float(y)
        request.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        request.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        with self._lock:
            # Recheck after slow validation; STOP bypasses the action lock.
            if self._autonomy_stop_latched or not self.allow_nav2_goal_requests:
                raise ApiError(409, 'Software STOP or readiness loss interrupted goal submission.')
            with self._goal_lock:
                if self._goal_requests:
                    raise ApiError(409, 'A Nav2 request is unresolved; wait for its terminal result before another goal.')
                goal_id = secrets.token_hex(16)
                record = {
                    'id': goal_id, 'stop_generation': self._stop_generation,
                    'handle': None, 'abandoned': False, 'cancel_sent': False,
                    'cancel_acknowledged': False, 'error': None,
                    'accepted': threading.Event(),
                }
                self._goal_requests[goal_id] = record
                self._goal_id = goal_id
                self._goal_handle = None
                self._goal_state = 'requesting'
                self._goal_detail = f'Planner goal requested at ({x:.2f}, {y:.2f}) m.'
        try:
            future = self._nav_action.send_goal_async(request, feedback_callback=lambda msg: self._on_goal_feedback(goal_id, msg))
            with self._goal_lock:
                record['acceptance_future'] = future
            future.add_done_callback(
                lambda done: self._on_goal_acceptance(goal_id, done))
        except Exception as error:
            # A transport exception does not prove the server never received
            # the request. Keep it unresolved and require intervention.
            with self._goal_lock:
                record['abandoned'] = True
                record['error'] = str(error)
                self._goal_state = 'unresolved'
                self._goal_detail = 'Goal transport failed; server execution is unknown. Use STOP.'
            raise ApiError(503, self._goal_detail)
        if not record['accepted'].wait(3.0):
            with self._goal_lock:
                record['abandoned'] = True
                if goal_id in self._goal_requests:
                    self._goal_state = 'acceptance_pending'
                    self._goal_detail = 'Acceptance timed out; any late accepted goal will be canceled.'
            # Acceptance can race the deadline. Check the handle once more.
            self._request_goal_cancel(goal_id)
            raise ApiError(504, 'Goal acceptance timed out; request remains tracked and late acceptance will be canceled.')
        with self._goal_lock:
            error = record['error']
            rejected = record.get('rejected', False)
            abandoned = record['abandoned']
            detail = self._goal_detail
        if error:
            raise ApiError(503, error)
        if rejected:
            raise ApiError(409, 'Nav2 rejected the goal.')
        if abandoned:
            raise ApiError(409, 'Goal was invalidated; cancellation requested. Wait for its terminal result.')
        return {'ok': True, 'message': detail, 'goal_id': goal_id, 'planner_only': self._bridge_profile != 'command'}

    def _goal_status(self):
        with self._goal_lock:
            return {
                'active_goal_id': self._goal_id,
                'active_goal_state': self._goal_state,
                'active_goal_detail': self._goal_detail,
                'unresolved_goal_count': len(self._goal_requests),
            }

    def _on_goal_acceptance(self, goal_id, future):
        with self._goal_lock:
            record = self._goal_requests.get(goal_id)
        if record is None:
            return
        try:
            handle = future.result()
            with self._lock:
                invalidated = (self._autonomy_stop_latched or
                               record['stop_generation'] != self._stop_generation)
                with self._goal_lock:
                    record['abandoned'] |= invalidated
                    if not handle.accepted:
                        record['rejected'] = True
                        self._goal_requests.pop(goal_id, None)
                        if self._goal_id == goal_id:
                            self._goal_state = 'rejected'
                            self._goal_detail = 'Nav2 rejected the goal.'
                        return
                    record['handle'] = handle
                    if self._goal_id == goal_id:
                        self._goal_handle = handle
                        self._goal_state = 'accepted'
                        self._goal_detail = 'Nav2 accepted the planner goal.'
            # Observe the result before cancellation, including immediate
            # cancellation/result completion from an action server.
            result_future = handle.get_result_async()
            with self._goal_lock:
                record['result_future'] = result_future
            result_future.add_done_callback(
                lambda done: self._on_goal_result(goal_id, done))
            if record['abandoned']:
                self._request_goal_cancel(goal_id)
        except Exception as error:
            with self._goal_lock:
                record['error'] = f'Nav2 action tracking failed: {error}'
                record['abandoned'] = True
                if self._goal_id == goal_id:
                    self._goal_state = 'unresolved'
                    self._goal_detail = record['error']
            self._request_goal_cancel(goal_id)
        finally:
            record['accepted'].set()

    def _request_goal_cancel(self, goal_id):
        with self._goal_lock:
            record = self._goal_requests.get(goal_id)
            if record is None:
                return
            record['abandoned'] = True
            handle = record['handle']
            if handle is None or record['cancel_sent']:
                return
            record['cancel_sent'] = True
            if self._goal_id == goal_id:
                self._goal_state = 'unresolved' if record['error'] else 'canceling'
                self._goal_detail = (
                    (record['error'] + ' ') if record['error'] else '') + (
                    'Cancellation requested; terminal result is still pending.')
        try:
            future = handle.cancel_goal_async()
            future.add_done_callback(
                lambda done: self._on_goal_cancel(goal_id, done))
        except Exception as error:
            self._goal_cancel_failed(goal_id, str(error))

    def _goal_cancel_failed(self, goal_id, reason):
        with self._goal_lock:
            record = self._goal_requests.get(goal_id)
            if record is None:
                return
            record['cancel_sent'] = False  # An explicit STOP/cancel may retry.
            if self._goal_id == goal_id:
                self._goal_state = 'cancel_unconfirmed'
                self._goal_detail = 'Cancellation unconfirmed; terminal result pending: ' + reason

    def _on_goal_cancel(self, goal_id, future):
        try:
            response = future.result()
            with self._goal_lock:
                record = self._goal_requests.get(goal_id)
                if record is None:
                    return
                expected = bytes(record['handle'].goal_id.uuid)
                acknowledged = any(bytes(item.goal_id.uuid) == expected
                                   for item in response.goals_canceling)
                if acknowledged:
                    record['cancel_acknowledged'] = True
                    if self._goal_id == goal_id:
                        self._goal_state = 'unresolved' if record['error'] else 'canceling'
                        self._goal_detail = (
                            (record['error'] + ' ') if record['error'] else '') + (
                            'Nav2 acknowledged cancellation; terminal result pending.')
            if not acknowledged:
                self._goal_cancel_failed(goal_id, 'No matching goal ID in cancellation response.')
        except Exception as error:
            self._goal_cancel_failed(goal_id, str(error))

    def set_nav_goal_requests(self, enabled, map_name=None, map_id=None):
        if not isinstance(enabled, bool):
            raise ApiError(400, 'enabled must be a boolean.')
        if enabled:
            self._assert_target_map(map_name, map_id)
            if not self._runtime.valid_lease():
                raise ApiError(409, 'Acquire the operator lease before planner opt-in.')
            components = self.status()['components']
            if self._autonomy_stop_latched:
                raise ApiError(409, 'Software STOP is latched. Restart localization/Nav2 before opting in.')
            if not isinstance(map_name, str) or map_name != self._navigation_map_name:
                raise ApiError(409, 'Load the same saved robot map used by Localization/Nav2 before opting in.')
            if not components['navigation']['planner_ready']:
                raise ApiError(503, components['navigation']['reason'])
        with self._lock:
            if enabled and (self._autonomy_stop_latched or self._stop_cleanup_pending):
                raise ApiError(409, 'Software STOP was requested; planner opt-in was refused.')
            self.allow_nav2_goal_requests = enabled
        return {
            'ok': True,
            'goal_requests_enabled': enabled,
            'message': (
                'Planner goal requests enabled; velocity output remains isolated.'
                if enabled else 'Planner goal requests disabled.'),
        }

    def set_initial_pose(self, x, y, yaw=0.0):
        with self._goal_lock:
            if self._goal_requests:
                raise ApiError(409, 'Wait for the active goal to terminate before changing the initial pose.')
        values = (x, y, yaw)
        if not all(not isinstance(value, bool) and isinstance(value, (int, float)) and
                   math.isfinite(value) for value in values):
            raise ApiError(400, 'Initial pose x, y, and yaw must be finite numbers.')
        components = self.status()['components']
        if not components['navigation_stack']['process_running']:
            raise ApiError(409, 'Start localization/Nav2 before setting its AMCL initial pose.')
        if components['map']['state'] != 'ready':
            raise ApiError(409, 'Wait for the selected robot map to arrive before setting the initial pose.')
        if self._initial_pose_pub.get_subscription_count() < 1:
            raise ApiError(409, 'AMCL is not subscribed to /initialpose yet. Wait for localization to start, then retry the clicked robot pose.')
        with self._lock:
            grid = self._map
        if grid is None:
            raise ApiError(409, 'A current map is required before setting the robot start pose.')
        self._validate_goal_cell(grid, float(x), float(y))
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        message.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        # A hand-picked location is a useful local prior: 20 cm position and
        # about 15 degrees heading standard deviation. AMCL must still publish
        # fresh, consistent scan-supported estimates before goals are enabled.
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.07
        with self._lock:
            self._path_preview = None
            self._localization_generation += 1
            self._localization_not_before_ns = self.get_clock().now().nanoseconds
            self._amcl_pose = None
            self._last_rx['localization'] = None
        self._request_session_rebind()
        with self._localization_lock:
            self._localization_refresh_active = True
            self._localization_refresh_inflight = False
            self._localization_refresh_started_at = time.monotonic()
            self._localization_refresh_requested_at = 0.0
            self._localization_converged = False
            self._localization_stable_samples = 0
            self._localization_previous_candidate = None
            self._localization_reason = 'Known initial pose sent; checking it against live scans.'
        self._initial_pose_pub.publish(message)
        return {'ok': True, 'message': f'Robot position estimate set to ({x:.2f}, {y:.2f}) m. AMCL is checking live scans before navigation is enabled.',
                'planner_only': True}

    def request_global_localization(self):
        status = self.status()['components']
        if not status['navigation_stack']['process_running']:
            raise ApiError(409, 'Start localization/Nav2 with the saved map first.')
        if status['mapping']['process_running']:
            raise ApiError(409, 'Stop mapping before global localization.')
        if not all(status[name]['state'] == 'ready' for name in (
                'map', 'camera', 'scan', 'odometry')):
            raise ApiError(409, 'Fresh map, camera, scan, and odometry are required for global localization.')
        with self._goal_lock:
            if self._goal_requests:
                raise ApiError(409, 'Cancel the active Nav2 goal before relocalizing.')
        if not self._global_localization_client.service_is_ready():
            raise ApiError(503, 'AMCL global-localization service is not ready.')
        if not self._localization_refresh_client.service_is_ready():
            raise ApiError(503, 'AMCL no-motion update service is not ready; localization was not started.')
        with self._lock:
            self._path_preview = None
            self._localization_generation += 1
            self._localization_not_before_ns = self.get_clock().now().nanoseconds
            self._amcl_pose = None
            self._last_rx['localization'] = None
        self._request_session_rebind()
        with self._localization_lock:
            self._localization_refresh_active = False
            self._localization_refresh_inflight = False
            self._localization_refresh_started_at = 0.0
            self._localization_refresh_requested_at = 0.0
            self._localization_converged = False
            self._localization_stable_samples = 0
            self._localization_previous_candidate = None
            self._localization_reason = 'AMCL global localization requested; waiting for pose updates.'
        future = self._global_localization_client.call_async(Empty.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(3.0):
            self.stop_autonomy()
            raise ApiError(504, 'AMCL global-localization request timed out.')
        try:
            future.result()
        except Exception as error:
            self.stop_autonomy()
            raise ApiError(503, f'AMCL global localization failed: {error}')
        if self._autonomy_stop_latched:
            raise ApiError(409, 'STOP interrupted global localization')
        with self._localization_lock:
            self._localization_refresh_started_at = time.monotonic()
            self._localization_refresh_requested_at = 0.0
            self._localization_refresh_active = True
        return {
            'ok': True,
            'message': 'AMCL is searching the saved map with no-motion scan updates. Wait for a stable estimate and verify the marker against the robot before selecting a goal.',
            'motion_command_sent': False,
        }

    def _validate_goal_cell(self, grid, x, y, *, use_operating_region=True):
        origin = grid.info.origin
        q = origin.orientation
        resolution = float(grid.info.resolution)
        width, height = int(grid.info.width), int(grid.info.height)
        geometry = (x, y, resolution, origin.position.x, origin.position.y, q.x, q.y, q.z, q.w)
        if (not all(math.isfinite(value) for value in geometry) or resolution <= 0 or
                width <= 0 or height <= 0 or len(grid.data) != width * height or
                abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01 or
                abs(q.x) > 1e-6 or abs(q.y) > 1e-6):
            raise ApiError(400, 'Current map has invalid planar geometry or dimensions.')
        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        dx, dy = x - origin.position.x, y - origin.position.y
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        cx, cy = math.floor(local_x / resolution), math.floor(local_y / resolution)
        if not (0 <= cx < width and 0 <= cy < height):
            raise ApiError(400, 'Goal lies outside the current map.')
        if use_operating_region and self._operating_region is not None:
            x0, y0, x1, y1 = self._operating_region
            if not x0 <= cx < x1 or not y0 <= cy < y1:
                raise ApiError(400, 'Target is outside the reviewed operating region.')
        clearance = self._goal_clearance_radius
        radius = max(1, int(math.ceil(clearance / resolution)))
        if cx < radius or cy < radius or cx >= width - radius or cy >= height - radius:
            raise ApiError(400, f'Goal lacks a full {clearance:.2f} m clearance inside the map boundary.')
        for row in range(max(0, cy - radius), min(height, cy + radius + 1)):
            for col in range(max(0, cx - radius), min(width, cx + radius + 1)):
                if (col - cx) ** 2 + (row - cy) ** 2 > radius ** 2:
                    continue
                value = int(grid.data[row * width + col])
                if value < 0 or value > 15:
                    raise ApiError(400, f'Goal is not in a known-free map region with {clearance:.2f} m clearance.')

    def _validate_live_clearance(self, x, y):
        global_grid = self._costmaps.get('global_costmap')
        if global_grid is None or global_grid.header.frame_id != 'map':
            raise ApiError(409, 'A current map-frame global costmap is required')
        self._validate_goal_cell(global_grid, x, y, use_operating_region=False)
        local = self._costmaps.get('local_costmap')
        if local is None:
            raise ApiError(409, 'A current local costmap is required')
        lx, ly = x, y
        if local.header.frame_id != 'map':
            try:
                transform = self._tf_buffer.lookup_transform(local.header.frame_id, 'map', Time()).transform
                q, t = transform.rotation, transform.translation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                lx, ly = t.x+math.cos(yaw)*x-math.sin(yaw)*y, t.y+math.sin(yaw)*x+math.cos(yaw)*y
            except (TransformException, ValueError) as error:
                raise ApiError(409, 'Local costmap transform unavailable: '+str(error))
        origin = local.info.origin.position
        inside = (origin.x <= lx < origin.x+local.info.width*local.info.resolution and
                  origin.y <= ly < origin.y+local.info.height*local.info.resolution)
        if inside:
            self._validate_goal_cell(local, lx, ly, use_operating_region=False)

    def _on_goal_result(self, goal_id, future):
        with self._goal_lock:
            if goal_id not in self._goal_requests:
                return
        try:
            wrapped_result = future.result()
            states = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}
            state = states.get(wrapped_result.status)
            if state is None:
                raise ValueError(f'Non-terminal action result status {wrapped_result.status}')
            detail = f'Nav2 goal {state}.'
        except Exception as error:
            # Losing the result transport does not establish that execution
            # ended. Keep the request/handle and inhibit replacement goals.
            with self._goal_lock:
                if goal_id in self._goal_requests and self._goal_id == goal_id:
                    self._goal_requests[goal_id]['error'] = str(error)
                    self._goal_state = 'unresolved'
                    self._goal_detail = f'Nav2 terminal result unavailable: {error}'
            return
        with self._goal_lock:
            if self._goal_requests.pop(goal_id, None) is None:
                return
            if self._goal_id == goal_id:
                self._goal_state = state
                self._goal_detail = detail
                self._goal_handle = None
        self._disable_goal_outputs()

    def _disable_goal_outputs(self):
        self._path_preview = None
        for client in self._autonomy_disable_clients.values():
            try:
                if client.service_is_ready():
                    client.call_async(SetBool.Request(data=False))
            except Exception as error:
                self.get_logger().error(f'Output disable unconfirmed: {error}')

    def cancel_nav_goal(self):
        self._disable_goal_outputs()
        with self._goal_lock:
            goal_ids = list(self._goal_requests)
            if not goal_ids:
                raise ApiError(409, 'There is no unresolved Nav2 goal to cancel.')
        for goal_id in goal_ids:
            # Marks pending acceptance abandoned as well as accepted handles.
            self._request_goal_cancel(goal_id)
        return {
            'ok': True, 'message': 'Cancellation requested; wait for a terminal action result.',
            'goal_ids': goal_ids, 'terminal_confirmed': False,
            'planner_only': self._bridge_profile == 'telemetry',
        }

    def _load_token(self):
        self.token_file.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        try:
            token = self.token_file.read_text(encoding='ascii').strip()
        except FileNotFoundError:
            return None
        if len(token) < 32:
            raise RuntimeError('API token file must contain at least 32 characters')
        os.chmod(str(self.token_file), 0o600)
        return token

    def pair_api(self, token):
        if not isinstance(token, str) or not re.fullmatch(r'[A-Za-z0-9_-]{32,128}', token):
            raise ApiError(400, 'Pairing key must be 32-128 URL-safe characters.')
        with self._token_lock:
            if self.api_token is not None:
                raise ApiError(409, 'This robot is already paired. Use the saved API key.')
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
            try:
                fd = os.open(str(self.token_file), flags, 0o600)
            except FileExistsError:
                self.api_token = self._load_token()
                raise ApiError(409, 'This robot was paired by another browser; enter its API key.')
            with os.fdopen(fd, 'w', encoding='ascii') as output:
                output.write(token + '\n')
            self.api_token = token
        return {'ok': True, 'message': 'This browser is paired with the robot.'}

    def issue_browser_session(self):
        if self.api_token is None:
            try:
                self.pair_api(secrets.token_urlsafe(48))
            except ApiError as error:
                if error.status != 409:
                    raise
        session_id = secrets.token_urlsafe(32)
        now = time.monotonic()
        with self._session_lock:
            self._browser_sessions = {
                key: expiry for key, expiry in self._browser_sessions.items()
                if expiry > now
            }
            self._browser_sessions[session_id] = now + self._session_ttl_seconds
        return session_id

    def valid_browser_session(self, session_id):
        if not isinstance(session_id, str) or not session_id:
            return False
        now = time.monotonic()
        with self._session_lock:
            expiry = self._browser_sessions.get(session_id)
            if expiry is None:
                return False
            if expiry <= now:
                self._browser_sessions.pop(session_id, None)
                return False
            return True

    def start_mapping(self):
        if self._bridge_profile != 'telemetry':
            raise ApiError(409, 'Select the telemetry-only bridge profile before mapping')
        with self._lock:
            if self._mapping_process is not None and self._mapping_process.poll() is None:
                raise ApiError(409, 'Mapping is already running.')
            if self._navigation_process is not None and self._navigation_process.poll() is None:
                raise ApiError(409, 'Stop localization/navigation before starting mapping.')
            if self._bridge_alive():
                raise ApiError(409, 'The project odometry bridge is already running.')
            if self._component('camera', time.monotonic())['state'] != 'ready':
                raise ApiError(409, 'Depth camera data is not fresh; mapping was not started.')
            self._mapping_paused = False
            self._stitched_pointcloud.clear()
            self._stitched_point_count = 0
            self._last_stitch_time = 0.0
            self._last_rx.pop('stitched_pointcloud', None)
            if not self._bridge_config.is_file():
                raise ApiError(500, f'Project bridge config missing: {self._bridge_config}')

        # ROS 1 queries can take seconds. Do not hold the state lock they share
        # with priority STOP while waiting for those subprocesses.
        if not self._adopt_or_start_bridge():
            raise ApiError(503, 'Could not establish the project odometry-only bridge.')

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if not self._bridge_alive():
                self.stop_bridge()
                raise ApiError(503, 'The odometry-only ROS bridge exited; see robot_api_bridge.log.')
            if self._component('vendor_odometry', time.monotonic())['state'] == 'ready':
                break
            time.sleep(0.1)
        else:
            self.stop_bridge()
            raise ApiError(503, 'No fresh /leg_odom2 arrived through the bridge; mapping was not started.')

        with self._lock:
            log_dir = self._workspace / 'logs'
            try:
                self._mapping_log = (log_dir / 'robot_api_mapping.log').open('ab')
                self._mapping_process = subprocess.Popen(
                    ['ros2', 'launch', 'lite3_odom_tf_adapter', 'mapping.launch.py'],
                    cwd=str(self._workspace), env=os.environ.copy(),
                    stdout=self._mapping_log, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception:
                if self._mapping_log:
                    self._mapping_log.close()
                    self._mapping_log = None
                self.stop_bridge()
                raise
        self._expected_map_signature = self._navigation_identity = None
        self._new_pipeline_session('Mapping started')
        self.get_logger().warning('Mapping launched; it does not publish motion commands.')
        return {'ok': True, 'message': 'Mapping started. Guide the robot with the RC in manual mode.'}

    def start_navigation(self, map_name):
        with self._lock:
            if self._stop_cleanup_pending:
                raise ApiError(409, 'Software STOP cleanup is still pending.')
            if self._stop_cleanup_error:
                raise ApiError(409, 'Software STOP cleanup failed; retry STOP before restarting navigation.')
            stop_generation = self._stop_generation
        if not isinstance(map_name, str) or not MAP_NAME.fullmatch(map_name):
            raise ApiError(400, 'Select a saved robot map using its map name.')
        map_yaml = self.maps_dir / f'{map_name}.yaml'
        map_image = self.maps_dir / f'{map_name}.pgm'
        if not map_yaml.is_file() or not map_image.is_file():
            raise ApiError(404, 'Matching saved map YAML and PGM were not found on the robot.')
        try:
            bundle = read_bundle(self.maps_dir / map_name,
                                 require_reviewed=self._bridge_profile == 'command',
                                 allow_legacy=self._bridge_profile != 'command')
        except (OSError, ValueError, KeyError) as error:
            raise ApiError(409, 'Map review/integrity check failed: ' + str(error))
        if self._component('camera', time.monotonic())['state'] != 'ready':
            raise ApiError(409, 'Depth camera data is not fresh; navigation was not started.')
        with self._lock:
            if self._mapping_process is not None and self._mapping_process.poll() is None:
                raise ApiError(409, 'Stop mapping before starting localization/navigation.')
            if self._navigation_process is not None and self._navigation_process.poll() is None:
                raise ApiError(409, 'Localization/navigation is already running.')
            self._amcl_pose = None
            self._last_rx['localization'] = None
        with self._localization_lock:
            self._localization_refresh_active = False
            self._localization_refresh_inflight = False
            self._localization_converged = False
            self._localization_stable_samples = 0
            self._localization_previous_candidate = None
            self._localization_reason = 'Waiting for an AMCL localization request.'
        command_names = {'lite3_cmd_vel_gate', 'lite3_motion_arbiter', 'lite3_vendor_cmd_vel_bridge', 'lite3_motion_supervisor', 'lite3_supervised_nav_relay'}
        existing = {name for name, namespace in self.get_node_names_and_namespaces()}
        if existing.intersection(command_names):
            raise ApiError(409, 'Existing command-stage nodes must be reconciled before startup: ' + ', '.join(sorted(existing.intersection(command_names))))
        self._start_bridge()
        self._new_pipeline_session('Localization/Nav2 started')
        self._goal_clearance_radius = 0.30
        if self._bridge_profile == 'command':
            record = self._commissioning_record()
            self._goal_clearance_radius = max(0.30, max(math.hypot(*point) for point in record['footprint'])+0.05)
        self._operating_region = bundle.get('manifest', {}).get('review', {}).get('operating_region_cells')
        self._navigation_identity = bundle['map_id']
        self._navigation_map_trial_eligible = bool(
            not bundle['manifest'].get('legacy_unverified') and
            bundle['manifest'].get('synthetic') is not True)
        self._navigation_map_reviewed = bool(
            bundle['approval_state'] == 'reviewed' and
            bundle['manifest'].get('review', {}).get('map_id') == bundle['map_id'] and
            bundle['manifest'].get('synthetic') is not True)
        self._expected_map_signature = bundle['grid_signature']
        log_dir = self._workspace / 'logs'
        log_dir.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self._navigation_log = (log_dir / 'robot_api_navigation.log').open('ab')
            try:
                if self._stop_generation != stop_generation:
                    raise ApiError(409, 'Software STOP interrupted navigation startup.')
                self._navigation_started_at = time.monotonic()
                self._navigation_process = subprocess.Popen(
                    ['ros2', 'launch', 'lite3_odom_tf_adapter', 'navigation.launch.py',
                     f'map:={map_yaml}', f'commissioned_motion:={str(self._bridge_profile == "command").lower()}',
                     f'supervised_trial:={str(self._bridge_profile == "supervised_trial").lower()}'], cwd=str(self._workspace), env=os.environ.copy(),
                    stdout=self._navigation_log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                self._navigation_map_name = map_name
                self._autonomy_stop_latched = False
            except Exception:
                self._navigation_log.close()
                self._navigation_log = None
                self.stop_bridge()
                raise
        detail = ('A verified unreviewed map permits only a short supervised trial through known-free space.'
                  if self._bridge_profile == 'supervised_trial' and self._navigation_map_trial_eligible
                  and not self._navigation_map_reviewed else
                  'Targets remain disabled until this map receives a measured review.'
                  if not self._navigation_map_reviewed else 'Motion requires explicit arming.')
        return {'ok': True, 'message': f'Localization/Nav2 starting with robot map {map_name}. {detail}'}

    def stop_navigation(self):
        with self._goal_lock:
            goal_ids = list(self._goal_requests)
        for goal_id in goal_ids:
            self._request_goal_cancel(goal_id)
        # Planner goal requests require a fresh opt-in after Nav2 is restarted.
        self.allow_nav2_goal_requests = False
        with self._lock:
            process = self._navigation_process
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=8.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3.0)
                except (ProcessLookupError, subprocess.TimeoutExpired) as error:
                    raise ApiError(500, f'Navigation shutdown failed: {error}')
        with self._lock:
            if self._navigation_process is process:
                self._navigation_process = None
                self._navigation_map_name = None
                self._navigation_started_at = 0.0
        with self._localization_lock:
            self._localization_refresh_active = False
            self._localization_refresh_inflight = False
        if self._navigation_log:
            self._navigation_log.close()
            self._navigation_log = None
        self.stop_bridge()
        self._navigation_identity = self._expected_map_signature = None
        self._navigation_map_reviewed = False
        self._navigation_map_trial_eligible = False
        self._new_pipeline_session('Localization/Nav2 stopped')
        return {'ok': True, 'message': 'Localization/Nav2 stopped and goal outputs disabled.'}

    def stop_autonomy(self):
        """Latch immediately; disable outputs without waiting for slow actions.

        Navigation cleanup runs under the ordinary action lock in a worker.
        This is a project software stop, never an independent physical stop.
        """
        with self._lock:
            self._autonomy_stop_latched = True
            self._runtime.release()
            self._path_preview = None
            self._path_request_id = None
            self.allow_nav2_goal_requests = False
            start_cleanup = not self._stop_cleanup_pending
            if start_cleanup:
                self._stop_generation += 1
                self._stop_cleanup_pending = True
                self._stop_cleanup_error = None
            generation = self._stop_generation
            try:
                self._readiness_pub.publish(Bool(data=False))
            except Exception as error:
                # A failed ROS publisher must not prevent the independent
                # disable requests or leave the cleanup worker unstarted.
                self._stop_disable_results['readiness'] = 'publish_failed'
                self.get_logger().error(f'STOP readiness publication failed: {error}')

        for name, client in self._autonomy_disable_clients.items():
            try:
                with self._lock:
                    self._stop_disable_results[name] = 'requested'
                if not client.service_is_ready():
                    with self._lock:
                        self._stop_disable_results[name] = 'unavailable'
                    continue
                future = client.call_async(SetBool.Request(data=False))
                future.add_done_callback(
                    lambda done, key=name, epoch=generation:
                    self._record_stop_disable(key, epoch, done))
            except Exception as error:
                with self._lock:
                    self._stop_disable_results[name] = 'failed'
                self.get_logger().error(f'{name} disable request failed: {error}')

        if start_cleanup:
            try:
                self._stop_cleanup_thread = threading.Thread(
                    target=self._finish_autonomy_stop, daemon=True)
                self._stop_cleanup_thread.start()
            except Exception as error:
                with self._lock:
                    self._stop_cleanup_pending = False
                    self._stop_cleanup_error = str(error)

        cleanup = self._stop_cleanup_status()
        return {
            'ok': True,
            'message': (
                'Software stop latched; output disable requested. '
                'Check safety stop status for navigation cleanup and disable confirmation. '
                'This is not a physical emergency stop; use the vendor controller or hardware stop.'
            ),
            'stop_latched': True,
            'gate_disable_confirmed': cleanup['disable_results'].get('gate') == 'confirmed',
            **cleanup,
            'physical_estop': False,
        }

    def _stop_cleanup_status(self):
        with self._lock:
            return {
                'cleanup_pending': self._stop_cleanup_pending,
                'cleanup_error': self._stop_cleanup_error,
                'disable_results': dict(self._stop_disable_results),
            }

    def _record_stop_disable(self, name, generation, future):
        try:
            result = 'confirmed' if future.result().success else 'refused'
        except Exception:
            result = 'failed'
        with self._lock:
            if generation == self._stop_generation:
                self._stop_disable_results[name] = result

    def _finish_autonomy_stop(self):
        try:
            with self._api_action_lock:
                with self._goal_lock:
                    goal_active = bool(self._goal_requests)
                if goal_active:
                    try:
                        self.cancel_nav_goal()
                    except Exception as error:
                        self.get_logger().warning(
                            f'Goal cancellation unconfirmed; stopping Nav2: {error}')
                self.stop_navigation()
                with self._goal_lock:
                    if not self._goal_requests:
                        self._goal_handle = None
                        self._goal_state = 'stopped'
                        self._goal_detail = 'Software autonomy stop latched; Nav2 cleanup complete.'
        except Exception as error:
            with self._lock:
                self._stop_cleanup_error = str(error)
            self.get_logger().error(f'Software STOP cleanup failed: {error}')
        finally:
            with self._lock:
                self._autonomy_stop_latched = True
                self.allow_nav2_goal_requests = False
                self._stop_cleanup_pending = False

    def _start_bridge(self):
        with self._lock:
            if self._bridge_alive():
                return
            if not self._bridge_config.is_file():
                raise ApiError(500, f'Project bridge config missing: {self._bridge_config}')
        if not self._adopt_or_start_bridge():
            raise ApiError(503, 'Could not establish the project odometry-only bridge.')
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if not self._bridge_alive():
                self.stop_bridge()
                raise ApiError(503, 'The odometry-only ROS bridge exited; see robot_api_bridge.log.')
            if self._component('vendor_odometry', time.monotonic())['state'] == 'ready':
                return
            time.sleep(0.1)
        self.stop_bridge()
        raise ApiError(503, 'No fresh /leg_odom2 arrived through the ROS bridge.')

    @staticmethod
    def _bridge_process_identity(pid):
        """Identify a live bridge by PID and Linux start time, avoiding PID reuse."""
        try:
            process = Path('/proc') / str(int(pid))
            argv = (process / 'cmdline').read_bytes().split(b'\0')
            # Fields after the closing comm parenthesis start at field 3.
            fields = (process / 'stat').read_text().rpartition(')')[2].split()
            if argv[0] != b'/opt/ros/foxy/lib/ros1_bridge/parameter_bridge':
                return None
            if fields[0] == 'Z':
                return None
            return (int(pid), int(fields[19]))
        except (OSError, ValueError, IndexError):
            return None

    def _bridge_alive(self):
        if self._bridge_external:
            identity = self._bridge_external_identity
            return bool(identity and self._bridge_process_identity(identity[0]) == identity)
        return self._bridge_process is not None and self._bridge_process.poll() is None

    def _bridge_topics_match(self, result):
        try:
            expected = yaml.safe_load(self._bridge_config.read_text(encoding='utf-8'))['topics']
            return result.returncode == 0 and yaml.safe_load(result.stdout) == expected
        except (OSError, KeyError, TypeError, yaml.YAMLError):
            return False

    def _adopt_or_start_bridge(self):
        """Reuse only the single already-running bridge with the exact narrow config."""
        existing = subprocess.run(
            # Linux TASK_COMM is truncated to 15 characters, so `pgrep -x
            # parameter_bridge` cannot match this 16-character executable name.
            # Match the known Foxy executable path as a full argv instead.
            ['pgrep', '-f', '^/opt/ros/foxy/lib/ros1_bridge/parameter_bridge$'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, check=False)
        occupied = self._run_noetic_command('rosparam get /topics', allow_failure=True)
        if existing.returncode == 0 or occupied.returncode == 0:
            try:
                pids = existing.stdout.split()
                identity = self._bridge_process_identity(pids[0]) if len(pids) == 1 else None
            except (ValueError, TypeError):
                return False
            if identity is None or not self._bridge_topics_match(occupied):
                return False
            self._bridge_external = True
            self._bridge_external_identity = identity
            self._last_rx.pop('odometry', None)
            return True

        if existing.returncode != 1:
            return False
        loaded = self._run_noetic_command(f"rosparam load '{self._bridge_config}'")
        if loaded.returncode != 0:
            return False
        self._bridge_param_owned = True
        self._bridge_external = False
        self._bridge_external_identity = None
        self._last_rx.pop('odometry', None)
        try:
            log_dir = self._workspace / 'logs'
            log_dir.mkdir(parents=True, exist_ok=True)
            self._bridge_log = (log_dir / 'robot_api_bridge.log').open('ab')
            bridge_env = os.environ.copy()
            # The recorder, mapping launch, and guarded project relay use
            # Cyclone DDS.  Keep this subscription-only bridge on the same
            # implementation so its /leg_odom2 publisher is discoverable by
            # the API and mapping nodes.
            bridge_env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
            self._bridge_process = subprocess.Popen(
                ['ros2', 'run', 'ros1_bridge', 'parameter_bridge'],
                cwd=str(self._workspace), env=bridge_env,
                stdout=self._bridge_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            self.stop_bridge()
            raise
        return True

    def stop_mapping(self):
        errors = []
        with self._lock:
            process = self._mapping_process
        if process is not None and process.poll() is None:
            try:
                if self._mapping_paused:
                    os.killpg(process.pid, signal.SIGCONT)
                    self._mapping_paused = False
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=8.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3.0)
                except (ProcessLookupError, subprocess.TimeoutExpired) as error:
                    errors.append(f'mapping shutdown: {error}')
        if errors:
            raise ApiError(500, '; '.join(errors))
        with self._lock:
            if self._mapping_process is process:
                self._mapping_process = None
            self._mapping_paused = False
        if self._mapping_log:
            self._mapping_log.close()
            self._mapping_log = None
        self.stop_bridge()
        self._new_pipeline_session('Mapping stopped')
        return {'ok': True, 'message': 'Mapping stopped; autonomous output remains inhibited.'}

    def pause_mapping(self):
        with self._lock:
            process = self._mapping_process
            if process is None or process.poll() is not None:
                raise ApiError(409, 'Mapping is not running.')
            if self._mapping_paused:
                return {'ok': True, 'message': 'Mapping is already paused.'}
            try:
                os.killpg(process.pid, signal.SIGSTOP)
            except ProcessLookupError:
                raise ApiError(409, 'The mapping process has already exited.')
            self._mapping_paused = True
        return {
            'ok': True,
            'message': 'Mapping paused. Keep the robot stationary with the manual controller; pause does not stop robot motion.',
        }

    def resume_mapping(self):
        with self._lock:
            process = self._mapping_process
            if process is None or process.poll() is not None:
                raise ApiError(409, 'Mapping is not running.')
            if not self._mapping_paused:
                return {'ok': True, 'message': 'Mapping is already running.'}
            try:
                os.killpg(process.pid, signal.SIGCONT)
            except ProcessLookupError:
                raise ApiError(409, 'The mapping process has already exited.')
            self._mapping_paused = False
        return {'ok': True, 'message': 'Mapping resumed.'}

    def stop_bridge(self):
        with self._lock:
            # An adopted process and its global ROS 1 parameters belong to its
            # original owner. Detach without sending signals or deleting state.
            if self._bridge_external:
                self._bridge_external = False
                self._bridge_external_identity = None
                return
            process = self._bridge_process
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    raise ApiError(500, 'Project odometry bridge did not stop cleanly; API left running.')
        with self._lock:
            if self._bridge_process is process:
                self._bridge_process = None
        if self._bridge_log:
            self._bridge_log.close()
            self._bridge_log = None
        if self._bridge_param_owned:
            active = self._run_noetic_command('rosparam get /topics', allow_failure=True)
            if self._bridge_topics_match(active):
                self._run_noetic_command('rosparam delete /topics', allow_failure=True)
            else:
                self.get_logger().warning(
                    'Bridge /topics changed or unavailable; leaving ROS 1 parameters untouched.')
            self._bridge_param_owned = False

    def prepare_for_poweroff(self):
        """Stop project-owned ROS launches, then close the project HTTP API."""
        with self._goal_lock:
            goal_active = bool(self._goal_requests)
        if goal_active:
            self.cancel_nav_goal()
        self.stop_navigation()
        self.stop_mapping()
        with self._lock:
            remaining = [
                label for label, process in (
                    ('mapping', self._mapping_process),
                    ('localization/Nav2', self._navigation_process),
                    ('odometry bridge', self._bridge_process),
                ) if process is not None and process.poll() is None
            ]
        if remaining:
            raise ApiError(500, 'Could not stop project processes: ' + ', '.join(remaining))
        self._shutdown_requested = True
        return {
            'ok': True,
            'message': (
                'Project mapping, localization/Nav2, and owned odometry bridge are stopped. '
                'The project API is closing. Vendor services and the robot computer remain on; '
                'use the normal robot power-off procedure before changing the battery.'
            ),
        }

    def close_http_server_after_reply(self):
        """Close the HTTP listener only after the shutdown reply can reach the browser."""
        time.sleep(0.5)
        if self._http_server_shutdown is not None:
            self._http_server_shutdown()

    @staticmethod
    def _run_noetic_command(command, allow_failure=False):
        script = f'source /opt/ros/noetic/setup.bash; {command}'
        env = {
            'HOME': str(Path.home()), 'USER': 'ysc', 'LOGNAME': 'ysc',
            'PATH': '/usr/bin:/bin',
        }
        for key in ('ROS_MASTER_URI', 'ROS_IP', 'ROS_HOSTNAME'):
            if key in os.environ:
                env[key] = os.environ[key]
        return subprocess.run(
            ['/bin/bash', '--noprofile', '--norc', '-c', script],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, timeout=5.0, check=not allow_failure,
        )

    def save_latest_map(self, name, message=None):
        if not isinstance(name, str) or not MAP_NAME.fullmatch(name):
            raise ApiError(400, 'Map name must use 1-64 letters, digits, underscore, or hyphen.')
        if message is None:
            with self._lock:
                mapping_live = self._mapping_process is not None and self._mapping_process.poll() is None
                navigation_live = self._navigation_process is not None and self._navigation_process.poll() is None
                map_recent = self._last_rx.get('map') is not None and time.monotonic() - self._last_rx['map'] <= MAP_STALE_AFTER
            if not mapping_live or navigation_live or not map_recent:
                raise ApiError(409, 'No fresh live SLAM map. Start mapping, wait for map updates, then save before stopping mapping.')
        with self._lock:
            message = self._map if message is None else message
        if message is None:
            raise ApiError(409, 'No map has been received yet.')
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        prefix = self.maps_dir / name
        if prefix.with_suffix('.pgm').exists() or prefix.with_suffix('.yaml').exists():
            raise ApiError(409, 'That map name already exists; choose a new name to preserve earlier maps.')
        try:
            image, yaml_file, manifest = save_grid(message, prefix, provenance={
                'software_id': self.software_identity(), 'session_epoch': self._runtime.epoch,
                'capture_report_sha256': None, 'effective_config_sha256': None,
                'checkpoint_source_recovered': False})
        except FileExistsError:
            raise ApiError(409, 'That map name is reserved or already exists; choose a new name.')
        except ValueError as error:
            raise ApiError(400, str(error))
        return {
            'ok': True,
            'name': name,
            'image_url': f'/maps/{image.name}',
            'yaml_url': f'/maps/{yaml_file.name}',
            'manifest_url': f'/maps/{manifest.name}',
            'width': message.info.width,
            'height': message.info.height,
            'approval_state': 'experimental',
        }

    def run_readonly_preflight(self):
        script = self._workspace / 'scripts' / 'robot_preflight.sh'
        if not script.is_file():
            raise ApiError(503, 'Robot preflight script is not installed in the project workspace.')
        try:
            result = subprocess.run(
                ['/bin/bash', str(script)], cwd=str(self._workspace),
                env=os.environ.copy(), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, timeout=45.0, check=False)
        except subprocess.TimeoutExpired:
            raise ApiError(504, 'Read-only preflight exceeded 45 seconds.')
        output = result.stdout or ''
        return {
            'ok': result.returncode == 0,
            'exit_code': result.returncode,
            'output': output[-30000:],
            'truncated': len(output) > 30000,
        }

    def list_maps(self):
        if not self.maps_dir.is_dir():
            return []
        items = []
        for yaml_file in sorted(self.maps_dir.glob('*.yaml')):
            image = yaml_file.with_suffix('.pgm')
            if image.is_file():
                manifest = load_manifest(yaml_file.with_suffix(''))
                try:
                    identity = read_bundle(yaml_file.with_suffix(''), allow_legacy=True)
                    integrity = {'map_id': identity['map_id'], 'approval_state': identity['approval_state'],
                                 'integrity': 'legacy_unverified' if identity['manifest'].get('legacy_unverified') else 'verified'}
                except (OSError, ValueError, KeyError):
                    integrity = {'map_id': None, 'approval_state': 'unverified', 'integrity': 'failed_or_legacy'}
                items.append({
                    'name': yaml_file.stem,
                    **integrity,
                    'image_url': f'/maps/{image.name}',
                    'yaml_url': f'/maps/{yaml_file.name}',
                    'manifest_url': (
                        f'/maps/{yaml_file.stem}.lite3-map.json' if manifest else None),
                    'has_registered_3d': bool(
                        manifest and manifest.get('reconstruction_3d', {}).get('available')),
                })
        return items

class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def make_handler(node):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(node.web_root), **kwargs)

        def log_message(self, _format, *_args):
            return

        def _json(self, status, payload):
            body = json.dumps(payload).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _binary(self, status, payload, headers=None):
            self.send_response(status)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            for key, value in (headers or {}).items():
                self.send_header(key, str(value))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/api/v1/maps/bundle':
                query = parse_qs(urlparse(self.path).query)
                name = query.get('name', [''])[0]
                identity = query.get('map_id', [''])[0]
                if not MAP_NAME.fullmatch(name):
                    return self._json(400, {'ok': False, 'error': 'Invalid map name'})
                try:
                    bundle = read_bundle(node.maps_dir / name, allow_legacy=True)
                    if bundle['map_id'] != identity:
                        raise ValueError('Map changed; refresh catalog and load again')
                    return self._json(200, {'ok': True, 'name': name, 'map_id': identity,
                                            'image_base64': base64.b64encode(bundle['_image']).decode(),
                                            'yaml_text': bundle['_yaml'].decode(),
                                            'approval_state': bundle['approval_state'],
                                            'review': bundle['manifest'].get('review')})
                except (OSError, ValueError, KeyError) as error:
                    return self._json(409, {'ok': False, 'error': str(error)})
            if path == '/api/v1/session':
                try:
                    session_id = node.issue_browser_session()
                except ApiError as error:
                    return self._json(error.status, {'ok': False, 'error': str(error)})
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header(
                    'Set-Cookie',
                    f'lite3_session={session_id}; HttpOnly; SameSite=Strict; Path=/; Max-Age={node._session_ttl_seconds}',
                )
                body = json.dumps({'ok': True, 'authorized': True}).encode('utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == '/api/v1/status':
                return self._json(200, node.status())
            if path == '/api/v1/map':
                result = node.status()
                return self._json(200, {'ok': True, 'map': result['map']})
            if path == '/api/v1/map/grid':
                with node._lock:
                    grid = node._map
                if grid is None or node._component('map', time.monotonic())['state'] != 'ready':
                    return self._json(503, {'ok': False, 'error': 'No fresh live map is available.'})
                origin = grid.info.origin
                orientation = origin.orientation
                origin_yaw = math.atan2(
                    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
                )
                return self._json(200, {
                    'ok': True,
                    'frame': grid.header.frame_id,
                    'width': int(grid.info.width),
                    'height': int(grid.info.height),
                    'resolution': float(grid.info.resolution),
                    'origin': {'x': float(origin.position.x), 'y': float(origin.position.y), 'yaw': origin_yaw},
                    'data': list(grid.data),
                })
            if path == '/api/v1/pointcloud':
                with node._lock:
                    points = node._pointcloud
                    frame = node._pointcloud_frame
                    count = node._pointcloud_count
                    stamp = node._last_rx.get('pointcloud')
                age = None if stamp is None else max(0.0, time.monotonic() - stamp)
                if points is None or age is None or age > STALE_AFTER:
                    return self._json(503, {'ok': False, 'error': 'No fresh camera point cloud is available.'})
                return self._binary(200, points, {
                    'X-Point-Frame': frame,
                    'X-Point-Count': count,
                    'X-Point-Age': f'{age:.3f}',
                })
            if path == '/api/v1/pointcloud/map':
                with node._lock:
                    points = bytes(node._stitched_pointcloud)
                    count = node._stitched_point_count
                if not points:
                    return self._json(503, {
                        'ok': False,
                        'error': 'No map-frame point cloud has been registered yet. Start mapping and allow TF registration.'})
                return self._binary(200, points, {
                    'X-Point-Frame': 'map',
                    'X-Point-Count': count,
                })
            if path == '/api/v1/maps':
                return self._json(200, {'ok': True, 'maps': node.list_maps()})
            if path.startswith('/api/'):
                return self._json(404, {'ok': False, 'error': 'Unknown API endpoint.'})
            return super().do_GET()

        def do_POST(self):
            path = urlparse(self.path).path
            if path not in (
                '/api/v1/mapping/start', '/api/v1/mapping/stop',
                '/api/v1/mapping/pause', '/api/v1/mapping/resume',
                '/api/v1/navigation/start', '/api/v1/navigation/stop',
                '/api/v1/safety/stop',
                '/api/v1/maps/save', '/api/v1/navigation/goal',
                '/api/v1/maps/finish', '/api/v1/maps/review', '/api/v1/navigation/preview',
                '/api/v1/control/acquire', '/api/v1/control/heartbeat',
                '/api/v1/motion/arm', '/api/v1/motion/arm-supervised', '/api/v1/bridge/profile',
                '/api/v1/navigation/goal-requests',
                '/api/v1/navigation/cancel',
                '/api/v1/localization/initial_pose',
                '/api/v1/localization/global',
                '/api/v1/diagnostics/preflight',
                '/api/v1/shutdown/prepare',
                '/api/v1/pair',
            ):
                return self._json(404, {'ok': False, 'error': 'Unknown API endpoint.'})
            if node._shutdown_requested and path != '/api/v1/safety/stop':
                return self._json(503, {'ok': False, 'error': 'Project API shutdown is in progress.'})
            if path == '/api/v1/pair':
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0 or length > 4096:
                        raise ApiError(400, 'Expected a generated pairing key.')
                    try:
                        body = json.loads(self.rfile.read(length).decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        raise ApiError(400, 'Request body must be valid JSON.')
                    if not isinstance(body, dict):
                        raise ApiError(400, 'JSON body must be an object.')
                    payload = node.pair_api(body.get('token'))
                    return self._json(200, payload)
                except ApiError as error:
                    return self._json(error.status, {'ok': False, 'error': str(error)})
            supplied_token = self.headers.get('X-Lite3-Token', '')
            token_valid = node.api_token is not None and hmac.compare_digest(
                supplied_token, node.api_token)
            session_valid = False
            if not token_valid:
                try:
                    cookie = SimpleCookie(self.headers.get('Cookie', ''))
                    session_cookie = cookie.get('lite3_session')
                    session_valid = bool(
                        session_cookie and node.valid_browser_session(session_cookie.value))
                except Exception:
                    session_valid = False
                origin = urlparse(self.headers.get('Origin', ''))
                same_origin = (
                    origin.scheme == 'http' and
                    origin.netloc.casefold() == self.headers.get('Host', '').casefold())
                session_valid = session_valid and same_origin
            if not token_valid and not session_valid:
                return self._json(401, {'ok': False, 'error': 'Invalid or missing robot API token.'})
            identity = supplied_token if token_valid else session_cookie.value
            actor = 'client:' + hashlib.sha256(identity.encode()).hexdigest()
            if path in ('/api/v1/control/acquire', '/api/v1/control/heartbeat'):
                # Lease renewal must not queue behind preflight/path/startup.
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if length < 0 or length > 4096:
                        raise ApiError(400, 'Invalid lease request size')
                    body = json.loads(self.rfile.read(length)) if length else {}
                    if not isinstance(body, dict):
                        raise ApiError(400, 'Lease body must be an object')
                    payload = (node.acquire_operator_lease(actor) if path.endswith('/acquire') else
                               node.renew_operator_lease(actor, body.get('lease_token'), body.get('session_epoch')))
                    return self._json(200, payload)
                except (ValueError, TypeError):
                    return self._json(400, {'ok': False, 'error': 'Malformed lease request'})
                except ApiError as error:
                    return self._json(error.status, {'ok': False, 'error': str(error)})
            # STOP must bypass the action lock: preflight/startup may hold it
            # for many seconds. Authentication still applies before latching.
            if path == '/api/v1/safety/stop':
                try:
                    payload = node.stop_autonomy()
                    return self._json(202 if payload['cleanup_pending'] else 200, payload)
                except Exception as error:
                    node.get_logger().error(f'Priority STOP request failed: {error}')
                    return self._json(500, {'ok': False, 'error': 'Software STOP request failed; check robot status.'})
            if not node._api_action_lock.acquire(blocking=False):
                return self._json(409, {'ok': False, 'error': 'Another robot control action is still running; retry after it finishes.'})
            try:
                if node._shutdown_requested:
                    raise ApiError(503, 'Project API shutdown is in progress.')
                if node._stop_cleanup_pending:
                    raise ApiError(409, 'Software STOP cleanup is still pending; other controls are inhibited.')
                if path in ('/api/v1/maps/finish', '/api/v1/maps/review', '/api/v1/navigation/preview',
                            '/api/v1/motion/arm', '/api/v1/motion/arm-supervised', '/api/v1/bridge/profile'):
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 16384:
                        raise ApiError(400, 'Expected a bounded JSON request')
                    try:
                        body = json.loads(self.rfile.read(length))
                    except (ValueError, TypeError):
                        raise ApiError(400, 'Malformed JSON request')
                    if not isinstance(body, dict):
                        raise ApiError(400, 'Expected a JSON object')
                    if path == '/api/v1/maps/finish':
                        if body.get('stationary_ack') is not True:
                            raise ApiError(400, 'Explicit stationary operator acknowledgment required')
                        payload = node.finish_scan(body.get('name'))
                    elif path == '/api/v1/maps/review':
                        payload = node.review_map(body.get('name'), body.get('review'))
                    elif path == '/api/v1/bridge/profile':
                        payload = node.select_bridge_profile(body.get('profile'))
                    else:
                        if body.get('session_epoch') != node._runtime.epoch:
                            raise ApiError(409, 'Navigation session changed; reload status and select again')
                        if path == '/api/v1/motion/arm':
                            if not node._runtime.valid_lease(actor, body.get('lease_token')):
                                raise ApiError(409, 'Current controlling lease is required')
                            payload = node.arm_motion()
                        elif path == '/api/v1/motion/arm-supervised':
                            if not node._runtime.valid_lease(actor, body.get('lease_token')):
                                raise ApiError(409, 'Current controlling lease is required')
                            payload = node.arm_supervised_trial(
                                body.get('auto_mode_confirmed'), body.get('app_stop_ready'),
                                body.get('area_clear'))
                        else:
                            payload = node.preview_nav_path(body.get('x'), body.get('y'), body.get('yaw', 0),
                                                            body.get('map_name'), body.get('map_id'))
                    return self._json(200, payload)
                if path == '/api/v1/shutdown/prepare':
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0 or length > 4096:
                        raise ApiError(400, 'Explicit shutdown confirmation is required.')
                    try:
                        body = json.loads(self.rfile.read(length).decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        raise ApiError(400, 'Request body must be valid JSON.')
                    if not isinstance(body, dict) or body.get('confirm') != 'STOP PROJECT':
                        raise ApiError(400, 'Type STOP PROJECT in the UI confirmation to continue.')
                    payload = node.prepare_for_poweroff()
                    threading.Thread(
                        target=node.close_http_server_after_reply, daemon=True).start()
                    self._json(200, payload)
                    return
                if path == '/api/v1/mapping/start':
                    payload = node.start_mapping()
                elif path == '/api/v1/mapping/pause':
                    payload = node.pause_mapping()
                elif path == '/api/v1/mapping/resume':
                    payload = node.resume_mapping()
                elif path == '/api/v1/mapping/stop':
                    payload = node.stop_mapping()
                elif path == '/api/v1/navigation/start':
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0 or length > 4096:
                        raise ApiError(400, 'Expected JSON containing the saved map name.')
                    try:
                        body = json.loads(self.rfile.read(length).decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        raise ApiError(400, 'Request body must be valid JSON.')
                    if not isinstance(body, dict):
                        raise ApiError(400, 'JSON body must be an object.')
                    payload = node.start_navigation(body.get('name'))
                elif path == '/api/v1/navigation/stop':
                    payload = node.stop_navigation()
                elif path == '/api/v1/navigation/goal-requests':
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0 or length > 4096:
                        raise ApiError(400, 'Expected JSON with enabled=true or false.')
                    try:
                        body = json.loads(self.rfile.read(length).decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        raise ApiError(400, 'Request body must be valid JSON.')
                    if not isinstance(body, dict) or 'enabled' not in body:
                        raise ApiError(400, 'Expected a boolean enabled field.')
                    if body['enabled'] and not node._runtime.valid_lease(actor, body.get('lease_token')):
                        raise ApiError(409, 'Acquire and renew this browser control lease first')
                    payload = node.set_nav_goal_requests(body['enabled'], body.get('map_name'), body.get('map_id'))
                elif path == '/api/v1/navigation/cancel':
                    payload = node.cancel_nav_goal()
                elif path == '/api/v1/localization/initial_pose':
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0 or length > 4096:
                        raise ApiError(400, 'Expected JSON containing x, y, and optional yaw.')
                    try:
                        body = json.loads(self.rfile.read(length).decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        raise ApiError(400, 'Request body must be valid JSON.')
                    if not isinstance(body, dict) or not all(k in body for k in ('x', 'y')):
                        raise ApiError(400, 'Initial pose JSON must contain x and y.')
                    if body.get('session_epoch') != node._runtime.epoch:
                        raise ApiError(409, 'Initial-pose session is stale')
                    node._assert_active_map(node._navigation_map_name, body.get('map_id'), require_reviewed=False)
                    payload = node.set_initial_pose(body['x'], body['y'], body.get('yaw', 0.0))
                elif path == '/api/v1/localization/global':
                    payload = node.request_global_localization()
                elif path == '/api/v1/diagnostics/preflight':
                    payload = node.run_readonly_preflight()
                else:
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0 or length > 4096:
                        raise ApiError(400, 'Expected a small JSON body containing a map name.')
                    try:
                        body = json.loads(self.rfile.read(length).decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        raise ApiError(400, 'Request body must be valid JSON.')
                    if not isinstance(body, dict):
                        raise ApiError(400, 'JSON body must be an object.')
                    if path == '/api/v1/navigation/goal':
                        if not all(k in body for k in ('x', 'y')):
                            raise ApiError(400, 'Goal JSON must contain x and y map coordinates.')
                        if (body.get('session_epoch') != node._runtime.epoch or
                                not node._runtime.valid_lease(actor, body.get('lease_token'))):
                            raise ApiError(409, 'Goal session/operator lease is invalid')
                        payload = node.submit_nav_goal(
                            body['x'], body['y'], body.get('yaw', 0.0), body.get('map_name'),
                            body.get('map_id'), body.get('preview_id'))
                    else:
                        if not isinstance(body.get('name'), str):
                            raise ApiError(400, 'Map-save JSON must contain a string field named name.')
                        payload = node.save_latest_map(body['name'])
                return self._json(200, payload)
            except ApiError as error:
                return self._json(error.status, {'ok': False, 'error': str(error)})
            except subprocess.TimeoutExpired:
                return self._json(503, {'ok': False, 'error': 'Robot ROS service command timed out.'})
            except Exception as error:
                node.get_logger().error(f'API request failed: {error}')
                return self._json(500, {'ok': False, 'error': 'Internal gateway error; see robot API log.'})
            finally:
                node._api_action_lock.release()

    return Handler


def spin_api(node):
    """Rebuild the executor around subscription changes on the spin thread.

    A Foxy wait-set iterator can retain ready handlers across spin_once calls,
    so merely moving destroy_subscription into a timer callback is not enough.
    Detaching and shutting down the old executor discards those references.
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            if node._session_rebind_pending():
                executor.remove_node(node)
                executor.shutdown()
                node._service_session_rebind()
                executor = SingleThreadedExecutor()
                executor.add_node(node)
    finally:
        executor.remove_node(node)
        executor.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RobotApi()
    spin_thread = threading.Thread(target=spin_api, args=(node,), daemon=True)
    spin_thread.start()
    server = ThreadingHTTPServer(
        (node.bind_address, node.port), make_handler(node))
    node._http_server_shutdown = server.shutdown
    node.get_logger().info(
        f'Lite3 mapping API listening on {node.bind_address}:{node.port}; '
        f'Nav2 goal opt-in={node.allow_nav2_goal_requests}; motion output isolated.')
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if node._goal_handle is not None:
            try:
                node.cancel_nav_goal()
            except ApiError as error:
                node.get_logger().warning(f'Could not cancel active Nav2 goal on shutdown: {error}')
        node.stop_navigation()
        node.stop_mapping()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
