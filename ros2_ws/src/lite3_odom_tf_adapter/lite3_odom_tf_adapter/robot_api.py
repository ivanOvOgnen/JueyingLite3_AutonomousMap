#!/usr/bin/env python3
"""HTTP gateway for Lite3 mapping and gated Nav2 planner requests."""

import json
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
from urllib.parse import unquote, urlparse
import yaml

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
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


STALE_AFTER = 2.0
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
        self._last_rx = {}
        self._map = None
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
        # Fail closed on process startup. Starting localization/Nav2 is the
        # explicit software re-arm; pressing STOP latches goals off again.
        self._autonomy_stop_latched = True
        self._nav_action = None
        self._nav_action_error = None
        self._global_localization_client = self.create_client(
            Empty, '/reinitialize_global_localization')
        self._autonomy_enable_client = self.create_client(
            SetBool, '/autonomy/set_enabled')
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
        self._bridge_param_owned = False
        self._mapping_log = None
        self._bridge_log = None
        self._workspace = Path.home() / 'lite3_autonomy_ws'
        self._bridge_config = (
            self._workspace / 'install' / 'lite3_odom_tf_adapter' / 'share' /
            'lite3_odom_tf_adapter' / 'config' / 'mapping_ros1_bridge.yaml')

        self.create_subscription(
            Image, '/camera/depth/image_rect_raw',
            lambda _msg: self._mark_rx('camera'), qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points',
            self._on_pointcloud, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/leg_odom2',
            lambda _msg: self._mark_rx('odometry'), 10)
        self.create_subscription(
            LaserScan, '/scan',
            lambda _msg: self._mark_rx('scan'), qos_profile_sensor_data)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_localization, amcl_qos)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            lambda message: self._on_costmap('local_costmap', message), 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            lambda message: self._on_costmap('global_costmap', message), 10)

    def _mark_rx(self, key):
        with self._lock:
            self._last_rx[key] = time.monotonic()

    def _on_map(self, message):
        with self._lock:
            self._map = message
            self._last_rx['map'] = time.monotonic()

    def _on_costmap(self, name, message):
        # Nav2's public `*/costmap` topics are OccupancyGrid. The
        # nav2_msgs/Costmap type is published on the separate `*/costmap_raw`
        # topics; using it here silently prevents a DDS endpoint match.
        info = message.info
        size_x = int(info.width)
        size_y = int(info.height)
        valid = (
            size_x > 0 and size_y > 0 and
            math.isfinite(float(info.resolution)) and float(info.resolution) > 0.0 and
            len(message.data) == size_x * size_y)
        with self._lock:
            self._costmap_valid[name] = valid
            if valid:
                self._last_rx[name] = time.monotonic()

    def _tf_available(self, target_frame, source_frame):
        try:
            return self._tf_buffer.can_transform(target_frame, source_frame, Time())
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

    def _on_localization(self, message):
        pose = message.pose.pose
        covariance = message.pose.covariance
        x, y = float(pose.position.x), float(pose.position.y)
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cov_x, cov_y, cov_yaw = (
            float(covariance[0]), float(covariance[7]), float(covariance[35]))
        finite = all(math.isfinite(value) for value in
                     (x, y, yaw, cov_x, cov_y, cov_yaw))
        low_uncertainty = finite and 0.0 <= cov_x <= 0.25 and \
            0.0 <= cov_y <= 0.25 and 0.0 <= cov_yaw <= 0.30
        with self._lock:
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
        if not all(components[name]['state'] == 'ready' for name in
                   ('map', 'camera', 'scan', 'odometry')):
            with self._localization_lock:
                self._localization_refresh_active = False
                self._localization_reason = (
                    'Localization refresh stopped because map, camera, scan, or odometry became stale.')
            return
        with self._goal_lock:
            goal_active = self._goal_state in (
                'accepted', 'executing', 'canceling', 'requesting')
        if goal_active or not self._localization_refresh_client.service_is_ready():
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

        def refresh_done(done_future):
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
        fresh_limit = 10.0 if name == 'map' else STALE_AFTER
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
        navigation = self.status()['components']['navigation']
        message = Bool()
        message.data = bool(
            navigation['planner_ready'] and
            navigation['goal_requests_enabled'] and
            not navigation['stop_latched'])
        self._readiness_pub.publish(message)

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
                'camera', 'odometry', 'scan', 'map', 'pointcloud')}
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
            ready = costmap_valid[key] and age is not None and age <= STALE_AFTER
            costmaps[key] = {
                'state': 'ready' if ready else 'not_ready',
                'age_seconds': None if age is None else round(age, 2),
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
        localization_ready = components['localization']['state'] == 'ready'
        map_ready = components['map']['state'] == 'ready' and grid is not None
        sensor_ready = all(components[k]['state'] == 'ready' for k in ('camera', 'odometry', 'scan'))
        known_fraction = (
            sum(1 for value in grid.data if value >= 0) / len(grid.data)
            if grid is not None and grid.data else 0.0)
        map_has_observed_cells = known_fraction > 0.0
        planner_ready = (
            action_ready and localization_ready and map_ready and sensor_ready and
            map_has_observed_cells and not mapping_running and
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
        elif not localization_ready or not map_ready or not sensor_ready:
            nav_reason = 'Fresh map, localization, camera, scan, and odometry are required.'
        elif components['transforms']['state'] != 'ready':
            nav_reason = 'Required map, odometry, scan, or depth-camera TF is missing.'
        elif components['costmaps']['state'] != 'ready':
            nav_reason = 'Fresh, valid local and global Nav2 costmaps are required.'
        elif not self.allow_nav2_goal_requests:
            nav_reason = 'Navigation checks pass, but planner goal requests remain disabled by default.'
        else:
            nav_reason = 'Navigation checks pass. The robot motion output remains disconnected.'
        if self._autonomy_stop_latched:
            nav_reason += ' Software STOP is latched; restart localization/Nav2 explicitly before another planner request.'
        components['navigation'] = {
            'state': 'ready' if planner_ready and self.allow_nav2_goal_requests else 'not_ready',
            'planner_ready': planner_ready,
            'goal_requests_opt_in_available': planner_ready,
            'goal_requests_enabled': self.allow_nav2_goal_requests,
            'action_server_ready': action_ready,
            'active_map_name': navigation_map_name,
            'map_known_fraction': round(known_fraction, 3),
            'active_goal_state': self._goal_state,
            'active_goal_detail': self._goal_detail,
            'stop_latched': self._autonomy_stop_latched,
            'reason': nav_reason,
        }
        components['safety_stop'] = {
            'state': 'latched' if self._autonomy_stop_latched else 'not_latched',
            'physical_estop_verified': False,
            'reason': 'Software stop only; the vendor motor path has no verified independent stop/interlock.',
        }
        components['motion_output'] = {
            'state': 'not_ready',
            'reason': 'Velocity gate output is isolated from the robot command transport.',
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
            'control_mode': 'manual',
            'api_auth_required': True,
            'api_paired': self.api_token is not None,
            'components': components,
            'map': map_info,
            'server_monotonic_seconds': round(now, 2),
        }

    def submit_nav_goal(self, x, y, yaw=0.0, map_name=None):
        if not self.allow_nav2_goal_requests:
            raise ApiError(503, 'Nav2 goal requests are disabled; API default is planner-off.')
        if self._autonomy_stop_latched:
            raise ApiError(409, 'Autonomy is stop-latched. Explicitly restart localization/Nav2 to re-arm planner requests.')
        if not isinstance(map_name, str) or map_name != self._navigation_map_name:
            raise ApiError(409, 'Load and click the same saved robot map that localization/Nav2 is using.')
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
        self._validate_goal_cell(grid, float(x), float(y))
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
        with self._goal_lock:
            if self._goal_state in ('accepted', 'executing', 'canceling'):
                raise ApiError(409, 'A Nav2 goal is already active; cancel it before sending another.')
            self._goal_state = 'requesting'
            self._goal_detail = f'Planner goal requested at ({x:.2f}, {y:.2f}) m.'
        future = self._nav_action.send_goal_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(3.0):
            with self._goal_lock:
                self._goal_state = 'unknown'
                self._goal_detail = 'Timed out waiting for Nav2 goal acceptance.'
            raise ApiError(504, 'Timed out waiting for Nav2 to accept the goal.')
        try:
            handle = future.result()
        except Exception as error:
            with self._goal_lock:
                self._goal_state = 'failed'
                self._goal_detail = 'Nav2 goal request failed.'
            raise ApiError(503, f'Nav2 goal request failed: {error}')
        if not handle.accepted:
            with self._goal_lock:
                self._goal_state = 'rejected'
                self._goal_detail = 'Nav2 rejected the goal.'
            raise ApiError(409, 'Nav2 rejected the goal.')
        with self._goal_lock:
            self._goal_handle = handle
            self._goal_state = 'accepted'
            self._goal_detail = f'Nav2 accepted planner goal at ({x:.2f}, {y:.2f}) m.'
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)
        return {'ok': True, 'message': self._goal_detail, 'planner_only': True}

    def set_nav_goal_requests(self, enabled, map_name=None):
        if not isinstance(enabled, bool):
            raise ApiError(400, 'enabled must be a boolean.')
        if enabled:
            components = self.status()['components']
            if self._autonomy_stop_latched:
                raise ApiError(409, 'Software STOP is latched. Restart localization/Nav2 before opting in.')
            if not isinstance(map_name, str) or map_name != self._navigation_map_name:
                raise ApiError(409, 'Load the same saved robot map used by Localization/Nav2 before opting in.')
            if not components['navigation']['planner_ready']:
                raise ApiError(503, components['navigation']['reason'])
        self.allow_nav2_goal_requests = enabled
        return {
            'ok': True,
            'goal_requests_enabled': enabled,
            'message': (
                'Planner goal requests enabled; velocity output remains isolated.'
                if enabled else 'Planner goal requests disabled.'),
        }

    def set_initial_pose(self, x, y, yaw=0.0):
        values = (x, y, yaw)
        if not all(not isinstance(value, bool) and isinstance(value, (int, float)) and
                   math.isfinite(value) for value in values):
            raise ApiError(400, 'Initial pose x, y, and yaw must be finite numbers.')
        components = self.status()['components']
        if not components['navigation_stack']['process_running']:
            raise ApiError(409, 'Start localization/Nav2 before setting its AMCL initial pose.')
        if components['map']['state'] != 'ready':
            raise ApiError(409, 'Wait for the selected robot map to arrive before setting the initial pose.')
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
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.20
        with self._lock:
            self._amcl_pose = None
            self._last_rx['localization'] = None
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
        return {'ok': True, 'message': f'AMCL initial pose published at ({x:.2f}, {y:.2f}) m.',
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
            if self._goal_state in ('accepted', 'executing', 'canceling', 'requesting'):
                raise ApiError(409, 'Cancel the active Nav2 goal before relocalizing.')
        if not self._global_localization_client.service_is_ready():
            raise ApiError(503, 'AMCL global-localization service is not ready.')
        if not self._localization_refresh_client.service_is_ready():
            raise ApiError(503, 'AMCL no-motion update service is not ready; localization was not started.')
        with self._lock:
            self._amcl_pose = None
            self._last_rx['localization'] = None
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
            raise ApiError(504, 'AMCL global-localization request timed out.')
        try:
            future.result()
        except Exception as error:
            raise ApiError(503, f'AMCL global localization failed: {error}')
        with self._localization_lock:
            self._localization_refresh_started_at = time.monotonic()
            self._localization_refresh_requested_at = 0.0
            self._localization_refresh_active = True
        return {
            'ok': True,
            'message': 'AMCL is searching the saved map with no-motion scan updates. Wait for a stable estimate and verify the marker against the robot before selecting a goal.',
            'motion_command_sent': False,
        }

    def _validate_goal_cell(self, grid, x, y):
        origin = grid.info.origin
        q = origin.orientation
        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        dx, dy = x - origin.position.x, y - origin.position.y
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        resolution = float(grid.info.resolution)
        cx, cy = int(local_x / resolution), int(local_y / resolution)
        width, height = int(grid.info.width), int(grid.info.height)
        if not (0 <= cx < width and 0 <= cy < height):
            raise ApiError(400, 'Goal lies outside the current map.')
        radius = max(1, int(math.ceil(0.30 / resolution)))
        if cx < radius or cy < radius or cx >= width - radius or cy >= height - radius:
            raise ApiError(400, 'Goal lacks a full 0.30 m clearance inside the map boundary.')
        for row in range(max(0, cy - radius), min(height, cy + radius + 1)):
            for col in range(max(0, cx - radius), min(width, cx + radius + 1)):
                if (col - cx) ** 2 + (row - cy) ** 2 > radius ** 2:
                    continue
                value = int(grid.data[row * width + col])
                if value < 0 or value > 15:
                    raise ApiError(400, 'Goal is not in a known-free map region with 0.30 m clearance.')

    def _on_goal_result(self, future):
        try:
            wrapped_result = future.result()
            state = f'finished_{wrapped_result.status}'
            detail = f'Nav2 action finished with status {wrapped_result.status}.'
        except Exception as error:
            state = 'failed'
            detail = f'Nav2 goal result unavailable: {error}'
        with self._goal_lock:
            self._goal_state = state
            self._goal_detail = detail
            self._goal_handle = None

    def cancel_nav_goal(self):
        with self._goal_lock:
            handle = self._goal_handle
            if handle is None:
                raise ApiError(409, 'There is no accepted Nav2 goal to cancel.')
            self._goal_state = 'canceling'
            self._goal_detail = 'Cancel requested for Nav2 planner goal.'
        future = handle.cancel_goal_async()
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(3.0):
            raise ApiError(504, 'Timed out waiting for Nav2 goal cancellation.')
        try:
            cancellation = future.result()
        except Exception as error:
            raise ApiError(503, f'Nav2 cancellation request failed: {error}')
        if not cancellation.goals_canceling:
            raise ApiError(409, 'Nav2 did not accept the cancellation request.')
        with self._goal_lock:
            self._goal_state = 'canceled'
            self._goal_detail = 'Nav2 goal canceled.'
            self._goal_handle = None
        return {'ok': True, 'message': self._goal_detail, 'planner_only': True}

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
        with self._lock:
            if self._mapping_process is not None and self._mapping_process.poll() is None:
                raise ApiError(409, 'Mapping is already running.')
            if self._navigation_process is not None and self._navigation_process.poll() is None:
                raise ApiError(409, 'Stop localization/navigation before starting mapping.')
            if self._bridge_process is not None and self._bridge_process.poll() is None:
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

            if not self._adopt_or_start_bridge():
                raise ApiError(503, 'Could not establish the project odometry-only bridge.')

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if self._bridge_process.poll() is not None:
                self.stop_bridge()
                raise ApiError(503, 'The odometry-only ROS bridge exited; see robot_api_bridge.log.')
            if self._component('odometry', time.monotonic())['state'] == 'ready':
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
        self.get_logger().warning('Mapping launched; it does not publish motion commands.')
        return {'ok': True, 'message': 'Mapping started. Guide the robot with the RC in manual mode.'}

    def start_navigation(self, map_name):
        if not isinstance(map_name, str) or not MAP_NAME.fullmatch(map_name):
            raise ApiError(400, 'Select a saved robot map using its map name.')
        map_yaml = self.maps_dir / f'{map_name}.yaml'
        map_image = self.maps_dir / f'{map_name}.pgm'
        if not map_yaml.is_file() or not map_image.is_file():
            raise ApiError(404, 'Matching saved map YAML and PGM were not found on the robot.')
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
        self._start_bridge()
        log_dir = self._workspace / 'logs'
        log_dir.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self._navigation_log = (log_dir / 'robot_api_navigation.log').open('ab')
            try:
                self._navigation_started_at = time.monotonic()
                self._navigation_process = subprocess.Popen(
                    ['ros2', 'launch', 'lite3_odom_tf_adapter', 'navigation.launch.py',
                     f'map:={map_yaml}'], cwd=str(self._workspace), env=os.environ.copy(),
                    stdout=self._navigation_log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                self._navigation_map_name = map_name
                self._autonomy_stop_latched = False
            except Exception:
                self._navigation_log.close()
                self._navigation_log = None
                self.stop_bridge()
                raise
        return {'ok': True, 'message': f'Localization/Nav2 starting with robot map {map_name}; output remains isolated.'}

    def stop_navigation(self):
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
        return {'ok': True, 'message': 'Localization/Nav2 stopped; output remained isolated.'}

    def stop_autonomy(self):
        """Latch planner requests off, disable the project gate, and stop Nav2.

        This only controls the project planner/gate. It cannot stop a vendor
        controller or serve as a physical emergency stop.
        """
        self._autonomy_stop_latched = True
        gate_disabled = False
        with self._goal_lock:
            goal_active = self._goal_handle is not None

        # Disable the software gate first so it emits zero if it was enabled.
        # The gate output is currently isolated from the robot transport.
        if self._autonomy_enable_client.service_is_ready():
            future = self._autonomy_enable_client.call_async(SetBool.Request(data=False))
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if completed.wait(1.0):
                try:
                    response = future.result()
                    gate_disabled = bool(response.success)
                except Exception as error:
                    self.get_logger().error(f'Autonomy gate disable failed: {error}')

        cancel_result = 'no accepted goal'
        if goal_active:
            try:
                self.cancel_nav_goal()
                cancel_result = 'active goal cancellation accepted'
            except ApiError as error:
                # Stopping the whole Nav2 launch below removes its action and
                # gate processes if cancellation is unavailable or times out.
                cancel_result = f'goal cancel did not confirm ({error})'

        self.stop_navigation()
        with self._goal_lock:
            self._goal_handle = None
            self._goal_state = 'stopped'
            self._goal_detail = 'Software autonomy stop latched; Nav2 is stopped.'
        return {
            'ok': True,
            'message': (
                f'Software stop latched; Nav2 stopped; {cancel_result}. '
                f'Gate disable service response: {"confirmed" if gate_disabled else "unavailable/unconfirmed"}. '
                'This is not a physical emergency stop; use the vendor controller or hardware stop.'
            ),
            'stop_latched': True,
            'gate_disable_confirmed': gate_disabled,
            'physical_estop': False,
        }

    def _start_bridge(self):
        with self._lock:
            if self._bridge_process is not None and self._bridge_process.poll() is None:
                return
            if not self._bridge_config.is_file():
                raise ApiError(500, f'Project bridge config missing: {self._bridge_config}')
            if not self._adopt_or_start_bridge():
                raise ApiError(503, 'Could not establish the project odometry-only bridge.')
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if self._bridge_process is not None and self._bridge_process.poll() is not None:
                self.stop_bridge()
                raise ApiError(503, 'The odometry-only ROS bridge exited; see robot_api_bridge.log.')
            if self._component('odometry', time.monotonic())['state'] == 'ready':
                return
            time.sleep(0.1)
        self.stop_bridge()
        raise ApiError(503, 'No fresh /leg_odom2 arrived through the ROS bridge.')

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
                expected_topics = yaml.safe_load(
                    self._bridge_config.read_text(encoding='utf-8'))['topics']
                active_topics = yaml.safe_load(occupied.stdout)
            except (OSError, KeyError, TypeError, yaml.YAMLError):
                return False
            if len(pids) != 1 or active_topics != expected_topics:
                return False
            self._bridge_external = True
            return True

        loaded = self._run_noetic_command(f"rosparam load '{self._bridge_config}'")
        if loaded.returncode != 0:
            return False
        self._bridge_param_owned = True
        self._bridge_external = False
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
        return {'ok': True, 'message': 'Mapping stopped; the robot remains in manual mode.'}

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
            self._run_noetic_command('rosparam delete /topics', allow_failure=True)
            self._bridge_param_owned = False

    def prepare_for_poweroff(self):
        """Stop project-owned ROS launches, then close the project HTTP API."""
        with self._goal_lock:
            goal_active = self._goal_handle is not None
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

    def save_latest_map(self, name):
        if not MAP_NAME.fullmatch(name):
            raise ApiError(400, 'Map name must use 1-64 letters, digits, underscore, or hyphen.')
        with self._lock:
            message = self._map
        if message is None:
            raise ApiError(409, 'No map has been received yet.')
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        prefix = self.maps_dir / name
        if prefix.with_suffix('.pgm').exists() or prefix.with_suffix('.yaml').exists():
            raise ApiError(409, 'That map name already exists; choose a new name to preserve earlier maps.')
        image, yaml_file, manifest = save_grid(message, prefix)
        return {
            'ok': True,
            'name': name,
            'image_url': f'/maps/{image.name}',
            'yaml_url': f'/maps/{yaml_file.name}',
            'manifest_url': f'/maps/{manifest.name}',
            'width': message.info.width,
            'height': message.info.height,
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
                items.append({
                    'name': yaml_file.stem,
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
                '/api/v1/navigation/goal-requests',
                '/api/v1/navigation/cancel',
                '/api/v1/localization/initial_pose',
                '/api/v1/localization/global',
                '/api/v1/diagnostics/preflight',
                '/api/v1/shutdown/prepare',
                '/api/v1/pair',
            ):
                return self._json(404, {'ok': False, 'error': 'Unknown API endpoint.'})
            if node._shutdown_requested:
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
            if not node._api_action_lock.acquire(blocking=False):
                return self._json(409, {'ok': False, 'error': 'Another robot control action is still running; retry after it finishes.'})
            try:
                if node._shutdown_requested:
                    raise ApiError(503, 'Project API shutdown is in progress.')
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
                    payload = node.set_nav_goal_requests(body['enabled'], body.get('map_name'))
                elif path == '/api/v1/safety/stop':
                    payload = node.stop_autonomy()
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
                        payload = node.submit_nav_goal(
                            body['x'], body['y'], body.get('yaw', 0.0), body.get('map_name'))
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


def main(args=None):
    rclpy.init(args=args)
    node = RobotApi()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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
