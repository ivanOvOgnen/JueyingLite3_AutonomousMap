"""Inert ROS imports for lifecycle tests; no ROS context or hardware is used."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock
import threading

import pytest


PACKAGE = Path(__file__).resolve().parents[1] / 'ros2_ws/src/lite3_odom_tf_adapter'


@pytest.fixture
def api_module(monkeypatch):
    def stub(name, **members):
        module = ModuleType(name)
        module.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, module)

    def forbidden(*args, **kwargs):
        raise AssertionError('A lifecycle unit test attempted to start ROS or a process')

    stub('rclpy', init=forbidden, spin=forbidden)
    stub('rclpy.node', Node=type('Node', (), {}))
    stub('rclpy.executors', SingleThreadedExecutor=forbidden)
    stub('rclpy.time', Time=SimpleNamespace)
    stub('rclpy.action', ActionClient=forbidden)
    stub('rclpy.type_support', check_for_type_support=forbidden)
    stub('rclpy.qos', DurabilityPolicy=SimpleNamespace, HistoryPolicy=SimpleNamespace,
         QoSProfile=SimpleNamespace, ReliabilityPolicy=SimpleNamespace,
         qos_profile_sensor_data=object())
    for package, messages in {
        'geometry_msgs': ['PoseWithCovarianceStamped'],
        'nav_msgs': ['OccupancyGrid', 'Odometry'],
        'std_msgs': ['Bool', 'String'],
        'sensor_msgs': ['Image', 'LaserScan', 'PointCloud2'],
    }.items():
        stub(package)
        stub(package + '.msg', **{name: SimpleNamespace for name in messages})
    stub('nav2_msgs')
    stub('nav2_msgs.action', NavigateToPose=SimpleNamespace, ComputePathToPose=SimpleNamespace)
    stub('std_srvs')
    stub('std_srvs.srv', Empty=SimpleNamespace, SetBool=SimpleNamespace(Request=SimpleNamespace))
    stub('tf2_ros', Buffer=forbidden, TransformListener=forbidden,
         TransformException=type('TransformException', (Exception,), {}))
    monkeypatch.syspath_prepend(str(PACKAGE))
    spec = importlib.util.spec_from_file_location(
        '_lite3_robot_api_unit_tests', PACKAGE / 'lite3_odom_tf_adapter/robot_api.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(module.subprocess, 'run', forbidden)
    # killpg is POSIX-only; create the test double on Windows as well so the
    # lifecycle suite can run without weakening the production code.
    monkeypatch.setattr(module.os, 'killpg', forbidden, raising=False)
    return module


@pytest.fixture
def api(api_module, tmp_path):
    node = api_module.RobotApi.__new__(api_module.RobotApi)
    node._lock = threading.RLock()
    node._goal_lock = threading.Lock()
    node._localization_lock = threading.Lock()
    node._api_action_lock = threading.Lock()
    node._workspace = tmp_path
    (tmp_path / 'logs').mkdir()
    node._bridge_config = tmp_path / 'bridge.yaml'
    node._bridge_config.write_text(
        'topics:\n  - topic: /leg_odom2\n    type: nav_msgs/msg/Odometry\n    queue_size: 10\n')
    node._bridge_process = None
    node._bridge_external = False
    node._bridge_external_identity = None
    node._bridge_param_owned = False
    node._bridge_log = None
    node._mapping_process = None
    node._mapping_log = None
    node._mapping_paused = False
    node._navigation_process = None
    node._navigation_log = None
    node._navigation_map_name = None
    node._navigation_started_at = 0.0
    node._stitched_pointcloud = bytearray()
    node._stitched_point_count = 0
    node._last_stitch_time = 0.0
    from lite3_odom_tf_adapter.runtime_state import RuntimeState
    node._runtime = RuntimeState(lease_seconds=60)
    node._runtime.acquire('inert-test-owner')
    node._session_subscriptions = []
    node._bind_session_topics = Mock()
    node._request_session_rebind = Mock()
    node._navigation_identity = None
    node._navigation_map_reviewed = False
    node._navigation_map_trial_eligible = False
    node._expected_map_signature = None
    node._map_condition = threading.Condition(node._lock)
    node._localization_generation = 0
    node._localization_not_before_ns = 0
    node._stationary_since = None
    node._operating_region = None
    node._map_sequence = 0
    node._path_preview = None
    node._path_request_id = None
    node._path_action = None
    node._goal_feedback = None
    node._observation_health = {}
    node._stage_health = {}
    node._sensor_frames = {}
    node._costmaps = {}
    node._goal_clearance_radius = 0.30
    node._costmap_valid = {'local_costmap': False, 'global_costmap': False}
    node._map = None
    node._last_rx = {}
    node._amcl_pose = None
    node._goal_handle = None
    node._goal_state = 'idle'
    node._goal_detail = ''
    node._goal_id = None
    node._goal_requests = {}
    node._autonomy_stop_latched = False
    node.allow_nav2_goal_requests = True
    node._stop_generation = 0
    node._stop_cleanup_pending = False
    node._stop_cleanup_error = None
    node._stop_cleanup_thread = None
    node._stop_disable_results = {}
    node._autonomy_disable_clients = {}
    node._readiness_pub = Mock()
    node._context_pub = Mock()
    node._bridge_profile = 'telemetry'
    node._software_id = None
    node._software_hash_at = 0.0
    node._shutdown_requested = False
    node.api_token = 'inert-unit-test-key'
    node.maps_dir = tmp_path / 'maps'
    node.maps_dir.mkdir()
    logger = Mock()
    node.get_logger = lambda: logger
    node.get_node_names_and_namespaces = lambda: []
    node._component = lambda *_args: {'state': 'ready'}
    yield node
    if node._api_action_lock.locked():
        node._api_action_lock.release()
    if node._stop_cleanup_thread:
        node._stop_cleanup_thread.join(timeout=2.0)
        assert not node._stop_cleanup_thread.is_alive(), 'STOP worker did not finish'
    for name in ('_bridge_log', '_mapping_log', '_navigation_log'):
        stream = getattr(node, name)
        if stream and not stream.closed:
            stream.close()


@pytest.fixture
def command_nodes(api_module, monkeypatch):
    """Construct real command-node classes with inert ROS publishers/timers."""
    class InertTwist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class InertNode:
        def __init__(self, _name):
            self.parameters = {}
            self.logger = Mock()
            self._context = SimpleNamespace(handle=object())

        def declare_parameter(self, name, value):
            self.parameters[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, *_args):
            return Mock()

        def create_subscription(self, *_args):
            return Mock()

        def destroy_subscription(self, *_args):
            return True

        def create_service(self, *_args):
            return Mock()

        def create_timer(self, *_args):
            assert hasattr(self._context, 'handle'), 'Node._context was overwritten by application state'
            return Mock()

        def get_logger(self):
            return self.logger

    monkeypatch.setattr(sys.modules['geometry_msgs.msg'], 'Twist', InertTwist, raising=False)
    monkeypatch.setattr(sys.modules['std_msgs.msg'], 'String', SimpleNamespace, raising=False)
    monkeypatch.setattr(sys.modules['rclpy.node'], 'Node', InertNode)
    monkeypatch.setattr(sys.modules['rclpy.qos'], 'ReliabilityPolicy',
                        SimpleNamespace(RELIABLE='reliable'))
    monkeypatch.setattr(sys.modules['rclpy.qos'], 'DurabilityPolicy',
                        SimpleNamespace(VOLATILE='volatile'))
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr(api_module.time, 'monotonic', lambda: clock.now)
    classes = {}
    for kind, filename, classname in (
        ('gate', 'cmd_vel_gate', 'CmdVelGate'),
        ('arbiter', 'motion_arbiter', 'MotionArbiter'),
        ('relay', 'vendor_cmd_vel_bridge', 'VendorCmdVelBridge'),
        ('trial', 'supervised_nav_relay', 'SupervisedNavRelay'),
    ):
        spec = importlib.util.spec_from_file_location(
            '_lite3_' + filename + '_unit_tests', PACKAGE / ('lite3_odom_tf_adapter/' + filename + '.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        classes[kind] = getattr(module, classname)

    def build(kind):
        node = classes[kind]()
        if kind == 'arbiter':
            node._read_safety_record = Mock(return_value=(True, 'inert clear safety record'))
        elif kind == 'relay':
            node._safety_ok = Mock(return_value=(True, 'inert clear safety record'))
        return node

    return SimpleNamespace(build=build, clock=clock, Twist=InertTwist)


@pytest.fixture
def saver(api_module):
    spec = importlib.util.spec_from_file_location(
        '_shared_save_map_tests', PACKAGE / 'lite3_odom_tf_adapter/save_map.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def grid():
    return SimpleNamespace(info=SimpleNamespace(width=3,height=2,resolution=0.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.,z=0.),
                              orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))),
        data=[0,0,0,0,0,0],header=SimpleNamespace(frame_id='map',stamp=SimpleNamespace(sec=42,nanosec=3)))
