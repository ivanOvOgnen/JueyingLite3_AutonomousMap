"""No ROS process or physical command is started by these relay policy tests."""

import importlib.util
from pathlib import Path
import struct
import sys
import time
from types import ModuleType, SimpleNamespace


PATH = (Path(__file__).resolve().parents[1] /
        'ros2_ws/src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/supervised_nav_relay.py')


def load(monkeypatch):
    for name in ('rclpy', 'rclpy.node', 'geometry_msgs', 'geometry_msgs.msg',
                 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv'):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules['rclpy.node'].Node = type('Node', (), {})
    for name in ('Twist',):
        setattr(sys.modules['geometry_msgs.msg'], name, SimpleNamespace)
    for name in ('Bool', 'String'):
        setattr(sys.modules['std_msgs.msg'], name, SimpleNamespace)
    sys.modules['std_srvs.srv'].SetBool = SimpleNamespace
    spec = importlib.util.spec_from_file_location('supervised_nav_relay_test', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_trial_needs_matching_active_goal_live_lease_and_stop(monkeypatch):
    relay = load(monkeypatch)
    context = {'ready': True, 'operator_lease_valid': True,
               'goal_active': True, 'active_goal_id': 'abc'}
    assert relay.eligible(context, 0.1, True, True, 'abc')
    assert not relay.eligible(context, 0.6, True, True, 'abc')
    assert not relay.eligible(context, 0.1, True, False, 'abc')
    assert not relay.eligible(context, 0.1, True, True, 'different')
    context['operator_lease_valid'] = False
    assert not relay.eligible(context, 0.1, True, True, 'abc')


def test_trial_rejects_stale_or_stopped_vendor_observation(monkeypatch, tmp_path):
    relay = load(monkeypatch)
    path = tmp_path/'safety'
    payload = struct.pack('<I???5xQ', 0x00010000, False, True, True, 0)
    path.write_bytes(struct.pack('<4sIQQI', b'EBL3', 1, 2,
                                 time.monotonic_ns(), len(payload)) + payload)
    assert relay.safety_clear(path)
    stopped = struct.pack('<I???5xQ', 0x00010000, True, True, True, 0)
    path.write_bytes(struct.pack('<4sIQQI', b'EBL3', 1, 4,
                                 time.monotonic_ns(), len(stopped)) + stopped)
    assert not relay.safety_clear(path)
    path.write_bytes(struct.pack('<4sIQQI', b'EBL3', 1, 6,
                                 time.monotonic_ns()-1_000_000_000, len(payload)) + payload)
    assert not relay.safety_clear(path)
