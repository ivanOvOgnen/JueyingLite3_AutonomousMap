import importlib.util
from pathlib import Path
import struct
import time


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/robot_supervised_cmd_vel_trial.py'
SPEC = importlib.util.spec_from_file_location('robot_supervised_cmd_vel_trial', SCRIPT)
TRIAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRIAL)


def test_safety_state_rejects_stale_and_torn_records(tmp_path):
    path = tmp_path / 'relay'
    payload = struct.pack('<I???5xQ', 0x00010000, False, True, True, 0)
    path.write_bytes(struct.pack('<4sIQQI', b'EBL3', 1, 2, time.monotonic_ns(), len(payload)) + payload)
    assert TRIAL.safety_state(path)['valid'] is True
    path.write_bytes(struct.pack('<4sIQQI', b'EBL3', 1, 3, time.monotonic_ns(), len(payload)) + payload)
    assert TRIAL.safety_state(path)['valid'] is False
    path.write_bytes(struct.pack('<4sIQQI', b'EBL3', 1, 4, time.monotonic_ns()-1_000_000_000, len(payload)) + payload)
    assert TRIAL.safety_state(path)['valid'] is False


def test_preflight_rejects_competing_command_publisher(monkeypatch):
    sequence = iter(({'valid': True, 'sequence': 2, 'stop': False,
                      'observed_fresh': True, 'axes_zero': True},
                     {'valid': True, 'sequence': 4, 'stop': False,
                      'observed_fresh': True, 'axes_zero': True}))
    monkeypatch.setattr(TRIAL, 'safety_state', lambda: next(sequence))
    monkeypatch.setattr(TRIAL, 'graph_state', lambda: {
        'cmd_vel_publishers': ['/other'], 'cmd_vel_subscribers': ['/ros2qnx'],
        'corrected_publishers': ['/safety_node']})
    monkeypatch.setattr(TRIAL.time, 'sleep', lambda _: None)
    result = TRIAL.preflight()
    assert result['ready'] is False
    assert any('Another ROS 1 /cmd_vel publisher' in reason for reason in result['reasons'])
