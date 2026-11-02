import importlib.util
from pathlib import Path
import struct
import time


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/robot_vendor_interface_audit.py'
SPEC = importlib.util.spec_from_file_location('robot_vendor_interface_audit', SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def test_source_evidence_is_bounded_and_filters_credentials(tmp_path):
    source = tmp_path / 'ros2qnx.cpp'
    source.write_text('void CmdVelCallback();\npassword = "secret";\nint mode = 1;\n')
    evidence = AUDIT.file_evidence(source, include_lines=True)
    assert evidence['present'] is True
    assert evidence['matching_line_count'] == 2
    assert all('secret' not in line for _number, line in evidence['matching_lines'])


def test_sanitize_removes_nested_credentials():
    result = AUDIT.sanitize({'token': 'hidden', 'message': 'mode changed\npassword: hidden'})
    assert result['token'] == '[redacted]'
    assert result['message'] == 'mode changed\n[redacted line]'


def test_safety_record_requires_fresh_stable_sequence(tmp_path):
    path = tmp_path / 'safety_state'
    payload = struct.pack('<I???5xQ', 0x00010000, False, True, True, 1)
    path.write_bytes(struct.pack('<4sIQQI', b'EBL3', 1, 2, time.monotonic_ns(), len(payload)) + payload)
    assert AUDIT.safety_record(path)['valid_fresh'] is True
    path.write_bytes(struct.pack('<4sIQQI', b'EBL3', 1, 3, time.monotonic_ns(), len(payload)) + payload)
    assert AUDIT.safety_record(path)['valid_fresh'] is False
