"""Synthetic capture corruption cases; never used as robot acceptance data."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    '_capture_validator', Path(__file__).resolve().parents[1] / 'scripts/validate_mapping_capture.py')
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


@pytest.fixture
def capture(tmp_path):
    (tmp_path / 'depth').mkdir()
    events = []
    rotation = {'x': 0, 'y': 0, 'z': 0, 'w': 1}
    xyz = {'x': 0, 'y': 0, 'z': 0}
    pose = {'position': xyz, 'orientation': rotation}

    def add(kind, stamp, frame='map', **fields):
        events.append({'kind': kind, 'received_unix_ns': stamp + 1_000_000,
                       'header': {'stamp_ns': stamp, 'frame_id': frame}, **fields})

    for index, stamp in enumerate((1_000_000_000, 2_000_000_000)):
        payload = bytes([1, 2, 3, 4])
        name = f'depth/{index:08d}.bin'
        (tmp_path / name).write_bytes(payload)
        add('depth', stamp, 'depth', file=name, bytes=4, sha256=hashlib.sha256(payload).hexdigest(),
            width=2, height=1, encoding='16UC1', step=4)
        add('camera_info', stamp, 'depth', width=2, height=1, k=[1, 0, 1, 0, 1, 0, 0, 0, 1])
        add('scan', stamp, 'scan', angle_increment=0.1, range_min=0.1, range_max=5,
            ranges=[1, 'Infinity', 'NaN'])
        for kind in ('vendor_odom', 'normalized_odom'):
            add(kind, stamp, pose=pose)
        add('tf', stamp, transforms=[
            {'parent': parent, 'child': child, 'stamp_ns': stamp, 'translation': xyz, 'rotation': rotation}
            for parent, child in (('map', 'odom'), ('odom', 'base_link'))])
    add('tf_static', 1_000_000_000, transforms=[
        {'parent': 'base_link', 'child': child, 'stamp_ns': 0, 'translation': xyz, 'rotation': rotation}
        for child in ('depth', 'scan')])
    add('map', 1_000_000_000, width=2, height=1, resolution=0.05, origin=pose, data=[0, -1])
    metadata = {'format': VALIDATOR.FORMAT}
    summary = {'format': VALIDATOR.FORMAT, 'writer_finished': True, 'writer_error': None,
               'queued_depth': 0, 'saved_depth_bytes': 8, 'reserved_depth_bytes': 8, 'dropped': {}}

    def write():
        summary['counts'] = dict(Counter(item['kind'] for item in events))
        (tmp_path / 'metadata.json').write_text(json.dumps(metadata))
        (tmp_path / 'summary.json').write_text(json.dumps(summary))
        (tmp_path / 'events.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in events))

    def validate(**kwargs):
        write()
        return VALIDATOR.validate_capture(tmp_path, min_duration=1, min_samples=2, max_gap=1.1, **kwargs)

    return tmp_path, events, summary, validate


def codes(report):
    return {failure['code'] for failure in report['failures']}


def test_complete_synthetic_capture_passes_only_offline_scope(capture):
    _, _, _, validate = capture
    report = validate()
    assert report['result'] == 'pass', report['failures']
    assert report['robot_acceptance'] == 'not_established'
    assert report['tf_bracket_coverage']['depth']['fraction'] == 1
    assert report['normalized_odom_max_displacement_from_first_metres'] == 0
    assert report['unverified']


def test_payload_corruption(capture):
    root, _, _, validate = capture
    (root / 'depth/00000000.bin').write_bytes(b'fail')
    assert 'depth_payload_hash' in codes(validate())


@pytest.mark.parametrize('field,value,code', [
    ('writer_finished', False, 'writer_unfinished'),
    ('writer_error', 'disk failed', 'writer_error'),
    ('queued_depth', 1, 'queued_depth'),
    ('saved_depth_bytes', 0, 'saved_bytes'),
    ('reserved_depth_bytes', 12, 'reserved_bytes'),
    ('dropped', {'depth_writer_queue': 1}, 'capture_drops'),
])
def test_summary_failure_is_never_a_pass(capture, field, value, code):
    _, _, summary, validate = capture
    summary[field] = value
    assert code in codes(validate())


def test_missing_stream_and_exact_intrinsic_pairing(capture):
    _, events, _, validate = capture
    events[:] = [row for row in events if row['kind'] != 'camera_info']
    result = codes(validate())
    assert {'missing_stream', 'unpaired_depth'} <= result


def test_nearest_tf_is_insufficient(capture):
    _, events, _, validate = capture
    for row in events:
        if row['kind'] == 'tf':
            for transform in row['transforms']:
                transform['stamp_ns'] += 100_000_000
    assert 'tf_bracket_coverage' in codes(validate())


def test_invalid_quaternion(capture):
    _, events, _, validate = capture
    next(row for row in events if row['kind'] == 'normalized_odom')['pose'] = {
        'position': {'x': 0, 'y': 0, 'z': 0}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 0}}
    assert 'normalized_odom_quaternion' in codes(validate())


def test_path_escape_is_refused(capture):
    _, events, _, validate = capture
    events[0]['file'] = '../private.bin'
    assert 'malformed_event' in codes(validate())


def test_orphan_file_is_reported(capture):
    root, _, _, validate = capture
    (root / 'depth/orphan.bin').write_bytes(b'1234')
    assert 'orphan_payloads' in codes(validate())


def test_legacy_nonstandard_json_scan_values_are_identified(capture):
    _, events, _, validate = capture
    next(row for row in events if row['kind'] == 'scan')['ranges'] = [float('inf')]
    assert 'legacy_or_invalid_scan_number' in codes(validate())


def test_sensor_clock_mismatch_is_reported(capture):
    _, events, _, validate = capture
    for row in events:
        if row['kind'] == 'depth':
            row['received_unix_ns'] += 10_000_000_000
    assert 'sensor_source_age' in codes(validate())


def test_depth_only_profile_does_not_imply_full_pipeline(capture):
    _, events, summary, validate = capture
    events[:] = [row for row in events if row['kind'] in ('depth', 'camera_info')]
    report = validate(profile='depth-only')
    assert report['result'] == 'pass'
    assert report['robot_acceptance'] == 'not_established'
    assert report['tf_bracket_coverage']['depth']['fraction'] == 0


def test_missing_capture_generates_failure_report(tmp_path):
    report = VALIDATOR.validate_capture(tmp_path)
    assert report['result'] == 'fail'
    assert 'missing_events' in codes(report)
