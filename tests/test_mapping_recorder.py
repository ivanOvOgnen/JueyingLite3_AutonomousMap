"""Real recorder writer/queue operations against disposable local files."""
import hashlib
import importlib.util
import json
from pathlib import Path
import queue
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from conftest import PACKAGE


@pytest.fixture
def recorder_module(api_module, monkeypatch):
    for module, names in {
        'geometry_msgs.msg': ['TransformStamped'],
        'sensor_msgs.msg': ['CameraInfo', 'Imu'],
    }.items():
        for name in names:
            monkeypatch.setattr(sys.modules[module], name, NS, raising=False)
    monkeypatch.setitem(sys.modules, 'tf2_msgs', ModuleType('tf2_msgs'))
    module = ModuleType('tf2_msgs.msg')
    module.TFMessage = NS
    monkeypatch.setitem(sys.modules, 'tf2_msgs.msg', module)
    spec = importlib.util.spec_from_file_location(
        '_lite3_recorder_tests', PACKAGE / 'lite3_odom_tf_adapter/mapping_recorder.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def recorder(recorder_module, tmp_path):
    r = recorder_module.MappingRecorder.__new__(recorder_module.MappingRecorder)
    r.output = tmp_path
    r.depth_dir = tmp_path / 'depth'
    r.depth_dir.mkdir()
    r.events = (tmp_path / 'events.jsonl').open('x')
    r.max_bytes = 12
    r.saved_bytes = r.reserved_bytes = r.depth_index = 0
    r.stopping = False
    r.writer_error = None
    r.counts, r.dropped = {}, {}
    r.state_lock, r.event_lock = threading.Lock(), threading.Lock()
    r.depth_queue = queue.Queue(maxsize=2)
    r.writer = threading.Thread(target=r._write_depth, daemon=True)
    yield r
    if r.writer.ident is not None:
        r.close()
    r.events.close()


def depth():
    return NS(data=bytes([1, 2, 3, 4]), width=2, height=1, step=4, encoding='16UC1',
              is_bigendian=False, header=NS(frame_id='depth', stamp=NS(sec=1, nanosec=0)))


def test_cap_includes_queued_frames(recorder):
    r = recorder
    r.max_bytes = 4
    r._depth(depth())
    r._depth(depth())
    assert r.saved_bytes == 0
    assert r.reserved_bytes == 4
    assert r.dropped == {'depth_disk_limit': 1}
    assert r.depth_queue.qsize() == 1


def test_full_queue_releases_reservation(recorder):
    r = recorder
    for _ in range(3):
        r._depth(depth())
    assert r.reserved_bytes == 8
    assert r.dropped == {'depth_writer_queue': 1}


def test_depth_keeps_callback_receipt_time_and_hash(recorder, recorder_module, monkeypatch):
    r = recorder
    monkeypatch.setattr(recorder_module.time, 'time_ns', lambda: 123456)
    r._depth(depth())
    monkeypatch.setattr(recorder_module.time, 'time_ns', lambda: 987654)
    r.writer.start()
    r.close()
    event = json.loads((r.output / 'events.jsonl').read_text())
    summary = json.loads((r.output / 'summary.json').read_text())
    assert event['received_unix_ns'] == 123456
    assert event['sha256'] == hashlib.sha256(depth().data).hexdigest()
    assert summary['saved_depth_bytes'] == summary['reserved_depth_bytes'] == 4
    assert summary['writer_finished'] and not summary['writer_error']
    assert summary['queued_depth'] == 0


def test_writer_failure_is_visible_and_full_queue_close_does_not_hang(recorder, monkeypatch):
    r = recorder
    original = Path.open

    def disk_failure(path, *args, **kwargs):
        if path.suffix == '.bin':
            raise OSError('inert disk failure')
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'open', disk_failure)
    r._depth(depth())
    r.writer.start()
    r.writer.join(1)
    assert not r.writer.is_alive()
    assert 'disk failure' in r.writer_error
    r.depth_queue.put_nowait('inert remaining payload')
    r.depth_queue.put_nowait('inert remaining payload')
    started = time.monotonic()
    r.close()
    assert time.monotonic() - started < 0.5
    summary = json.loads((r.output / 'summary.json').read_text())
    assert summary['writer_finished']
    assert summary['writer_error']
    assert summary['queued_depth'] == 2
    r.stopping = False
    r._depth(depth())
    assert r.dropped['depth_writer_failed'] == 1
    r.stopping = True


def test_nonfinite_ranges_are_explicit_strict_json(recorder):
    r = recorder
    r._event('scan', ranges=[1.0, float('inf'), float('-inf'), float('nan')])
    r.events.flush()
    value = json.loads((r.output / 'events.jsonl').read_text(),
                       parse_constant=lambda v: pytest.fail('Invalid JSON number: ' + v))
    assert value['ranges'] == [1.0, 'Infinity', '-Infinity', 'NaN']
