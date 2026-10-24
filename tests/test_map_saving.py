"""Map bundle files, thresholds, no-overwrite and partial-publish rollback."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest
import yaml

from conftest import PACKAGE


@pytest.fixture
def saver(api_module):
    spec = importlib.util.spec_from_file_location(
        '_save_map_tests', PACKAGE / 'lite3_odom_tf_adapter/save_map.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def grid():
    return NS(info=NS(width=3, height=2, resolution=0.05,
                      origin=NS(position=NS(x=-1.5, y=2.5, z=0.0),
                                orientation=NS(x=0.0, y=0.0, z=math.sin(0.2), w=math.cos(0.2)))),
              data=[0, 19, 20, 64, 65, -1], header=NS(frame_id='map', stamp=NS(sec=42, nanosec=3)))


def test_bundle_roundtrip_orientation_thresholds_and_checksums(saver, grid, tmp_path):
    image, config, sidecar = saver.save_grid(grid, tmp_path / 'room')
    assert image.read_bytes() == b'P5\n3 2\n255\n' + bytes([205, 0, 205, 254, 254, 205])
    settings = yaml.safe_load(config.read_text())
    assert settings['origin'] == pytest.approx([-1.5, 2.5, 0.4])
    assert settings['resolution'] == 0.05
    manifest = json.loads(sidecar.read_text())
    assert manifest['quality']['approval_state'] == 'experimental'
    assert manifest['bundle_status'] == 'complete'
    assert manifest['source_grid_stamp_ns'] == 42_000_000_003
    for path in (image, config):
        assert manifest['files_sha256'][path.name] == hashlib.sha256(path.read_bytes()).hexdigest()


def test_existing_map_never_changes(saver, grid, tmp_path):
    paths = saver.save_grid(grid, tmp_path / 'room')
    before = {path: path.read_bytes() for path in paths}
    grid.data[:] = [100] * 6
    with pytest.raises(FileExistsError):
        saver.save_grid(grid, tmp_path / 'room')
    assert {path: path.read_bytes() for path in paths} == before


@pytest.mark.parametrize('change', ['dimensions', 'resolution_nan', 'quaternion', 'cell', 'pitch'])
def test_invalid_map_produces_no_bundle(saver, grid, tmp_path, change):
    if change == 'dimensions':
        grid.info.width = 4
    elif change == 'resolution_nan':
        grid.info.resolution = float('nan')
    elif change == 'quaternion':
        grid.info.origin.orientation.w = 0
    elif change == 'cell':
        grid.data[0] = 101
    else:
        grid.info.origin.orientation.x = 0.1
    with pytest.raises(ValueError):
        saver.save_grid(grid, tmp_path / 'room')
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('failure_index', [2, 3])
def test_publish_failure_rolls_back_owned_files(saver, grid, tmp_path, monkeypatch, failure_index):
    original = saver.os.link
    counter = [0]

    def link(source, destination):
        counter[0] += 1
        assert not (tmp_path / 'room.yaml').exists()
        if counter[0] == failure_index:
            raise OSError('inert publication failure')
        return original(source, destination)

    monkeypatch.setattr(saver.os, 'link', link)
    with pytest.raises(OSError, match='publication'):
        saver.save_grid(grid, tmp_path / 'room')
    assert not list(tmp_path.iterdir())


def test_concurrent_same_name_has_exactly_one_winner(saver, grid, tmp_path):
    barrier = threading.Barrier(2)
    results, errors = [], []

    def save():
        barrier.wait()
        try:
            results.append(saver.save_grid(grid, tmp_path / 'room'))
        except FileExistsError as error:
            errors.append(error)

    threads = [threading.Thread(target=save) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(2)
        assert not thread.is_alive()
    assert len(results) == len(errors) == 1
    assert (tmp_path / 'room.yaml').is_file()
    assert len(list(tmp_path.iterdir())) == 3


def test_unrelated_file_created_during_publish_is_preserved(saver, grid, tmp_path, monkeypatch):
    original = saver.os.link

    def link(source, destination):
        if destination.name.endswith('.lite3-map.json'):
            destination.write_text('other owner')
        return original(source, destination)

    monkeypatch.setattr(saver.os, 'link', link)
    with pytest.raises(FileExistsError):
        saver.save_grid(grid, tmp_path / 'room')
    assert (tmp_path / 'room.lite3-map.json').read_text() == 'other owner'
    assert not (tmp_path / 'room.pgm').exists()
    assert not (tmp_path / 'room.yaml').exists()
