"""The rollback recovery page must match the saved API and bypass old JS."""

import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import repair_rollback_ui as laptop
import robot_repair_rollback_ui as robot


def test_saved_ui_gets_a_cache_key_for_its_actual_javascript():
    manifest, assets = laptop.recovery_assets(laptop.SNAPSHOT)
    assert manifest['source']['app.js'] == hashlib.sha256(assets['app.js']).hexdigest()
    assert b'app.js?v=rollback-69aae1d66f46' in assets['index.html']
    assert b'app.js?v=25' not in assets['index.html']


def test_robot_refuses_ui_when_installed_api_lacks_its_routes(tmp_path, monkeypatch):
    workspace = tmp_path/'workspace'
    stage = tmp_path/'stage'
    stage.mkdir()
    (workspace/'webapp').mkdir(parents=True)
    api = workspace/'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/robot_api.py'
    api.parent.mkdir(parents=True)
    api.write_bytes(b'different api')
    installed = workspace/'install/lite3_odom_tf_adapter/lib/python3.8/site-packages/lite3_odom_tf_adapter/robot_api.py'
    installed.parent.mkdir(parents=True)
    installed.write_bytes(b'different api')
    manifest, assets = laptop.recovery_assets(laptop.SNAPSHOT)
    (stage/'manifest.json').write_text(json.dumps(manifest))
    for name, data in assets.items():
        (stage/name).write_bytes(data)
        (workspace/'webapp'/name).write_bytes(b'current '+name.encode())
    monkeypatch.setattr(robot, 'fetch', lambda _opener, _url:
                        json.dumps({'ok': True, 'components': {}}).encode())
    with pytest.raises(RuntimeError, match='lacks UI routes'):
        robot.install(stage, workspace, apply=True)
    assert (workspace/'webapp/index.html').read_bytes() == b'current index.html'


def test_robot_checks_ui_and_api_without_writing(tmp_path, monkeypatch):
    workspace = tmp_path/'workspace'
    stage = tmp_path/'stage'
    stage.mkdir()
    (workspace/'webapp').mkdir(parents=True)
    api = workspace/'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/robot_api.py'
    api.parent.mkdir(parents=True)
    installed = workspace/'install/lite3_odom_tf_adapter/lib/python3.8/site-packages/lite3_odom_tf_adapter/robot_api.py'
    installed.parent.mkdir(parents=True)
    manifest, assets = laptop.recovery_assets(laptop.SNAPSHOT)
    original_api = (laptop.SNAPSHOT/'robot_api.py').read_bytes()
    api.write_bytes(original_api)
    installed.write_bytes(original_api)
    (stage/'manifest.json').write_text(json.dumps(manifest))
    for name, data in assets.items():
        (stage/name).write_bytes(data)
        (workspace/'webapp'/name).write_bytes(b'current '+name.encode())
    monkeypatch.setattr(robot, 'fetch', lambda _opener, _url:
                        json.dumps({'ok': True, 'components': {'camera': {'state': 'ready'}}}).encode())
    report = robot.install(stage, workspace, apply=False)
    assert report['applied'] is False
    assert report['components']['camera'] == 'ready'
    assert (workspace/'webapp/index.html').read_bytes() == b'current index.html'


def test_robot_backs_up_then_serves_matching_assets(tmp_path, monkeypatch):
    workspace = tmp_path/'workspace'
    stage = tmp_path/'stage'
    stage.mkdir()
    (workspace/'webapp').mkdir(parents=True)
    api = workspace/'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/robot_api.py'
    api.parent.mkdir(parents=True)
    installed = workspace/'install/lite3_odom_tf_adapter/lib/python3.8/site-packages/lite3_odom_tf_adapter/robot_api.py'
    installed.parent.mkdir(parents=True)
    manifest, assets = laptop.recovery_assets(laptop.SNAPSHOT)
    original_api = (laptop.SNAPSHOT/'robot_api.py').read_bytes()
    api.write_bytes(original_api)
    installed.write_bytes(original_api)
    (stage/'manifest.json').write_text(json.dumps(manifest))
    for name, data in assets.items():
        (stage/name).write_bytes(data)
        (workspace/'webapp'/name).write_bytes(b'current '+name.encode())

    def served(_opener, url):
        if url.endswith('/api/v1/status'):
            return json.dumps({'ok': True, 'components': {}}).encode()
        name = url.split('/')[-1].split('?')[0]
        return (workspace/'webapp'/name).read_bytes()

    monkeypatch.setattr(robot, 'fetch', served)
    report = robot.install(stage, workspace, apply=True)
    assert report['applied'] is True
    assert (Path(report['backup'])/'index.html').read_bytes() == b'current index.html'
    assert (workspace/'webapp/index.html').read_bytes() == assets['index.html']


def test_modern_api_selects_modern_ui(tmp_path):
    workspace = tmp_path/'workspace'
    stage = tmp_path/'stage'
    stage.mkdir()
    api = workspace/'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/robot_api.py'
    api.parent.mkdir(parents=True)
    installed = workspace/'install/lite3_odom_tf_adapter/lib/python3.8/site-packages/lite3_odom_tf_adapter/robot_api.py'
    installed.parent.mkdir(parents=True)
    modern_api = (laptop.ROOT/'ros2_ws/src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/robot_api.py').read_bytes()
    api.write_bytes(modern_api)
    installed.write_bytes(modern_api)
    manifest, old_assets = laptop.recovery_assets(laptop.SNAPSHOT)
    (stage/'manifest.json').write_text(json.dumps(manifest))
    for name, data in old_assets.items():
        (stage/name).write_bytes(data)
    modern = laptop.modern_assets(laptop.ROOT/'webapp')
    for name, data in modern.items():
        (stage/('modern_'+name)).write_bytes(data)
    status = {'api_contract_version': 2, 'capabilities': list(robot.MODERN_CAPABILITIES)}
    selected, profile, identity = robot.checked_inputs(stage, workspace, status)
    assert profile == 'modern_v30'
    assert selected['app.js'] == modern['app.js']
    assert identity['route_count'] > 20
