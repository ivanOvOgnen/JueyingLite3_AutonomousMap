"""Actual file deployment/rollback against temporary folders and fake HTTP."""
import importlib.util
import json
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    '_ui_deployment_tests', Path(__file__).resolve().parents[1] / 'scripts/deploy_web_ui.py')
DEPLOY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DEPLOY)


@pytest.fixture
def deployment(tmp_path, monkeypatch):
    source, workspace = tmp_path / 'source', tmp_path / 'robot'
    source.mkdir()
    target = workspace / 'webapp'
    target.mkdir(parents=True)
    assets = {'app.js': b'const UI_DEPLOY_VERSION = 30;', 'styles.css': b'body {}',
              'index.html': b'<script src="app.js?v=30"></script>'}
    for name, payload in assets.items():
        (source / name).write_bytes(payload)
        (target / name).write_bytes(b'previous:' + payload)
    (target / 'maps').mkdir()
    (target / 'maps/map.pgm').write_bytes(b'preserved map')
    status = {'ok': True, 'api_contract_version': 2, 'capabilities': list(DEPLOY.REQUIRED_CAPABILITIES)}

    def fetch(url):
        if url.endswith('/api/v1/status'):
            return json.dumps(status).encode()
        return (target / url.rsplit('/', 1)[1]).read_bytes()

    monkeypatch.setattr(DEPLOY, 'fetch', fetch)
    return source, workspace, target, assets, status


def test_check_only_never_writes_assets_or_backup(deployment):
    source, workspace, target, assets, _ = deployment
    report = DEPLOY.deploy(source, workspace)
    assert not report['applied']
    assert not (workspace / 'backups').exists()
    assert (target / 'app.js').read_bytes() == b'previous:' + assets['app.js']


def test_explicit_deployment_preserves_backup_and_maps(deployment):
    source, workspace, target, assets, _ = deployment
    report = DEPLOY.deploy(source, workspace, apply=True)
    assert report['applied']
    for name, data in assets.items():
        assert (target / name).read_bytes() == data
        assert (Path(report['backup']) / name).read_bytes() == b'previous:' + data
    assert (target / 'maps/map.pgm').read_bytes() == b'preserved map'


def test_incompatible_backend_refuses_any_write(deployment):
    source, workspace, target, assets, status = deployment
    status['capabilities'] = []
    with pytest.raises(ValueError, match='contract'):
        DEPLOY.deploy(source, workspace, apply=True)
    assert not (workspace / 'backups').exists()
    assert (target / 'app.js').read_bytes() == b'previous:' + assets['app.js']


def test_wrong_served_root_rolls_back_all_assets(deployment, monkeypatch):
    source, workspace, target, assets, _ = deployment
    original = DEPLOY.fetch
    monkeypatch.setattr(DEPLOY, 'fetch', lambda url: original(url) if url.endswith('/status') else b'wrong served root')
    with pytest.raises(ValueError, match='Served asset hash'):
        DEPLOY.deploy(source, workspace, apply=True)
    for name, data in assets.items():
        assert (target / name).read_bytes() == b'previous:' + data
    assert len(list((workspace / 'backups').glob('*/*/app.js'))) == 1


def test_partial_publish_failure_rolls_back(deployment, monkeypatch):
    source, workspace, target, assets, _ = deployment
    original = DEPLOY.os.replace

    def replace(src, dest):
        if Path(src).name == 'styles.css':
            raise OSError('inert publish failure')
        return original(src, dest)

    monkeypatch.setattr(DEPLOY.os, 'replace', replace)
    with pytest.raises(OSError, match='publish failure'):
        DEPLOY.deploy(source, workspace, apply=True)
    for name, data in assets.items():
        assert (target / name).read_bytes() == b'previous:' + data
