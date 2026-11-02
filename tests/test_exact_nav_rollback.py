"""Rollback must use the backup from this installation, not an older snapshot."""

import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import robot_rollback_supervised_nav_trial as rollback


def snapshot(tmp_path, monkeypatch):
    monkeypatch.setattr(rollback, 'FILES', {'robot_api.py': 'src/robot_api.py'})
    monkeypatch.setattr(rollback, 'PACKAGE_MODULES', ('robot_api.py',))
    monkeypatch.setattr(rollback, 'installed_resources',
                        lambda _workspace: {'navigation.launch.py': Path('/unused')})
    workspace = tmp_path/'robot'
    saved = workspace/'backups/20261024/supervised_nav_current'
    saved.mkdir(parents=True)
    files = {
        'robot_api.py': b'original source',
        'installed_navigation.launch.py': b'original launch',
        'installed_python_robot_api.py': b'original installed api',
    }
    for name, data in files.items():
        (saved/name).write_bytes(data)
    digest = lambda name: hashlib.sha256(files[name]).hexdigest()
    (saved/'manifest.json').write_text(json.dumps({
        'format': 'lite3_supervised_nav_backup/v2',
        'workspace': str(workspace),
        'source': {'robot_api.py': digest('robot_api.py')},
        'installed_resources': {
            'navigation.launch.py': digest('installed_navigation.launch.py')},
        'installed_python': {'robot_api.py': digest('installed_python_robot_api.py')},
    }))
    return workspace, saved


def test_exact_snapshot_is_verified_before_restore(tmp_path, monkeypatch):
    workspace, saved = snapshot(tmp_path, monkeypatch)
    assert rollback.verified_snapshot(workspace, saved) == saved
    (saved/'installed_python_robot_api.py').write_bytes(b'wrong installed api')
    with pytest.raises(RuntimeError, match='hash mismatch'):
        rollback.verified_snapshot(workspace, saved)


def test_backup_outside_workspace_is_rejected(tmp_path, monkeypatch):
    workspace, saved = snapshot(tmp_path, monkeypatch)
    foreign = tmp_path/'foreign/backups/20261024/supervised_nav_current'
    foreign.parent.mkdir(parents=True)
    saved.rename(foreign)
    with pytest.raises(RuntimeError, match='not a supervised Nav2 snapshot'):
        rollback.verified_snapshot(workspace, foreign)
