"""The downloaded robot snapshot must match its verified manifest."""

import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import nav2_one_shot as one_shot


def test_backup_copy_checks_every_present_hash(monkeypatch, tmp_path):
    backup = '/home/ysc/lite3_autonomy_ws/backups/20261024/supervised_nav_test'
    expected = hashlib.sha256(b'old api').hexdigest()

    def fake_scp(_command, check):
        folder = tmp_path/'download/supervised_nav_test'
        folder.mkdir(parents=True)
        (folder/'robot_api.py').write_bytes(b'old api')
        (folder/'manifest.json').write_text(json.dumps({
            'format': 'lite3_supervised_nav_backup/v2',
            'source': {'robot_api.py': expected},
            'installed_resources': {}, 'installed_python': {},
        }))

    monkeypatch.setattr(one_shot.subprocess, 'run', fake_scp)
    copy = one_shot.copy_robot_backup(backup, '192.168.2.1',
                                     '192.168.1.103', tmp_path/'download')
    assert (copy/'robot_api.py').read_bytes() == b'old api'


def test_backup_copy_rejects_corruption(monkeypatch, tmp_path):
    backup = '/home/ysc/lite3_autonomy_ws/backups/20261024/supervised_nav_test'

    def fake_scp(_command, check):
        folder = tmp_path/'download/supervised_nav_test'
        folder.mkdir(parents=True)
        (folder/'robot_api.py').write_bytes(b'corrupt')
        (folder/'manifest.json').write_text(json.dumps({
            'format': 'lite3_supervised_nav_backup/v2',
            'source': {'robot_api.py': hashlib.sha256(b'old api').hexdigest()},
            'installed_resources': {}, 'installed_python': {},
        }))

    monkeypatch.setattr(one_shot.subprocess, 'run', fake_scp)
    with pytest.raises(RuntimeError, match='hash mismatch'):
        one_shot.copy_robot_backup(backup, '192.168.2.1',
                                   '192.168.1.103', tmp_path/'download')
