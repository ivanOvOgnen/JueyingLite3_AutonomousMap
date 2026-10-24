#!/usr/bin/env python3
"""Explicit UI deployment on the robot; connection helpers never deploy.

Run this saved script on the target with a reviewed source webapp directory.
Default is a read-only compatibility/change report. --apply creates a dated
backup, stages assets, publishes index.html last and verifies HTTP hashes.
The API is never started/restarted and maps are never copied or removed.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
from urllib.parse import urlparse
from urllib.error import URLError
from urllib.request import ProxyHandler, Request, build_opener
import uuid

ASSETS = ('app.js', 'styles.css', 'index.html')
UI_VERSION = 30
REQUIRED_CAPABILITIES = {'priority_software_stop', 'tracked_goal_lifecycle', 'map_identity', 'path_preview', 'operator_lease'}


def sha256(value):
    return hashlib.sha256(value).hexdigest()


def fetch(url):
    try:
        opener = build_opener(ProxyHandler({}))
        with opener.open(Request(url, headers={'Cache-Control': 'no-cache'}), timeout=5) as response:
            return response.read()
    except URLError as error:
        raise ValueError('Robot project API is unavailable at '+url+': '+str(error.reason)+
                         '. Run deploy_mapping_mode_ui.py --apply --recover-api, then inspect its report if recovery fails.') from error


def deploy(source, workspace, api_url='http://127.0.0.1:8080', *, apply=False):
    parsed = urlparse(api_url)
    if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError('API URL must be HTTP(S) without embedded credentials')
    base = api_url.rstrip('/')
    source, workspace = Path(source).resolve(), Path(workspace).resolve()
    target = workspace / 'webapp'
    if not target.is_dir() or target.is_symlink():
        raise ValueError('Expected an existing, non-symlink workspace/webapp directory')
    assets = {}
    previous = {}
    for name in ASSETS:
        if (source / name).is_symlink() or (target / name).is_symlink():
            raise ValueError('Asset symlinks are not supported')
        assets[name] = (source / name).read_bytes()
        previous[name] = (target / name).read_bytes()
    if not re.search(rb'UI_DEPLOY_VERSION\s*=\s*30\b', assets['app.js']):
        raise ValueError('Source does not contain the reviewed UI version 30')
    if b'app.js?v=30' not in assets['index.html']:
        raise ValueError('Source index.html does not select UI version 30')
    status = json.loads(fetch(base + '/api/v1/status'))
    if (status.get('ok') is not True or status.get('api_contract_version') != 2 or status.get('simulation') is True or
            not REQUIRED_CAPABILITIES.issubset(status.get('capabilities', []))):
        raise ValueError('Backend contract/capabilities do not support this UI; no assets changed')
    report = {'format': 'lite3_ui_deployment/v1', 'ui_version': UI_VERSION,
              'applied': False, 'assets': {name: {'before_sha256': sha256(previous[name]),
                                               'after_sha256': sha256(assets[name])} for name in ASSETS}}
    if not apply:
        return report
    # Serialize cooperating deployments. A stale lock file is harmless; flock
    # releases the actual lock on process death.
    with (workspace / '.ui-deployment.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for name in ASSETS:
            if (target / name).read_bytes() != previous[name]:
                raise ValueError('Target changed during compatibility checks; retry with a fresh review')
        now = datetime.now(timezone.utc)
        backup = workspace / 'backups' / now.strftime('%Y%m%d') / (
            'ui_predeploy_' + now.strftime('%H%M%S') + '_' + uuid.uuid4().hex[:8])
        backup.mkdir(parents=True, exist_ok=False)
        report['backup'] = str(backup)
        for name in ASSETS:
            shutil.copy2(target / name, backup / name)
            if (backup / name).read_bytes() != previous[name]:
                raise ValueError('Backup hash mismatch; no assets changed')
        changed = []
        try:
            with tempfile.TemporaryDirectory(prefix='.ui-stage-', dir=str(workspace)) as temporary:
                stage = Path(temporary)
                for name in ASSETS:
                    with (stage / name).open('xb') as stream:
                        stream.write(assets[name])
                        stream.flush()
                        os.fsync(stream.fileno())
                    os.chmod(stage / name, 0o644)
                # The new index/cache version becomes visible after both assets.
                for name in ASSETS:
                    os.replace(stage / name, target / name)
                    changed.append(name)
                for name in ASSETS:
                    if sha256(fetch(base + '/' + name)) != sha256(assets[name]):
                        raise ValueError('Served asset hash mismatch: ' + name)
            report['applied'] = True
            (backup / 'deployment.json').write_text(json.dumps(report, indent=2) + '\n')
        except Exception:
            for name in reversed(changed):
                # Preserve the rollback copy even after restoration.
                with tempfile.NamedTemporaryFile(dir=str(target), prefix='.ui-rollback-', delete=False) as stream:
                    rollback = Path(stream.name)
                    stream.write(previous[name])
                    stream.flush()
                    os.fsync(stream.fileno())
                shutil.copystat(backup / name, rollback)
                os.replace(rollback, target / name)
            raise
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='Reviewed staged webapp assets')
    parser.add_argument('--workspace', type=Path, default=Path.home() / 'lite3_autonomy_ws')
    parser.add_argument('--api-url', default='http://127.0.0.1:8080')
    parser.add_argument('--apply', action='store_true', help='Apply after compatibility and backup checks')
    options = parser.parse_args(argv)
    try:
        report = deploy(options.source, options.workspace, options.api_url, apply=options.apply)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, str(error) + '\n')
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
