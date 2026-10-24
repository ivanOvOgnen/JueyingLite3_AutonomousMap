#!/usr/bin/env python3
"""Publish a cache-consistent Room Mapper UI matching the running project API.

Only webapp assets are changed. The running API's contract and installed route
inventory select the UI; this never restarts ROS or changes vendor modes.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
from urllib.request import ProxyHandler, build_opener


ASSETS = ('app.js', 'styles.css', 'index.html')
MODERN_CAPABILITIES = {'priority_software_stop', 'tracked_goal_lifecycle',
                       'map_identity', 'path_preview', 'operator_lease'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def fetch(opener, url):
    with opener.open(url, timeout=5) as response:
        return response.read()


def checked_inputs(stage, workspace, status):
    manifest = json.loads((stage/'manifest.json').read_text())
    if manifest.get('format') != 'lite3_supervised_nav_backup/v2':
        raise RuntimeError('Unexpected UI recovery manifest')
    source_api = workspace/'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/robot_api.py'
    installed = list((workspace/'install/lite3_odom_tf_adapter/lib').glob(
        'python*/site-packages/lite3_odom_tf_adapter/robot_api.py'))
    if len(installed) != 1:
        raise RuntimeError('Cannot identify one installed robot API')
    api_source = source_api.read_bytes()
    api_installed = installed[0].read_bytes()
    modern = (status.get('api_contract_version') == 2 and
              MODERN_CAPABILITIES.issubset(status.get('capabilities') or []))
    prefix = 'modern_' if modern else ''
    assets = {name: (stage/(prefix+name)).read_bytes() for name in ASSETS}
    profile = 'modern_v30' if modern else 'rollback_v24'
    for name in ('app.js', 'styles.css'):
        if not modern and digest(assets[name]) != manifest.get('source', {}).get(name):
            raise RuntimeError('Staged '+name+' differs from the saved backup')
    marker = b'UI_DEPLOY_VERSION = 30' if modern else b'UI_DEPLOY_VERSION = 24'
    if marker not in assets['app.js']:
        raise RuntimeError('Staged JavaScript version does not match '+profile)
    script_ref = b'app.js?v=rollback-'+digest(assets['app.js']).encode()[:12]
    style_ref = b'styles.css?v=rollback-'+digest(assets['styles.css']).encode()[:12]
    if script_ref not in assets['index.html'] or style_ref not in assets['index.html']:
        raise RuntimeError('Staged page does not select the exact saved assets')
    routes = set(re.findall(rb'/api/v1/[A-Za-z0-9_/-]+', assets['app.js']))
    if len(routes) < 15:
        raise RuntimeError('Staged UI has too few recognized API routes')
    missing = sorted(route.decode() for route in routes if route not in api_installed)
    if missing:
        raise RuntimeError('Installed robot API lacks UI routes '+str(missing)+
                           '; source sha256='+digest(api_source)+
                           ', installed sha256='+digest(api_installed))
    return assets, profile, {'source_sha256': digest(api_source),
                             'installed_sha256': digest(api_installed),
                             'contract_version': status.get('api_contract_version'),
                             'route_count': len(routes)}


def install(stage, workspace, *, apply):
    workspace = workspace.resolve()
    target = workspace/'webapp'
    if not target.is_dir() or target.is_symlink():
        raise RuntimeError('Robot webapp directory is missing or symlinked')
    if any((target/name).is_symlink() for name in ASSETS):
        raise RuntimeError('Robot UI contains a symlink; refusing to overwrite it')
    opener = build_opener(ProxyHandler({}))
    base = 'http://127.0.0.1:8080'
    status = json.loads(fetch(opener, base+'/api/v1/status'))
    if status.get('ok') is not True or not isinstance(status.get('components'), dict):
        raise RuntimeError('Robot API is not healthy')
    if status.get('simulation') is True:
        raise RuntimeError('Expected the physical robot API, not a simulation')
    assets, profile, api_identity = checked_inputs(stage, workspace, status)
    before = {name: (target/name).read_bytes() for name in ASSETS}
    report = {
        'format': 'lite3_rollback_ui_repair/v2',
        'applied': False,
        'selected_ui': profile,
        'api': api_identity,
        'before': {name: digest(value) for name, value in before.items()},
        'after': {name: digest(value) for name, value in assets.items()},
        'components': {key: value.get('state') for key, value in status['components'].items()
                       if key in ('camera', 'odometry', 'mapping', 'navigation_stack', 'navigation', 'map')
                       and isinstance(value, dict)},
    }
    if not apply:
        return report
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    backup = workspace/'backups'/datetime.now(timezone.utc).strftime('%Y%m%d')/('ui_repair_'+timestamp)
    backup.mkdir(parents=True, exist_ok=False)
    for name in ASSETS:
        shutil.copy2(target/name, backup/name)
        if digest((backup/name).read_bytes()) != digest(before[name]):
            raise RuntimeError('Current UI backup failed verification: '+name)
    report['backup'] = str(backup)
    changed = []
    try:
        for name in ASSETS:
            if (target/name).read_bytes() != before[name]:
                raise RuntimeError('Robot UI changed during recovery')
        # index.html is published last, after its exact JS and CSS bytes.
        for name in ASSETS:
            with tempfile.NamedTemporaryFile(dir=str(target), prefix='.ui-repair-', delete=False) as stream:
                staged = Path(stream.name)
                stream.write(assets[name])
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(staged, 0o644)
            os.replace(staged, target/name)
            changed.append(name)
        for name in ASSETS:
            if digest(fetch(opener, base+'/'+name+'?repair='+timestamp)) != digest(assets[name]):
                raise RuntimeError('Robot serves unexpected UI bytes: '+name)
        report['applied'] = True
        (backup/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        return report
    except Exception:
        for name in reversed(changed):
            with tempfile.NamedTemporaryFile(dir=str(target), prefix='.ui-repair-rollback-', delete=False) as stream:
                staged = Path(stream.name)
                stream.write(before[name])
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(staged, target/name)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', required=True, type=Path)
    parser.add_argument('--workspace', type=Path, default=Path.home()/'lite3_autonomy_ws')
    parser.add_argument('--apply', action='store_true')
    options = parser.parse_args()
    print(json.dumps(install(options.stage, options.workspace, apply=options.apply), indent=2), flush=True)


if __name__ == '__main__':
    main()
