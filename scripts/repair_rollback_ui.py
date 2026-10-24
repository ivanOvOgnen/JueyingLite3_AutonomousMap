#!/usr/bin/env python3
"""Repair the Room Mapper UI after the 2026-10-24 trial rollback.

Run once after manually joining robot Wi-Fi. Select the saved or modern UI
from the running API contract and installed route inventory. Back up the
current web assets and publish a fresh browser cache key. ROS and vendor modes
are never changed.
"""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shlex
import socket
import subprocess
import sys
import tempfile
import uuid

from connect_lite3_linux import command, perception_address, robot_gateway
from deploy_supervised_nav_trial import cached_perception_address


ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = (ROOT/'log/nav2_robot_backup_20261024_155806/'
            'supervised_nav_20240912_064352')
ASSETS = ('app.js', 'styles.css', 'index.html')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def recovery_assets(snapshot):
    manifest = json.loads((snapshot/'manifest.json').read_text())
    if manifest.get('format') != 'lite3_supervised_nav_backup/v2':
        raise ValueError('Rollback snapshot manifest is invalid')
    original = {name: (snapshot/name).read_bytes() for name in ASSETS}
    for name in ASSETS:
        if sha(original[name]) != manifest.get('source', {}).get(name):
            raise ValueError('Rollback snapshot hash mismatch: '+name)
    if b'UI_DEPLOY_VERSION = 24' not in original['app.js']:
        raise ValueError('Rollback snapshot JavaScript version is unexpected')
    script_ref = b'app.js?v=rollback-'+sha(original['app.js']).encode()[:12]
    style_ref = b'styles.css?v=rollback-'+sha(original['styles.css']).encode()[:12]
    index, script_count = re.subn(rb'app\.js\?v=[^"\s]+', script_ref, original['index.html'])
    index, style_count = re.subn(rb'styles\.css\?v=[^"\s]+', style_ref, index)
    if script_count != 1 or style_count != 1:
        raise ValueError('Rollback page does not have one script and stylesheet reference')
    return manifest, {'app.js': original['app.js'], 'styles.css': original['styles.css'],
                      'index.html': index}


def modern_assets(webapp):
    original = {name: (webapp/name).read_bytes() for name in ASSETS}
    if b'UI_DEPLOY_VERSION = 30' not in original['app.js']:
        raise ValueError('Local modern UI version is unexpected')
    script_ref = b'app.js?v=rollback-'+sha(original['app.js']).encode()[:12]
    style_ref = b'styles.css?v=rollback-'+sha(original['styles.css']).encode()[:12]
    index, script_count = re.subn(rb'app\.js\?v=[^"\s]+', script_ref, original['index.html'])
    index, style_count = re.subn(rb'styles\.css\?v=[^"\s]+', style_ref, index)
    if script_count != 1 or style_count != 1:
        raise ValueError('Modern page does not have one script and stylesheet reference')
    return {'app.js': original['app.js'], 'styles.css': original['styles.css'],
            'index.html': index}


def free_port():
    for port in range(18080, 18091):
        with socket.socket() as sock:
            try:
                sock.bind(('127.0.0.1', port))
            except OSError:
                continue
            return port
    raise RuntimeError('No free local Room Mapper port between 18080 and 18090')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true',
                        help='back up and republish the matching UI; otherwise inspect only')
    parser.add_argument('--no-browser', action='store_true')
    options = parser.parse_args()
    manifest, assets = recovery_assets(SNAPSHOT)
    modern = modern_assets(ROOT/'webapp')
    ssid, motion = robot_gateway()
    perception = cached_perception_address(motion)
    if not perception:
        config = command('ssh', '-o', 'ConnectTimeout=10', 'ysc@'+motion,
                         "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
        perception = perception_address(config)
    print('Robot Wi-Fi:', ssid, 'API host:', perception, flush=True)
    stage = '/tmp/lite3_ui_repair_'+uuid.uuid4().hex[:12]
    control = '/tmp/lite3_ui_ssh_'+uuid.uuid4().hex[:12]
    common = ['-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
              '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
              '-o', 'ControlPersist=120', '-o', 'ControlPath='+control]
    target = 'ysc@'+perception
    jump = 'ysc@'+motion
    ssh = ['ssh', *common, '-J', jump, target]
    scp = ['scp', *common, '-o', 'ProxyJump='+jump]
    with tempfile.TemporaryDirectory(prefix='lite3-ui-repair-') as temporary:
        files = Path(temporary)
        for name, content in assets.items():
            (files/name).write_bytes(content)
        for name, content in modern.items():
            (files/('modern_'+name)).write_bytes(content)
        (files/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        helper = ROOT/'scripts/robot_repair_rollback_ui.py'
        subprocess.run([*ssh, 'mkdir -m 700 '+shlex.quote(stage)], check=True)
        names = (*ASSETS, *(('modern_'+name) for name in ASSETS), 'manifest.json')
        subprocess.run([*scp, *(str(files/name) for name in names),
                        str(helper), target+':'+stage+'/'], check=True)
        remote = ('python3 '+shlex.quote(stage+'/robot_repair_rollback_ui.py')+
                  ' --stage '+shlex.quote(stage)+(' --apply' if options.apply else ''))
        result = subprocess.run([*ssh, remote], check=True, text=True,
                                stdout=subprocess.PIPE)
        print(result.stdout, end='', flush=True)
        report = json.loads(result.stdout)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        local_report = ROOT/'log'/('rollback_ui_repair_'+stamp+'.json')
        local_report.write_text(json.dumps(report, indent=2)+'\n')
        print('Repair report:', local_report, flush=True)
    if not options.apply:
        print('Read-only check complete. Re-run with --apply to repair the UI.')
        return 0
    if options.no_browser:
        return 0
    port = free_port()
    print('Opening a fresh Room Mapper tab; old localhost tabs may have cached trial controls.',
          flush=True)
    return subprocess.call([sys.executable, str(ROOT/'scripts/connect_lite3_linux.py'),
                            '--local-port', str(port)])


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError,
            subprocess.TimeoutExpired) as error:
        print('Room Mapper UI repair stopped:', error, file=sys.stderr)
        raise SystemExit(1)
