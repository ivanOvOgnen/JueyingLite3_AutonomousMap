#!/usr/bin/env python3
"""One-command, backed-up Nav2 trial install and laptop app session.

Run this from the repo after *you* join the robot Wi-Fi. It performs local
tests, a robot idle preflight, a verified project software STOP and backup, a package build, API health
checks, a local copy of the robot backup, a Manual+Move reassertion, and opens
the app. It never sends a navigation goal or arms robot velocity by itself.
If the post-install manual joystick check fails, it rolls back automatically.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from connect_lite3_linux import robot_gateway, perception_address, command
from restore_vendor_mode import restore as restore_manual


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT/'scripts'
BACKUP_RE = re.compile(r'^Backup: (/home/ysc/lite3_autonomy_ws/backups/[0-9]{8}/supervised_nav_[0-9A-Za-z_]+)$')
SSH_OPTIONS = ('-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
               '-o', 'ServerAliveCountMax=3')


def run_with_output(args):
    process = subprocess.Popen(args, cwd=ROOT, text=True, stdout=subprocess.PIPE)
    lines = []
    try:
        for line in process.stdout:
            print(line, end='', flush=True)
            lines.append(line.rstrip('\n'))
    except KeyboardInterrupt:
        process.wait()
        raise
    result = process.wait()
    if result:
        raise subprocess.CalledProcessError(result, args)
    return lines


def copy_robot_backup(backup_path, motion, perception, destination):
    destination.mkdir(parents=True, exist_ok=False)
    target = 'ysc@'+perception+':'+backup_path
    cmd = ['scp', '-r', *SSH_OPTIONS, '-o', 'ProxyJump=ysc@'+motion,
           target, str(destination)]
    subprocess.run(cmd, check=True)
    copy = destination/Path(backup_path).name
    manifest = copy/'manifest.json'
    if not manifest.is_file():
        raise RuntimeError('Downloaded backup has no manifest: '+str(copy))
    data = json.loads(manifest.read_text())
    if data.get('format') != 'lite3_supervised_nav_backup/v2':
        raise RuntimeError('Downloaded backup manifest is unexpected')
    for field, prefix in (('source', ''),
                          ('installed_resources', 'installed_'),
                          ('installed_python', 'installed_python_')):
        for name, expected in data.get(field, {}).items():
            if expected is None:
                continue
            copied = copy/(prefix+name)
            if (not copied.is_file() or
                    hashlib.sha256(copied.read_bytes()).hexdigest() != expected):
                raise RuntimeError('Downloaded backup hash mismatch: '+str(copied))
    print('Local copy of robot backup:', copy, flush=True)
    return copy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-only', action='store_true',
                        help='run local tests and robot preflight without deployment')
    parser.add_argument('--no-browser', action='store_true',
                        help='install and verify without opening the app')
    args = parser.parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    report_path = ROOT/'log'/('nav2_one_shot_'+stamp+'.json')
    report_path.parent.mkdir(exist_ok=True)
    report = {'format': 'lite3_nav2_one_shot/v1',
              'started_utc': datetime.now(timezone.utc).isoformat(),
              'deployed': False, 'backup_robot': None,
              'backup_laptop': None, 'manual_verified': False}

    def save():
        report_path.write_text(json.dumps(report, indent=2)+'\n')

    save()
    print('Checking the full local trial bundle...', flush=True)
    subprocess.run([sys.executable, '-m', 'pytest', '-q', 'tests'],
                   cwd=ROOT, check=True)
    ssid, motion = robot_gateway()
    config = command('ssh', *SSH_OPTIONS, 'ysc@'+motion,
                     "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
    perception = perception_address(config)
    print('Robot network:', ssid, 'motion:', motion, 'perception:', perception,
          flush=True)
    report.update({'ssid': ssid, 'motion_host': motion,
                   'perception_host': perception})
    save()
    deploy = [sys.executable, str(SCRIPTS/'deploy_supervised_nav_trial.py'),
              '--motion-host', motion, '--perception-host', perception]
    if args.check_only:
        run_with_output(deploy)
        report['check_only'] = True
        save()
        print('Read-only preflight complete; no robot files changed.')
        return 0

    print('Keep the DEEP joysticks neutral. The installer will latch the project '
          'software STOP after checking mapping/Nav2/goals are idle. Confirm '
          'manual driving worked before this install.', flush=True)
    if input('Type YES if manual joystick works now: ').strip() != 'YES':
        raise RuntimeError('Manual baseline was not confirmed; no install started')

    try:
        lines = run_with_output(deploy+['--apply'])
        report['deployed'] = True
        save()
        backup = next((match.group(1) for line in lines
                       if (match := BACKUP_RE.fullmatch(line))), None)
        if not backup:
            raise RuntimeError('Install returned without a verified backup path')
        report['backup_robot'] = backup
        save()
        local_backup = ROOT/'log'/('nav2_robot_backup_'+stamp)
        report['backup_laptop'] = str(copy_robot_backup(
            backup, motion, perception, local_backup))
        save()
        restore_manual(assume_neutral=True)
        print('Check the DEEP joystick briefly in a clear area, then release '
              'it to neutral.', flush=True)
        if input('Type YES if manual joystick still works: ').strip() != 'YES':
            raise RuntimeError('Manual joystick was not verified after install')
        report['manual_verified'] = True
        save()
    except (KeyboardInterrupt, OSError, RuntimeError, subprocess.CalledProcessError,
            subprocess.TimeoutExpired):
        if report['deployed']:
            print('Verification failed. Restoring the pre-install robot '
                  'snapshot now...', file=sys.stderr, flush=True)
            try:
                if not report['backup_robot']:
                    raise RuntimeError('Install backup path was not captured; refusing to restore an unrelated snapshot')
                subprocess.run([sys.executable,
                                str(SCRIPTS/'rollback_supervised_nav_trial.py'),
                                '--motion-host', motion,
                                '--perception-host', perception,
                                '--backup', report['backup_robot'], '--apply'],
                               cwd=ROOT, check=True)
                report['rollback_completed'] = True
            except Exception as recovery_error:
                report['rollback_error'] = str(recovery_error)
                print('Rollback needs inspection:', recovery_error,
                      file=sys.stderr, flush=True)
            else:
                try:
                    restore_manual(assume_neutral=True)
                    report['manual_mode_sent_after_rollback'] = True
                except Exception as mode_error:
                    report['manual_mode_error_after_rollback'] = str(mode_error)
                    print('Project rollback completed, but Manual + Move mode '
                          'could not be sent:', mode_error,
                          file=sys.stderr, flush=True)
        save()
        raise

    print('Installed and backed up. The DEEP joystick still works.', flush=True)
    print('The app will open; map-click walking remains an attended, explicitly '
          'armed trial. The installer has not started Nav2 or motor output.',
          flush=True)
    print('Run report:', report_path, flush=True)
    if args.no_browser:
        return 0
    try:
        return subprocess.call([sys.executable,
                                str(SCRIPTS/'connect_lite3_linux.py')], cwd=ROOT)
    finally:
        # The app tunnel may close after a trial. Reassert manual mode only
        # when the read-only graph proves no goal/command route remains.
        try:
            restore_manual(assume_neutral=True)
        except Exception as error:
            print('Manual return was not verified. Stop Nav2 in the app, then '
                  'run python3 scripts/restore_vendor_mode.py --apply. '
                  'Reason: '+str(error), file=sys.stderr, flush=True)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (KeyboardInterrupt, OSError, ValueError, RuntimeError,
            subprocess.CalledProcessError,
            subprocess.TimeoutExpired) as error:
        print('Nav2 one-shot stopped:', error, file=sys.stderr)
        raise SystemExit(1)
