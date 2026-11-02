#!/usr/bin/env python3
"""Read-only vendor mode/STOP/command-interface inspection on one Lite3 host.

Run through collect_vendor_interface_audit.py. No ROS publications, UDP sends,
service changes, mode switches, or motor commands are made here.
"""

import argparse
from datetime import datetime, timezone
import glob
import hashlib
import json
from pathlib import Path
import re
import shlex
import struct
import subprocess
import time


SENSITIVE = re.compile(r'password|passwd|token|secret|credential|authorization|private.key', re.I)
SOURCE_TERMS = re.compile(r'CmdVelCallback|SimpleCMD|handle_state|0x21010[Cc]0[23]|navigation.?mode|manual.?mode|auto.?mode|stop|mode', re.I)
LOG_TERMS = re.compile(r'Auto Mode|Manual Mode|navigation mode|manual mode|Soft Emergency|is_robot_lose_control|STOP', re.I)


def sanitize(value):
    if isinstance(value, dict):
        return {key: ('[redacted]' if SENSITIVE.search(str(key)) else sanitize(item))
                for key, item in value.items()}
    if isinstance(value, list):
        return [sanitize(item) for item in value]
    if isinstance(value, str):
        return '\n'.join('[redacted line]' if SENSITIVE.search(line) else line
                         for line in value.split('\n'))
    return value


def run(command, timeout=8):
    try:
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=timeout, check=False)
        return {'exit_code': result.returncode, 'output': sanitize(result.stdout[-25000:])}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'error': type(error).__name__, 'detail': sanitize(str(error))}


def file_evidence(path, include_lines=False):
    path = Path(path)
    try:
        data = path.read_bytes()
    except OSError as error:
        return {'present': False, 'error': type(error).__name__}
    evidence = {'present': True, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    if include_lines and len(data) < 2_000_000:
        text = data.decode('utf-8', 'replace')
        lines = text.splitlines()
        match_numbers = {number for number, line in enumerate(lines)
                         if SOURCE_TERMS.search(line) and not SENSITIVE.search(line)}
        context_numbers = {near for number in match_numbers
                           for near in range(max(0, number - 3), min(len(lines), number + 4))}
        evidence['matching_lines'] = [(number + 1, lines[number].strip())
                                      for number in sorted(context_numbers)
                                      if not SENSITIVE.search(lines[number])][:140]
        evidence['matching_line_count'] = len(match_numbers)
    return evidence


def safety_record(path=Path('/dev/shm/emotion_bot_lite3_safety_state')):
    path = Path(path)
    header = struct.Struct('<4sIQQI')
    wire = struct.Struct('<I???5xQ')
    try:
        with path.open('rb') as stream:
            first_raw = stream.read(header.size)
            payload = stream.read(wire.size)
            stream.seek(0)
            second_raw = stream.read(header.size)
        magic, version, sequence, receive_ns, length = header.unpack(first_raw)
        record_version, stop, observed_fresh, axes_zero, _ = wire.unpack(payload)
        age = (time.monotonic_ns() - receive_ns) / 1e9
        valid = (first_raw == second_raw and magic == b'EBL3' and version == 1
                 and sequence % 2 == 0 and length == wire.size
                 and record_version == 0x00010000 and 0 <= age <= 1.0)
        return {'present': True, 'valid_fresh': valid, 'sequence': sequence,
                'age_seconds': round(age, 3), 'stop': bool(stop),
                'observed_fresh': bool(observed_fresh), 'axes_zero': bool(axes_zero)}
    except (OSError, ValueError, struct.error) as error:
        return {'present': path.exists(), 'valid_fresh': False,
                'error': type(error).__name__}


def ros1_master_graph():
    code = '''import socket, rosgraph
socket.setdefaulttimeout(3.0)
master = rosgraph.Master('/lite3_vendor_readonly_audit')
pubs, subs, _ = master.getSystemState()
types = dict(master.getTopicTypes())
pubs, subs = dict(pubs), dict(subs)
for topic in ('/cmd_vel','/cmd_vel_corrected','/simple_cmd','/handle_state','/commands/joy_raw'):
 print(topic, types.get(topic,'<unknown>'), 'publishers=',pubs.get(topic,[]),'subscribers=',subs.get(topic,[]))
for node in ('/safety_node','/ros2qnx'):
 print('NODE', node)
 print(' publishes=', [(topic, types.get(topic,'<unknown>')) for topic,nodes in pubs.items() if node in nodes])
 print(' subscribes=', [(topic, types.get(topic,'<unknown>')) for topic,nodes in subs.items() if node in nodes])'''
    shell = ('source /opt/ros/noetic/setup.bash >/dev/null 2>&1; '
             'python3 -c ' + shlex.quote(code))
    return run(['/bin/bash', '-c', shell], 12)


def vendor_log_evidence():
    paths = glob.glob('/home/ysc/jy_exe/log/*/deeprcs.*.log')
    if not paths:
        return {'present': False}
    try:
        path = max(paths, key=lambda item: Path(item).stat().st_mtime_ns)
        # Limit reading even if the log has grown indefinitely. Mode/STOP
        # transitions older than this window must be checked separately.
        with open(path, 'rb') as stream:
            stream.seek(max(0, stream.seek(0, 2) - 2_000_000))
            tail = stream.read().decode('utf-8', 'replace')
        lines = [line for line in tail.splitlines() if LOG_TERMS.search(line)]
        return {'present': True, 'log_name': Path(path).name,
                'matching_lines': sanitize(lines[-80:])}
    except OSError as error:
        return {'present': True, 'error': type(error).__name__}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=('motion', 'perception'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = {'format': 'lite3_vendor_interface_audit/v1', 'role': args.role,
              'collected_at_utc': datetime.now(timezone.utc).isoformat(),
              'read_only': True,
              'host': run(['hostname'], 3), 'clock': run(['date', '-Is'], 3)}
    process_list = run(['ps', '-eo', 'pid,args'], 5)
    report['relevant_processes'] = [line for line in process_list.get('output', '').splitlines()
                                    if re.search(r'deeprcs|ros2qnx|safety_node|safety_supervisor|stop_relay_receiver|motion_stop_observer', line, re.I)
                                    and 'robot_vendor_interface_audit' not in line][:40]
    service_list = run(['systemctl', 'list-units', '--all', '--type=service', '--no-legend'], 6)
    report['relevant_service_units'] = [line for line in service_list.get('output', '').splitlines()
                                        if re.search(r'emotion-bot|safety|stop-relay|transfer', line, re.I)][:80]
    if args.role == 'motion':
        report['services'] = {name: run(['systemctl', 'is-active', name], 4)
                              for name in ('emotion-bot-stop-observer@ysc.service',
                                           'emotion-bot-hardware-core@ysc.service',
                                           'emotion-bot-hardware-telemetry@ysc.service')}
        report['observer'] = file_evidence('/opt/emotion-bot-motion/motion_stop_observer.py')
        report['vendor_log'] = vendor_log_evidence()
        report['udp'] = run(['ss', '-lunp'], 5)
    else:
        base = Path.home() / 'lite_cog/transfer/src/message_transformer/src'
        report['vendor_source'] = {
            name: file_evidence(base / name, include_lines=True)
            for name in ('ros2qnx.cpp', 'qnx2ros.cpp', 'nx2app.cpp')}
        report['ros1_graph'] = ros1_master_graph()
        report['bridge_topics'] = run(['/bin/bash', '-c',
            'source /opt/ros/noetic/setup.bash >/dev/null 2>&1; '
            'timeout -k 1s 4s rosparam get /topics 2>&1'], 6)
        report['safety_services'] = {name: run(['systemctl', 'is-active', name], 4)
                                     for name in ('emotion-bot-stop-relay-receiver@ysc.service',
                                                  'emotion-bot-safety-supervisor@ysc.service')}
        first = safety_record()
        time.sleep(0.2)
        second = safety_record()
        report['safety_record'] = {'first': first, 'second': second,
                                   'sequence_advanced': (first.get('sequence') is not None
                                                         and second.get('sequence', 0) > first['sequence'])}
    args.output.write_text(json.dumps(sanitize(report), indent=2, sort_keys=True) + '\n')
    print('Read-only vendor interface audit saved:', args.output)


if __name__ == '__main__':
    main()
