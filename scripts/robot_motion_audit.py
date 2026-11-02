#!/usr/bin/env python3
"""Collect a bounded, read-only Lite3 motion-path report on the perception host.

This program never publishes a ROS message, calls a control endpoint, changes a
parameter, arms a motor, or claims to verify the physical stop circuit.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
from datetime import datetime, timezone
from urllib.request import ProxyHandler, build_opener


PRIVATE_KEY = re.compile(r'(password|passwd|secret|token|authorization|cookie|credential|private.key)', re.I)
PRIVATE_LINE = re.compile(r'(?im)^([^\n]*(?:password|passwd|secret|token|authorization|cookie|credential|private.key)[^\n]*)$')
PROJECT = Path.home() / 'lite3_autonomy_ws'
PACKAGE = 'lite3_odom_tf_adapter'
FILES = (
    'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/robot_api.py',
    'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/motion_supervisor.py',
    'src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/vendor_cmd_vel_bridge.py',
    'src/lite3_odom_tf_adapter/config/vendor_cmd_vel_bridge.yaml',
    'src/lite3_odom_tf_adapter/config/nav2_camera_overrides.yaml',
    'src/lite3_odom_tf_adapter/launch/navigation.launch.py',
    'scripts/start_vendor_cmd_vel_bridge.sh',
)


def redact(value):
    if isinstance(value, dict):
        return {key: ('[redacted]' if PRIVATE_KEY.search(str(key)) else redact(item))
                for key, item in value.items()}
    if isinstance(value, list):
        return [redact(item) for item in value]
    if isinstance(value, str):
        return PRIVATE_LINE.sub('[redacted line]', value)
    return value


def run(args, timeout):
    try:
        result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=timeout, check=False)
        return {'exit_code': result.returncode, 'output': redact(result.stdout[-100000:])}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'error': type(error).__name__, 'detail': redact(str(error))}


def api_status():
    opener = build_opener(ProxyHandler({}))
    try:
        with opener.open('http://127.0.0.1:8080/api/v1/status', timeout=4) as response:
            return {'http_status': response.status, 'body': redact(json.load(response))}
    except (OSError, ValueError) as error:
        return {'error': type(error).__name__, 'detail': redact(str(error))}


def file_hashes():
    result = {}
    for relative in FILES:
        path = PROJECT / relative
        try:
            data = path.read_bytes()
            result[relative] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        except OSError as error:
            result[relative] = {'error': type(error).__name__}
    return result


def commissioning_metadata():
    path = Path.home() / '.config/lite3_autonomy/commissioning.json'
    if not path.is_file():
        return {'present': False}
    try:
        record = json.loads(path.read_text())
        if not isinstance(record, dict):
            return {'present': True, 'valid_json_object': False}
        fields = ('vendor_exclusivity_verified', 'independent_stop_verified',
                  'downstream_timeout_verified', 'footprint_measured', 'visibility_reviewed')
        return {'present': True, 'format': record.get('format'),
                'commissioning_id_present': bool(record.get('commissioning_id')),
                'checks': {field: record.get(field) is True for field in fields},
                'allowed_map_count': len(record.get('allowed_map_ids', []))
                if isinstance(record.get('allowed_map_ids'), list) else 0}
    except (OSError, ValueError) as error:
        return {'present': True, 'error': type(error).__name__}


def status_findings(status):
    body = status.get('body', {})
    components = body.get('components', {}) if isinstance(body, dict) else {}
    if not isinstance(components, dict):
        components = {}
    motion = components.get('motion_output', {})
    navigation = components.get('navigation', {})
    stop = components.get('safety_stop', {})
    return {
        'motor_output_state': motion.get('state') if isinstance(motion, dict) else None,
        'motor_output_reason': motion.get('reason') if isinstance(motion, dict) else None,
        'bridge_profile': motion.get('bridge_profile') if isinstance(motion, dict) else None,
        'navigation_state': navigation.get('state') if isinstance(navigation, dict) else None,
        'physical_estop_claimed_verified': stop.get('physical_estop_verified')
        if isinstance(stop, dict) else None,
        'interpretation': 'An accepted Nav2 goal is planner evidence only; physical motion '
                          'requires separately commissioned command ownership, stop, and timeout.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preflight', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    if not options.preflight.is_file():
        parser.error('preflight script does not exist')
    status = api_status()
    report = {
        'format': 'lite3_motion_audit/v1',
        'collected_at_utc': datetime.now(timezone.utc).isoformat(),
        'read_only': True,
        'physical_stop_verified_by_this_audit': False,
        'api_status': status,
        'status_findings': status_findings(status),
        'commissioning_metadata': commissioning_metadata(),
        'project_file_hashes': file_hashes(),
        'services': {name: run(['systemctl', 'is-active', name], 4)
                     for name in ('transfer.service', 'transfer_ros2.service', 'realsense_ros2.service')},
        'preflight': run(['bash', str(options.preflight)], 65),
    }
    options.output.write_text(json.dumps(redact(report), indent=2, sort_keys=True) + '\n')
    print('Read-only motion audit saved:', options.output)


if __name__ == '__main__':
    main()
