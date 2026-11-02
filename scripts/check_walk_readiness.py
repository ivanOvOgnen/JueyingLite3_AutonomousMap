#!/usr/bin/env python3
"""Explain walk blockers from a read-only robot motion audit.

This tool changes nothing. It deliberately cannot certify the physical stop,
vendor control handoff, obstacle coverage, or measured robot movement.
"""

import argparse
import json
from pathlib import Path
import re
import sys


def topic_counts(preflight, topic):
    pattern = r'(?m)^--- ' + re.escape(topic) + r' ---\n(.*?)(?=^--- |^=== |\Z)'
    match = re.search(pattern, preflight, re.S | re.M)
    if not match:
        return None
    publishers = re.search(r'^Publisher count: (\d+)$', match.group(1), re.M)
    subscribers = re.search(r'^Subscription count: (\d+)$', match.group(1), re.M)
    if not publishers or not subscribers:
        return None
    return int(publishers.group(1)), int(subscribers.group(1))


def assess(report):
    blockers = []
    status = report.get('api_status', {}).get('body', {})
    components = status.get('components', {}) if isinstance(status, dict) else {}
    if not isinstance(components, dict):
        components = {}
    nav = components.get('navigation') or {}
    motion = components.get('motion_output') or {}
    stop = components.get('safety_stop') or {}
    preflight = report.get('preflight', {}).get('output', '')

    if report.get('api_status', {}).get('http_status') != 200:
        blockers.append('Robot API status was unavailable; verify the API before planning.')
    if nav.get('state') != 'ready':
        blockers.append('Localization/Nav2 is not ready on the active map.')
    if nav.get('active_goal_state') not in (None, 'idle', 'stopped', 'rejected',
                                             'canceled', 'aborted', 'finished_2',
                                             'finished_4', 'finished_5', 'finished_6'):
        blockers.append('A goal is still active or unresolved; settle it before a deployment.')
    if motion.get('state') != 'armed':
        blockers.append('Motor output is isolated; the accepted Nav2 goal cannot move the robot.')
    if stop.get('physical_estop_verified') is not True:
        blockers.append('An independent physical stop has not been verified.')
    if topic_counts(preflight, '/autonomy/manual_override') in (None, (0, 1)):
        blockers.append('No authoritative manual/autonomy ownership publisher is present.')
    if topic_counts(preflight, '/autonomy/cmd_vel_arbited') is None or \
            topic_counts(preflight, '/autonomy/cmd_vel_arbited')[0] == 0:
        blockers.append('The autonomous command arbiter is not publishing.')
    if topic_counts(preflight, '/cmd_vel') is None or \
            topic_counts(preflight, '/cmd_vel')[1] == 0:
        blockers.append('The guarded command output has no ROS 2 subscriber/bridge.')
    if re.search(r'ROS 1 /cmd_vel graph:.*?Publishers:\s*None', preflight, re.S):
        blockers.append('The vendor ROS 1 /cmd_vel consumer has no publisher.')
    metadata = report.get('commissioning_metadata', {})
    if not metadata.get('present'):
        blockers.append('No motion commissioning record exists; do not create one without measured evidence.')
    return blockers


def advisories(report):
    status = report.get('api_status', {}).get('body', {})
    components = status.get('components', {}) if isinstance(status, dict) else {}
    nav = components.get('navigation', {}) if isinstance(components, dict) else {}
    known = nav.get('map_known_fraction') if isinstance(nav, dict) else None
    notes = []
    if isinstance(known, (float, int)) and known < 0.5:
        notes.append('The map has {:.1%} known cells. Review the specific start, goal, '
                     'connected free-space route, footprint clearance, and live obstacles; '
                     'this percentage alone cannot approve or reject a route.'.format(known))
    return notes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path, help='robot_motion_audit_*.json from collect_robot_motion_audit.py')
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    if report.get('format') != 'lite3_motion_audit/v1':
        parser.error('expected a lite3_motion_audit/v1 report')
    blockers = assess(report)
    print('Robot walk readiness: BLOCKED' if blockers else 'Automated checks: no blocker found')
    for index, blocker in enumerate(blockers, 1):
        print('{}. {}'.format(index, blocker))
    for note in advisories(report):
        print('Map review: ' + note)
    print('Always required before physical walking: supervised vendor handoff and independent-stop '
          'tests, observed timeout/manual takeover, reviewed obstacle coverage and footprint, '
          'and a measured short-goal trial. This script cannot certify those physical results.')
    return 2 if blockers else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print('Readiness check failed:', error, file=sys.stderr)
        sys.exit(1)
