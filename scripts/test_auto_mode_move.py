#!/usr/bin/env python3
"""Attended, bounded Auto Mode movement check using the existing vendor trial.

Default: read-only preflight. --apply stages the robot helper, asks the operator
to arm it at the robot, sends at most 0.05 m/s for three seconds, and records
whether the DEEP app STOP halted it. This does not send a Nav2 goal.
"""

import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='run the attended motion check')
    parser.add_argument('--motion-host', help='verified motion computer IPv4')
    parser.add_argument('--perception-host', help='verified perception computer IPv4')
    args = parser.parse_args()
    runner = Path(__file__).with_name('run_supervised_vendor_trial.py')
    command = [sys.executable, str(runner)]
    if args.apply:
        command.append('--apply')
    for option in ('motion_host', 'perception_host'):
        value = getattr(args, option)
        if value:
            command.extend(('--' + option.replace('_', '-'), value))
    return subprocess.call(command)


if __name__ == '__main__':
    raise SystemExit(main())
