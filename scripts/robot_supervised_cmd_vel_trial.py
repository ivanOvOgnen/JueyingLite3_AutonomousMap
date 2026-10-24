#!/usr/bin/env python3
"""Bounded Lite3 vendor /cmd_vel commissioning trial on the perception host.

Run only while standing beside the robot with the vendor app STOP in reach.
The default is read-only. --apply requires an interactive confirmation and
sends at most 0.05 m/s forward for at most three seconds. It is not navigation.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import sys
import time


SAFETY_PATH = Path('/dev/shm/emotion_bot_lite3_safety_state')
HEADER = struct.Struct('<4sIQQI')
WIRE = struct.Struct('<I???5xQ')


def safety_state(path=SAFETY_PATH):
    try:
        with path.open('rb') as stream:
            first = stream.read(HEADER.size)
            payload = stream.read(WIRE.size)
            stream.seek(0)
            second = stream.read(HEADER.size)
        magic, version, sequence, receive_ns, length = HEADER.unpack(first)
        record_version, stop, observed_fresh, axes_zero, _ = WIRE.unpack(payload)
        age = (time.monotonic_ns() - receive_ns) / 1e9
        valid = (first == second and magic == b'EBL3' and version == 1 and
                 sequence % 2 == 0 and length == WIRE.size and
                 record_version == 0x00010000 and 0 <= age <= 0.25)
        return {'valid': valid, 'stop': bool(stop), 'observed_fresh': bool(observed_fresh),
                'axes_zero': bool(axes_zero), 'age_seconds': round(age, 3),
                'sequence': sequence}
    except (OSError, ValueError, struct.error):
        return {'valid': False, 'stop': None, 'observed_fresh': False,
                'axes_zero': False, 'age_seconds': None, 'sequence': None}


def graph_state():
    import rosgraph
    socket.setdefaulttimeout(3.0)
    master = rosgraph.Master('/lite3_supervised_cmd_vel_trial')
    publishers, subscribers, _ = master.getSystemState()
    publishers, subscribers = dict(publishers), dict(subscribers)
    return {'cmd_vel_publishers': publishers.get('/cmd_vel', []),
            'cmd_vel_subscribers': subscribers.get('/cmd_vel', []),
            'corrected_publishers': publishers.get('/cmd_vel_corrected', [])}


def preflight():
    graph = graph_state()
    first = safety_state()
    time.sleep(0.15)
    second = safety_state()
    reasons = []
    if graph['cmd_vel_publishers']:
        reasons.append('Another ROS 1 /cmd_vel publisher is present')
    if '/ros2qnx' not in graph['cmd_vel_subscribers']:
        reasons.append('Vendor /ros2qnx is not subscribed to /cmd_vel')
    if '/safety_node' not in graph['corrected_publishers']:
        reasons.append('Expected vendor /safety_node output is missing')
    if not (first['valid'] and second['valid'] and second['sequence'] > first['sequence']):
        reasons.append('STOP observer record is missing, stale, or not advancing')
    if second['stop'] or not second['observed_fresh'] or not second['axes_zero']:
        reasons.append('STOP is active, observation stale, or controller axes are nonzero')
    return {'graph': graph, 'safety_first': first, 'safety_second': second,
            'ready': not reasons, 'reasons': reasons}


def run_trial(duration, linear_x):
    # The perception host's ROS 1 graph is local; advertise a loopback URI so
    # the local vendor subscriber can connect without the robot Wi-Fi address.
    os.environ.setdefault('ROS_MASTER_URI', 'http://127.0.0.1:11311')
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node('lite3_supervised_cmd_vel_trial', anonymous=True,
                    disable_signals=True, disable_rosout=True)
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    deadline = time.monotonic() + 2.0
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    if publisher.get_num_connections() != 1:
        raise RuntimeError('Expected exactly one /cmd_vel subscriber; no command sent')

    start = time.monotonic()
    nonzero_count = 0
    stop_observed = False
    fault = None
    command = Twist()
    command.linear.x = linear_x
    zero = Twist()
    try:
        while not rospy.is_shutdown() and time.monotonic() - start < duration:
            state = safety_state()
            if state['stop'] is True and state['valid'] and state['observed_fresh']:
                stop_observed = True
                break
            if not state['valid'] or not state['observed_fresh'] or not state['axes_zero']:
                fault = 'STOP observation became stale or controller axes moved'
                break
            if publisher.get_num_connections() != 1:
                fault = 'Vendor subscriber count changed'
                break
            publisher.publish(command)
            nonzero_count += 1
            time.sleep(0.05)
    finally:
        # Repeated zeroes cover the normal ROS process-exit path. The vendor
        # velocity-command timeout still needs a measured fault test.
        for _ in range(10):
            publisher.publish(zero)
            time.sleep(0.05)
    return {'elapsed_seconds': round(time.monotonic() - start, 3),
            'nonzero_messages': nonzero_count, 'stop_observed': stop_observed,
            'fault': fault, 'final_safety': safety_state()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Perform the bounded motion trial')
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--linear-x', type=float, default=0.05)
    parser.add_argument('--output', type=Path, default=Path('/tmp/lite3_supervised_cmd_vel_trial.json'))
    args = parser.parse_args()
    if not (math.isfinite(args.duration) and 0 < args.duration <= 3.0):
        parser.error('duration must be > 0 and <= 3 seconds')
    if not (math.isfinite(args.linear_x) and 0 < args.linear_x <= 0.05):
        parser.error('linear-x must be > 0 and <= 0.05 m/s')

    report = {'format': 'lite3_supervised_cmd_vel_trial/v1', 'apply': args.apply,
              'limits': {'duration_seconds': args.duration, 'linear_x_mps': args.linear_x}}
    try:
        report['preflight'] = preflight()
    except Exception as error:
        report['preflight'] = {'ready': False, 'reasons': [str(error)]}
    print(json.dumps(report, indent=2), flush=True)
    if not args.apply:
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        return 0 if report['preflight']['ready'] else 2
    if not report['preflight']['ready']:
        print('Refusing motion: preflight is not ready.', file=sys.stderr)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        return 2

    print('Stand beside the robot in a clear area. Confirm it is standing, Auto Mode is ON,')
    print('and the vendor app STOP is open and reachable. Press app STOP during this trial.')
    try:
        confirmation = input('Type ARM and press Enter to begin (Ctrl+C cancels): ')
    except (EOFError, KeyboardInterrupt):
        print('\nTrial canceled before any command.', flush=True)
        confirmation = ''
    if confirmation != 'ARM':
        print('Confirmation not given; no command sent.', file=sys.stderr)
        report['confirmation'] = 'declined'
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        return 2
    # Recheck after the operator confirmation; an old clear state is invalid.
    report['preflight_after_confirmation'] = preflight()
    if not report['preflight_after_confirmation']['ready']:
        print('State changed during confirmation; no command sent.', file=sys.stderr)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        return 2
    previous_alarm_handler = signal.getsignal(signal.SIGALRM)
    def trial_deadline(_signum, _frame):
        raise TimeoutError('Motion trial exceeded its eight-second process deadline')
    signal.signal(signal.SIGALRM, trial_deadline)
    signal.setitimer(signal.ITIMER_REAL, 8.0)
    try:
        report['trial'] = run_trial(args.duration, args.linear_x)
    except KeyboardInterrupt:
        report['trial'] = {'fault': 'Operator interrupted the trial', 'stop_observed': False}
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print('Trial interrupted; report saved:', args.output, flush=True)
        return 130
    except Exception as error:
        report['trial'] = {'fault': str(error), 'stop_observed': False}
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_alarm_handler)
    try:
        moved = input('Did the robot visibly move? [yes/no/unknown]: ').strip().lower()
        halted = input('Did the app STOP halt motion? [yes/no/unknown]: ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        moved = halted = 'unknown'
    report['operator_observation'] = {
        'robot_moved': moved if moved in ('yes', 'no') else 'unknown',
        'app_stop_halted_motion': halted if halted in ('yes', 'no') else 'unknown',
    }
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print('Trial report:', args.output)
    print(json.dumps(report['trial'], indent=2))
    return 0 if (report['trial'].get('stop_observed') and not report['trial'].get('fault')
                 and report['operator_observation']['robot_moved'] == 'yes'
                 and report['operator_observation']['app_stop_halted_motion'] == 'yes') else 2


if __name__ == '__main__':
    raise SystemExit(main())
