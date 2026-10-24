import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/check_walk_readiness.py'
SPEC = importlib.util.spec_from_file_location('check_walk_readiness', SCRIPT)
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


def test_report_cannot_hide_isolated_motor_or_missing_stop():
    report = {'api_status': {'http_status': 200, 'body': {'components': {
        'navigation': {'state': 'ready', 'active_goal_state': 'finished_6', 'map_known_fraction': .377},
        'motion_output': {'state': 'not_ready'},
        'safety_stop': {'physical_estop_verified': False},
    }}}, 'preflight': {'output': ''}, 'commissioning_metadata': {'present': False}}
    blockers = CHECK.assess(report)
    assert any('Motor output is isolated' in item for item in blockers)
    assert any('independent physical stop' in item for item in blockers)
    assert not any('map' in item.lower() for item in blockers)
    assert any('37.7%' in item for item in CHECK.advisories(report))


def test_topic_count_parser():
    output = '--- /autonomy/cmd_vel ---\nType: geometry_msgs/msg/Twist\nPublisher count: 1\nSubscription count: 0\n--- /map ---\n'
    assert CHECK.topic_counts(output, '/autonomy/cmd_vel') == (1, 0)
    assert CHECK.topic_counts(output, '/cmd_vel') is None
