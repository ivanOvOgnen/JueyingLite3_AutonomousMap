import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/robot_motion_audit.py'
SPEC = importlib.util.spec_from_file_location('robot_motion_audit', SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def test_report_redacts_nested_credentials_and_preflight_lines():
    report = AUDIT.redact({
        'api': {'access_token': 'sensitive', 'nested': [{'password': 'sensitive'}]},
        'preflight': 'node --password=sensitive\nROS graph healthy\n',
    })
    assert report['api']['access_token'] == '[redacted]'
    assert report['api']['nested'][0]['password'] == '[redacted]'
    assert 'sensitive' not in report['preflight']
    assert 'ROS graph healthy' in report['preflight']


def test_commissioning_metadata_never_treats_missing_record_as_verified(monkeypatch, tmp_path):
    monkeypatch.setattr(AUDIT.Path, 'home', lambda: tmp_path)
    assert AUDIT.commissioning_metadata() == {'present': False}


def test_status_findings_keep_planner_and_motor_states_separate():
    findings = AUDIT.status_findings({'body': {'components': {
        'navigation': {'state': 'ready'},
        'motion_output': {'state': 'not_ready', 'reason': 'isolated'},
        'safety_stop': {'physical_estop_verified': False},
    }}})
    assert findings['navigation_state'] == 'ready'
    assert findings['motor_output_state'] == 'not_ready'
    assert findings['physical_estop_claimed_verified'] is False
