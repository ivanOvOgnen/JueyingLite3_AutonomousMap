"""Inert checks for the single-purpose robot hotfix deployer."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/robot_apply_amcl_hotfix.py'
SPEC = importlib.util.spec_from_file_location('robot_apply_amcl_hotfix_test', SCRIPT)
HOTFIX = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HOTFIX)


def status(*, goal='stopped', mapping=False, navigation=False, motion='not_ready'):
    return {'components': {
        'mapping': {'process_running': mapping},
        'navigation_stack': {'process_running': navigation},
        'navigation': {'active_goal_state': goal, 'unresolved_goal_count': 0},
        'motion_output': {'state': motion},
    }}


def test_hotfix_requires_idle_isolated_project():
    HOTFIX.check_idle(status())
    HOTFIX.check_idle(status(navigation=True))
    for blocked in (status(goal='accepted'), status(mapping=True), status(motion='ready')):
        with pytest.raises(RuntimeError):
            HOTFIX.check_idle(blocked)
    pending = status()
    pending['components']['safety_stop'] = {'cleanup_pending': True}
    with pytest.raises(RuntimeError):
        HOTFIX.check_idle(pending)


def test_hotfix_names_unresolved_goal():
    busy = status(goal='canceling', navigation=True)
    busy['components']['navigation']['unresolved_goal_count'] = 1
    with pytest.raises(RuntimeError, match='state=canceling, unresolved=1'):
        HOTFIX.check_idle(busy)


def test_hotfix_path_check_is_python38_compatible(tmp_path):
    inside = tmp_path / 'install' / 'robot_api.py'
    outside = tmp_path.parent / 'elsewhere.py'
    assert HOTFIX.is_within(inside, tmp_path)
    assert not HOTFIX.is_within(outside, tmp_path)
