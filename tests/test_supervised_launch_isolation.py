"""Static topology guard: supervised trials must have one command relay only."""

from pathlib import Path


LAUNCH = (Path(__file__).resolve().parents[1] /
          'ros2_ws/src/lite3_odom_tf_adapter/launch/navigation.launch.py')


def test_supervised_profile_excludes_legacy_motion_chain():
    source = LAUNCH.read_text(encoding='utf-8')
    supervised_branch = source.split('if supervised:', 1)[1].split(
        'return [perception, gate, arbiter, relay, supervisor, nav2_group]', 1)[0]
    assert 'return [perception, nav2_group, trial_relay]' in supervised_branch
    assert 'gate,' not in supervised_branch
    assert 'arbiter,' not in supervised_branch
    assert 'relay,' not in supervised_branch.replace('trial_relay', '')
    assert 'supervisor,' not in supervised_branch
