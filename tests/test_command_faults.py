from types import SimpleNamespace
from unittest.mock import Mock

import pytest


KINDS = ('gate', 'arbiter', 'relay')


def readiness(node, kind, value=True):
    callback = node._on_ready if kind == 'relay' else node._on_readiness
    callback(SimpleNamespace(data=value))
    node._on_ownership(SimpleNamespace(data=True))


def manual(node, kind, value=False):
    callback = node._on_manual if kind == 'relay' else node._on_manual_override
    callback(SimpleNamespace(data=value))


def arm(node, kind):
    readiness(node, kind)
    if kind != 'gate':
        node._physical_stop_verified = True  # Inert test only, never a live parameter.
        manual(node, kind)
    result = node._set_enabled(SimpleNamespace(data=True), SimpleNamespace())
    assert result.success
    node._publisher.reset_mock()


def nonzero(test_nodes):
    command = test_nodes.Twist()
    command.linear.x = 0.08
    command.angular.z = 0.12
    return command


def assert_zero(message):
    assert all(getattr(vector, axis) == 0.0 for vector in (message.linear, message.angular)
               for axis in ('x', 'y', 'z'))


@pytest.mark.parametrize('kind', KINDS)
def test_nodes_start_disabled(command_nodes, kind):
    node = command_nodes.build(kind)
    assert not node._enabled
    if kind != 'gate':
        assert node._manual_override
        assert node._manual_override_at is None
        assert not node._physical_stop_verified


@pytest.mark.parametrize('kind', KINDS)
def test_late_command_cannot_beat_watchdog_and_recovery_requires_rearm(command_nodes, kind):
    node = command_nodes.build(kind)
    arm(node, kind)
    command_nodes.clock.now += node._command_timeout + 0.01
    # Deliberately do not run the timer. The callback must check the deadline.
    node._on_command(nonzero(command_nodes))
    assert not node._enabled
    assert_zero(node._publisher.publish.call_args.args[0])
    count = node._publisher.publish.call_count
    readiness(node, kind)
    if kind != 'gate':
        manual(node, kind)
    node._on_command(nonzero(command_nodes))
    assert node._publisher.publish.call_count == count
    arm(node, kind)
    node._on_command(nonzero(command_nodes))
    assert node._publisher.publish.call_args.args[0].linear.x == 0.08


@pytest.mark.parametrize('kind', KINDS)
@pytest.mark.parametrize('bad_value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_command_latches_disabled(command_nodes, kind, bad_value):
    node = command_nodes.build(kind)
    arm(node, kind)
    command = nonzero(command_nodes)
    command.angular.x = bad_value
    node._on_command(command)
    assert not node._enabled
    assert_zero(node._publisher.publish.call_args.args[0])
    count = node._publisher.publish.call_count
    node._on_command(nonzero(command_nodes))
    assert node._publisher.publish.call_count == count


@pytest.mark.parametrize('kind', KINDS)
def test_readiness_expiry_is_checked_before_forwarding(command_nodes, kind):
    node = command_nodes.build(kind)
    arm(node, kind)
    command_nodes.clock.now += node._readiness_timeout + 0.01
    node._last_command_at = command_nodes.clock.now
    if kind != 'gate':
        manual(node, kind)
    node._on_command(nonzero(command_nodes))
    assert not node._enabled
    assert_zero(node._publisher.publish.call_args.args[0])


@pytest.mark.parametrize('kind', ['arbiter', 'relay'])
def test_missing_manual_state_refuses_enable(command_nodes, kind):
    node = command_nodes.build(kind)
    node._physical_stop_verified = True
    readiness(node, kind)
    result = node._set_enabled(SimpleNamespace(data=True), SimpleNamespace())
    assert not result.success
    assert not node._enabled
    assert 'manual' in result.message.lower()


@pytest.mark.parametrize('kind', ['arbiter', 'relay'])
def test_stale_manual_state_stops_before_forwarding(command_nodes, kind):
    node = command_nodes.build(kind)
    arm(node, kind)
    command_nodes.clock.now += node._manual_override_timeout + 0.01
    node._last_command_at = command_nodes.clock.now
    readiness(node, kind)
    node._on_command(nonzero(command_nodes))
    assert not node._enabled
    assert_zero(node._publisher.publish.call_args.args[0])


@pytest.mark.parametrize('kind', ['arbiter', 'relay'])
def test_manual_takeover_immediately_disables(command_nodes, kind):
    node = command_nodes.build(kind)
    arm(node, kind)
    manual(node, kind, True)
    assert not node._enabled
    assert_zero(node._publisher.publish.call_args.args[0])


@pytest.mark.parametrize('kind', ['arbiter', 'relay'])
def test_safety_loss_is_checked_on_command_without_timer(command_nodes, kind):
    node = command_nodes.build(kind)
    arm(node, kind)
    source = '_read_safety_record' if kind == 'arbiter' else '_safety_ok'
    setattr(node, source, Mock(return_value=(False, 'inert STOP active')))
    node._on_command(nonzero(command_nodes))
    assert not node._enabled
    assert_zero(node._publisher.publish.call_args.args[0])


@pytest.mark.parametrize('kind', KINDS)
def test_velocity_limits_and_zero_unused_axes_are_preserved(command_nodes, kind):
    node = command_nodes.build(kind)
    arm(node, kind)
    command = nonzero(command_nodes)
    command.linear.x = 5.0
    command.angular.z = -5.0
    command.linear.y = 3.0
    node._on_command(command)
    output = node._publisher.publish.call_args.args[0]
    assert output.linear.x == 0.10
    assert output.angular.z == -0.20
    assert output.linear.y == output.linear.z == output.angular.x == output.angular.y == 0.0


def test_gate_fault_zero_does_not_keep_downstream_armed(command_nodes):
    nodes = {kind: command_nodes.build(kind) for kind in KINDS}
    for kind, node in nodes.items():
        arm(node, kind)
    nodes['gate']._publisher.publish.side_effect = nodes['arbiter']._on_command
    nodes['arbiter']._publisher.publish.side_effect = nodes['relay']._on_command
    nodes['gate']._on_command(nonzero(command_nodes))
    assert nodes['relay']._publisher.publish.call_args.args[0].linear.x == 0.08
    # All callbacks see expired source commands, including the gate's final zero.
    command_nodes.clock.now += 0.31
    nodes['gate']._check_watchdog()
    assert all(not node._enabled for node in nodes.values())
    assert_zero(nodes['relay']._publisher.publish.call_args.args[0])
