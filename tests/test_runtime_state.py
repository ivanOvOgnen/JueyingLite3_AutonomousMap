import importlib.util
from pathlib import Path
import pytest

SPEC=importlib.util.spec_from_file_location('runtime_state_tests',Path(__file__).resolve().parents[1]/'ros2_ws/src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/runtime_state.py')
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


def test_epoch_rejects_queued_old_data_and_clears_lease():
    state=M.RuntimeState(clock=lambda:10)
    state.acquire('a')
    old=state.epoch
    state.reset('new map')
    assert not state.observe('scan',1,1,epoch=old)
    assert not state.valid_lease()
    assert state.health('scan')['state']=='not_ready'


def test_receipt_freshness_does_not_hide_old_or_frozen_acquisition():
    state=M.RuntimeState(clock=lambda:10)
    state.observe('scan',1_000_000_000,10_000_000_000)
    assert state.health('scan')['receipt_age_seconds']==0
    assert state.health('scan')['state']=='not_ready'
    state.observe('scan',10_000_000_000,10_000_000_000)
    assert state.health('scan')['state']=='ready'
    state.observe('scan',10_000_000_000,10_000_000_000)
    assert state.health('scan')['state']=='not_ready'


def test_operator_lease_is_exclusive_expires_and_cannot_resurrect():
    now=[10.0]
    state=M.RuntimeState(clock=lambda:now[0], lease_seconds=.8)
    token=state.acquire('a')
    with pytest.raises(ValueError):state.acquire('b')
    with pytest.raises(ValueError):state.renew('a',None,state.epoch)
    with pytest.raises(ValueError):state.renew('a','wrong',state.epoch)
    now[0]+=.4
    state.renew('a',token,state.epoch)
    now[0]+=.81
    assert not state.valid_lease()
    with pytest.raises(ValueError):state.renew('a',token,state.epoch)
    with pytest.raises(ValueError):state.acquire('a')
    state.reset('explicit stop/restart')
    assert state.acquire('b')!=token


def test_ui_lease_survives_short_stall_while_motion_heartbeat_fails_closed():
    now = [10.0]
    state = M.RuntimeState(clock=lambda: now[0], lease_seconds=5.0)
    state.acquire('operator')
    now[0] += 0.9
    assert state.valid_lease()
    assert not state.fresh_lease(0.8)
    now[0] += 4.11
    assert not state.valid_lease()
