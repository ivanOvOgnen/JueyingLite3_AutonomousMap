import importlib.util
from pathlib import Path
import pytest

SPEC=importlib.util.spec_from_file_location('supervisor_tests',Path(__file__).resolve().parents[1]/'ros2_ws/src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/motion_supervisor.py')
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


@pytest.fixture
def policy():
    now=[10.0]
    record={'format':'lite3_motion_commissioning/v1','commissioning_id':'SYNTHETIC-ONLY',
            'allowed_map_ids':['map'],'software_id':'software'}
    for key in ('vendor_exclusivity_verified','independent_stop_verified','downstream_timeout_verified',
                'footprint_measured','visibility_reviewed'):record[key]=True
    p=M.SupervisorPolicy(record,clock=lambda:now[0],wall_clock=lambda:int(now[0]*1e9))
    p.update_vendor({'instance_id':'synthetic','sequence':1,'source_unix_ns':int(now[0]*1e9),
                     'mode':'autonomy','posture':'standing','manual_input_active':False,'stop_clear':True,
                     'exclusive_owner':'lite3_autonomy','commissioning_id':'SYNTHETIC-ONLY'})
    p.update_context({'ready':True,'operator_lease_valid':True,'map_id':'map','software_id':'software'})
    return p,now


def test_unknown_vendor_and_uncommissioned_never_permit():
    p=M.SupervisorPolicy()
    with pytest.raises(ValueError):p.arm_transaction(lambda *_:True)
    assert not p.status()['ownership_permit']


@pytest.mark.parametrize('record', [[], ['invalid'], 'invalid', 42])
def test_malformed_commissioning_record_inhibits(record):
    p = M.SupervisorPolicy(record)
    assert p.inhibit_reason() == 'No valid commissioning record'
    assert not p.status()['ownership_permit']


def test_transaction_arms_gate_last_and_requires_status(policy):
    p,now=policy
    calls=[]
    def enable(name,value):
        calls.append((name,value))
        p.stages[name]={'enabled':value,'rx':now[0]}
        return True
    p.arm_transaction(enable)
    assert calls==[('relay',True),('arbiter',True),('gate',True)]
    assert p.status()['enabled']
    now[0]+=.31
    assert not p.status()['ownership_permit']
    assert p.state=='stopped'
    now[0]=10
    assert not p.status()['ownership_permit'] # Recovery never re-arms.


def test_partial_arm_failure_rolls_back_all_stages(policy):
    p,_=policy;calls=[]
    def enable(name,value):
        calls.append((name,value))
        return name!='arbiter' or not value
    with pytest.raises(ValueError):p.arm_transaction(enable)
    assert calls[-3:]==[('gate',False),('arbiter',False),('relay',False)]
    assert not p.status()['enabled']


def test_stop_during_enable_cannot_be_overwritten(policy):
    p,_=policy
    def enable(name,value):
        if value:p.stop('operator STOP')
        return True
    with pytest.raises(ValueError):p.arm_transaction(enable)
    assert not p.status()['ownership_permit']


@pytest.mark.parametrize('field,value',[('mode','manual'),('manual_input_active',True),('stop_clear',False),
                                        ('exclusive_owner','other'),('posture','lying')])
def test_manual_or_stop_condition_inhibits(policy,field,value):
    p,_=policy
    packet={**p.vendor,field:value,'sequence':2}
    p.update_vendor(packet)
    with pytest.raises(ValueError):p.arm_transaction(lambda *_:True)


def test_vendor_replay_does_not_refresh_permissions(policy):
    p,_=policy
    p.update_vendor(dict(p.vendor))
    assert p.vendor=={}
    assert not p.status()['ownership_permit']
