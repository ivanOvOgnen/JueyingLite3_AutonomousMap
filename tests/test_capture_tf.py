import importlib.util
from pathlib import Path
import math
import numpy as np
import pytest

SPEC=importlib.util.spec_from_file_location('capture_tf_tests',Path(__file__).resolve().parents[1]/'ros2_ws/src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/capture_tf.py')
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


def tf(parent,child,stamp,x=0,yaw=0):
    return {'parent':parent,'child':child,'stamp_ns':stamp,'translation':{'x':x,'y':0,'z':0},
            'rotation':{'x':0,'y':0,'z':math.sin(yaw/2),'w':math.cos(yaw/2)}}


def test_interpolated_translation_slerp_composition_and_inverse():
    ledger=M.CaptureTf(max_gap_seconds=2)
    ledger.add(tf('map','base',1_000_000_000,x=0,yaw=0))
    ledger.add(tf('map','base',3_000_000_000,x=2,yaw=math.pi))
    ledger.add(tf('base','camera',0,x=1),static=True)
    result=ledger.lookup('map','camera',2_000_000_000)
    assert result[:3,3]==pytest.approx([1,1,0])
    assert result[:3,:3]@np.array([1,0,0])==pytest.approx([0,1,0])
    assert ledger.lookup('camera','map',2_000_000_000)@result==pytest.approx(np.eye(4))
    with pytest.raises(ValueError):ledger.lookup('map','camera',3_000_000_001)


def test_nearby_transform_and_large_gap_do_not_prove_lookup():
    ledger=M.CaptureTf()
    ledger.add(tf('map','base',1_000_000_000))
    ledger.add(tf('map','base',2_000_000_000))
    with pytest.raises(ValueError):ledger.lookup('map','base',1_500_000_000)


def test_conflicting_parents_duplicate_transform_and_cycles_are_rejected():
    ledger=M.CaptureTf();ledger.add(tf('map','base',0),static=True)
    with pytest.raises(ValueError):ledger.add(tf('odom','base',0),static=True)
    with pytest.raises(ValueError):ledger.add(tf('map','base',0,x=2),static=True)
    ledger=M.CaptureTf();ledger.add(tf('map','base',0),static=True)
    with pytest.raises(ValueError):ledger.add(tf('base','map',0),static=True)
