import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest

SPEC=importlib.util.spec_from_file_location('depth_projection_tests',Path(__file__).resolve().parents[1]/'ros2_ws/src/lite3_odom_tf_adapter/lite3_odom_tf_adapter/depth_projection.py')
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


def pair(encoding='16UC1',big=False):
    header=NS(stamp=NS(sec=10,nanosec=0),frame_id='depth')
    dtype=('>' if big else '<')+('u2' if encoding=='16UC1' else 'f4')
    data=np.array([[1000,2000],[0,3000]],dtype=dtype) if encoding=='16UC1' else np.array([[1,2],[np.nan,3]],dtype=dtype)
    image=NS(header=header,width=2,height=2,encoding=encoding,is_bigendian=big,step=2*data.dtype.itemsize,data=data.tobytes())
    info=NS(header=header,width=2,height=2,p=[2,0,.5,0,0,2,.5,0,0,0,1,0])
    return image,info


@pytest.mark.parametrize('encoding',['16UC1','32FC1'])
@pytest.mark.parametrize('big',[False,True])
def test_metric_deprojection_preserves_optical_convention(encoding,big):
    image,info=pair(encoding,big)
    points,health=M.project_depth(image,info,now_ns=10_100_000_000,stride=1,max_range=4)
    assert points.tolist()==pytest.approx(np.array([[-.25,-.25,1],[.5,-.5,2],[.75,.75,3]]))
    assert health['stamp_ns']==10_000_000_000
    assert health['valid_fraction']==.75


@pytest.mark.parametrize('kind',['stale','future','unpaired','stride','encoding','intrinsics','empty'])
def test_invalid_observations_cannot_make_a_cloud(kind):
    image,info=pair()
    now=10_100_000_000
    if kind=='stale':now+=1_000_000_000
    elif kind=='future':now-=1_000_000_000
    elif kind=='unpaired':info.header=NS(stamp=NS(sec=9,nanosec=0),frame_id='depth')
    elif kind=='stride':image.step=1
    elif kind=='encoding':image.encoding='8UC1'
    elif kind=='intrinsics':info.p[0]=0
    else:image.data=bytes(8)
    with pytest.raises(ValueError):M.project_depth(image,info,now_ns=now,stride=1)


def test_measured_camera_transport_delay_is_accepted_but_one_second_is_not():
    image,info=pair()
    _,health=M.project_depth(image,info,now_ns=10_750_000_000,stride=1)
    assert health['age_seconds']==pytest.approx(.75)
    with pytest.raises(ValueError,match='stale'):
        M.project_depth(image,info,now_ns=11_050_000_000,stride=1)
