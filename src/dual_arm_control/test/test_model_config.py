from pathlib import Path
import pytest
from dual_arm_control.model_config import validate_handoff, fixed_point_in_root


def test_shared_demo_b_targets_map_to_same_world_point():
    root=Path(__file__).resolve().parents[2]
    model=(root/'dual_arm_description/urdf/dual_arm_demo.urdf').read_text()
    a=dict(base_frame='arm1_base',b_place=[.15,-.04,.02])
    b=dict(base_frame='arm2_base',b_pick=[.15,-.04,.02,1.,0.,0.,0.])
    validate_handoff(model,a,b)
    with pytest.raises(ValueError,match='B targets disagree'):
        validate_handoff(model,a,{**b,'b_pick':[.20,-.04,.02,1.,0.,0.,0.]})
    with pytest.raises(ValueError,match='missing'):
        fixed_point_in_root(model,'missing_base',[0,0,0])


def test_moving_base_cannot_be_used_as_static_handoff_calibration():
    model='<robot><link name="world"/><link name="base"/><joint name="j" type="revolute"><parent link="world"/><child link="base"/></joint></robot>'
    with pytest.raises(ValueError,match='fixed mounting'):
        fixed_point_in_root(model,'base',[0,0,0])
