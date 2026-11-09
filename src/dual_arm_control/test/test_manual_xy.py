from copy import deepcopy
import math
import time
from pathlib import Path

import pytest
import yaml

from dual_arm_control.manual_config import configure, target_xy, check_plane
from dual_arm_control.xy_calibration import fit_references, Kinematics, verify_evidence, calibration

ROOT = Path(__file__).resolve().parents[2]
URDF = (ROOT/'dual_arm_description/urdf/arm1.urdf').read_text()


def demo():
    return yaml.safe_load((ROOT/'dual_arm_control/config/manual_xy_demo.yaml').read_text())


def test_xy_uses_configured_plane_and_checks_bounds():
    c, _ = configure(demo(), URDF, True)
    assert target_xy(c, .15, .04) == [.15, .04, .04]
    for xy in ((float('nan'), .04), (.15, float('inf')), (10., .04)):
        with pytest.raises(ValueError):
            target_xy(c, *xy)


def test_hardware_template_and_demo_cannot_enable_hardware():
    c = yaml.safe_load((ROOT/'dual_arm_control/config/manual_xy_hardware.yaml').read_text())
    with pytest.raises(ValueError):
        configure(c, URDF, False)
    with pytest.raises(ValueError, match='calibration flags'):
        configure(demo(), URDF, False)


def test_flags_alone_cannot_certify_hardware():
    c = demo()
    c['task_coordinator']['ros__parameters']['calibrated'] = True
    c['cm530_bridge']['ros__parameters'].update(hardware_calibrated=True, serial_port='/not-opened')
    c['manual_xy']['obstacles'] = [dict(center=[0., -.4, 0.], size=[.1, .1, .1])]
    c['manual_xy']['calibration'] = {}
    with pytest.raises(ValueError, match='physical measurement'):
        configure(c, URDF, False)


def test_plane_and_orientation_rejections():
    check_plane([.15, .04, .04], [0, 0, -1], .04)
    with pytest.raises(ValueError):
        check_plane([.15, .04, .045], [0, 0, -1], .04)
    with pytest.raises(ValueError):
        check_plane([.15, .04, .04], [0, 0, 1], .04)


def test_reference_fit_and_inconsistent_measurements():
    scale = 1023 / math.radians(300)
    samples = [[dict(radians=q, raw=round(z+d*q*scale)) for q in (-.3, 0., .3)]
               for z, d in zip([400, 500, 600, 700], [1, -1, 1, -1])]
    assert fit_references(samples) == {'arm1.zero_raw': [400, 500, 600, 700], 'arm1.direction': [1, -1, 1, -1]}
    bad = deepcopy(samples)
    bad[0][1]['raw'] += 20
    with pytest.raises(ValueError):
        fit_references(bad)
    with pytest.raises(ValueError):
        fit_references([s[:2] for s in samples])


def test_measured_tcp_changes_forward_geometry():
    c = demo()
    _, m1 = configure(c, URDF, True)
    c['manual_xy']['tool']['xyz'][2] += .01
    _, m2 = configure(c, URDF, True)
    xyz1, _ = Kinematics(m1).forward([0., 0., 0., 0.])
    xyz2, _ = Kinematics(m2).forward([0., 0., 0., 0.])
    assert math.dist(xyz1, xyz2) == pytest.approx(.01)


def test_independent_validation_rejects_bad_position_orientation_and_duplicates():
    # Synthetic fixture only; these data must never become a hardware config.
    c, model = configure(demo(), URDF, True)
    b = dict(c['bridge'], **{'arm1.zero_raw': [512]*4, 'arm1.direction': [1]*4})
    scale = 1023/math.radians(300)
    evidence = dict(tcp_measured=True, plane_measured=True, limits_measured=True, scene_measured=True,
                    joint_references=[[dict(radians=q, raw=round(512+q*scale)) for q in (-.3, 0., .3)] for _ in range(4)],
                    validation_poses=[])
    for raw in ([563, 630, 676, 844], [561, 643, 655, 852], [569, 634, 670, 846]):
        xyz, axis = Kinematics(model).forward(calibration(b).to_rad(raw))
        evidence['validation_poses'].append(dict(raw=raw, measured_xyz=xyz, measured_tool_z=axis))
    assert len(verify_evidence(dict(calibration=evidence), b, model)) == 3
    bad = deepcopy(evidence)
    bad['validation_poses'][0]['measured_xyz'][0] += .01
    with pytest.raises(ValueError, match='acceptance'):
        verify_evidence(dict(calibration=bad), b, model)
    bad = deepcopy(evidence)
    bad['validation_poses'][0]['measured_tool_z'] = [0., 0., 1.]
    with pytest.raises(ValueError, match='acceptance'):
        verify_evidence(dict(calibration=bad), b, model)
    bad = deepcopy(evidence)
    bad['validation_poses'] = [bad['validation_poses'][0]]*3
    with pytest.raises(ValueError, match='distinct'):
        verify_evidence(dict(calibration=bad), b, model)


def test_user_confirmed_legacy_profile_and_expired_clearance():
    c = demo()
    c['task_coordinator']['ros__parameters']['calibrated'] = True
    c['cm530_bridge']['ros__parameters'].update(hardware_calibrated=True, serial_port='/not-opened',
        **{'arm1.zero_raw': [512]*4, 'arm1.direction': [1]*4})
    m = c['manual_xy']
    m['tool'] = dict(xyz=[0.,0.,0.], rpy=[0.,0.,0.], box_size=[.01]*3, box_center=[0.,0.,.005])
    m['calibration'] = dict(method='legacy_user_confirmed', source='test fixture',
        joint_mapping_user_confirmed=True, unchanged_installation_user_confirmed=True,
        workspace_clear_user_confirmed=True, physical_base_alignment_confirmed=True,
        clearance_valid_until_unix_s=time.time()+60)
    result, model = configure(c, URDF, False)
    assert result['task']['simulation'] is False
    assert '<box size="0.01 0.01 0.01"' in model
    m['calibration']['physical_base_alignment_confirmed'] = False
    with pytest.raises(ValueError, match='physical_base_alignment_confirmed'):
        configure(c, URDF, False)
    m['calibration']['physical_base_alignment_confirmed'] = True
    m['calibration']['clearance_valid_until_unix_s'] = 0
    with pytest.raises(ValueError, match='clearance expired'):
        configure(c, URDF, False)
