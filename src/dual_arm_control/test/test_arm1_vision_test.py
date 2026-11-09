from types import SimpleNamespace as S
import pytest
from dual_arm_control.vision_target import target_from_pick

CFG = dict(a_min=[0.08, 0.0, 0.017], a_max=[0.20, 0.10, 0.023],
           pick_z=0.02, height_tolerance=0.003, approach_height=0.02)


def pick(**changes):
    values = dict(success=True, message='selected from multiple caps', rank=0,
                  x=0.15, y=0.04, z=0.02)
    values.update(changes)
    return S(**values)


def test_selected_camera_coordinate_becomes_hover_target():
    assert target_from_pick(pick(), CFG) == [0.15, 0.04, 0.04]


def test_hover_uses_calibrated_height_after_validating_camera_height():
    assert target_from_pick(pick(z=0.021), CFG)[2] == 0.04


@pytest.mark.parametrize('change', [
    {'success': False, 'message': 'no reachable caps'},
    {'x': float('nan')}, {'y': float('inf')}, {'z': float('nan')},
    {'x': 0.4}, {'y': -0.1}, {'z': 0.03},
])
def test_invalid_or_unsafe_camera_target_is_rejected(change):
    with pytest.raises(ValueError):
        target_from_pick(pick(**change), CFG)


def test_height_tolerance_is_checked_even_inside_a_bounds():
    cfg = {**CFG, 'a_max': [0.20, 0.10, 0.10]}
    with pytest.raises(ValueError, match='height'):
        target_from_pick(pick(z=0.025), cfg)
