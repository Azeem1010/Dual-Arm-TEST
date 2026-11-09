"""Validation for a single arm1 camera target, independent of ROS runtime."""
import math


def target_from_pick(result, cfg):
    if not result.success:
        raise ValueError('Vision target unavailable: ' + result.message)
    # GetPickTarget specifies arm1_base coordinates in metres.
    xyz = [result.x, result.y, result.z]
    if not all(math.isfinite(v) for v in xyz):
        raise ValueError('Vision returned non-finite coordinates')
    if any(not lo <= v <= hi for lo, v, hi in zip(cfg['a_min'], xyz, cfg['a_max'])):
        raise ValueError(f'Vision target outside calibrated A bounds: {xyz}')
    if abs(xyz[2] - cfg['pick_z']) > cfg['height_tolerance']:
        raise ValueError('Vision height differs from calibrated pick height')
    return [xyz[0], xyz[1], cfg['pick_z'] + cfg['approach_height']]
