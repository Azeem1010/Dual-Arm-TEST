"""Fail-closed validation, independent of ROS and wall/simulation clock choice."""
import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def position_budget(hole_width_m, cap_diameter_m, cup_diameter_m,
                    mechanical_error_m, margin_m):
    values = [hole_width_m, cap_diameter_m, cup_diameter_m, mechanical_error_m, margin_m]
    if not all(finite(v) and v >= 0 for v in values) or min(values[:3]) <= 0:
        raise ValueError('Measured dimensions and nonnegative uncertainty required')
    clearance = min((hole_width_m-cap_diameter_m)/2, (cap_diameter_m-cup_diameter_m)/2)
    budget = clearance - mechanical_error_m - margin_m
    if budget <= 0:
        raise ValueError('No positive vision error budget remains')
    return budget


def matrix_valid(value):
    if not isinstance(value, list) or len(value) != 9 or not all(finite(v) for v in value):
        return False
    a,b,c,d,e,f,g,h,i = value
    return abs(a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g)) > 1e-15


def validate(data, *, mode, camera_id, frame_id, reference_frame, image_size,
             now, image_stamp, max_age_s, mount_revision, pose_signature):
    errors = []
    if not isinstance(data, dict):
        return ['Calibration is not a mapping']
    if mode not in ('sim', 'hardware'):
        errors.append('Unknown operating mode')
    for key, expected in dict(mode=mode, camera_id=camera_id, frame_id=frame_id,
                              reference_frame=reference_frame, mount_revision=mount_revision,
                              pose_signature=pose_signature).items():
        if not expected or data.get(key) != expected:
            errors.append(f'{key} mismatch or absent')
    if data.get('image_size') != list(image_size):
        errors.append('Image resolution mismatch')
    if data.get('status') != 'PASS':
        errors.append('Calibration not validated')
    if data.get('evidence_verified') is not True or data.get('evidence_kind') != {'sim':'gazebo','hardware':'physical'}.get(mode):
        errors.append('Evidence provenance unverified')
    if data.get('budget_verified') is not True:
        errors.append('Measured tolerance budget unverified')
    if camera_id == 'arm2' and data.get('slot_order_verified') is not True:
        errors.append('Slot numbering unverified')
    if not all(finite(v) for v in [now, image_stamp, max_age_s]) or max_age_s <= 0 or not 0 <= now-image_stamp <= max_age_s:
        errors.append('Image stale, future-dated or age policy invalid')
    start, end = data.get('calibrated_at'), data.get('expires_at')
    if not all(finite(v) for v in [start, end, now]) or not start <= now < end:
        errors.append('Calibration expired or timestamp invalid')
    for error, limit in [('workspace_error_m', 'allowed_position_error_m'), ('reprojection_error_px', 'allowed_reprojection_error_px')]:
        a, b = data.get(error), data.get(limit)
        if not finite(a) or not finite(b) or b <= 0 or not 0 <= a <= b:
            errors.append(f'{error} outside validated allowance')
    for key in ('camera_matrix', 'homography'):
        if not matrix_valid(data.get(key)):
            errors.append(f'{key} absent, nonfinite or singular')
    k = data.get('camera_matrix')
    if matrix_valid(k) and (k[0] <= 0 or k[4] <= 0 or k[6:] != [0., 0., 1.]):
        errors.append('Invalid pinhole intrinsics')
    d = data.get('distortion')
    if not isinstance(d, list) or len(d) not in (4,5,8,12,14) or not all(finite(v) for v in d):
        errors.append('Invalid pinhole distortion coefficients')
    return errors
