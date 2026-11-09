#!/usr/bin/env python3
"""Validate a fitted homography against independent measured points.

Input JSON: fit_pixels, fit_world_m, check_pixels, check_world_m,
hole_width_m, cap_diameter_m, cup_diameter_m, mechanical_error_m, margin_m.
Output is a measurement report, never a physical calibration authorization.
Use upstream calibrate_intrinsics and calibrate_homography for acquisition.
"""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from dual_arm_vision_adapter.validity import position_budget


def points(value):
    value = np.asarray(value, dtype=float)
    if value.ndim != 2 or value.shape[1] != 2 or len(value) < 4 or not np.isfinite(value).all():
        raise ValueError('At least four finite 2D points required')
    if np.linalg.matrix_rank(value-value.mean(axis=0)) != 2:
        raise ValueError('Collinear points cannot validate a plane')
    return value


def evaluate(data):
    fp, fw, cp, cw = [points(data[key]) for key in ('fit_pixels','fit_world_m','check_pixels','check_world_m')]
    if len(fp) != len(fw) or len(cp) != len(cw):
        raise ValueError('Correspondence counts differ')
    if any(np.linalg.norm(p-q) < 1e-9 for p in fp for q in cp):
        raise ValueError('Validation points must be independent of fitting points')
    h, _ = cv2.findHomography(fp, fw, method=0)
    if h is None or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-15:
        raise ValueError('Singular homography')
    projected = cv2.perspectiveTransform(cp.reshape(-1,1,2), h).reshape(-1,2)
    errors = np.linalg.norm(projected-cw, axis=1)
    budget = position_budget(*[data[k] for k in ('hole_width_m','cap_diameter_m','cup_diameter_m','mechanical_error_m','margin_m')])
    maximum = float(errors.max())
    return dict(status='PASS' if maximum <= budget else 'FAIL', homography=h.ravel().tolist(),
                workspace_error_m=maximum, rms_error_m=float(np.sqrt(np.mean(errors**2))),
                allowed_position_error_m=budget, validation_count=len(cp),
                errors_m=errors.tolist(), scope='Independent plane measurement only; intrinsic, mount, TF and physical evidence gates remain required')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('measurements',type=Path)
    parser.add_argument('report',type=Path)
    args = parser.parse_args()
    raw = args.measurements.read_bytes()
    result = evaluate(json.loads(raw))
    result['input_sha256'] = hashlib.sha256(raw).hexdigest()
    args.report.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['status']=='PASS' else 1)


if __name__ == '__main__':
    main()
