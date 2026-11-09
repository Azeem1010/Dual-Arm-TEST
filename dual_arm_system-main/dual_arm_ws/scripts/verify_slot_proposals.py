"""Recheck saved proposal joint solutions against the current, unchanged CAD model.

This is a discrete kinematic check, not collision or continuous-path acceptance.
It does not change CAD, runtime slots, or calibration validity.
"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/dual_arm_coordinator'))
from dual_arm_coordinator.kinematics import CadKinematics


def main():
    source = ROOT / 'docs/evidence/slot_adjustment.json'
    urdf = ROOT / 'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro'
    model = CadKinematics(urdf.read_text(), 'arm2')
    study = json.loads(source.read_text())
    candidates = [
        (c['kind'], c['offset_m'], c['dense_1mm_approach'])
        for c in study['candidates'] if 'offset_m' in c
    ]
    extra = study['ten_mm_candidate']
    candidates.append(('toward_user_y_10mm', extra['offset_m'], extra['validation']))
    report = {
        'scope': 'Saved samples rechecked with current CAD FK; no collision or hardware acceptance',
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
        'candidates': [],
    }
    for name, offset, samples in candidates:
        if len(samples) != 8 * 31:
            raise ValueError(f'{name}: expected 248 samples')
        errors, dots, margins, steps = [], [], [], []
        critical = None
        for slot in range(1, 9):
            sequence = sorted((s for s in samples if s['slot'] == slot), key=lambda s: s['lift_m'])
            lifts = [s['lift_m'] for s in sequence]
            if len(sequence) != 31 or not np.allclose(lifts, np.linspace(0, .03, 31), atol=1e-12, rtol=0):
                raise ValueError(f'{name}: incomplete approach for slot {slot}')
            previous = None
            for sample in sequence:
                q = np.asarray(sample['joints'])
                if sample['status'] != 'PASS' or q.shape != (4,) or not np.isfinite(q).all():
                    raise ValueError(f'{name}: invalid sample')
                row, col = divmod(slot - 1, 2)
                target = np.array([-.1675 + col * .035, -.015 + row * .035,
                                   study['contact_height_m'] + sample['lift_m']]) + offset
                pose = model.forward(q)
                error = float(np.linalg.norm(pose[:3, 3] - target))
                dot = float(np.dot(pose[:3, 2], [0, 0, -1]))
                margin = np.minimum(q - model.lower, model.upper - q)
                if error >= 1e-5 or dot <= .99999 or np.min(margin) < 0:
                    raise ValueError(f'{name}: FK or joint-limit failure at slot {slot}')
                errors.append(error)
                dots.append(dot)
                margins.append(float(margin.min()))
                if critical is None or margin.min() < critical['margin_rad']:
                    critical = {'slot': slot, 'lift_m': sample['lift_m'],
                                'joint': model.names[int(margin.argmin())],
                                'margin_rad': float(margin.min())}
                if previous is not None:
                    steps.append(float(np.max(np.abs(q - previous))))
                previous = q
        report['candidates'].append({
            'name': name, 'offset_m': offset, 'sample_count': len(samples),
            'discrete_fk_status': 'PASS', 'maximum_position_error_m': max(errors),
            'minimum_downward_dot': min(dots), 'minimum_limit_margin_rad': min(margins),
            'minimum_limit_margin_deg': float(np.rad2deg(min(margins))),
            'maximum_adjacent_sample_joint_step_rad': max(steps),
            'critical_sample': critical,
            'collision_status': 'NOT_RUN', 'continuous_trajectory_status': 'NOT_RUN',
        })
    (ROOT / 'docs/evidence/slot_proposal_verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
