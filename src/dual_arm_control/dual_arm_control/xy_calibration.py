"""Offline model/calibration checks. Never opens a serial port or moves a motor."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .cm530 import Calibration
from .model_config import transform_point

JOINTS = ['arm1_' + j for j in ('arm_shoulder_pan_joint', 'arm_shoulder_lift_joint',
                               'arm_elbow_flex_joint', 'arm_wrist_flex_joint')]


def vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length or not all(
            isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in value):
        raise ValueError(f'{name}: expected {length} finite numbers')
    return value


def calibration(bridge):
    defaults = ([512]*4, [1]*4, [-2.61799, -2.09439, -2.426, -1.7104],
                [2.61799, 2.09439, 2.426, 1.7976], [0]*4, [1023]*4)
    return Calibration(*(bridge.get('arm1.' + key, default) for key, default in zip(
        ('zero_raw', 'direction', 'lower_rad', 'upper_rad', 'raw_min', 'raw_max'), defaults)))


def tool_model(urdf, tool):
    """Set measured TCP and a conservative cylindrical tool envelope in memory."""
    xyz = vector(tool['xyz'], 3, 'tool.xyz')
    rpy = vector(tool['rpy'], 3, 'tool.rpy')
    box = 'box_size' in tool
    if box:
        if min(vector(tool['box_size'], 3, 'tool.box_size')) <= 0:
            raise ValueError('Positive tool box dimensions required')
        vector(tool['box_center'], 3, 'tool.box_center')
    else:
        for key in ('radius', 'length'):
            if not isinstance(tool[key], (float, int)) or not math.isfinite(tool[key]) or tool[key] <= 0:
                raise ValueError('Positive measured tool ' + key + ' required')
    robot = ET.fromstring(urdf)
    joint = robot.find("joint[@name='arm1_suction_fixed']")
    if joint is None or joint.get('type') != 'fixed':
        raise ValueError('Missing fixed suction TCP joint')
    joint.find('origin').set('xyz', ' '.join(map(str, xyz)))
    joint.find('origin').set('rpy', ' '.join(map(str, rpy)))
    link = robot.find("link[@name='arm1_suction_tcp']")
    for kind in ('visual', 'collision'):
        item = link.find(kind)
        if box:
            item.find('origin').set('xyz', ' '.join(map(str, tool['box_center'])))
            geometry = item.find('geometry')
            geometry.clear()
            ET.SubElement(geometry, 'box', size=' '.join(map(str, tool['box_size'])))
        else:
            item.find('origin').set('xyz', f"0 0 {-tool['length']/2}")
            item.find('geometry/cylinder').set('radius', str(tool['radius']))
            item.find('geometry/cylinder').set('length', str(tool['length']))
    return ET.tostring(robot, encoding='unicode')


class Kinematics:
    def __init__(self, urdf):
        root = ET.fromstring(urdf)
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []
        child = 'arm1_suction_tcp'
        while child != 'arm1_base':
            if child not in by_child or len(self.chain) > 50:
                raise ValueError('Invalid arm1 TCP chain')
            j = by_child[child]
            self.chain.insert(0, j)
            child = j.find('parent').get('link')
        if [j.get('name') for j in self.chain if j.get('type') != 'fixed'] != JOINTS:
            raise ValueError('Unexpected arm1 joint chain')

    def forward(self, q):
        vector(q, 4, 'joint angles')
        angles = dict(zip(JOINTS, q))
        total = np.eye(4)
        for j in self.chain:
            origin = j.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            t = np.eye(4)
            t[:3, 3] = xyz
            t[:3, :3] = np.array([transform_point(e, [0]*3, rpy) for e in np.eye(3)]).T
            if j.get('type') != 'fixed':
                a = np.array([float(v) for v in j.find('axis').get('xyz').split()])
                a /= np.linalg.norm(a)
                x, y, z = a
                k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                theta = angles[j.get('name')]
                t[:3, :3] = t[:3, :3] @ (np.eye(3) + math.sin(theta)*k + (1-math.cos(theta))*(k@k))
            total = total @ t
        return total[:3, 3].tolist(), total[:3, 2].tolist()

    def check_limits(self, cal):
        for i, j in enumerate(j for j in self.chain if j.get('type') != 'fixed'):
            limit = j.find('limit')
            if cal.lower[i] < float(limit.get('lower')) - 1e-6 or cal.upper[i] > float(limit.get('upper')) + 1e-6:
                raise ValueError('Calibrated joint limits exceed URDF: ' + j.get('name'))


def angle_between(a, b):
    a, b = np.array(a, dtype=float), np.array(b, dtype=float)
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)) or min(np.linalg.norm(a), np.linalg.norm(b)) < 1e-9:
        raise ValueError('Invalid measured tool axis')
    return math.degrees(math.acos(float(np.clip(np.dot(a, b)/np.linalg.norm(a)/np.linalg.norm(b), -1, 1))))


def fit_references(samples):
    if len(samples) != 4:
        raise ValueError('Need reference samples for all four joints (IDs 17,3,2,15)')
    zero, direction = [], []
    scale = 1023 / math.radians(300)
    for refs in samples:
        if len(refs) < 3:
            raise ValueError('At least three reference angles per joint required')
        for r in refs:
            vector([r['radians'], r['raw']], 2, 'reference')
            if not isinstance(r['raw'], int) or not 0 <= r['raw'] <= 1023:
                raise ValueError('Reference raw must be an integer in 0..1023')
        q, raw = np.array([r['radians'] for r in refs]), np.array([r['raw'] for r in refs])
        if len(set(q)) < 3 or np.ptp(q) < 0.2:
            raise ValueError('Reference angles must be distinct and span at least 0.2 rad')
        candidates = [(float(np.max(np.abs(raw - (round(float(np.mean(raw - d*q*scale))) + d*q*scale)))),
                       round(float(np.mean(raw - d*q*scale))), d) for d in (-1, 1)]
        residual, z, d = min(candidates)
        if residual > 2:
            raise ValueError('Joint references disagree with AX 300-degree scale by more than 2 ticks')
        zero.append(z)
        direction.append(d)
    return {'arm1.zero_raw': zero, 'arm1.direction': direction}


def verify_evidence(manual, bridge, urdf):
    evidence = manual['calibration']
    for key in ('tcp_measured', 'plane_measured', 'limits_measured', 'scene_measured'):
        if evidence.get(key) is not True:
            raise ValueError('Missing physical measurement: ' + key)
    fitted = fit_references(evidence['joint_references'])
    for key, values in fitted.items():
        if bridge.get(key) != values:
            raise ValueError(f'Joint references require {key}={values}')
    cal = calibration(bridge)
    model = Kinematics(urdf)
    model.check_limits(cal)
    poses = evidence['validation_poses']
    if len(poses) < 3 or len({tuple(p['raw']) for p in poses}) < 3:
        raise ValueError('At least three distinct independent validation poses required')
    results = []
    for p in poses:
        q = cal.to_rad(p['raw'])
        cal.to_raw(q)
        xyz, axis = model.forward(q)
        measured = vector(p['measured_xyz'], 3, 'measured_xyz')
        measured_axis = vector(p['measured_tool_z'], 3, 'measured_tool_z')
        position_error = math.dist(xyz, measured)
        orientation_error = angle_between(axis, measured_axis)
        downward_error = angle_between(measured_axis, [0, 0, -1])
        if position_error > .005 or orientation_error > 3 or downward_error > 3:
            raise ValueError('Independent pose failed 5 mm / 3 degree calibration acceptance')
        results.append(dict(predicted_xyz=xyz, position_error_m=position_error,
                            orientation_error_deg=orientation_error, downward_error_deg=downward_error))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--fit', action='store_true', help='Print candidate zero/direction only; does not certify calibration')
    args = parser.parse_args()
    try:
        config = yaml.safe_load(Path(args.config).read_text())
        manual = config['manual_xy']
        if args.fit:
            print(yaml.safe_dump(fit_references(manual['calibration']['joint_references'])))
        else:
            model = tool_model(Path(args.urdf).read_text(), manual['tool'])
            print(json.dumps(verify_evidence(manual, config['cm530_bridge']['ros__parameters'], model), indent=2))
    except (ValueError, KeyError, TypeError) as e:
        parser.exit(1, f'Calibration incomplete/invalid: {e}\n')
