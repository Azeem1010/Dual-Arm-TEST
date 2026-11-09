#!/usr/bin/env python3
"""Extract nominal geometry from this SolidWorks AP203 assembly, without moving hardware.

This is a file-specific placement reader, not a general STEP/CAD importer. Output
is measurement evidence only; it does not generate or enable hardware settings.
"""
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'dual_arm_3d-main/雙臂_交付/雙臂總組合.STEP'


def extract():
    data = SOURCE.read_bytes()
    entities = {int(n): v.strip() for n, v in re.findall(
        r'#(\d+)\s*=\s*(.*?);', data.decode('cp950', errors='replace'), re.S)}
    def refs(value):
        return list(map(int, re.findall(r'#(\d+)', value)))
    def decode(value):
        return re.sub(r'\\X2\\([0-9a-fA-F]+)\\X0\\',
                      lambda m: bytes.fromhex(m[1]).decode('utf-16-be'), value)
    def name(pd):
        formation = refs(entities[pd])[0]
        product = refs(entities[formation])[0]
        return decode(re.findall(r"'([^']*)'", entities[product])[0])
    def vec(n):
        return np.array(list(map(float, re.search(
            r'\(\s*([^()]+)\s*\)\s*\)', entities[n])[1].split(','))))
    # This export uses conversion-based INCH units, explicitly 0.0254 metres.
    conversions = [v for v in entities.values() if 'CONVERSION_BASED_UNIT' in v]
    assert conversions and all("'INCH'" in v for v in conversions)
    for v in conversions:
        measure = entities[refs(v)[0]]
        assert 'LENGTH_MEASURE_WITH_UNIT' in measure
        scale = float(re.search(r'LENGTH_MEASURE\(\s*([\d.Ee+-]+)', measure)[1])
        assert abs(scale - .0254) < 1e-12
        assert '.METRE.' in entities[refs(measure)[0]]
    def axis(n):
        p, z, x = refs(entities[n])
        z, x = vec(z), vec(x)
        matrix = np.eye(4)
        matrix[:3, :3] = np.column_stack((x, np.cross(z, x), z))
        matrix[:3, 3] = vec(p) * 25.4
        assert np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3))
        return matrix
    relations = []
    for v in entities.values():
        if v.startswith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'):
            rel, shape = refs(v)
            occurrence = refs(entities[shape])[-1]
            parent, child = refs(entities[occurrence])
            _, _, transformation = refs(entities[rel])
            p, q = refs(entities[transformation])
            relations.append((parent, child, axis(p) @ np.linalg.inv(axis(q)), occurrence))
    roots = set(p for p, _, _, _ in relations) - set(c for _, c, _, _ in relations)
    assert len(roots) == 1
    found = {}
    def walk(pd, matrix, path):
        for parent, child, t, occurrence in relations:
            if parent != pd:
                continue
            g = matrix @ t
            child_path = path + [name(child)]
            nm = name(child)
            if any(key in nm for key in ('輸出盤', 'ax-12a_2019', '總高20')):
                found['/'.join(child_path)] = dict(
                    occurrence_entity=occurrence, product_definition_entity=child,
                    origin_mm=g[:3, 3].tolist(), rotation=g[:3, :3].tolist())
            walk(child, g, child_path)
    walk(roots.pop(), np.eye(4), [])
    arms = {}
    for arm in ('完成_右臂_平台2_含五金', '完成_左臂_平台3_含五金'):
        parts = {p: v for p, v in found.items() if p.startswith(arm + '/')}
        def part(suffix):
            matches = [v for p, v in parts.items() if p.endswith(suffix)]
            assert len(matches) == 1
            return matches[0]
        shoulder = part('輸出盤35度')
        elbow = part('輸出盤55度')
        wrist = part('ax-12a_2019')
        cup = part('總高20')
        sp, ep, wp, cp = [np.array(v['origin_mm']) for v in (shoulder, elbow, wrist, cup)]
        # Motor shaft cylinders are coaxial with each motor part's local Z.
        # The cup extends 20 mm along its own +Y (verified in delivery notes).
        tcp = cp + np.array(cup['rotation'])[:, 1] * 20.
        shaft = np.array(wrist['rotation'])[:, 2]
        arms[arm] = dict(shoulder_to_elbow_mm=float(np.linalg.norm(ep-sp)),
            elbow_to_wrist_mm=float(np.linalg.norm(wp-ep)),
            wrist_axis_to_cup_mount_mm=float(np.linalg.norm(np.cross(cp-wp, shaft))),
            wrist_axis_to_tcp_mm=float(np.linalg.norm(np.cross(tcp-wp, shaft))),
            cup_mount_to_lip_mm=20., tcp_in_cad_assembly_mm=tcp.tolist(),
            shoulder_height_above_platform_top_mm=float(sp[1]-10.), parts=parts)
        assert abs(arms[arm]['shoulder_to_elbow_mm']-93.5) < 1e-6
        assert abs(arms[arm]['elbow_to_wrist_mm']-93.5) < 1e-6
        assert abs(arms[arm]['wrist_axis_to_tcp_mm']-71.) < 1e-6
    return dict(source=str(SOURCE.relative_to(ROOT)), sha256=hashlib.sha256(data).hexdigest(),
                units='mm', cad_vertical_axis='+Y',
                warning='Nominal CAD geometry; arm1 identity, physical dimensions, base transform and motor zeros remain unverified.',
                arms=arms)


if __name__ == '__main__':
    result = json.dumps(extract(), indent=2, ensure_ascii=False) + '\n'
    if len(sys.argv) == 2:
        Path(sys.argv[1]).write_text(result)
    else:
        print(result, end='')
