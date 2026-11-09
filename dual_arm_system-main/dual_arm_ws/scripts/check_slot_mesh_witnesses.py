"""Inspect one box-collision witness per failed slot with source STL surfaces.

Prepare with system Python (SciPy); inspect with .cad-venv Python (VTK).
Surface crossings confirm interference; their absence does not rule out
containment, missing components or collisions elsewhere in the trajectory.
"""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'docs/evidence'


def prepare():
    import numpy as np
    from scipy.spatial.transform import Rotation
    sys.path.insert(0, str(ROOT / 'src/dual_arm_coordinator'))
    from dual_arm_coordinator.kinematics import CadKinematics
    model = CadKinematics((ROOT / 'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro').read_text(), 'arm2')

    def frames(q):
        result, transform, index = {}, np.eye(4), 0
        for joint in model.chain:
            local = np.eye(4)
            origin = joint.find('origin')
            if origin is not None:
                local[:3, 3] = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                local[:3, :3] = Rotation.from_euler('xyz', [float(x) for x in origin.get('rpy', '0 0 0').split()]).as_matrix()
            transform = transform @ local
            if joint.get('type') == 'revolute':
                axis = np.array([float(x) for x in joint.find('axis').get('xyz').split()])
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                transform = transform @ rotation
                index += 1
            result[joint.find('child').get('link')] = transform.copy()
        return result

    zero = frames([0.] * 4)
    cad_to_world = np.eye(4)
    cad_to_world[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) / 1000
    cad_to_world[:3, 3] = [0, .03, 0]
    study = json.loads((EVIDENCE / 'slot_adjustment.json').read_text())
    candidates = {c['kind']: c['dense_1mm_approach'] for c in study['candidates'] if 'offset_m' in c}
    candidates['toward_user_y_10mm'] = study['ten_mm_candidate']['validation']
    failures = json.loads((EVIDENCE / 'slot_self_collision.json').read_text())
    witnesses = []
    for candidate in failures['candidates']:
        seen = set()
        for failure in candidate['invalid_samples']:
            if failure['slot'] in seen:
                continue
            seen.add(failure['slot'])
            sample = next(s for s in candidates[candidate['name']]
                          if s['slot'] == failure['slot'] and s['lift_m'] == failure['lift_m'])
            current = frames(sample['joints'])
            matrices = {link: (current[link] @ np.linalg.inv(zero[link]) @ cad_to_world).tolist()
                        for link in ('arm2_cad_link_2', 'arm2_cad_link_4')}
            witnesses.append({'candidate': candidate['name'], **failure, 'matrices': matrices})
    (EVIDENCE / 'slot_mesh_witness_inputs.json').write_text(json.dumps(witnesses, indent=2) + '\n')
    print('Prepared', len(witnesses), 'witnesses')


def inspect():
    from vtkmodules.vtkCommonMath import vtkMatrix4x4
    from vtkmodules.vtkIOGeometry import vtkSTLReader
    from vtkmodules.vtkFiltersModeling import vtkCollisionDetectionFilter

    groups = {'arm2_cad_link_2': [241, 229, 243], 'arm2_cad_link_4': [231, 232, 240]}
    meshes = {}
    for indices in groups.values():
        for index in indices:
            reader = vtkSTLReader()
            reader.SetFileName(str(ROOT / f'src/dual_arm_description/meshes/cad/component_{index:03d}.stl'))
            reader.Update()
            meshes[index] = reader.GetOutput()
            if meshes[index].GetNumberOfCells() == 0:
                raise RuntimeError(f'Empty mesh {index}')
    results = []
    for witness in json.loads((EVIDENCE / 'slot_mesh_witness_inputs.json').read_text()):
        transforms = {}
        for name, values in witness['matrices'].items():
            matrix = vtkMatrix4x4()
            for row in range(4):
                for col in range(4):
                    matrix.SetElement(row, col, values[row][col])
            transforms[name] = matrix
        contacts = []
        for left in groups['arm2_cad_link_2']:
            for right in groups['arm2_cad_link_4']:
                collision = vtkCollisionDetectionFilter()
                collision.SetInputData(0, meshes[left])
                collision.SetInputData(1, meshes[right])
                collision.SetMatrix(0, transforms['arm2_cad_link_2'])
                collision.SetMatrix(1, transforms['arm2_cad_link_4'])
                collision.SetBoxTolerance(0.)
                collision.SetCellTolerance(0.)
                collision.SetCollisionModeToFirstContact()
                collision.Update()
                if collision.GetNumberOfContacts():
                    point = collision.GetContactsOutput().GetPoint(0)
                    contacts.append({'components': [left, right], 'contact_world_m': point})
        entry = {k: v for k, v in witness.items() if k != 'matrices'}
        entry['surface_crossings'] = contacts
        entry['collision_status'] = 'FAIL' if contacts else 'NOT_RUN'
        results.append(entry)
        print(entry['candidate'], 'slot', entry['slot'], 'surface_crossings', len(contacts), flush=True)
    report = {'scope': 'Six selected witnesses, primary component STL surfaces only; no full-path clearance certification',
              'no_crossing_meaning': 'Does not exclude containment or other components; retain conservative collision rejection',
              'witnesses': results}
    (EVIDENCE / 'slot_mesh_witnesses.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    if sys.argv[1:] == ['prepare']:
        prepare()
    elif sys.argv[1:] == ['inspect']:
        inspect()
    else:
        raise SystemExit('Usage: check_slot_mesh_witnesses.py prepare|inspect')
