"""Verify CAD source-pose preservation independently of Gazebo rendering."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import json

root = Path(__file__).resolve().parents[1]
robot = ET.parse(root/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro').getroot()
transforms = {'world':np.eye(4)}
def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3,3] = [float(x) for x in element.get('xyz','0 0 0').split()]
        t[:3,:3] = Rotation.from_euler('xyz',[float(x) for x in element.get('rpy','0 0 0').split()]).as_matrix()
    return t
pending = list(robot.findall('joint'))
while pending:
    before = len(pending)
    for joint in pending.copy():
        parent = joint.find('parent').get('link')
        if parent in transforms:
            transforms[joint.find('child').get('link')] = transforms[parent]@origin(joint.find('origin'))
            pending.remove(joint)
    assert len(pending)<before,'Disconnected graph'
expected = np.eye(4)
expected[:3,:3] = Rotation.from_euler('x',np.pi/2).as_matrix()
expected[:3,3] = [0,.03,0]
count = 0
for link in robot.findall('link'):
    for visual in link.findall('visual'):
        actual = transforms[link.get('name')]@origin(visual.find('origin'))
        assert np.allclose(actual,expected,atol=1e-9), link.get('name')
        count += 1
assert count==264,count
assert len([j for j in robot.findall('joint') if j.get('type')=='revolute'])==8
report = {'status':'PASS','visual_components':count,'scope':'At zero joint displacement, each exported CAD visual has exactly the source assembly transform; does not validate fastener assignment during motion or dynamics.'}
(root/'docs/evidence/cad_pose_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
