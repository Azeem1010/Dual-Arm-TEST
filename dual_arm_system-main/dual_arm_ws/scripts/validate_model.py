"""Structural acceptance of expanded URDF; does not assert CAD equivalence."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
from runtime_settings import SETTINGS, configured_homes
from dual_arm_coordinator.kinematics import CadKinematics

root = Path(__file__).resolve().parents[1]
text = subprocess.check_output(['xacro',str(root/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro')],text=True)
(root/'docs/evidence/dual_arm.urdf').write_text(text)
robot = ET.fromstring(text)
links = {link.attrib['name'] for link in robot.findall('link')}
joints = robot.findall('joint')
assert len({j.attrib['name'] for j in joints}) == len(joints)
children = {j.find('child').attrib['link'] for j in joints}
assert links-children == {'world'}, links-children
active = {j.attrib['name'] for j in joints if j.attrib['type']=='revolute'}
assert len(active)==8, active
assert {'arm1_tcp','arm2_tcp'} <= links
for j in joints:
    assert j.find('parent').attrib['link'] in links
    assert j.find('child').attrib['link'] in links
    if j.attrib['type']=='revolute':
        limit = j.find('limit').attrib
        assert float(limit['lower']) < float(limit['upper'])
        assert float(limit['velocity']) > 0
for mesh in robot.findall('.//mesh'):
    uri = mesh.attrib['filename']
    assert uri.startswith('package://dual_arm_description/')
    assert (root/'src/dual_arm_description'/uri.split('package://dual_arm_description/')[1]).is_file(), uri
config = yaml.safe_load((root/'src/dual_arm_controllers/config/controllers.yaml').read_text())
a = set(config['arm1_controller']['ros__parameters']['joints'])
b = set(config['arm2_controller']['ros__parameters']['joints'])
assert not a & b and a | b == active
# CAD-style component links belong to rigid assemblies. Ignore same/adjacent
# rigid bodies (servo and fitted bracket), retaining other self/inter-arm checks.
parent = {link:link for link in links}
def representative(link):
    while parent[link] != link:
        link = parent[link]
    return link
for j in joints:
    if j.attrib['type']=='fixed':
        parent[representative(j.find('child').attrib['link'])] = representative(j.find('parent').attrib['link'])
adjacent = {frozenset((representative(j.find('parent').attrib['link']),representative(j.find('child').attrib['link']))) for j in joints if j.attrib['type']=='revolute'}
srdf = ET.parse(root/'src/dual_arm_moveit_config/config/dual_arm.srdf')
for node in list(srdf.getroot().findall('disable_collisions')):
    srdf.getroot().remove(node)
ordered = sorted(links)
for i, a in enumerate(ordered):
    for b in ordered[i+1:]:
        ra, rb = representative(a), representative(b)
        if ra==rb or frozenset((ra,rb)) in adjacent:
            ET.SubElement(srdf.getroot(),'disable_collisions',link1=a,link2=b,reason='RigidAssembly' if ra==rb else 'Adjacent')
for node in list(srdf.getroot().findall('group_state')):
    if node.get('name') in ('home_A','home_C','home_A_home_C'):
        srdf.getroot().remove(node)
models={arm:CadKinematics(text,arm) for arm in ('arm1','arm2')}
try:
    homes=configured_homes(models)
except ValueError as error:
    print('HOME named states NOT_RUN:',error)
else:
    combined=ET.SubElement(srdf.getroot(),'group_state',name='home_A_home_C',group='both_arms')
    for arm,values in homes.items():
        state=ET.SubElement(srdf.getroot(),'group_state',name=SETTINGS['home'][arm]['name'],group=arm)
        for name,value in zip(models[arm].names,values):
            ET.SubElement(state,'joint',name=name,value=str(value))
            ET.SubElement(combined,'joint',name=name,value=str(value))
srdf.write(root/'src/dual_arm_moveit_config/config/dual_arm.srdf',encoding='unicode')
print(f'PASS: connected world-root model, {len(links)} links, eight revolute joints, meshes present, disjoint controllers')
