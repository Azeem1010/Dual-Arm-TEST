"""Articulate extracted CAD at its source pose, preserving each visual component.

Rotation axes are taken from the AX-12 output cylinder surfaces. Fixed fastener
membership is inferred by closest main component bounding box and recorded for
review. Dynamics and travel limits are simulation estimates, not calibration.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

root = Path(__file__).resolve().parents[1]
items = json.loads((root/'docs/evidence/cad_inventory.json').read_text())
by_id = {int(x['file'][10:13]):x for x in items}
rotation = np.array([[1,0,0],[0,0,-1],[0,1,0]])
shift = np.array([0,.03,0])
def world(point):
    return rotation @ np.array(point)/1000+shift
def xyz(v):
    return ' '.join(f'{a:.10g}' for a in v)

groups = {1:[[0,1,2],[3,4],[5,6,7],[8,9,12],[10,11,19]],
          2:[[259,260,261],[262,263],[241,229,243],[242,230,233],[231,232,240]]}
motor_ids = {1:[2,4,7,12],2:[260,263,243,233]}
joint_names = ['arm_shoulder_pan_joint','arm_shoulder_lift_joint','arm_elbow_flex_joint','arm_wrist_flex_joint']
assignment = {}
corrections = json.loads((root/'cad/corrections.json').read_text(encoding='utf-8-sig'))
for item in items:
    index = int(item['file'][10:13])
    arm = 1 if '右臂' in '/'.join(item['ancestry']) else 2
    known = next((g for g,ids in enumerate(groups[arm]) if index in ids),None)
    if known is None:
        center = (np.array(item['bounds_mm'][:3])+item['bounds_mm'][3:])/2
        def distance(i):
            bounds = by_id[i]['bounds_mm']
            return np.linalg.norm(np.maximum(np.maximum(np.array(bounds[:3])-center,center-np.array(bounds[3:])),0))
        known = min(range(5),key=lambda group:min(distance(i) for i in groups[arm][group]))
    assignment[index] = (arm,known)
    for correction in corrections['rotations']:
        if index in correction['component_indices']:
            assignment[index] = (correction['arm'],4)

robot = ET.Element('robot',name='dual_arm',attrib={'xmlns:xacro':'http://ros.org/wiki/xacro'})
ET.SubElement(robot,'xacro:arg',name='mode',default='mock')
ET.SubElement(robot,'xacro:arg',name='port',default='/dev/ttyUSB0')
ET.SubElement(robot,'xacro:arg',name='calibration_file',default='')
ET.SubElement(robot,'xacro:arg',name='enable_motion',default='false')
ET.SubElement(robot,'link',name='world')
axis_report = {}
tip_offsets = {}
for arm in (1,2):
    pivots, axes = [], []
    for index in motor_ids[arm]:
        motor = by_id[index]
        cylinder = next(c for c in motor['cylinders'] if abs(c['radius_mm']-9.1)<1e-5)
        # All points on a cylinder axis define the same rotation line.
        pivots.append(world(cylinder['point_mm']))
        axes.append(rotation@np.array(cylinder['axis']))
    base = np.array([.1 if arm==1 else -.1,-.08725,0.])
    origins = [base]+pivots
    axis_report[f'arm{arm}'] = {'pivots_m':[p.tolist() for p in pivots],'axes':[a.tolist() for a in axes]}
    for group in range(5):
        link_name = f'arm{arm}_base_link' if group==0 else f'arm{arm}_cad_link_{group}'
        link = ET.SubElement(robot,'link',name=link_name)
        # Positive coarse inertial values permit simulation; measured masses required for hardware.
        mass = .1 if group==0 else .08
        inertial = ET.SubElement(link,'inertial')
        ET.SubElement(inertial,'mass',value=str(mass))
        ET.SubElement(inertial,'inertia',ixx='0.0001',iyy='0.0001',izz='0.0001',ixy='0',ixz='0',iyz='0')
        for index,owner in assignment.items():
            if owner!=(arm,group):
                continue
            item = by_id[index]
            visual = ET.SubElement(link,'visual',name=f'component_{index}')
            ET.SubElement(visual,'origin',xyz=xyz(shift-origins[group]),rpy=f'{math.pi/2} 0 0')
            geometry = ET.SubElement(visual,'geometry')
            ET.SubElement(geometry,'mesh',filename='package://dual_arm_description/meshes/cad/'+item['file'],scale='0.001 0.001 0.001')
            color = '0.08 0.08 0.09 1' if ('ax12' in item['name'].lower() or 'ax-12' in item['name'].lower()) else '0.45 0.45 0.48 1'
            if '平台' in item['name']:
                color = '1 0.15 0.55 1'
            if '吸盤_六角' in item['name']:
                color = '0.6 0.7 0.8 0.8'
            material = ET.SubElement(visual,'material',name=f'cad_{index}')
            ET.SubElement(material,'color',rgba=color)
        # Conservative component boxes; platform collision remains in workstation SDF
        # to preserve its physical holes. Fasteners do not add separate collisions.
        for index in groups[arm][group]:
            item = by_id[index]
            if '平台' in item['name']:
                continue
            lo,hi = np.array(item['bounds_mm'][:3]),np.array(item['bounds_mm'][3:])
            center,size = world((lo+hi)/2),np.abs(rotation)@(hi-lo)/1000
            collision = ET.SubElement(link,'collision',name=f'box_{index}')
            ET.SubElement(collision,'origin',xyz=xyz(center-origins[group]))
            ET.SubElement(ET.SubElement(collision,'geometry'),'box',size=xyz(size))
        joint = ET.SubElement(robot,'joint',name=f'arm{arm}_mount' if group==0 else f'arm{arm}_{joint_names[group-1]}',type='fixed' if group==0 else 'revolute')
        previous = 'world' if group==0 else (f'arm{arm}_base_link' if group==1 else f'arm{arm}_cad_link_{group-1}')
        ET.SubElement(joint,'parent',link=previous)
        ET.SubElement(joint,'child',link=link_name)
        ET.SubElement(joint,'origin',xyz=xyz(base if group==0 else origins[group]-origins[group-1]))
        if group:
            ET.SubElement(joint,'axis',xyz=xyz(axes[group-1]))
            # Wrist upper limit fits firmware HOME raw 825 (91.79 deg from the raw-512 straight pose); simulation estimate only.
            ET.SubElement(joint,'limit',lower='-1.5',upper='1.65' if group==4 else '1.5',effort='1.5',velocity='0.5')
            ET.SubElement(joint,'dynamics',damping='0.05',friction='0.01')
    cup = by_id[19 if arm==1 else 240]
    matrix = np.array(cup['transform'])
    tip = world(matrix[:3,3]+matrix[:3,1]*20)
    ET.SubElement(robot,'link',name=f'arm{arm}_tcp')
    joint = ET.SubElement(robot,'joint',name=f'arm{arm}_tcp_fixed',type='fixed')
    ET.SubElement(joint,'parent',link=f'arm{arm}_cad_link_4')
    ET.SubElement(joint,'child',link=f'arm{arm}_tcp')
    ET.SubElement(joint,'origin',xyz=xyz(tip-origins[4]),rpy=f'{-math.pi/2} 0 0')
    axis_report[f'arm{arm}']['tcp_m'] = tip.tolist()
    tip_offsets[arm] = tip-origins[4]

control = ET.SubElement(robot,'ros2_control',name='DualArmSystem',type='system')
hardware = ET.SubElement(control,'hardware')
ET.SubElement(ET.SubElement(hardware,'xacro:if',value="${'$(arg mode)' == 'sim'}"),'plugin').text='gz_ros2_control/GazeboSimSystem'
cm530 = ET.SubElement(hardware,'xacro:if',value="${'$(arg mode)' == 'hardware'}")
ET.SubElement(cm530,'plugin').text='cm530_hardware/CM530SystemInterface'
for name in ('port','calibration_file','enable_motion'):
    ET.SubElement(cm530,'param',name=name).text=f'$(arg {name})'
ET.SubElement(ET.SubElement(hardware,'xacro:if',value="${'$(arg mode)' not in ('sim', 'hardware')}"),'plugin').text='mock_components/GenericSystem'
for arm in (1,2):
    for name in joint_names:
        joint = ET.SubElement(control,'joint',name=f'arm{arm}_{name}')
        ET.SubElement(joint,'command_interface',name='position')
        ET.SubElement(ET.SubElement(joint,'state_interface',name='position'),'param',name='initial_value').text='0.0'
        ET.SubElement(joint,'state_interface',name='velocity')
gazebo = ET.SubElement(ET.SubElement(robot,'xacro:if',value="${'$(arg mode)' == 'sim'}"),'gazebo')
plugin = ET.SubElement(gazebo,'plugin',filename='libgz_ros2_control-system.so',name='gz_ros2_control::GazeboSimROS2ControlPlugin')
ET.SubElement(plugin,'parameters').text='$(find dual_arm_controllers)/config/controllers.yaml'
suction = ET.SubElement(gazebo,'plugin',filename='$(find dual_arm_gripper)/../../lib/libdual_arm_suction.so',name='dual_arm_gripper::SuctionSystem')
for arm,offset in tip_offsets.items():
    ET.SubElement(suction,f'arm{arm}_tip').text=xyz(offset)
ET.indent(robot)
ET.ElementTree(robot).write(root/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro',encoding='unicode')
(root/'docs/evidence/cad_joint_mapping.json').write_text(json.dumps({'axis_geometry':axis_report,'component_assignment':assignment,'dynamics_status':'NOT_RUN','travel_limits_status':'NOT_RUN'},indent=2)+'\n')
print('Generated articulated CAD model: 264 visuals, eight geometry-derived axes; fastener membership and dynamics require validation')
