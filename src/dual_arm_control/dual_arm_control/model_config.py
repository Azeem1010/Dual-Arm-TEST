"""Validate that both calibrated B contact targets describe one physical point."""
import math
import xml.etree.ElementTree as ET


def transform_point(xyz, translation, rpy):
    x,y,z=xyz
    r,p,yaw=rpy
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(yaw),math.sin(yaw)
    return [translation[0]+cy*cp*x+(cy*sp*sr-sy*cr)*y+(cy*sp*cr+sy*sr)*z,
            translation[1]+sy*cp*x+(sy*sp*sr+cy*cr)*y+(sy*sp*cr-cy*sr)*z,
            translation[2]-sp*x+cp*sr*y+cp*cr*z]


def fixed_point_in_root(urdf, frame, point):
    root=ET.fromstring(urdf)
    links={e.attrib['name'] for e in root.findall('link')}
    joints={j.find('child').attrib['link']:j for j in root.findall('joint')}
    if frame not in links:
        raise ValueError('Model is missing base frame: '+frame)
    visited=set()
    while frame in joints:
        if frame in visited: raise ValueError('Cyclic URDF')
        visited.add(frame)
        joint=joints[frame]
        if joint.attrib['type']!='fixed': raise ValueError('Arm base must have a fixed mounting transform')
        origin=joint.find('origin')
        xyz=[float(v) for v in (origin.get('xyz','0 0 0') if origin is not None else '0 0 0').split()]
        rpy=[float(v) for v in (origin.get('rpy','0 0 0') if origin is not None else '0 0 0').split()]
        if len(xyz)!=3 or len(rpy)!=3 or not all(math.isfinite(v) for v in xyz+rpy):
            raise ValueError('Invalid fixed origin')
        point=transform_point(point,xyz,rpy)
        frame=joint.find('parent').attrib['link']
    return frame,point


def validate_handoff(urdf, arm1, arm2, tolerance=0.005):
    a,pa=fixed_point_in_root(urdf,arm1['base_frame'],arm1['b_place'])
    b,pb=fixed_point_in_root(urdf,arm2['base_frame'],arm2['b_pick'][:3])
    if a!=b or math.dist(pa,pb)>tolerance:
        raise ValueError(f'B targets disagree in common model frame: {a}:{pa} vs {b}:{pb}')
