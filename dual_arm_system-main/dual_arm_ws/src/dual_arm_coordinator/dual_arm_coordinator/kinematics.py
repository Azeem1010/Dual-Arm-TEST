"""Four-axis CAD IK with explicit suction direction constraint (simulation model).

Uses the exact revolute origins/axes in the expanded URDF. Numerical position-only
IK is not sufficient for suction; the TCP +Z axis must point downward.
"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class CadKinematics:
    def __init__(self, urdf, arm):
        robot=ET.fromstring(urdf)
        joints={j.find('child').get('link'):j for j in robot.findall('joint')}
        self.chain=[]
        child=f'{arm}_tcp'
        while child!='world':
            joint=joints[child]
            self.chain.insert(0,joint)
            child=joint.find('parent').get('link')
        active=[j for j in self.chain if j.get('type')=='revolute']
        self.names=[j.get('name') for j in active]
        self.lower=np.array([float(j.find('limit').get('lower')) for j in active])
        self.upper=np.array([float(j.find('limit').get('upper')) for j in active])
        if len(active)!=4:
            raise ValueError('Expected exactly four active joints')

    def forward(self,q):
        t=np.eye(4)
        index=0
        for joint in self.chain:
            origin=joint.find('origin')
            local=np.eye(4)
            if origin is not None:
                local[:3,3]=[float(x) for x in origin.get('xyz','0 0 0').split()]
                local[:3,:3]=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix()
            t=t@local
            if joint.get('type')=='revolute':
                axis=np.array([float(x) for x in joint.find('axis').get('xyz').split()])
                rotation=np.eye(4)
                rotation[:3,:3]=Rotation.from_rotvec(axis*q[index]).as_matrix()
                t=t@rotation
                index+=1
        return t

    def solve(self,target,seed=None):
        target=np.asarray(target,dtype=float)
        if target.shape!=(3,) or not np.isfinite(target).all():
            raise ValueError('Finite world XYZ target required')
        def residual(q):
            t=self.forward(q)
            return np.r_[t[:3,3]-target,.1*(t[:3,2]-[0,0,-1])]
        seeds=[np.zeros(4),[0,-.5,.8,1.2],[0,.5,.5,.5],[0,-1.,1.4,1.2]]
        if seed is not None:
            seeds.insert(0,seed)
        for start in seeds:
            result=least_squares(residual,np.clip(start,self.lower,self.upper),bounds=(self.lower,self.upper),max_nfev=300,gtol=1e-10,ftol=1e-10,xtol=1e-10)
            pose=self.forward(result.x)
            if np.linalg.norm(pose[:3,3]-target)<1e-5 and np.dot(pose[:3,2],[0,0,-1])>.99999:
                return result.x.tolist()
        raise ValueError('No four-DOF solution satisfies XYZ and downward suction axis within model limits')
