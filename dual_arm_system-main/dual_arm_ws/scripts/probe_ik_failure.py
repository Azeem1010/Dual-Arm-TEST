from pathlib import Path
import sys
import numpy as np
from scipy.optimize import least_squares
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/dual_arm_coordinator'))
from dual_arm_coordinator.kinematics import CadKinematics
for arm,target in [('arm1',[.2,.085,.024]),('arm2',[-.1675,.09,.024])]:
    model=CadKinematics((root/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro').read_text(),arm)
    def residual(q):
        t=model.forward(q)
        return np.r_[t[:3,3]-target,.1*(t[:3,2]-[0,0,-1])]
    for limit in (1.5,2.62):
        best=min((least_squares(residual,np.random.default_rng(seed).uniform(-limit,limit,4),bounds=(-limit,limit),max_nfev=300) for seed in range(25)),key=lambda r:np.linalg.norm(r.fun))
        print(arm,'hypothetical_limit',limit,'residual',np.linalg.norm(best.fun),'q',best.x.tolist())
