"""Measure candidate layout changes; never edits CAD or runtime geometry.

Geometric feasibility only. Collision-aware planning and measured hardware
limits remain separate acceptance gates. Approach heights are explicit probes.
"""
from pathlib import Path
import sys
import json
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/dual_arm_coordinator'))
from dual_arm_coordinator.kinematics import CadKinematics
from runtime_settings import SETTINGS
model=CadKinematics((root/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro').read_text(),'arm2')
slots=[[-.1675+col*.035,-.015+row*.035] for row in range(4) for col in range(2)]
# Current white pads are flush with the foam. Historical recess results are archived.
contact_z=SETTINGS['surface_z_m']+SETTINGS['cap_height_m']
def check(dx=0.,dy=0.,dz=0.,lifts=(.03,.02,.01,0.)):
    details=[]
    for number,(x,y) in enumerate(slots,1):
        seed=None
        for lift in lifts:
            target=[x+dx,y+dy,contact_z+dz+lift]
            try:
                seed=model.solve(target,seed)
                details.append(dict(slot=number,lift_m=lift,status='PASS',joints=seed))
            except ValueError:
                return False,dict(slot=number,lift_m=lift,target=target,status='FAIL')
    return True,details
report={'scope':'Numerical CAD IK, four downward approach samples per slot; not collision/hardware acceptance',
        'contact_height_m':contact_z,'baseline':check()[1],'candidates':[]}
for kind,direction in [('toward_user_y',(0.,-1.,0.)),('toward_arm_base_x',(1.,0.,0.)),('raise_insert_z',(0.,0.,1.))]:
    found=None
    for millimetres in range(0,61):
        offset=np.array(direction)*millimetres/1000
        ok,details=check(*offset)
        if ok:
            found=dict(kind=kind,minimum_on_1mm_grid_mm=millimetres,offset_m=offset.tolist(),validation=details)
            break
    report['candidates'].append(found or dict(kind=kind,status='FAIL',searched_mm=60))
    print(kind, 'minimum_mm',found['minimum_on_1mm_grid_mm'] if found else 'none',flush=True)
for candidate in report['candidates']:
    if 'offset_m' in candidate:
        valid,detail=check(*candidate['offset_m'],lifts=np.linspace(.03,0.,31))
        candidate['dense_1mm_approach_status']='PASS' if valid else 'FAIL'
        candidate['dense_1mm_approach']=detail
        if valid:
            vectors=np.array([entry['joints'] for entry in detail])
            candidate['smallest_joint_limit_margin_rad']=float(np.min(np.minimum(vectors-model.lower,model.upper-vectors)))
ok,details=check(0.,-.010,0.,lifts=np.linspace(.03,0.,31))
report['ten_mm_candidate']={'status':'PASS' if ok else 'FAIL','offset_m':[0.,-.010,0.],'validation':details}
if ok:
    vectors=np.array([entry['joints'] for entry in details])
    report['ten_mm_candidate']['smallest_joint_limit_margin_rad']=float(np.min(np.minimum(vectors-model.lower,model.upper-vectors)))
(root/'docs/evidence/slot_adjustment.json').write_text(json.dumps(report,indent=2)+'\n')
