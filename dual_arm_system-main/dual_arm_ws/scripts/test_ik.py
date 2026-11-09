from pathlib import Path
import subprocess
import json
from dual_arm_coordinator.kinematics import CadKinematics

root=Path(__file__).resolve().parents[1]
urdf=(root/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro').read_text()
results=[]
for arm in ('arm1','arm2'):
    model=CadKinematics(urdf,arm)
    targets=([[.14+col*.06,-.02+row*.035,.024] for row in range(4) for col in range(2)] if arm=='arm1' else [[-.1675+col*.035,-.015+row*.035,.024] for row in range(4) for col in range(2)])
    targets.append([0,.05,.024])
    for target in targets:
        try:
            q=model.solve(target)
            results.append(dict(arm=arm,target=target,status='PASS',joints=q))
        except ValueError as exc:
            results.append(dict(arm=arm,target=target,status='FAIL',reason=str(exc)))
(root/'docs/evidence/ik_validation.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
raise SystemExit(0 if all(r['status']=='PASS' for r in results) else 1)
