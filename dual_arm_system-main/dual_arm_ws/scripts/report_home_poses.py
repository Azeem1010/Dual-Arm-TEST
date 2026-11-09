"""Record supplied motor counts and distinguish shaft angles from robot poses."""
import json
import math
from runtime_settings import ROOT, SETTINGS
from dual_arm_coordinator.home_pose import convert_home
from dual_arm_coordinator.kinematics import CadKinematics

report={'formula':'q_rad = direction * (raw - raw_at_cad_zero) * radians(300) / 1023',
        'mapping_mode':SETTINGS['home'].get('mapping_mode','unspecified'),
        'hardware_calibration_status':SETTINGS['home']['calibration_status'],
        'hardware_use_allowed':SETTINGS['home'].get('hardware_use_allowed',False),
        'reference':'https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/',
        'note':'Relative-to-512 shaft angles are arithmetic only, not CAD joint angles or physical pose calibration.',
        'arms':{}}
for arm in ('arm1','arm2'):
    config=SETTINGS['home'][arm]
    entry={'name':config['name'],'motor_ids':config['motor_ids'],'raw':config['raw'],
           'shaft_deg_relative_512':[float((raw-512)*300/1023) for raw in config['raw']],
           'status':'NOT_RUN'}
    try:
        q=convert_home(config['raw'],config['raw_at_cad_zero'],config['direction'])
        model=CadKinematics((ROOT/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro').read_text(),arm)
        entry['joint_rad']=q
        entry['tcp_transform']=model.forward(q).tolist()
        entry['within_model_limits']=all(lo<=value<=hi for value,lo,hi in zip(q,model.lower,model.upper))
        entry['status']='PASS' if entry['within_model_limits'] else 'FAIL'
        entry['scope']='Arithmetic/FK only; collision and hardware acceptance not included'
    except ValueError as error:
        entry['reason']=str(error)
    report['arms'][arm]=entry
(ROOT/'docs/evidence/home_poses.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
