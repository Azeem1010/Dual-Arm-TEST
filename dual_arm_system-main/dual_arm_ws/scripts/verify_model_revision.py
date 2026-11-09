"""Check exact correction transforms, unchanged kinematics and flush-pad geometry."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np

root=Path(__file__).resolve().parents[1]
baseline=root/'docs/evidence/before_revision_20261109'
old=json.loads((baseline/'cad_inventory.json').read_text())
new=json.loads((root/'docs/evidence/cad_inventory.json').read_text())
config=json.loads((root/'cad/corrections.json').read_text(encoding='utf-8-sig'))
assert len(old)==len(new)==264
changed={}
for correction in config['rotations']:
    transform=np.eye(4)
    transform[:3,:3]=np.diag([-1.,-1.,1.])
    pivot=np.array(correction['pivot_cad_mm'])
    transform[:3,3]=pivot-transform[:3,:3]@pivot
    for index in correction['component_indices']:
        changed[index]=transform
for index,(before,after) in enumerate(zip(old,new)):
    original=np.eye(4)
    original[:3,:]=before['transform']
    expected=changed.get(index,np.eye(4))@original
    assert np.allclose(expected[:3,:],after['transform'],atol=1e-8,rtol=0),index
for index in (10,231):
    before=(old[index]['bounds_mm'][1]+old[index]['bounds_mm'][4])/2
    after=(new[index]['bounds_mm'][1]+new[index]['bounds_mm'][4])/2
    assert before<172.090716 and after>172.090716,(index,before,after)
old_axes=json.loads((baseline/'cad_joint_mapping.json').read_text())['axis_geometry']
new_axes=json.loads((root/'docs/evidence/cad_joint_mapping.json').read_text())['axis_geometry']
for arm in ('arm1','arm2'):
    for key in ('pivots_m','axes','tcp_m'):
        assert np.allclose(old_axes[arm][key],new_axes[arm][key],atol=1e-9,rtol=0),(arm,key)
world=ET.parse(root/'src/dual_arm_gazebo/worlds/dual_arm_workspace.sdf').getroot().find('world')
pads=[model for model in world.findall('model') if model.get('name','').startswith('slot_')]
assert len(pads)==8
for pad in pads:
    position=[float(v) for v in pad.findtext('pose').split()[:3]]
    size=[float(v) for v in pad.findtext('link/collision/geometry/box/size').split()]
    assert np.allclose(size,[.025,.025,.014])
    assert abs(position[2]+size[2]/2-.014)<1e-12
    assert pad.findtext('link/visual/material/diffuse')=='1 1 1 1'
report={'status':'PASS','components':264,'corrected_components':sorted(changed),
        'camera_support_forward':'PASS','motor_axes_and_TCP_preserved':'PASS',
        'white_flush_pads':8,'pad_top_z_m':.014,'scope':'Geometry consistency, not physical collision or hardware acceptance'}
(root/'docs/evidence/model_revision_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
