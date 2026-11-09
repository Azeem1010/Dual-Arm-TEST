"""Compare original vision detections to actual Gazebo poses and CAD holes.

Uses CameraInfo as simulation intrinsic baseline, not physical calibration.
Reports errors without inventing an unmeasured acceptance budget.
"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/vision_pkg'))
from vision_pkg.vision_core.cap_detect import CapParams,detect_caps
from vision_pkg.vision_core.holes import HoleParams,locate_holes
runtime=json.loads((root/'docs/evidence/sim_runtime.json').read_text())
report={'mode':'sim','hardware_calibration_status':'NOT_RUN','cameras':{}}
for name,cx,cz in [('env',0.,.8),('arm1',.2,.5),('arm2',-.2,.5)]:
    info=runtime['camera_info'][name]
    k=np.array(info['k']).reshape(3,3)
    frame=cv2.imread(str(root/f'docs/evidence/{name}_gazebo.png'))
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    if name=='arm2':
        pink=cv2.inRange(hsv,np.array([150,35,110]),np.array([179,255,255])) | cv2.inRange(hsv,np.array([0,35,110]),np.array([8,255,255]))
        layout=locate_holes(pink,HoleParams(150.,20000.,1.5,.75,.1,.2),0)
        if not layout.ok:
            report['cameras'][name]={'status':'FAIL','reason':layout.message}
            continue
        pixels=np.array([[h.cx,h.cy] for h in layout.holes])
        truth=np.array([[-.1675+col*.035,-.015+row*.035] for row in range(4) for col in range(2)])
        plane=.014
        source='CAD hole centers, fixed source geometry'
    else:
        blue=cv2.inRange(hsv,np.array([95,100,50]),np.array([130,255,255]))
        caps,_=detect_caps(blue,CapParams(30.,30000.,.75,.65,True,2))
        pixels=np.array([[c.cx,c.cy] for c in caps])
        actual=[runtime.get('ground_truth',{}).get(f'cap_{n}') for n in range(1,9)]
        if not all(actual) or not len(caps):
            report['cameras'][name]={'status':'FAIL','reason':'Missing independent actual cap poses or detections'}
            continue
        truth=np.array([a['position'][:2] for a in actual])
        plane=float(np.median([a['position'][2]+.005 for a in actual]))
        source='Gazebo PosePublisher cap body positions plus 5 mm cap half-height'
    depth=cz-plane
    h=np.array([[depth/k[0,0],0,cx-depth*k[0,0]**-1*k[0,2]],[0,-depth/k[1,1],depth*k[1,1]**-1*k[1,2]],[0,0,1]])
    mapped=cv2.perspectiveTransform(pixels.astype(float).reshape(-1,1,2),h).reshape(-1,2)
    if name=='arm2':
        expected=truth
    else:
        rows,cols=linear_sum_assignment(np.linalg.norm(mapped[:,None,:]-truth[None,:,:],axis=2))
        expected=truth[cols]
        mapped=mapped[rows]
        pixels=pixels[rows]
    error=np.linalg.norm(mapped-expected,axis=1)
    projected=cv2.perspectiveTransform(expected.reshape(-1,1,2),np.linalg.inv(h)).reshape(-1,2)
    report['cameras'][name]={'measurement_status':'PASS','calibration_validity':'NOT_RUN',
        'reason':'Error measured; synchronized acquisition/pose provenance and measured combined tolerance budget still required',
        'detections':len(pixels),'truth_source':source,'plane_z_m':plane,
        'camera_matrix':k.ravel().tolist(),'distortion':info['d'],'world_homography':h.ravel().tolist(),
        'maximum_error_m':float(error.max()),'rms_error_m':float(np.sqrt(np.mean(error**2))),
        'maximum_reprojection_error_px':float(np.linalg.norm(projected-pixels,axis=1).max()),
        'pixels':pixels.tolist(),'mapped_xy_m':mapped.tolist(),'expected_xy_m':expected.tolist()}
    if len(pixels)==8:
        # Four extreme points fit the actual detector/plane mapping. Four inner
        # points are held out, so a fit residual cannot masquerade as validation.
        fit=[0,1,6,7]
        check=[2,3,4,5]
        calibrated,_=cv2.findHomography(pixels[fit],expected[fit],method=0)
        if calibrated is not None:
            verified=cv2.perspectiveTransform(pixels[check].reshape(-1,1,2),calibrated).reshape(-1,2)
            errors=np.linalg.norm(verified-expected[check],axis=1)
            report['cameras'][name]['fitted_plane']={'homography':calibrated.ravel().tolist(),'fit_indices':fit,'held_out_indices':check,'held_out_maximum_error_m':float(errors.max()),'held_out_rms_error_m':float(np.sqrt(np.mean(errors**2))),'validity':'NOT_RUN','note':'Single static scene only; needs separately acquired observations, intrinsic acquisition validation, live TF/mount and tolerance budget.'}
(root/'docs/evidence/sim_coordinate_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
