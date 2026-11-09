"""Run upstream vision_core on real saved Gazebo frames, report measurements.

This does not authorize calibration: no independent measured error budget or
acquisition-correlated ground truth has yet been established.
"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/vision_pkg'))
from vision_pkg.vision_core.cap_detect import CapParams,detect_caps
from vision_pkg.vision_core.holes import HoleParams,locate_holes

runtime=json.loads((root/'docs/evidence/sim_runtime.json').read_text())
report={}
for name in ('env','arm1','arm2'):
    frame=cv2.imread(str(root/f'docs/evidence/{name}_gazebo.png'))
    if frame is None:
        raise RuntimeError('Missing actual Gazebo camera frame')
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    blue=cv2.inRange(hsv,np.array([95,100,50]),np.array([130,255,255]))
    # Simulation palette is more saturated than the upstream physical defaults.
    pink=cv2.inRange(hsv,np.array([150,35,110]),np.array([179,255,255])) | cv2.inRange(hsv,np.array([0,35,110]),np.array([8,255,255]))
    caps,rejected=detect_caps(blue,CapParams(30.,30000.,.75,.65,True,2))
    layout=locate_holes(pink,HoleParams(150.,20000.,1.5,.75,.1,.2),0)
    info=runtime['camera_info'].get(name)
    report[name]={'detected_caps':len(caps),'cap_pixels':[[c.cx,c.cy] for c in caps],
                  'grid_status':'PASS' if layout.ok else 'FAIL','grid_message':layout.message,
                  'slot_pixels':[[h.cx,h.cy] for h in layout.holes],
                  'camera_info_status':'PASS' if info and info['width']==frame.shape[1] and info['height']==frame.shape[0] else 'FAIL',
                  'calibration_status':'NOT_RUN'}
(root/'docs/evidence/sim_detection.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
