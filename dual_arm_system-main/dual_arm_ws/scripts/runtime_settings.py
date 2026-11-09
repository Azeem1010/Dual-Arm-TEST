"""Shared simulation geometry, speeds and explicit HOME calibration requirements."""
from pathlib import Path
import json
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/dual_arm_coordinator'))
from dual_arm_coordinator.home_pose import convert_home

SETTINGS=json.loads((ROOT/'src/dual_arm_coordinator/config/sim/runtime.json').read_text())
SURFACE_Z=SETTINGS['surface_z_m']
CAP_CENTER_Z=SURFACE_Z+SETTINGS['cap_height_m']/2
CONTACT_Z=SURFACE_Z+SETTINGS['cap_height_m']+.0003


def configured_homes(models):
    homes={}
    for arm,model in models.items():
        config=SETTINGS['home'][arm]
        try:
            q=convert_home(config['raw'],config['raw_at_cad_zero'],config['direction'])
        except ValueError as error:
            raise ValueError(config['name']+': '+str(error)+'. See docs/HOME_POSES.md.') from error
        if any(value<lo or value>hi for value,lo,hi in zip(q,model.lower,model.upper)):
            raise ValueError(config['name']+' exceeds current model limits; do not widen limits to bypass this')
        homes[arm]=q
    return homes
