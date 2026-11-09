from pathlib import Path
import subprocess
import json

root = Path(__file__).resolve().parents[1]
manifest = {}
for name in ['phantomx_pincher','dual_arm_3d','CM530_ROS_BRIDGE','three-camera-vision-ros2']:
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root/'third_party'/name), *args], text=True).strip()
    manifest[name] = dict(url=git('remote','get-url','origin'), commit=git('rev-parse','HEAD'), branch=git('branch','--show-current'))
(root/'docs/sources.lock.json').write_text(json.dumps(manifest, indent=2)+'\n')
print(json.dumps(manifest, indent=2))
