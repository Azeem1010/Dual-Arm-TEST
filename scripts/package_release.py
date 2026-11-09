#!/usr/bin/env python3
"""Package ROS sources, models, configuration, instructions and test evidence."""
import hashlib
from pathlib import Path
import zipfile

root=Path(__file__).resolve().parents[1]
out=root/'releases';out.mkdir(exist_ok=True)
archive=out/'dual_arm_ros2_cm530_v17.zip'
files=[root/'README.md']
for folder in ('src','scripts','docs'):
    files.extend(p for p in (root/folder).rglob('*') if p.is_file()
        and not any(part in ('__pycache__','.pytest_cache','build') or part.endswith('.egg-info') for part in p.parts)
        and p.suffix not in ('.pyc','.pyo'))
manifest=[]
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as z:
    for p in sorted(files):
        name=p.relative_to(root).as_posix()
        data=p.read_bytes()
        z.writestr('dual_arm_ros2_cm530_v17/'+name,data)
        manifest.append(hashlib.sha256(data).hexdigest()+'  '+name)
    z.writestr('dual_arm_ros2_cm530_v17/SHA256SUMS','\n'.join(manifest)+'\n')
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
(out/'SHA256SUMS').write_text(hashlib.sha256(archive.read_bytes()).hexdigest()+'  '+archive.name+'\n')
print(f'{archive}: {len(files)} source/document files, {archive.stat().st_size} bytes')
