"""Package integration modules without touching upstream source checkouts."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
for name, entries, deps in [
    ('dual_arm_vision_adapter', ['calibration_status = dual_arm_vision_adapter.status_node:main'], ['rclpy','sensor_msgs','diagnostic_msgs','python3-yaml','vision_pkg']),
    ('dual_arm_coordinator', [], ['rclpy','vision_interfaces']),
]:
    p = root/'src'/name
    (p/'resource').mkdir(exist_ok=True)
    (p/'resource'/name).write_text('')
    (p/'setup.cfg').write_text(f'[develop]\nscript_dir=$base/lib/{name}\n[install]\ninstall_scripts=$base/lib/{name}\n')
    (p/'setup.py').write_text(f'''from setuptools import setup, find_packages
from glob import glob
setup(name={name!r}, version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages',['resource/{name}']),('share/{name}',['package.xml'])]+[('share/{name}/'+d,glob(d+'/*.yaml')) for d in ['config/sim','config/hardware']], install_requires=['setuptools'], entry_points={{'console_scripts':{entries!r}}})
''')
    (p/'package.xml').write_text(f'''<package format="3"><name>{name}</name><version>0.1.0</version><description>Dual arm integration</description><maintainer email="maintainer@example.com">Dual Arm Team</maintainer><license>Apache-2.0</license><buildtool_depend>ament_python</buildtool_depend>{''.join(f'<exec_depend>{d}</exec_depend>' for d in deps)}<export><build_type>ament_python</build_type></export></package>''')
for mode in ('sim','hardware'):
    p = root/'src/dual_arm_vision_adapter/config'/mode
    p.mkdir(parents=True,exist_ok=True)
    for camera in ('env','arm1','arm2'):
        (p/f'{camera}_camera.yaml').write_text(f'''mode: {mode}
camera_id: {camera}
frame_id: {camera}_camera_optical_frame
reference_frame: {'world' if camera == 'env' else camera+'_base_link'}
status: NOT_RUN
evidence_verified: false
budget_verified: false
slot_order_verified: false
camera_matrix: null
distortion: null
homography: null
reason: No independently validated calibration evidence
''')
