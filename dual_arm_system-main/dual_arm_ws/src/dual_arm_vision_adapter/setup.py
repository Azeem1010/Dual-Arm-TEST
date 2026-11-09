from setuptools import setup, find_packages
from glob import glob
setup(name='dual_arm_vision_adapter', version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages',['resource/dual_arm_vision_adapter']),('share/dual_arm_vision_adapter',['package.xml'])]+[('share/dual_arm_vision_adapter/'+d,glob(d+'/*.yaml')) for d in ['config/sim','config/hardware']], install_requires=['setuptools'], entry_points={'console_scripts':['calibration_status = dual_arm_vision_adapter.status_node:main']})
