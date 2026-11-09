from setuptools import setup, find_packages
from glob import glob
setup(name='dual_arm_coordinator', version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages',['resource/dual_arm_coordinator']),('share/dual_arm_coordinator',['package.xml'])]+[('share/dual_arm_coordinator/'+d,glob(d+'/*.yaml')+glob(d+'/*.json')) for d in ['config/sim','config/hardware']], install_requires=['setuptools'], entry_points={'console_scripts':[]})
