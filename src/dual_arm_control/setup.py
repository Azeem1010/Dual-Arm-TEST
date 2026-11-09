from glob import glob
from setuptools import find_packages, setup

setup(name='dual_arm_control', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/dual_arm_control']),
                  ('share/dual_arm_control', ['package.xml']),
                  ('share/dual_arm_control/launch', glob('launch/*.launch.py')),
                  ('share/dual_arm_control/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Dual arm team', maintainer_email='maintainer@example.com',
      description='Integrated A to B to C controller with CM530 protocol 5 integration.',
      license='BSD-3-Clause', tests_require=['pytest'],
      entry_points={'console_scripts': [
          'manual_xy = dual_arm_control.manual_xy:main',
          'manual_xy_server = dual_arm_control.manual_xy:server_main',
          'check_xy_calibration = dual_arm_control.xy_calibration:main',
          'dual_coordinator = dual_arm_control.dual_coordinator:main',
          'dual_mock_vision = dual_arm_control.dual_mock_vision:main',
          'task_coordinator = dual_arm_control.coordinator:main',
          'arm1_vision_test = dual_arm_control.arm1_vision_test:main',
          'arm2_coordinator = dual_arm_control.arm2_coordinator:main',
          'arm2_mock_vision = dual_arm_control.arm2_mock_vision:main',
          'cm530_bridge = dual_arm_control.bridge_node:main',
          'mock_vision = dual_arm_control.mock_vision:main',
          'operator_console = dual_arm_control.operator_console:main',
      ]})
