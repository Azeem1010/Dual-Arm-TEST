from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    actions = [DeclareLaunchArgument('calib_dir', default_value='~/.ros/dual_arm_calib/sim'), IncludeLaunchDescription(PythonLaunchDescriptionSource(share('dual_arm_bringup') + '/launch/sim.launch.py'))]
    for name in ['env', 'arm1', 'arm2']:
        actions.append(Node(package='vision_pkg', executable=name + '_vision_node', parameters=[share('vision_pkg') + '/config/' + name + '_vision.yaml', {'use_sim_time': True, 'show_debug': False, 'calib_dir': LaunchConfiguration('calib_dir')}], output='screen'))
    return LaunchDescription(actions)
