from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = ParameterValue(Command(['xacro ', share('dual_arm_description'), '/urdf/dual_arm_cad.urdf.xacro mode:=sim']), value_type=str)
    config = Path(share('dual_arm_moveit_config')) / 'config'
    params = {'use_sim_time': True, 'robot_description': description, 'robot_description_semantic': (config / 'dual_arm.srdf').read_text(), 'robot_description_kinematics': yaml.safe_load((config / 'kinematics.yaml').read_text()), 'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': yaml.safe_load((config / 'ompl_planning.yaml').read_text())}
    params['robot_description_planning'] = yaml.safe_load((config / 'joint_limits.yaml').read_text())
    params.update(yaml.safe_load((config / 'moveit_controllers.yaml').read_text()))
    actions = [DeclareLaunchArgument('gz_args', default_value='-r -s'), IncludeLaunchDescription(PythonLaunchDescriptionSource(share('ros_gz_sim') + '/launch/gz_sim.launch.py'), launch_arguments={'gz_args': [LaunchConfiguration('gz_args'), ' ', share('dual_arm_gazebo'), '/worlds/dual_arm_workspace.sdf']}.items()), Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description, 'use_sim_time': True}]), Node(package='ros_gz_sim', executable='create', arguments=['-name', 'dual_arm', '-topic', '/robot_description'], output='screen'), Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': share('dual_arm_gazebo') + '/config/bridge.yaml'}]), Node(package='dual_arm_bringup', executable='move_group_lifetime', parameters=[params], output='screen')]
    for name in ['joint_state_broadcaster', 'arm1_controller', 'arm2_controller']:
        actions.append(Node(package='controller_manager', executable='spawner', arguments=[name, '--controller-manager-timeout', '120', '--switch-timeout', '120'], parameters=[{'use_sim_time': True}]))
    for name, x, z in [('env', 0.0, 0.8), ('arm1', 0.2, 0.5), ('arm2', -0.2, 0.5)]:
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x', str(x), '--y', '0', '--z', str(z), '--roll', '3.14159265359', '--frame-id', 'world', '--child-frame-id', name + '_camera_optical_frame']))
    actions.insert(0, SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(Path(share('dual_arm_description')).parent) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH','')))
    return LaunchDescription(actions)
