from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = ParameterValue(Command(['xacro ', share('dual_arm_description'), '/urdf/dual_arm_cad.urdf.xacro mode:=mock']), value_type=str)
    config = Path(share('dual_arm_moveit_config')) / 'config'
    params = {'robot_description': description, 'robot_description_semantic': (config / 'dual_arm.srdf').read_text(), 'robot_description_kinematics': yaml.safe_load((config / 'kinematics.yaml').read_text()), 'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': yaml.safe_load((config / 'ompl_planning.yaml').read_text())}
    params['robot_description_planning'] = yaml.safe_load((config / 'joint_limits.yaml').read_text())
    params.update(yaml.safe_load((config / 'moveit_controllers.yaml').read_text()))
    actions = [DeclareLaunchArgument('rviz', default_value='true'), Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description}]), Node(package='controller_manager', executable='ros2_control_node', parameters=[share('dual_arm_controllers') + '/config/controllers.yaml'], remappings=[('~/robot_description', '/robot_description')]), Node(package='dual_arm_bringup', executable='move_group_lifetime', parameters=[params], output='screen', prefix='gdb -batch -ex "handle SIGINT nostop noprint pass" -ex run -ex bt --args' if os.environ.get('DUAL_ARM_DEBUG_SHUTDOWN')=='1' else ''), Node(package='rviz2', executable='rviz2', parameters=[params], condition=IfCondition(LaunchConfiguration('rviz')))]
    for name in ['joint_state_broadcaster', 'arm1_controller', 'arm2_controller']:
        actions.append(Node(package='controller_manager', executable='spawner', arguments=[name, '--controller-manager-timeout', '60']))
    return LaunchDescription(actions)
