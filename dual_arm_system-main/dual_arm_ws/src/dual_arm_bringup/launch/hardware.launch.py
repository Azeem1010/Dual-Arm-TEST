"""CM-530 bringup. Read-only by default: no arm controllers, no motor writes.

Motion needs enable_motion:=true, a measured calibration and a controllers_file
with measured goal tolerances. Even then the hardware starts inactive, so torque
is enabled only by an explicit operator command:

  ros2 control set_hardware_component_state DualArmSystem active
  ros2 control switch_controllers --activate arm1_controller arm2_controller

A hardware emergency stop independent of this software is required.
"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HARDWARE_NAME = 'DualArmSystem'
MOCK_PORT = '/tmp/cm530_mock_serial'


def require_bool(context, name):
    value = LaunchConfiguration(name).perform(context)
    if value not in ('true', 'false'):
        raise RuntimeError(f'{name} must be true or false, got {value!r}')
    return value == 'true'


def spawner(name, *extra):
    return Node(package='controller_manager', executable='spawner',
                arguments=[name, '--controller-manager-timeout', '60', *extra])


def setup(context):
    enable_motion = require_bool(context, 'enable_motion')
    use_mock = require_bool(context, 'use_mock_serial')
    use_moveit = require_bool(context, 'use_moveit')
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    calibration = LaunchConfiguration('calibration_file').perform(context)
    port = MOCK_PORT if use_mock else LaunchConfiguration('port').perform(context)
    if enable_motion and not controllers_file:
        raise RuntimeError('enable_motion:=true requires controllers_file with measured goal tolerances')

    description = ParameterValue(Command([
        'xacro ', share('dual_arm_description'), '/urdf/dual_arm_cad.urdf.xacro mode:=hardware',
        ' port:=', port, ' calibration_file:=', calibration,
        ' enable_motion:=', 'true' if enable_motion else 'false']), value_type=str)

    control_params = [controllers_file or share('dual_arm_controllers') + '/config/controllers.yaml']
    if enable_motion:
        control_params.append({'hardware_components_initial_state.inactive': [HARDWARE_NAME]})

    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='controller_manager', executable='ros2_control_node', parameters=control_params,
             remappings=[('~/robot_description', '/robot_description')], output='screen'),
        spawner('joint_state_broadcaster'),
    ]
    if enable_motion:
        actions += [spawner('arm1_controller', '--inactive'), spawner('arm2_controller', '--inactive')]
        if use_moveit:
            config = Path(share('dual_arm_moveit_config')) / 'config'
            params = {
                'robot_description': description,
                'robot_description_semantic': (config / 'dual_arm.srdf').read_text(),
                'robot_description_kinematics': yaml.safe_load((config / 'kinematics.yaml').read_text()),
                'robot_description_planning': yaml.safe_load((config / 'joint_limits.yaml').read_text()),
                'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl',
                'ompl': yaml.safe_load((config / 'ompl_planning.yaml').read_text()),
            }
            params.update(yaml.safe_load((config / 'moveit_controllers.yaml').read_text()))
            actions.append(Node(package='dual_arm_bringup', executable='move_group_lifetime',
                                parameters=[params], output='screen'))
    if not use_mock:
        return actions
    # The mock must create its pseudo-terminal before the driver opens it.
    mock = Node(package='cm530_hardware', executable='cm530_mock_serial',
                arguments=['--link', MOCK_PORT], output='screen')
    return [mock, TimerAction(period=1.5, actions=actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('calibration_file',
                              default_value=share('cm530_hardware') + '/config/calibration.yaml'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('controllers_file', default_value=''),
        DeclareLaunchArgument('use_mock_serial', default_value='false'),
        DeclareLaunchArgument('use_moveit', default_value='true'),
        OpaqueFunction(function=setup),
    ])
