"""One CM530, one planning scene, one A→B→C coordinator."""
from pathlib import Path
import hashlib
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('mode')
    if mode not in ('demo', 'hardware'):
        raise ValueError('mode must be demo or hardware')
    demo = mode == 'demo'
    start_vision = value('vision').lower() == 'true' if value('vision') else not demo
    control = Path(get_package_share_directory('dual_arm_control'))
    description = Path(get_package_share_directory('dual_arm_description'))
    config = yaml.safe_load(Path(value('config') or control / 'config' / f'dual_arm_{mode}.yaml').read_text())
    task = dict(config['dual_coordinator']['ros__parameters'])
    bridge = dict(config['cm530_bridge']['ros__parameters'])
    task['simulation'], bridge['mock_hardware'] = demo, demo
    task.setdefault('journal_path', '~/.ros/dual_arm/' + mode + '_integrated.json')
    task['auto_confirm_demo'] = value('auto_confirm_demo').lower() == 'true'
    if task['auto_confirm_demo'] and not demo:
        raise ValueError('auto_confirm_demo is simulation only')
    if value('journal'):
        task['journal_path'] = value('journal')
    if value('cycles'):
        task['cycles'] = int(value('cycles'))
    if not 1 <= task.get('cycles', 1) <= 8:
        raise ValueError('cycles must be in 1..8')
    if not demo and (not value('robot_urdf') or not value('robot_srdf') or
                     not bridge.get('hardware_calibrated') or not bridge.get('serial_port')):
        raise ValueError('Hardware requires measured dual-arm URDF/SRDF, calibration and explicit CM530 serial port')
    urdf = Path(value('robot_urdf') or description / 'urdf/dual_arm_demo.urdf').read_text()
    srdf = Path(value('robot_srdf') or description / 'srdf/dual_arm_demo.srdf').read_text()
    signature = hashlib.sha256((urdf + srdf).encode()).hexdigest()
    from dual_arm_control.coordinator import DEFAULTS, validate_config
    from dual_arm_control.arm2_coordinator import ARM2_DEFAULTS
    from dual_arm_control.arm2_config import validate_arm2
    from dual_arm_control.cm530 import Calibration
    from dual_arm_control.model_config import validate_handoff
    checked_arms = {}
    # Validate both workflows and both motor mappings BEFORE opening the port.
    for arm, defaults in [('arm1', DEFAULTS), ('arm2', ARM2_DEFAULTS)]:
        task[arm + '.model_signature'] = signature
        checked = {**defaults, **{k[len(arm)+1:]:v for k,v in task.items() if k.startswith(arm+'.')},
                   'simulation':demo,'auto_confirm_demo':task['auto_confirm_demo'],'integrated':True}
        validate_config(checked)
        checked_arms[arm] = checked
        if arm == 'arm2':
            validate_arm2(checked)
        if not demo:
            calibration = Calibration(*[bridge[arm+'.'+k] for k in ('zero_raw','direction','lower_rad','upper_rad','raw_min','raw_max')])
            bridge[arm+'.expected_home_raw'] = calibration.to_raw(checked['home_joints'])
    validate_handoff(urdf, checked_arms['arm1'], checked_arms['arm2'])
    bridge['verify_home'] = not demo
    bridge.update(arms=['arm1','arm2'],require_safety_inputs=True,require_arm1_clear=False,require_coordinator=True)
    robot = {'robot_description':ParameterValue(urdf,value_type=str)}
    joints = {a:[a+'_'+j for j in ('arm_shoulder_pan_joint','arm_shoulder_lift_joint','arm_elbow_flex_joint','arm_wrist_flex_joint')]
              for a in ('arm1','arm2')}
    moveit = {**robot,'robot_description_semantic':ParameterValue(srdf,value_type=str),
        'robot_description_kinematics':{a:{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution':0.005,'kinematics_solver_timeout':0.2} for a in joints},
        'robot_description_planning':{'joint_limits':{j:{'has_velocity_limits':True,'max_velocity':0.52359878,
            'has_acceleration_limits':True,'max_acceleration':1.0} for names in joints.values() for j in names}},
        'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
        'ompl':{'planning_plugins':['ompl_interface/OMPLPlanner'],
            'request_adapters':['default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters':['default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution','default_planning_response_adapters/DisplayMotionPath'],
            'planner_configs':{'RRTConnectkConfigDefault':{'type':'geometric::RRTConnect','range':0.0}},
            **{a:{'planner_configs':['RRTConnectkConfigDefault'],'longest_valid_segment_fraction':0.005} for a in joints}},
        'allow_trajectory_execution':True,'trajectory_execution.allowed_start_tolerance':0.04,
        'trajectory_execution.allowed_execution_duration_scaling':2.0,
        'trajectory_execution.allowed_goal_duration_margin':15.0,
        'publish_robot_description_semantic':True,
        'planning_scene_monitor.publish_planning_scene':True,
        'planning_scene_monitor.publish_state_updates':True,
        'planning_scene_monitor.publish_transforms_updates':True,
        'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager':{'controller_names':[a+'_controller' for a in joints],
            **{a+'_controller':{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':names}
               for a,names in joints.items()}}}
    nodes = [Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[robot],output='screen'),
             Node(package='dual_arm_control',executable='cm530_bridge',parameters=[bridge],output='screen'),
             Node(package='moveit_ros_move_group',executable='move_group',parameters=[moveit],output='screen'),
             Node(package='dual_arm_control',executable='dual_coordinator',parameters=[task],output='screen')]
    if demo:
        mock = dict(config.get('dual_mock_vision',{}).get('ros__parameters',{}))
        mock['c_poses'] = task['arm2.c_poses']
        if not start_vision:
            nodes.append(Node(package='dual_arm_control',executable='dual_mock_vision',parameters=[mock],output='screen'))
    actions = list(nodes)
    if start_vision:
        vision_launch = Path(get_package_share_directory('vision_pkg')) / 'launch/vision.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vision_launch)),
            launch_arguments={k: value(k) for k in ('env_cam_device','arm1_cam_device','arm2_cam_device',
                                                   'calib_dir','show_debug','undistort')}.items()))
    return actions + [RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[
        EmitEvent(event=Shutdown(reason='Integrated process exited; no automatic restart'))])) for n in nodes]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in
        [('mode','demo'),('config',''),('auto_confirm_demo','false'),('journal',''),('cycles',''),
         ('robot_urdf',''),('robot_srdf',''),('vision',''),('env_cam_device',''),
         ('arm1_cam_device',''),('arm2_cam_device',''),('calib_dir',''),
         ('show_debug','false'),('undistort','')]] + [OpaqueFunction(function=setup)])
