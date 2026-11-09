"""Reproducibly adapt the pinned upstream arm; CAD validation remains separate."""
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
UP = ROOT / 'third_party/phantomx_pincher/phantomx_pincher_description'
OUT = ROOT / 'src/dual_arm_description'
(OUT / 'urdf').mkdir(parents=True, exist_ok=True)
shutil.copytree(UP / 'phantomx_pincher/meshes', OUT / 'meshes', dirs_exist_ok=True)
shutil.copy(ROOT / 'third_party/phantomx_pincher/LICENSE', OUT / 'LICENSE.upstream')
for name in ('hardware', 'utils', 'inertial'):
    text = (UP / f'urdf/phantomx_pincher_{name}.xacro').read_text()
    (OUT / f'urdf/phantomx_pincher_{name}.xacro').write_text(text.replace('phantomx_pincher_description', 'dual_arm_description'))
text = (UP / 'urdf/phantomx_pincher_arm.xacro').read_text()
text = text.replace('phantomx_pincher_description', 'dual_arm_description')
text = '\n'.join(line for line in text.splitlines() if 'phantomx_pincher.ros2_control' not in line)
text = text[:text.index('    <!-- Gripper -->')] + '''
    <!-- Reference suction geometry. CAD transform validation required. -->
    <link name="${prefix}tcp">
      <visual><origin xyz="0 0 -0.01"/><geometry><cylinder radius="0.009" length="0.02"/></geometry><material name="cup"><color rgba="0.6 0.7 0.8 0.7"/></material></visual>
      <collision><origin xyz="0 0 -0.01"/><geometry><cylinder radius="0.009" length="0.02"/></geometry></collision>
      <inertial><origin xyz="0 0 -0.01"/><mass value="0.01"/><inertia ixx="0.00000054" iyy="0.00000054" izz="0.0000004" ixy="0" ixz="0" iyz="0"/></inertial>
    </link>
    <joint name="${prefix}tcp_fixed" type="fixed"><parent link="${prefix}arm_wrist_F3_0_link"/><child link="${prefix}tcp"/><origin xyz="0 0 0.02"/></joint>
  </xacro:macro>
</robot>
'''
(OUT / 'urdf/arm.xacro').write_text(text)

def package(name, dependencies, folders):
    path = ROOT / 'src' / name
    path.mkdir(parents=True, exist_ok=True)
    for folder in folders:
        (path / folder).mkdir(exist_ok=True)
    (path / 'package.xml').write_text(f'''<?xml version="1.0"?>
<package format="3"><name>{name}</name><version>0.1.0</version><description>Dual arm integration</description><maintainer email="maintainer@example.com">Dual Arm Team</maintainer><license>Apache-2.0</license><buildtool_depend>ament_cmake</buildtool_depend>{''.join(f'<exec_depend>{d}</exec_depend>' for d in dependencies)}<export><build_type>ament_cmake</build_type></export></package>
''')
    (path / 'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.16)
project({name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY {' '.join(folders)} DESTINATION share/${{PROJECT_NAME}})
ament_package()
''')

package('dual_arm_description', ['xacro', 'robot_state_publisher'], ['urdf', 'meshes'])
package('dual_arm_controllers', ['controller_manager', 'joint_trajectory_controller', 'joint_state_broadcaster'], ['config'])
package('dual_arm_gazebo', ['ros_gz_sim', 'ros_gz_bridge', 'gz_ros2_control'], ['worlds', 'config'])
package('dual_arm_moveit_config', ['moveit_ros_move_group', 'moveit_kinematics', 'moveit_planners_ompl', 'moveit_simple_controller_manager'], ['config'])
package('dual_arm_bringup', ['dual_arm_description', 'dual_arm_controllers', 'dual_arm_gazebo', 'dual_arm_moveit_config', 'launch_ros', 'rviz2', 'vision_pkg'], ['launch', 'config'])

joints = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint']
control = ''.join(f'''<joint name="arm{arm}_{j}"><command_interface name="position"/><state_interface name="position"><param name="initial_value">0.0</param></state_interface><state_interface name="velocity"/></joint>''' for arm in (1, 2) for j in joints)
(OUT / 'urdf/dual_arm.urdf.xacro').write_text('''<?xml version="1.0"?>
<robot name="dual_arm" xmlns:xacro="http://ros.org/wiki/xacro">
<xacro:arg name="mode" default="mock"/>
<xacro:include filename="$(find dual_arm_description)/urdf/arm.xacro"/>
<link name="world"/>
<xacro:phantomx_pincher prefix="arm1_" ros2_control="false"/>
<xacro:phantomx_pincher prefix="arm2_" ros2_control="false"/>
<joint name="arm1_mount" type="fixed"><parent link="world"/><child link="arm1_base_link"/><origin xyz="0.15 0.22 0.03"/></joint>
<joint name="arm2_mount" type="fixed"><parent link="world"/><child link="arm2_base_link"/><origin xyz="-0.15 0.22 0.03"/></joint>
<ros2_control name="DualArmSystem" type="system"><hardware>
<xacro:if value="${'$(arg mode)' == 'sim'}"><plugin>gz_ros2_control/GazeboSimSystem</plugin></xacro:if>
<xacro:unless value="${'$(arg mode)' == 'sim'}"><plugin>mock_components/GenericSystem</plugin></xacro:unless>
</hardware>''' + control + '''</ros2_control>
<xacro:if value="${'$(arg mode)' == 'sim'}"><gazebo><plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin"><parameters>$(find dual_arm_controllers)/config/controllers.yaml</parameters></plugin></gazebo></xacro:if>
</robot>
''')
controller = 'controller_manager:\n  ros__parameters:\n    update_rate: 100\n    joint_state_broadcaster:\n      type: joint_state_broadcaster/JointStateBroadcaster\n'
for arm in (1, 2):
    controller += f'    arm{arm}_controller:\n      type: joint_trajectory_controller/JointTrajectoryController\n'
for arm in (1, 2):
    controller += f'arm{arm}_controller:\n  ros__parameters:\n    joints: [{", ".join(f"arm{arm}_{j}" for j in joints)}]\n    command_interfaces: [position]\n    state_interfaces: [position, velocity]\n    allow_partial_joints_goal: false\n'
(ROOT / 'src/dual_arm_controllers/config/controllers.yaml').write_text(controller)
srdf = '<robot name="dual_arm">'
for arm in (1, 2):
    srdf += f'<group name="arm{arm}"><chain base_link="arm{arm}_base_link" tip_link="arm{arm}_tcp"/></group>'
srdf += '<group name="both_arms"><group name="arm1"/><group name="arm2"/></group></robot>'
(ROOT / 'src/dual_arm_moveit_config/config/dual_arm.srdf').write_text(srdf)
(ROOT / 'src/dual_arm_moveit_config/config/kinematics.yaml').write_text(''.join(f'arm{arm}:\n  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin\n  position_only_ik: true\n  kinematics_solver_timeout: 0.1\n' for arm in (1, 2)))
(ROOT / 'src/dual_arm_moveit_config/config/ompl_planning.yaml').write_text('planning_plugins: [ompl_interface/OMPLPlanner]\nplanner_configs:\n  RRTConnect:\n    type: geometric::RRTConnect\n    range: 0.0\n' + ''.join(f'arm{arm}:\n  planner_configs: [RRTConnect]\n' for arm in (1, 2)) + 'both_arms:\n  planner_configs: [RRTConnect]\n')
moveit = 'moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager\nmoveit_simple_controller_manager:\n  controller_names: [arm1_controller, arm2_controller]\n'
for arm in (1, 2):
    moveit += f'  arm{arm}_controller:\n    type: FollowJointTrajectory\n    action_ns: follow_joint_trajectory\n    default: true\n    joints: [{", ".join(f"arm{arm}_{j}" for j in joints)}]\n'
(ROOT / 'src/dual_arm_moveit_config/config/moveit_controllers.yaml').write_text(moveit)
print('Generated reference description and configurations; CAD acceptance NOT_RUN')
