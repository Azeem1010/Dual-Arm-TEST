"""Query MoveIt for the configured HOME states; never commands a trajectory.

Covers discrete self-collision with current URDF boxes and SRDF exclusions only.
Table, foam, caps and platform are not in the MoveIt scene, and no path is checked.
"""
from pathlib import Path
import hashlib
import json
import os
import signal
import subprocess
import sys

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts'))
from runtime_settings import configured_homes
from dual_arm_coordinator.kinematics import CadKinematics


def query(node, client, names, values):
    request = GetStateValidity.Request()
    request.group_name = 'both_arms'
    request.robot_state.joint_state.name = names
    request.robot_state.joint_state.position = values
    request.robot_state.is_diff = False
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done() or future.result() is None:
        raise RuntimeError('State validity query did not return')
    response = future.result()
    pairs = sorted({tuple(sorted([c.contact_body_1, c.contact_body_2])) for c in response.contacts})
    return {'valid': response.valid, 'contacts': pairs}


def main():
    os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 100)
    evidence = ROOT / 'docs/evidence'
    model_path = Path(get_package_share_directory('dual_arm_description')) / 'urdf/dual_arm_cad.urdf.xacro'
    srdf_path = Path(get_package_share_directory('dual_arm_moveit_config')) / 'config/dual_arm.srdf'
    source_path = ROOT / 'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro'
    if model_path.read_bytes() != source_path.read_bytes():
        raise RuntimeError('Installed and source models differ; rebuild before measuring')
    text = model_path.read_text()
    models = {arm: CadKinematics(text, arm) for arm in ('arm1', 'arm2')}
    homes = configured_homes(models)
    names = models['arm1'].names + models['arm2'].names
    zero = [0.] * 4
    states = {'home_A_home_C': homes['arm1'] + homes['arm2'],
              'home_A_arm2_cad_zero': homes['arm1'] + zero,
              'arm1_cad_zero_home_C': zero + homes['arm2']}
    report = {'status': 'NOT_RUN',
              'scope': 'Discrete self/inter-arm collision with URDF boxes and SRDF exclusions',
              'world_collision_status': 'NOT_RUN', 'continuous_path_status': 'NOT_RUN',
              'urdf_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
              'srdf_sha256': hashlib.sha256(srdf_path.read_bytes()).hexdigest(),
              'joint_rad': homes, 'states': {}}
    with (evidence / 'home_collision_launch.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'dual_arm_bringup', 'mock.launch.py', 'rviz:=false'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node('home_collision_measurement')
        try:
            client = node.create_client(GetStateValidity, '/check_state_validity')
            if not client.wait_for_service(timeout_sec=180):
                raise RuntimeError('MoveIt state validity service unavailable')
            for name, values in states.items():
                report['states'][name] = query(node, client, names, values)
                print(name, report['states'][name], flush=True)
            report['status'] = 'PASS' if all(s['valid'] for s in report['states'].values()) else 'FAIL'
        except Exception as error:
            report['status'] = 'FAIL'
            report['error'] = str(error)
            raise
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    report['shutdown'] = 'FAIL'
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            (evidence / 'home_collision.json').write_text(json.dumps(report, indent=2) + '\n')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
