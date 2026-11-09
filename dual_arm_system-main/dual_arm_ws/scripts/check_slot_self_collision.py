"""Query MoveIt for saved proposal states; never commands a trajectory.

Only current URDF collision boxes and SRDF exclusions are covered. The other
arm stays at model zero; no table, foam, cap or platform world object is added.
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/dual_arm_coordinator'))
from dual_arm_coordinator.kinematics import CadKinematics


def main():
    os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 100)
    evidence = ROOT / 'docs/evidence'
    model_path = Path(get_package_share_directory('dual_arm_description')) / 'urdf/dual_arm_cad.urdf.xacro'
    srdf_path = Path(get_package_share_directory('dual_arm_moveit_config')) / 'config/dual_arm.srdf'
    source_path = ROOT / 'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro'
    if model_path.read_bytes() != source_path.read_bytes():
        raise RuntimeError('Installed and source models differ; rebuild before measuring')
    study_path = evidence / 'slot_adjustment.json'
    study = json.loads(study_path.read_text())
    arm1 = CadKinematics(model_path.read_text(), 'arm1')
    arm2 = CadKinematics(model_path.read_text(), 'arm2')
    candidates = [(c['kind'], c['dense_1mm_approach']) for c in study['candidates'] if 'offset_m' in c]
    candidates.append(('toward_user_y_10mm', study['ten_mm_candidate']['validation']))
    report = {
        'status': 'NOT_RUN',
        'scope': 'Discrete self-collision with current URDF boxes and SRDF exclusions; arm1 at model zero',
        'world_collision_status': 'NOT_RUN', 'continuous_path_status': 'NOT_RUN',
        'urdf_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
        'srdf_sha256': hashlib.sha256(srdf_path.read_bytes()).hexdigest(),
        'study_sha256': hashlib.sha256(study_path.read_bytes()).hexdigest(),
        'candidates': [],
    }
    with (evidence / 'slot_collision_launch.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'dual_arm_bringup', 'mock.launch.py', 'rviz:=false'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node('slot_self_collision_measurement')
        try:
            client = node.create_client(GetStateValidity, '/check_state_validity')
            if not client.wait_for_service(timeout_sec=60):
                raise RuntimeError('MoveIt state validity service unavailable')
            for name, samples in candidates:
                if len(samples) != 248:
                    raise ValueError('Expected 248 discrete states per proposal')
                result = {'name': name, 'checked': 0, 'invalid': 0, 'contacts': [], 'invalid_samples': []}
                pairs = set()
                for sample in samples:
                    request = GetStateValidity.Request()
                    request.group_name = 'both_arms'
                    request.robot_state.joint_state.name = arm1.names + arm2.names
                    request.robot_state.joint_state.position = [0.] * 4 + sample['joints']
                    request.robot_state.is_diff = False
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                    if not future.done() or future.result() is None:
                        raise RuntimeError('State validity query did not return')
                    response = future.result()
                    result['checked'] += 1
                    if not response.valid:
                        result['invalid'] += 1
                        result['invalid_samples'].append({'slot': sample['slot'], 'lift_m': sample['lift_m']})
                    for contact in response.contacts:
                        pair = tuple(sorted([contact.contact_body_1, contact.contact_body_2]))
                        if pair not in pairs:
                            pairs.add(pair)
                            result['contacts'].append({'bodies': pair, 'depth_m': contact.depth})
                result['status'] = 'PASS' if result['invalid'] == 0 else 'FAIL'
                report['candidates'].append(result)
                print(name, result['status'], 'invalid', result['invalid'], '/', result['checked'], flush=True)
            report['status'] = 'PASS' if all(c['status'] == 'PASS' for c in report['candidates']) else 'FAIL'
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
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            (evidence / 'slot_self_collision.json').write_text(json.dumps(report, indent=2) + '\n')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
