"""Read-only live MoveIt state checks for a single-object transfer demonstration."""
import os
os.environ['ROS_DOMAIN_ID'] = '42'
from pathlib import Path
import sys
import json
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/dual_arm_coordinator'))
from dual_arm_coordinator.kinematics import CadKinematics
from runtime_settings import CONTACT_Z

rclpy.init()
node = Node('transfer_target_probe')
client = node.create_client(GetStateValidity, '/check_state_validity')
models = {a: CadKinematics((ROOT/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro').read_text(), a) for a in ['arm1','arm2']}
report = []
try:
    if not client.wait_for_service(timeout_sec=20):
        raise RuntimeError('Live MoveIt unavailable')
    targets = []
    for i in range(8):
        targets.append(('A_cap_'+str(i+1), 'arm1', [.14+(i%2)*.06, -.1+(i//2)*.065, .024]))
    for y in [-.02, 0., .02, .04, .06]:
        for arm in models:
            targets.append(('B_'+str(y), arm, [0., y, .024]))
    for i in range(8):
        targets.append(('C_slot_'+str(i+1), 'arm2', [-.1675+(i%2)*.035, -.015+(i//2)*.035, CONTACT_Z]))
    for name, arm, xyz in targets:
        entry = {'name': name, 'arm': arm, 'target': xyz, 'samples': [], 'status': 'PASS'}
        seed = None
        for lift in [.03, .02, .01, 0.]:
            try:
                q = models[arm].solve([xyz[0],xyz[1],xyz[2]+lift], seed)
                seed = q
                request = GetStateValidity.Request()
                request.group_name = 'both_arms'
                request.robot_state.joint_state.name = models['arm1'].names + models['arm2'].names
                request.robot_state.joint_state.position = q+[0.]*4 if arm=='arm1' else [0.]*4+q
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done() or future.result() is None:
                    raise RuntimeError('State query timeout')
                response = future.result()
                entry['samples'].append({'lift': lift, 'q': q, 'valid': response.valid,
                    'contacts': [[c.contact_body_1,c.contact_body_2] for c in response.contacts]})
                if not response.valid:
                    entry['status'] = 'FAIL'
            except ValueError:
                entry['samples'].append({'lift': lift, 'error': 'IK failed'})
                entry['status'] = 'FAIL'
        report.append(entry)
        print(name, arm, entry['status'], flush=True)
finally:
    (ROOT/'docs/evidence/transfer_target_probe.json').write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
