"""One-object physical Gazebo transfer on the dedicated demo domain.

Uses simulator ground truth, not calibrated vision. Original CAD is unchanged.
MoveIt checks current robot collision geometry; complete workstation/payload
planning-scene acceptance remains outside this demonstration.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '42'
from pathlib import Path
import json
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/dual_arm_coordinator'))
from dual_arm_coordinator.kinematics import CadKinematics
from runtime_settings import SETTINGS, CAP_CENTER_Z, configured_homes


class Demo(Node):
    def __init__(self):
        super().__init__('single_object_gazebo_transfer')
        self.joints, self.object_position, self.gripper = {}, None, {}
        self.object_received = 0.
        self.clock = None
        self.models = {a: CadKinematics((ROOT/'src/dual_arm_description/urdf/dual_arm_cad.urdf.xacro').read_text(), a)
                       for a in ['arm1', 'arm2']}
        self.subscriptions_owned = [
            self.create_subscription(JointState, '/joint_states', lambda m: self.joints.update(zip(m.name,m.position)), 10),
            self.create_subscription(TFMessage, '/ground_truth/cap_5', self.object_callback, qos_profile_sensor_data),
            self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile_sensor_data)]
        self.publishers_owned = {}
        for arm in self.models:
            self.publishers_owned[arm] = self.create_publisher(String, f'/gripper/{arm}/command', 10)
            self.subscriptions_owned.append(self.create_subscription(String, f'/gripper/{arm}/state',
                lambda m,a=arm: self.gripper.__setitem__(a,m.data), 10))
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.sequence = time.time_ns() // 1000
        self.report = {'status':'FAIL','object':'cap_5','mode':'Gazebo ground-truth demonstration',
                       'full_scene_acceptance':'NOT_RUN','events':[]}

    def clock_callback(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec / 1e9

    def object_callback(self, msg):
        if msg.transforms:
            p = msg.transforms[0].transform.translation
            self.object_position = np.array([p.x,p.y,p.z])
            self.object_received = time.monotonic()

    def record(self, stage, **extra):
        entry = {'stage':stage,'wall_time':time.time(),'sim_time':self.clock, **extra}
        self.report['events'].append(entry)
        print(stage, json.dumps(extra), flush=True)
        self.save()

    def save(self):
        (ROOT/'docs/evidence/one_object_transfer.json').write_text(json.dumps(self.report,indent=2)+'\n')

    def wait(self, predicate, timeout, reason):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if predicate():
                return
        raise RuntimeError(reason)

    def settle(self, seconds=.3):
        if self.clock is None:
            raise RuntimeError('No simulation clock')
        start = self.clock
        self.wait(lambda: self.clock >= start+seconds, 45, 'Simulation stopped advancing')

    def object_xyz(self):
        if self.object_position is None or time.monotonic()-self.object_received>3:
            raise RuntimeError('Object pose feedback missing or stale')
        return self.object_position.copy()

    def move(self, arm, xyz=None, home=False, stage='move'):
        model = self.models[arm]
        current = [self.joints[n] for n in model.names]
        q = self.homes[arm] if home else model.solve(xyz, current)
        goal = MoveGroup.Goal()
        goal.request.group_name = arm
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = 10.
        goal.request.num_planning_attempts = 3
        goal.request.max_velocity_scaling_factor = SETTINGS['contact_speed_scale'] if ('contact' in stage or 'place' in stage) else SETTINGS['transport_speed_scale']
        goal.request.max_acceleration_scaling_factor = SETTINGS['acceleration_scale']
        goal.request.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
            joint_name=name, position=value, tolerance_above=.00001, tolerance_below=.00001, weight=1.)
            for name,value in zip(model.names,q)])]
        goal.planning_options.plan_only = False
        self.record(stage+'_start',arm=arm,target=xyz)
        future = self.client.send_goal_async(goal)
        self.wait(future.done,20,'MoveIt goal submission timed out')
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('MoveIt rejected goal')
        completed = handle.get_result_async()
        try:
            self.wait(completed.done,240,'MoveIt motion timed out')
        except RuntimeError:
            cancel = handle.cancel_goal_async()
            self.wait(cancel.done,10,'Motion cancellation timed out')
            raise
        code = completed.result().result.error_code.val
        if code != 1:
            raise RuntimeError(f'{stage}: MoveIt failed with code {code}')
        self.settle()
        actual = model.forward([self.joints[n] for n in model.names])
        if xyz is not None and np.linalg.norm(actual[:3,3]-xyz)>.001:
            raise RuntimeError(f'{stage}: TCP error exceeds 1 mm')
        self.record(stage+'_done',tcp=actual[:3,3].tolist(),object=self.object_xyz().tolist())
        return actual[:3,3]

    def suction(self, arm, attach):
        self.sequence += 1
        suffix = 'attach,cap_5' if attach else 'release'
        command = f'{self.sequence},{suffix}'
        expected = f'{self.sequence},'+('attached:cap_5' if attach else 'released')
        self.publishers_owned[arm].publish(String(data=command))
        self.wait(lambda: self.gripper.get(arm,'').startswith(str(self.sequence)+','),
                  20,'Suction status timeout; stopping without assuming attachment')
        if self.gripper[arm]!=expected:
            raise RuntimeError('Suction rejected contact: '+self.gripper[arm])
        self.record('attach_ack' if attach else 'release_ack',arm=arm,response=expected)

    def verify_carried(self, arm, prior):
        position = self.object_xyz()
        model = self.models[arm]
        tcp = model.forward([self.joints[n] for n in model.names])[:3,3]
        if position[2]-prior[2]<.015 or np.linalg.norm(position-(tcp-[0,0,.005]))>.002:
            raise RuntimeError('Object did not physically follow the lifted suction cup')
        self.record('physical_lift_verified',arm=arm,object=position.tolist())

    def verify_place(self, target, stage):
        self.settle(.5)
        p = self.object_xyz()
        if np.linalg.norm(p[:2]-target[:2])>.002 or abs(p[2]-target[2])>.0015:
            raise RuntimeError(f'{stage}: object failed placement tolerance: {p}')
        self.record(stage,object=p.tolist(),expected=target)

    def run(self):
        self.homes = configured_homes(self.models)
        self.wait(lambda: len(self.joints)==8 and self.object_position is not None and self.clock is not None,
                  60,'Simulator feedback not ready')
        self.settle()
        if not self.client.wait_for_server(timeout_sec=20):
            raise RuntimeError('MoveIt unavailable')
        a = self.object_xyz()
        if np.linalg.norm(a-np.array([.14,.03,.019]))>.003:
            raise RuntimeError('cap_5 is not at the expected A source; no automatic teleport/reset')
        b = [0.,.02,.019]
        c = [-.1675,.02,CAP_CENTER_Z]
        self.record('start',A=a.tolist(),B=b,C=c)
        self.move('arm1',home=True,stage='home_A_ready')
        self.move('arm2',home=True,stage='home_C_ready')
        self.move('arm1',[a[0],a[1],a[2]+.035],stage='A_approach')
        self.move('arm1',[a[0],a[1],a[2]+.0053],stage='A_contact')
        self.suction('arm1',True)
        self.move('arm1',[a[0],a[1],a[2]+.035],stage='A_lift')
        self.verify_carried('arm1',a)
        self.move('arm1',[b[0],b[1],b[2]+.035],stage='B_arm1_approach')
        self.move('arm1',[b[0],b[1],b[2]+.0053],stage='B_arm1_place')
        self.suction('arm1',False)
        self.move('arm1',[b[0],b[1],b[2]+.035],stage='B_arm1_retreat')
        self.move('arm1',home=True,stage='arm1_home')
        self.verify_place(b,'B_placement_verified')
        actual_b = self.object_xyz()
        self.move('arm2',[actual_b[0],actual_b[1],actual_b[2]+.035],stage='B_arm2_approach')
        self.move('arm2',[actual_b[0],actual_b[1],actual_b[2]+.0053],stage='B_arm2_contact')
        self.suction('arm2',True)
        self.move('arm2',[actual_b[0],actual_b[1],actual_b[2]+.035],stage='B_arm2_lift')
        self.verify_carried('arm2',actual_b)
        self.move('arm2',[c[0],c[1],.054],stage='C_approach')
        self.move('arm2',[c[0],c[1],c[2]+.0053],stage='C_place')
        self.suction('arm2',False)
        self.move('arm2',[c[0],c[1],.054],stage='C_retreat')
        self.move('arm2',home=True,stage='arm2_home')
        self.verify_place(c,'C_placement_verified')
        self.report['status']='PASS'
        self.record('completed')


if __name__=='__main__':
    rclpy.init()
    demo = Demo()
    try:
        demo.run()
    except Exception as error:
        demo.report['error']=str(error)
        demo.record('stopped',reason=str(error))
        raise
    finally:
        demo.save()
        demo.destroy_node()
        rclpy.shutdown()
