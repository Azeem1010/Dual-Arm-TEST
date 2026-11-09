"""Three-object Gazebo demonstration with overlapping A pickup / B pickup.

Domain 43 only. Uses simulator poses and original CAD. No camera calibration
claim. One combined MoveIt goal controls concurrent arm movements, avoiding
two competing move_action goals. B is cleared before the next deposit.
"""
import os
import time
import json
import argparse
import numpy as np
from demo_transfer_one import Demo, ROOT, rclpy, MoveGroup, Constraints, JointConstraint
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from runtime_settings import SETTINGS, CAP_CENTER_Z, CONTACT_Z, configured_homes

os.environ['ROS_DOMAIN_ID'] = '43'


class Pipeline(Demo):
    def __init__(self, count=3):
        self.poses = {}
        self.count = count
        super().__init__()
        self.held = {'arm1':None, 'arm2':None}
        self.b_object = None
        self.report = {'status':'FAIL','mode':'Gazebo ground truth, three-object pipeline',
                       'transport_speed_scale':SETTINGS['transport_speed_scale'],'contact_speed_scale':SETTINGS['contact_speed_scale'],
                       'full_scene_acceptance':'NOT_RUN','events':[]}
        for name in ['cap_4','cap_6']:
            self.subscriptions_owned.append(self.create_subscription(TFMessage, '/ground_truth/'+name,
                lambda m,n=name: self.observe(n,m), qos_profile_sensor_data))

    def object_callback(self, msg):
        super().object_callback(msg)
        self.observe('cap_5',msg)

    def observe(self,name,msg):
        if msg.transforms:
            p=msg.transforms[0].transform.translation
            self.poses[name]=(np.array([p.x,p.y,p.z]),time.monotonic())

    def position(self,name):
        if name not in self.poses or time.monotonic()-self.poses[name][1]>3:
            raise RuntimeError('Missing or stale object feedback: '+name)
        return self.poses[name][0].copy()

    def save(self):
        (ROOT/'docs/evidence/transfer_pipeline.json').write_text(json.dumps(self.report,indent=2)+'\n')

    def move_pair(self, targets, stage, contact=False):
        # None means configured home_A/home_C; never silently substitute CAD zero.
        desired={}
        for arm,xyz in targets.items():
            model=self.models[arm]
            q=self.homes[arm] if xyz is None else model.solve(xyz,[self.joints[n] for n in model.names])
            desired.update(zip(model.names,q))
        goal=MoveGroup.Goal()
        goal.request.group_name='both_arms' if len(targets)==2 else next(iter(targets))
        goal.request.start_state.is_diff=True
        goal.request.allowed_planning_time=10.
        goal.request.num_planning_attempts=3
        goal.request.max_velocity_scaling_factor=SETTINGS['contact_speed_scale'] if contact else SETTINGS['transport_speed_scale']
        goal.request.max_acceleration_scaling_factor=SETTINGS['acceleration_scale']
        goal.request.goal_constraints=[Constraints(joint_constraints=[JointConstraint(
            joint_name=n,position=q,tolerance_above=.00001,tolerance_below=.00001,weight=1.)
            for n,q in desired.items()])]
        goal.planning_options.plan_only=False
        self.record(stage+'_start',targets=targets,simultaneous=len(targets)==2)
        future=self.client.send_goal_async(goal)
        self.wait(future.done,20,'Goal submission timeout')
        handle=future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(stage+': goal rejected')
        result=handle.get_result_async()
        try:
            self.wait(result.done,240,'Motion timeout')
        except BaseException:
            cancel=handle.cancel_goal_async()
            self.wait(cancel.done,10,'Motion cancellation timeout')
            raise
        if result.result().result.error_code.val!=1:
            raise RuntimeError(stage+': MoveIt failure '+str(result.result().result.error_code.val))
        self.settle(.2)
        feedback={}
        for arm,xyz in targets.items():
            model=self.models[arm]
            actual=model.forward([self.joints[n] for n in model.names])
            error=0. if xyz is None else float(np.linalg.norm(actual[:3,3]-xyz))
            feedback[arm]={'tcp':actual[:3,3].tolist(),'error_m':error}
            if error>.001:
                self.record(stage+'_feedback_failure',feedback=feedback)
                raise RuntimeError(stage+': TCP error exceeds 1 mm')
        self.record(stage+'_done',feedback=feedback)

    def grip(self,arm,name,attach):
        if attach and (self.held[arm] is not None or name in self.held.values()):
            raise RuntimeError('Duplicate object ownership')
        self.sequence+=1
        suffix='attach,'+name if attach else 'release'
        expected=f'{self.sequence},'+('attached:'+name if attach else 'released')
        self.publishers_owned[arm].publish(String(data=f'{self.sequence},{suffix}'))
        self.wait(lambda:self.gripper.get(arm,'').startswith(str(self.sequence)+','),20,'Suction reply timeout')
        if self.gripper[arm]!=expected:
            raise RuntimeError(self.gripper[arm])
        self.held[arm]=name if attach else None
        self.record('attached' if attach else 'released',arm=arm,object=name)

    def lifted(self,arm,name,prior):
        position=self.position(name)
        model=self.models[arm]
        tcp=model.forward([self.joints[n] for n in model.names])[:3,3]
        if position[2]-prior[2]<.015 or np.linalg.norm(position-(tcp-[0,0,.005]))>.002:
            raise RuntimeError('Physical lift/follow check failed: '+name)
        self.record('physical_lift_verified',arm=arm,object=name,position=position.tolist())

    def placed(self,name,target,label):
        self.settle(.5)
        position=self.position(name)
        if np.linalg.norm(position[:2]-target[:2])>.002 or abs(position[2]-target[2])>.0015:
            self.record('placement_failure',object=name,actual=position.tolist(),target=target)
            raise RuntimeError(label+': placement outside tolerance')
        self.record(label,object=name,position=position.tolist())

    def deposit_b(self,name):
        if self.b_object is not None:
            raise RuntimeError('B is occupied; deposit denied')
        self.move_pair({'arm1':[0.,.02,.054]},'B_deposit_approach')
        self.move_pair({'arm1':[0.,.02,CONTACT_Z]},'B_deposit_contact',contact=True)
        self.grip('arm1',name,False)
        self.b_object=name
        self.move_pair({'arm1':[0.,.02,.054]},'B_arm1_retreat')
        self.move_pair({'arm1':None},'B_arm1_clear')
        self.placed(name,[0.,.02,CAP_CENTER_Z],'B_placement_verified')

    def run(self):
        self.homes=configured_homes(self.models)
        names=['cap_5','cap_6','cap_4'][:self.count]
        destinations=[[-.1675,.02,CAP_CENTER_Z],[-.1325,.02,CAP_CENTER_Z],[-.1675,.055,CAP_CENTER_Z]][:self.count]
        self.wait(lambda:len(self.joints)==8 and all(n in self.poses for n in names) and self.clock is not None,
                  90,'Simulator not ready')
        self.settle()
        if not self.client.wait_for_server(timeout_sec=20):
            raise RuntimeError('MoveIt unavailable')
        # Joint feedback can precede trajectory-controller activation at startup.
        self.controller_clients=[]
        for arm in ('arm1','arm2'):
            controller=ActionClient(self,FollowJointTrajectory,f'/{arm}_controller/follow_joint_trajectory')
            self.controller_clients.append(controller)
            if not controller.wait_for_server(timeout_sec=90):
                raise RuntimeError(arm+' trajectory controller not ready; no motion submitted')
        sources={n:self.position(n) for n in names}
        expected={'cap_5':[.14,.03,.019],'cap_6':[.2,.03,.019],'cap_4':[.2,-.035,.019]}
        for n in names:
            if np.linalg.norm(sources[n]-expected[n])>.003:
                raise RuntimeError('Source object moved; fresh scene required: '+n)
        # All destination release/approach orientations must be solvable before moving.
        for c in destinations:
            for z in [.054,.0253]:
                self.models['arm2'].solve([c[0],c[1],z])
        self.record('pipeline_start',objects=names,destinations=destinations)
        self.move_pair({'arm1':None,'arm2':None},'home_A_home_C_ready')
        first=names[0]
        a=sources[first]
        self.move_pair({'arm1':[float(a[0]),float(a[1]),.054]},'first_A_approach')
        self.move_pair({'arm1':[float(a[0]),float(a[1]),CONTACT_Z]},'first_A_contact',contact=True)
        self.grip('arm1',first,True)
        self.move_pair({'arm1':[float(a[0]),float(a[1]),.054]},'first_A_lift')
        self.lifted('arm1',first,a)
        self.deposit_b(first)
        for index,current in enumerate(names):
            following=names[index+1] if index+1<len(names) else None
            b=self.position(current)
            if self.b_object!=current:
                raise RuntimeError('B ownership mismatch')
            next_a=self.position(following) if following else None
            for stage,z in [('approach',.054),('contact',CONTACT_Z)]:
                targets={'arm2':[float(b[0]),float(b[1]),z]}
                if following:
                    targets['arm1']=[float(next_a[0]),float(next_a[1]),z]
                self.move_pair(targets,'parallel_A_B_'+stage,contact=stage=='contact')
            self.grip('arm2',current,True)
            if following:
                self.grip('arm1',following,True)
            targets={'arm2':[float(b[0]),float(b[1]),.054]}
            if following:
                targets['arm1']=[float(next_a[0]),float(next_a[1]),.054]
            self.move_pair(targets,'parallel_A_B_lift')
            self.lifted('arm2',current,b)
            if following:
                self.lifted('arm1',following,next_a)
            c=destinations[index]
            self.move_pair({'arm2':[c[0],c[1],.054]},'C_carry')
            # B ownership clears only after the real object has left the shared region.
            if self.position(current)[0]>-.10:
                raise RuntimeError('Object has not cleared B')
            self.b_object=None
            self.record('B_clear_verified',object=current)
            targets={'arm2':[c[0],c[1],.0253]}
            if following:
                targets['arm1']=[0.,.02,.054]
            self.move_pair(targets,'parallel_C_release_B_approach',contact=True)
            # Release above the flush white pad, then verify actual supported cap height.
            self.grip('arm2',current,False)
            self.move_pair({'arm2':[c[0],c[1],.054]},'C_retreat')
            self.placed(current,c,'C_placement_verified')
            self.move_pair({'arm2':None},'arm2_clear')
            if following:
                self.deposit_b(following)
        self.move_pair({'arm1':None,'arm2':None},'both_home')
        self.report['status']='PASS'
        self.record('pipeline_completed')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--count',type=int,choices=[1,2,3],default=3)
    args=parser.parse_args()
    rclpy.init()
    demo=Pipeline(args.count)
    try:
        demo.run()
    except BaseException as error:
        demo.report['error']=str(error)
        demo.record('stopped',reason=str(error))
        raise
    finally:
        demo.save()
        demo.destroy_node()
        rclpy.shutdown()
