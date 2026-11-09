"""Bounded runtime probe; owns and terminates only its launch process group."""
import argparse
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import time
import uuid
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, CameraInfo
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from tf2_msgs.msg import TFMessage

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('mode',choices=['mock','sim'])
parser.add_argument('--seconds',type=float,default=60)
parser.add_argument('--debug-shutdown',action='store_true')
parser.add_argument('--group',choices=['arm1','arm2','both_arms'],default='arm1')
parser.add_argument('--execute',action='store_true')
args = parser.parse_args()
os.environ['ROS_DOMAIN_ID'] = str(50+os.getpid()%100)
os.environ['GZ_PARTITION'] = 'dual_arm_acceptance_'+uuid.uuid4().hex
evidence = root/'docs/evidence'
log = (evidence/f'{args.mode}_launch.log').open('w')
cmd = ['ros2','launch','dual_arm_bringup', 'mock.launch.py' if args.mode=='mock' else 'vision_sim.launch.py']
if args.mode=='mock':
    cmd.append('rviz:=false')
process = subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init()
node = Node('runtime_acceptance_probe')
images = {}
camera_info = {}
ground_truth = {}
joint_state = {}
subs = [node.create_subscription(JointState,'/joint_states',lambda msg: joint_state.update(zip(msg.name,msg.position)),10)]
def ground_truth_callback(msg):
    for t in msg.transforms:
        ground_truth[t.child_frame_id] = {'parent':t.header.frame_id,'position':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z], 'quaternion':[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]}
subs.append(node.create_subscription(TFMessage,'/gazebo/ground_truth',ground_truth_callback,qos_profile_sensor_data))
def cap_pose_callback(name,msg):
    if msg.transforms:
        t=msg.transforms[0].transform
        ground_truth[name]={'position':[t.translation.x,t.translation.y,t.translation.z],'quaternion':[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]}
for number in range(1,9):
    name=f'cap_{number}'
    subs.append(node.create_subscription(TFMessage,'/ground_truth/'+name,lambda m,c=name:cap_pose_callback(c,m),qos_profile_sensor_data))
for camera in ('env','arm1','arm2'):
    subs.append(node.create_subscription(Image,f'/{camera}_cam/image_raw',lambda msg,c=camera:images.__setitem__(c,msg),qos_profile_sensor_data))
    subs.append(node.create_subscription(CameraInfo,f'/{camera}_cam/camera_info',lambda msg,c=camera:camera_info.__setitem__(c,msg),qos_profile_sensor_data))
for view in ('isometric','top','front','side'):
    subs.append(node.create_subscription(Image,f'/view_{view}/image_raw',lambda msg,c='view_'+view:images.__setitem__(c,msg),qos_profile_sensor_data))
results = {'mode':args.mode}
try:
    deadline = time.monotonic()+args.seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if len(joint_state)>=8 and (args.mode=='mock' or (len(images)==7 and len(camera_info)==3)):
            break
        if process.poll() is not None:
            break
    results['joint_feedback'] = 'PASS' if len(joint_state)==8 else 'FAIL'
    results['joints'] = joint_state.copy()
    results['images'] = {}
    results['camera_info'] = {name:{'k':list(msg.k),'d':list(msg.d),'width':msg.width,'height':msg.height,'frame_id':msg.header.frame_id,'distortion_model':msg.distortion_model} for name,msg in camera_info.items()}
    results['ground_truth'] = ground_truth.copy()
    for camera,msg in images.items():
        row = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
        img = row[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=='rgb8':
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        if msg.encoding not in ('rgb8','bgr8'):
            raise RuntimeError(f'Unexpected image encoding: {msg.encoding}')
        cv2.imwrite(str(evidence/f'{camera}_gazebo.png'),img)
        results['images'][camera] = {'frame':msg.header.frame_id,'size':[msg.width,msg.height]}
    results['three_camera_streams'] = ('PASS' if {'env','arm1','arm2'} <= images.keys() else 'FAIL') if args.mode=='sim' else 'NOT_RUN'
    results['workspace_views'] = ('PASS' if all('view_'+v in images for v in ('isometric','top','front','side')) else 'FAIL') if args.mode=='sim' else 'NOT_RUN'
    check = subprocess.run(['ros2','control','list_controllers'],capture_output=True,text=True,timeout=15)
    (evidence/f'{args.mode}_controllers.log').write_text(check.stdout+check.stderr)
    plain = re.sub(r'\x1b\[[0-9;]*m','',check.stdout)
    active = {line.split()[0] for line in plain.splitlines() if line.split() and line.split()[-1]=='active'}
    results['controllers'] = 'PASS' if check.returncode==0 and {'joint_state_broadcaster','arm1_controller','arm2_controller'} <= active else 'FAIL'
    client = ActionClient(node,MoveGroup,'/move_action')
    results['moveit_plan'] = 'NOT_RUN'
    if joint_state and client.wait_for_server(timeout_sec=10):
        goal = MoveGroup.Goal()
        goal.request.group_name = args.group
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = 5.
        goal.request.num_planning_attempts = 1
        goal.request.max_velocity_scaling_factor = .1
        goal.request.max_acceleration_scaling_factor = .1
        constraint = Constraints()
        for name,position in joint_state.items():
            if args.group=='both_arms' or name.startswith(args.group+'_'):
                constraint.joint_constraints.append(JointConstraint(joint_name=name,position=position+.05 if 'pan' in name else position,tolerance_above=.005,tolerance_below=.005,weight=1.))
        goal.request.goal_constraints = [constraint]
        goal.planning_options.plan_only = not args.execute
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if future.done() and future.result().accepted:
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(node,result_future,timeout_sec=90)
            if result_future.done():
                result = result_future.result().result
                results['moveit_error_code'] = result.error_code.val
                results['moveit_plan'] = 'PASS' if result.error_code.val==1 and len(result.planned_trajectory.joint_trajectory.points)>0 else 'FAIL'
                if args.execute:
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.1)
                    results['execution_feedback']='PASS' if result.error_code.val==1 and all(abs(joint_state.get(j.joint_name,float('inf'))-j.position)<.01 for j in constraint.joint_constraints) else 'FAIL'
                    results['final_joints']=joint_state.copy()
    results['planning_group']=args.group
    results['status'] = 'PASS' if results['joint_feedback']=='PASS' and results['controllers']=='PASS' and results['moveit_plan']=='PASS' and (args.mode=='mock' or results['three_camera_streams']=='PASS') else 'FAIL'
    if args.execute and results.get('execution_feedback')!='PASS':
        results['status']='FAIL'
finally:
    if args.mode=='sim' and process.poll() is None:
        stop=subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','stop: true'],capture_output=True,text=True,timeout=5)
        (evidence/'gazebo_stop.log').write_text(stop.stdout+stop.stderr)
        time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()
    debugger = None
    if args.debug_shutdown:
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():
                continue
            try:
                if os.getpgid(int(proc.name))==process.pid and 'move_group_lifetime' in (proc/'cmdline').read_text():
                    debug_log=(evidence/'moveit_shutdown_gdb.log').open('w')
                    debugger=subprocess.Popen(['gdb','-batch','-p',proc.name,'-ex','handle SIGINT nostop noprint pass','-ex','continue','-ex','thread apply all bt'],stdout=debug_log,stderr=subprocess.STDOUT)
                    time.sleep(2)
                    break
            except (OSError,ProcessLookupError):
                pass
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL)
            process.wait()
    log.close()
    if debugger is not None:
        try:
            debugger.wait(timeout=10)
        except subprocess.TimeoutExpired:
            debugger.terminate()
            debugger.wait()
        debug_log.close()
    # gz's Ruby wrapper can exit before its server child. Reap the entire
    # owned process group even when launch has already returned.
    try:
        os.killpg(process.pid,signal.SIGKILL)
    except ProcessLookupError:
        pass
    launch_text = (evidence/f'{args.mode}_launch.log').read_text()
    results['shutdown'] = 'FAIL' if any(s in launch_text for s in ('exit code -11','SIGSEGV','Segmentation fault','failed to terminate')) else 'PASS'
    results['mesh_resources'] = 'FAIL' if 'Unable to find file' in launch_text or 'Could not resolve file' in launch_text else 'PASS'
    if results['shutdown']=='FAIL' or results['mesh_resources']=='FAIL':
        results['status'] = 'FAIL'
    (evidence/f'{args.mode}_runtime.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
raise SystemExit(0 if results.get('status')=='PASS' else 1)
