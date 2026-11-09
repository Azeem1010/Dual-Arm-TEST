#!/usr/bin/env python3
"""Real ROS bridge → pyserial → PTY → unchanged v17 C parser. No physical I/O."""
import ctypes
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import threading
import time
import tty
import uuid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='166':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=166')
    root=Path(__file__).resolve().parents[1]
    firmware=root/'CM530_ROS_BRIDGE-main/17 ROS to CM530 ver. dual arm'
    out=Path('/tmp/dual-arm-ros2/serial_bridge')/uuid.uuid4().hex[:8]
    out.mkdir(parents=True)
    library=out/'peer.so'
    subprocess.run(['gcc','-shared','-fPIC','-Wall','-Wextra','-Werror','-I',str(firmware/'APP/inc'),
        str(firmware/'APP/src/bridge.c'),str(root/'src/dual_arm_control/test/firmware_peer.c'),'-o',str(library)],check=True)
    peer=ctypes.CDLL(str(library));peer.take_output.restype=ctypes.c_char_p
    peer.BridgeInit();peer.clear_output()
    master,slave=os.openpty();tty.setraw(slave)
    stop=threading.Event();inject=threading.Event();wire=[]
    def serve():
        line=bytearray()
        while not stop.is_set():
            if not select.select([master],[],[],.02)[0]:continue
            for b in os.read(master,256):
                line.append(b);peer.BridgeFeed(b)
                reply=peer.take_output()
                if reply:
                    if inject.is_set():reply=b'ERR,OVERFLOW\r\n';inject.clear()
                    wire.append(dict(tx=line.decode().strip(),rx=reply.decode().strip()));line.clear()
                    for byte in reply:os.write(master,bytes([byte]))
                    peer.clear_output()
    peer_thread=threading.Thread(target=serve);peer_thread.start()
    log=(out/'bridge.log').open('w')
    proc=subprocess.Popen(['ros2','run','dual_arm_control','cm530_bridge','--ros-args',
        '-p','mock_hardware:=false','-p','hardware_calibrated:=true','-p','serial_port:='+os.ttyname(slave),
        '-p','arms:=[arm1,arm2]','-p','startup_listen:=0.02','-p','command_timeout:=0.5',
        '-p','command_period:=0.1'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('serial_bridge_integration')
    health=[]
    node.create_subscription(String,'/cm530/status',lambda m:health.append(m.data),10)
    spin=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);spin.start()
    def wait(f,timeout=8):
        deadline=time.monotonic()+timeout
        while not f.done() and time.monotonic()<deadline:time.sleep(.01)
        if not f.done():raise TimeoutError('ROS response timeout')
        return f.result()
    def service(arm,op):
        client=node.create_client(Trigger,f'/cm530/{arm}/{op}')
        assert client.wait_for_service(timeout_sec=15),'Missing service '+op
        return wait(client.call_async(Trigger.Request()))
    def goal(arm):
        g=FollowJointTrajectory.Goal()
        g.trajectory.joint_names=[arm+'_'+j for j in ('arm_shoulder_pan_joint','arm_shoulder_lift_joint','arm_elbow_flex_joint','arm_wrist_flex_joint')]
        g.trajectory.points=[JointTrajectoryPoint(positions=[0.0]*4,time_from_start=Duration()),
            JointTrajectoryPoint(positions=[.03,-.03,.03,-.03],time_from_start=Duration(sec=1))]
        return g
    clients={a:ActionClient(node,FollowJointTrajectory,f'/{a}_controller/follow_joint_trajectory') for a in ('arm1','arm2')}
    passed=False
    try:
        assert service('arm1','get_home').success
        assert [x['tx'] for x in wire[:2]]==['VERSION','PING']
        assert not any(x['tx'].startswith(('AX,','HOME,','TORQUE,')) for x in wire), 'Unexpected startup motion'
        for a in clients:
            assert service(a,'enable').success
            assert clients[a].wait_for_server(timeout_sec=10)
        first=wait(clients['arm1'].send_goal_async(goal('arm1')));assert first.accepted
        duplicate=wait(clients['arm2'].send_goal_async(goal('arm2')));assert not duplicate.accepted
        outcome=wait(first.get_result_async());assert outcome.status==4,outcome.result.error_string
        second=wait(clients['arm2'].send_goal_async(goal('arm2')));assert second.accepted
        outcome=wait(second.get_result_async());assert outcome.status==4,outcome.result.error_string
        for a in clients:
            raw=json.loads(service(a,'read').message)
            assert raw==[518,506,518,506],raw
        # The action must have checked actual POS after its final AX before STOPPED.
        for a in clients:
            end=max(i for i,x in enumerate(wire) if x['tx'].startswith('AX,'+a+','))
            led=next(i for i in range(end+1,len(wire)) if wire[i]['tx']==f'LED,{a},STOPPED')
            assert sum(x['tx']==f'READ,{a}' for x in wire[end+1:led])>=3
        inject.set()
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not any('OVERFLOW' in h for h in health):time.sleep(.05)
        assert any('OVERFLOW' in h for h in health),health
        before=len(wire)
        assert not service('arm1','enable').success
        assert not service('arm1','hold').success
        assert len(wire)==before,'Sent command onto desynchronized wire'
        passed=True
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        stop.set();peer_thread.join(timeout=2);os.close(master);os.close(slave);log.close()
        rclpy.shutdown();spin.join(timeout=2);node.destroy_node()
        saved=root/'docs/validation';saved.mkdir(exist_ok=True)
        result=dict(passed=passed,transport='ROS action/service → pyserial PTY → unchanged firmware bridge.c',
                    limitations='AX12 transport is simulated; no physical motor or STM32 UART validation',wire=wire,health=health)
        (saved/'serial_bridge.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
        print(('PASS' if passed else 'FAIL')+': '+str(out))


if __name__=='__main__':main()
