#!/usr/bin/env python3
"""Full shared MoveIt + two arm workflows + simulated wire-independent actuator."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from dual_arm_interfaces.srv import ConfirmStep


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenario',choices=['full','eight','cancel','release_failure','safety_loss','existing_b'],default='full')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '165':
        raise SystemExit('Use ROS_DOMAIN_ID=165; this test only launches simulated devices')
    root=Path(__file__).resolve().parents[1]
    out=Path('/tmp/dual-arm-ros2/dual_integration')/(args.scenario+'-'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    config=yaml.safe_load((root/'src/dual_arm_control/config/dual_arm_demo.yaml').read_text())
    config['cm530_bridge']['ros__parameters']['mock_time_scale']=0.15
    config['dual_coordinator']['ros__parameters']['arm1.home_settle']=0.1
    config['dual_coordinator']['ros__parameters']['arm2.home_settle']=0.1
    if args.scenario=='safety_loss':
        config['dual_mock_vision']['ros__parameters']['freeze_safety_on_grasp']=True
    if args.scenario=='existing_b':
        config['dual_mock_vision']['ros__parameters']['initial_b']=True
    (out/'config.yaml').write_text(yaml.safe_dump(config))
    log=(out/'launch.log').open('w')
    proc=subprocess.Popen(['ros2','launch','dual_arm_control','dual_arm.launch.py','mode:=demo',
        'auto_confirm_demo:=false',f'config:={out}/config.yaml',f'journal:={out}/journal.json',
        'cycles:='+('8' if args.scenario=='eight' else '1')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(); node=Node('dual_integration_test')
    state={}; history=[]; tokens=set(); lock=threading.Lock()
    def receive(msg):
        v=json.loads(msg.data)
        with lock:
            state.clear(); state.update(v)
            if not history or history[-1]!=v['state']: history.append(v['state'])
    node.create_subscription(String,'/dual_arm/status',receive,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clients={name:node.create_client(kind,'/dual_arm/'+name) for name,kind in
             [('start',Trigger),('cancel',Trigger),('confirm',ConfirmStep)]}
    thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True); thread.start()
    def call(name,req):
        c=clients[name]
        if not c.wait_for_service(timeout_sec=10): raise RuntimeError('Missing '+name)
        f=c.call_async(req); deadline=time.monotonic()+10
        while not f.done() and time.monotonic()<deadline: time.sleep(.01)
        if not f.done(): raise TimeoutError(name)
        return f.result()
    ok=False
    try:
        if not clients['start'].wait_for_service(timeout_sec=45): raise RuntimeError('Bringup failed')
        time.sleep(4)
        reply=call('start',Trigger.Request()); assert reply.success,reply.message
        assert not call('start',Trigger.Request()).success, 'Duplicate start accepted'
        deadline=time.monotonic()+(800 if args.scenario=='eight' else 180)
        while time.monotonic()<deadline:
            if proc.poll() is not None: raise RuntimeError('Launch exited')
            with lock: v=dict(state)
            if v.get('state') in ('FAULT','SUCCEEDED','FULL') and not v.get('busy',True): break
            token,op=v.get('confirmation_token'),v.get('confirmation_operation')
            if token and token not in tokens and op!='recovery':
                assert not call('confirm',ConfirmStep.Request(token='stale',success=True)).accepted
                if args.scenario=='cancel' and v.get('active_arm')=='arm2' and op=='grasp':
                    assert call('cancel',Trigger.Request()).success
                elif args.scenario=='safety_loss' and v.get('active_arm')=='arm2' and op=='grasp':
                    pass  # Wait for missing heartbeat to cancel while B is reserved.
                else:
                    failed=args.scenario=='release_failure' and v.get('active_arm')=='arm2' and op=='release'
                    assert call('confirm',ConfirmStep.Request(token=token,success=not failed,message='simulated operator')).accepted
                    assert not call('confirm',ConfirmStep.Request(token=token,success=True)).accepted
                tokens.add(token)
            time.sleep(.05)
        else: raise TimeoutError('Integrated flow timeout')
        if args.scenario in ('cancel','release_failure','safety_loss'):
            assert v['state']=='FAULT',v
            assert v['holding']=='unknown',v
            if args.scenario!='release_failure': assert v['b_owner']=='arm2',v
            assert not call('start',Trigger.Request()).success
        else:
            assert v['state']==('FULL' if args.scenario=='eight' else 'SUCCEEDED'),v
            assert sum(v['occupied'])==(8 if args.scenario=='eight' else 1),v
            assert not v['b_locked'] and v['holding']=='empty',v
            if args.scenario=='existing_b': assert not any(x.startswith('ARM1:') for x in history)
            else: assert 'ARM1:SUCCEEDED' in history
            assert 'ARM2:B_EMPTY' in history
            if args.scenario=='eight': assert not call('start',Trigger.Request()).success
        ok=True
    finally:
        result = json.dumps(dict(passed=ok,scenario=args.scenario,state=state,history=history),indent=2)
        (out/'result.json').write_text(result)
        saved = root/'docs/validation'
        saved.mkdir(exist_ok=True)
        (saved/('dual_'+args.scenario+'.json')).write_text(result)
        os.killpg(proc.pid,signal.SIGINT)
        try: proc.wait(timeout=12)
        except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL); proc.wait()
        log.close(); rclpy.shutdown(); thread.join(timeout=2); node.destroy_node()
        print(('PASS' if ok else 'FAIL')+': '+str(out),flush=True)


if __name__=='__main__': main()
