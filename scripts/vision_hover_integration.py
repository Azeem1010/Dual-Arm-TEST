#!/usr/bin/env python3
"""Mock hardware/vision with real MoveIt: Home_A -> selected target hover -> stay.

Source the built workspace, set ROS_DOMAIN_ID=164, then run this script.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '164':
        raise SystemExit('Use ROS_DOMAIN_ID=164 for this mock-only test')
    out = Path('/tmp/dual-arm-ros2/hover') / uuid.uuid4().hex[:8]
    out.mkdir(parents=True)
    log = (out / 'launch.log').open('w')
    proc = subprocess.Popen(['ros2', 'launch', 'dual_arm_control', 'arm1.launch.py',
                             'mode:=demo', 'control:=vision_test'],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('hover_integration_test')
    history, latest, joints = [], {}, []
    lock = threading.Lock()
    def status(msg):
        value = json.loads(msg.data)
        with lock:
            latest.clear()
            latest.update(value)
            history.append(value['state'])
    def joint_state(msg):
        with lock:
            joints[:] = [msg]
    node.create_subscription(String, '/arm1/vision_test/status', status, 10)
    node.create_subscription(JointState, '/joint_states', joint_state, 10)
    start = node.create_client(Trigger, '/arm1/vision_test/start')
    fk = node.create_client(GetPositionFK, '/compute_fk')
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    def call(client, request, timeout=10.):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('Missing service: ' + client.srv_name)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline: raise TimeoutError(client.srv_name)
            time.sleep(.02)
        return future.result()
    try:
        assert start.wait_for_service(timeout_sec=40.), 'Bringup failed'
        time.sleep(3.)
        assert call(start, Trigger.Request()).success
        deadline = time.monotonic() + 180.
        while time.monotonic() < deadline:
            with lock: final = dict(latest)
            if final.get('state') in ('SUCCEEDED', 'FAULT'): break
            if proc.poll() is not None: raise RuntimeError('Bringup exited')
            time.sleep(.05)
        else: raise TimeoutError('Hover did not finish')
        assert final['state'] == 'SUCCEEDED', final
        assert final['target_above_a'] == [.15, .04, .04], final
        with lock:
            measured = joints[0]
            states = [s for s in history if s != 'IDLE']
        assert states == ['MOVING_HOME_A', 'WAITING_FOR_VISION', 'MOVING_ABOVE_A', 'SUCCEEDED'], states
        result = call(fk, GetPositionFK.Request(
            header=Header(frame_id='arm1_base'), fk_link_names=['arm1_suction_tcp'],
            robot_state=RobotState(joint_state=measured, is_diff=True)))
        assert result.error_code.val == 1, result.error_code
        p = result.pose_stamped[0].pose.position
        xyz = [p.x, p.y, p.z]
        assert max(abs(a-b) for a,b in zip(xyz, final['target_above_a'])) < .005, xyz
        time.sleep(2.)
        with lock:
            after = dict(zip(joints[0].name, joints[0].position))
            assert latest['state'] == 'SUCCEEDED'
        assert max(abs(after[n]-q) for n,q in zip(measured.name, measured.position)) < .001
        report = dict(passed=True, states=states, final=final, measured_tcp=xyz,
                      stayed_at_hover=True, real_moveit=True, hardware='mock', vision='mock')
        (out / 'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(dict(passed=True, report=str(out / 'result.json'))), flush=True)
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try: proc.wait(timeout=12.)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        rclpy.shutdown()
        spinner.join(timeout=3.)
        node.destroy_node()
        log.close()


if __name__ == '__main__':
    main()
