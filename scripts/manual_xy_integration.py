#!/usr/bin/env python3
"""Real MoveIt with mock CM530 only, isolated domain 164. No hardware commands."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from dual_arm_interfaces.srv import ManualXY
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '164':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=164')
    out = Path('/tmp/dual-arm-ros2/manual_xy_validation')
    out.mkdir(parents=True, exist_ok=True)
    log = (out/'launch.log').open('w')
    proc = subprocess.Popen(['ros2', 'launch', 'dual_arm_control', 'arm1.launch.py',
                             'mode:=demo', 'control:=manual'], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    rclpy.init()
    node = Node('manual_xy_integration')
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    xy = node.create_client(ManualXY, '/arm1/manual_xy')
    current = node.create_client(Trigger, '/arm1/manual_xy/current')
    scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    cancel = node.create_client(Trigger, '/arm1/manual_xy/cancel')
    raw_read = node.create_client(Trigger, '/cm530/arm1/read')
    results = []

    def call(client, req, timeout=90):
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError('Service missing: ' + client.srv_name)
        f = client.call_async(req)
        deadline = time.monotonic() + timeout
        while not f.done() and time.monotonic() < deadline:
            time.sleep(.02)
        if not f.done():
            raise TimeoutError(client.srv_name)
        return f.result()

    def move(x=.15, y=.04, execute=False, prepare=False, expected=True):
        r = call(xy, ManualXY.Request(x=x, y=y, execute=execute, prepare=prepare))
        results.append(dict(x=x, y=y, execute=execute, prepare=prepare, success=r.success, message=r.message))
        assert r.success is expected, r.message
        return json.loads(r.message) if r.success else r.message

    def read():
        r = call(current, Trigger.Request())
        assert r.success, r.message
        return json.loads(r.message)

    success = False
    try:
        assert xy.wait_for_service(timeout_sec=40)
        assert scene.wait_for_service(timeout_sec=30)
        before = read()
        cli = subprocess.run(['ros2', 'run', 'dual_arm_control', 'manual_xy', '--current'],
                             capture_output=True, text=True, timeout=20)
        assert cli.returncode == 0, cli.stderr
        assert json.loads(cli.stdout)['simulation'] is True
        move(prepare=True)
        after = read()
        assert before['estimated_raw'] == after['estimated_raw'], 'Plan-only caused motion'
        move(expected=False)  # initial pose not yet on the downward travel plane
        move(x=10.0, expected=False)
        move(x=.20, y=.12, prepare=True, expected=False)  # inside XY bounds, beyond downward IK reach
        move(prepare=True, execute=True)
        for x, y in ((.15, .04), (.155, .04), (.15, .04), (.15, .045), (.15, .04)):
            r = move(x=x, y=y, execute=True)
            assert math.dist(r['measured_fk_xyz'], [x, y, .04]) <= .005
        # A real MoveIt planning-scene obstacle blocks the target.
        obj = CollisionObject(id='integration_blocker', operation=CollisionObject.ADD)
        obj.header.frame_id = 'arm1_base'
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.015, .015, .08])]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = .19, .04, .04
        obj.primitive_poses = [pose]
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [obj]
        assert call(scene, ApplyPlanningScene.Request(scene=ps)).success
        move(x=.19, expected=False)
        # Verify firmware/action path remains available after a planning rejection.
        move(x=.155, execute=True)
        move(execute=True)
        # Concurrent request is rejected without interfering with the first.
        pending = xy.call_async(ManualXY.Request(x=.155, y=.04, execute=True))
        time.sleep(.5)
        busy = move(expected=False)
        assert 'in progress' in busy
        assert call(cancel, Trigger.Request()).success
        deadline = time.monotonic() + 15
        while not pending.done() and time.monotonic() < deadline:
            time.sleep(.02)
        assert pending.done() and not pending.result().success
        results.append(dict(cancellation_verified=True, message=pending.result().message))
        deadline = time.monotonic() + 5
        held_response = call(raw_read, Trigger.Request())
        while not held_response.success and time.monotonic() < deadline:
            time.sleep(.1)
            held_response = call(raw_read, Trigger.Request())
        assert held_response.success
        held = json.loads(held_response.message)
        time.sleep(.5)
        held_response = call(raw_read, Trigger.Request())
        assert held_response.success and json.loads(held_response.message) == held, 'Motion continued after cancellation'
        assert read()['measured_raw'] == held  # diagnostic reads work after cancellation/fault
        success = True
    finally:
        (out/'results.json').write_text(json.dumps(dict(success=success, simulation=True, results=results), indent=2))
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        rclpy.shutdown()
        spinner.join(timeout=2)
        node.destroy_node()
        log.close()
    print(out/'results.json')


if __name__ == '__main__':
    main()
