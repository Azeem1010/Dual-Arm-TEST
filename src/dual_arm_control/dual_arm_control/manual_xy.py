"""Arm1 manual XY service and CLI; planning has no actuator side effects."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from dual_arm_interfaces.srv import ManualXY
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

from .manual_config import check_plane, target_xy, validate_legacy_confirmation
from .moveit_client import MoveItClient, wait_future
from .trajectory import validate_trajectory
from .xy_calibration import JOINTS, Kinematics, calibration


class ManualServer(Node):
    def __init__(self):
        super().__init__('manual_xy_server')
        self.config = json.loads(self.declare_parameter('configuration', '').value)
        self.cfg = self.config['task']
        self.cancel = threading.Event()
        self.lock = threading.Lock()
        self.group = ReentrantCallbackGroup()
        self.motion = MoveItClient(self, self.cfg, self.group, self.cancel)
        self.read_client = self.create_client(Trigger, '/cm530/arm1/read', callback_group=self.group)
        self.stop_client = self.create_client(Trigger, '/cm530/arm1/stop', callback_group=self.group)
        self.model = Kinematics(self.config['urdf'])
        self.cal = calibration(self.config['bridge'])
        self.create_service(ManualXY, '/arm1/manual_xy', self.request, callback_group=self.group)
        self.create_service(Trigger, '/arm1/manual_xy/current', self.current, callback_group=self.group)
        self.create_service(Trigger, '/arm1/manual_xy/cancel', self.stop, callback_group=self.group)
        self.get_logger().info('Manual XY ready; default plan-only; no automatic task or torque enable')

    def measured(self):
        state = self.motion.state()
        q = list(state.joint_state.position)
        self.cal.to_raw(q)
        xyz, axis = self.model.forward(q)
        return state, q, xyz, axis

    def current(self, request, response):
        try:
            # A fresh READ also works after cancellation, when joint-state
            # polling has stopped due to the bridge's latched fault.
            if not self.read_client.wait_for_service(timeout_sec=self.cfg['service_timeout']):
                raise TimeoutError('CM530 READ service unavailable')
            result = wait_future(self.read_client.call_async(Trigger.Request()), self.cfg['service_timeout'])
            if not result.success:
                raise RuntimeError(result.message)
            raw = json.loads(result.message)
            q = self.cal.to_rad(raw)
            self.cal.to_raw(q)
            xyz, axis = self.model.forward(q)
            response.success = True
            response.message = json.dumps(dict(simulation=self.cfg['simulation'], frame='arm1_base',
                joint_radians=q, measured_raw=raw, estimated_raw=self.cal.to_raw(q), xyz=xyz, tool_z_axis=axis,
                pick_z=self.cfg['pick_z'], travel_z=self.cfg['travel_z']))
        except Exception as e:
            response.success, response.message = False, str(e)
        return response

    def stop(self, request, response):
        self.cancel.set()
        self.motion.stop()
        try:
            if not self.stop_client.wait_for_service(timeout_sec=self.cfg['service_timeout']):
                raise TimeoutError('Bridge stop service unavailable; physical stop not confirmed')
            stopped = wait_future(self.stop_client.call_async(Trigger.Request()), self.cfg['service_timeout'])
            response.success, response.message = stopped.success, stopped.message
        except Exception as e:
            response.success, response.message = False, str(e)
        return response

    def scene(self):
        self.motion.setup_scene()
        scene = PlanningScene(is_diff=True)
        for i, box in enumerate(self.config['manual']['obstacles']):
            obj = CollisionObject(id=f'manual_obstacle_{i}', operation=CollisionObject.ADD)
            obj.header.frame_id = 'arm1_base'
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, box['size'])))]
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x, pose.position.y, pose.position.z = map(float, box['center'])
            obj.primitive_poses = [pose]
            scene.world.collision_objects.append(obj)
        if scene.world.collision_objects:
            self.motion.apply_scene(scene)

    def validate_plan(self, trajectory, target, prepare):
        if trajectory.multi_dof_joint_trajectory.points:
            raise ValueError('Unexpected multi-DOF trajectory')
        times, points = validate_trajectory(trajectory.joint_trajectory, JOINTS, self.cal)
        if not prepare:
            # The bridge interpolates joint positions linearly. Check that actual
            # interpolation also remains on the plane, not only the IK waypoints.
            for a, b in zip(points, points[1:]):
                count = max(1, math.ceil(max(abs(x-y) for x, y in zip(a, b))/.005))
                for i in range(count+1):
                    q = [x+(y-x)*i/count for x, y in zip(a, b)]
                    check_plane(*self.model.forward(q), self.cfg['travel_z'])
        xyz, axis = self.model.forward(points[-1])
        check_plane(xyz, axis, self.cfg['travel_z'])
        if math.dist(xyz, target) > .001:
            raise ValueError('Planned endpoint differs from XY target by more than 1 mm')
        return times, points

    def request(self, request, response):
        if not self.lock.acquire(blocking=False):
            response.success, response.message = False, 'Manual operation already in progress'
            return response
        self.cancel.clear()
        execution_started = False
        try:
            target = target_xy(self.config, request.x, request.y)
            self.scene()
            start, initial, xyz, axis = self.measured()
            if not request.prepare:
                check_plane(xyz, axis, self.cfg['travel_z'])
            trajectory = (self.motion.plan_joints(self.motion.check_ik(target)) if request.prepare
                          else self.motion.plan_cartesian(target, start))
            times, points = self.validate_plan(trajectory, target, request.prepare)
            # Reject planning against a pose that changed while waiting for MoveIt.
            _, now, _, _ = self.measured()
            if max(abs(a-b) for a, b in zip(initial, now)) > .005 or max(
                    abs(a-b) for a, b in zip(initial, points[0])) > .005:
                raise RuntimeError('Measured start changed; plan again from a fresh stationary pose')
            record = dict(simulation=self.cfg['simulation'], frame='arm1_base', target_xyz=target,
                prepare=request.prepare, executed=False, start_xyz=xyz, start_joint_radians=initial,
                target_joint_radians=points[-1], target_raw=self.cal.to_raw(points[-1]),
                duration_s=times[-1], trajectory_points=len(points))
            if self.cancel.is_set():
                raise InterruptedError('Canceled before execution')
            if request.execute:
                if self.config['manual'].get('calibration', {}).get('method') == 'legacy_user_confirmed':
                    validate_legacy_confirmation(self.config['manual'], self.config['bridge'])
                execution_started = True
                self.motion.execute(trajectory)
                _, q, end, axis = self.measured()
                check_plane(end, axis, self.cfg['travel_z'])
                if math.dist(end, target) > .005:
                    raise RuntimeError('Measured FK endpoint differs from target by more than 5 mm')
                record.update(executed=True, measured_joint_radians=q, measured_raw=self.cal.to_raw(q),
                              measured_fk_xyz=end, completed_unix_s=time.time())
            response.success, response.message = True, json.dumps(record)
            self.get_logger().info(response.message)
        except Exception as e:
            response.success, response.message = False, str(e)
            if execution_started:
                stopped = self.stop(None, Trigger.Response())
                if not stopped.success:
                    response.message += '; ' + stopped.message
            self.get_logger().warning(str(e))
        finally:
            self.lock.release()
        return response


def server_main(args=None):
    rclpy.init(args=args)
    node = ManualServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.cancel.set()
        node.motion.stop()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='Arm1 XY in metres; default plans only. Z comes from launch config.')
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--prepare', action='store_true', help='Initial positioning: may change height/orientation along the path')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--current', action='store_true', help='Read measured joint state and calibrated FK')
    mode.add_argument('--cancel', action='store_true')
    parser.add_argument('--output', help='Save successful JSON result to this file')
    parser.add_argument('--timeout', type=float, default=120.0)
    parsed = parser.parse_args(remove_ros_args(args=args)[1:])
    if parsed.current or parsed.cancel:
        if parsed.x is not None or parsed.y is not None or parsed.execute or parsed.prepare:
            parser.error('--current/--cancel cannot be combined with motion options')
    elif parsed.x is None or parsed.y is None or not all(math.isfinite(v) for v in (parsed.x, parsed.y)):
        parser.error('Specify finite --x and --y (metres)')
    if not math.isfinite(parsed.timeout) or parsed.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    rclpy.init(args=args)
    node = Node('manual_xy_cli')
    service_type = Trigger if parsed.current or parsed.cancel else ManualXY
    name = '/arm1/manual_xy' + ('/current' if parsed.current else '/cancel' if parsed.cancel else '')
    client = node.create_client(service_type, name)
    sent = False
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Manual service unavailable; launch arm1.launch.py control:=manual first')
        request = Trigger.Request() if service_type is Trigger else ManualXY.Request(
            x=parsed.x, y=parsed.y, execute=parsed.execute, prepare=parsed.prepare)
        future = client.call_async(request)
        sent = True
        rclpy.spin_until_future_complete(node, future, timeout_sec=parsed.timeout)
        if not future.done():
            raise TimeoutError('Manual request timed out; outcome must be inspected')
        response = future.result()
        sent = False  # A received rejection must not cancel another client's active request.
        if not response.success:
            raise RuntimeError(response.message)
        if parsed.cancel:
            print(response.message)
        else:
            output = json.dumps(json.loads(response.message), indent=2, ensure_ascii=False)
            print(output)
            if parsed.output:
                Path(parsed.output).write_text(output + '\n')
    except (KeyboardInterrupt, Exception) as e:
        if sent and service_type is ManualXY:
            cancel = node.create_client(Trigger, '/arm1/manual_xy/cancel')
            if rclpy.ok() and cancel.wait_for_service(timeout_sec=1):
                rclpy.spin_until_future_complete(node, cancel.call_async(Trigger.Request()), timeout_sec=2)
        parser.exit(1, f'Manual XY failed: {e}\n')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
