"""Finite MoveIt demonstration on the dedicated Gazebo demo ROS domain only."""
import json
import os
from pathlib import Path
import time

os.environ['ROS_DOMAIN_ID'] = '42'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


def main():
    rclpy.init()
    node = Node('gazebo_visible_motion_demo')
    state, clocks = {}, []
    subscriptions = [
        node.create_subscription(JointState, '/joint_states', lambda m: state.update(zip(m.name, m.position)), 10),
        node.create_subscription(Clock, '/clock', lambda m: clocks.append(m.clock.sec + m.clock.nanosec / 1e9), 10),
    ]
    client = ActionClient(node, MoveGroup, '/move_action')
    report = {'scope': 'Gazebo domain 42, finite low-speed demonstration', 'status': 'FAIL', 'motions': []}
    try:
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if len(state) == 8 and len(clocks) > 2 and clocks[-1] > clocks[0]:
                break
        if len(state) != 8 or len(clocks) < 3 or clocks[-1] <= clocks[0]:
            raise RuntimeError('Eight joint feedback and advancing simulation clock required')
        if not client.wait_for_server(timeout_sec=20):
            raise RuntimeError('MoveIt action unavailable')
        baseline = state.copy()
        report['baseline'] = baseline
        for cycle in range(2):
            for label, offsets in [('turn_and_lift', (.20, -.12, .10, -.08)),
                                   ('turn_back', (-.15, -.06, .05, -.04)),
                                   ('return', (0., 0., 0., 0.))]:
                suffixes = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint', 'wrist_flex_joint']
                targets = {name: value + next(offset for suffix, offset in zip(suffixes, offsets) if name.endswith(suffix))
                           for name, value in baseline.items()}
                goal = MoveGroup.Goal()
                goal.request.group_name = 'both_arms'
                goal.request.start_state.is_diff = True
                goal.request.allowed_planning_time = 10.
                goal.request.num_planning_attempts = 3
                goal.request.max_velocity_scaling_factor = .15
                goal.request.max_acceleration_scaling_factor = .15
                goal.request.goal_constraints = [Constraints(joint_constraints=[
                    JointConstraint(joint_name=name, position=value, tolerance_above=.005,
                                    tolerance_below=.005, weight=1.) for name, value in targets.items()])]
                goal.planning_options.plan_only = False
                print(f'Cycle {cycle + 1}: {label}', flush=True)
                future = client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                if not future.done() or future.result() is None or not future.result().accepted:
                    raise RuntimeError('MoveIt did not accept demonstration goal')
                handle = future.result()
                completed = handle.get_result_async()
                rclpy.spin_until_future_complete(node, completed, timeout_sec=120)
                if not completed.done():
                    cancel = handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
                    raise RuntimeError('Motion timed out; cancellation requested')
                result = completed.result().result
                deadline = time.monotonic() + 1
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                error = max(abs(state[name] - value) for name, value in targets.items())
                report['motions'].append({'cycle': cycle + 1, 'motion': label,
                                          'moveit_code': result.error_code.val,
                                          'maximum_joint_error_rad': error, 'feedback': state.copy()})
                if result.error_code.val != 1 or error > .025:
                    raise RuntimeError(f'Motion failed: MoveIt {result.error_code.val}, feedback error {error}')
                print(f'Completed: maximum joint error {error:.5f} rad', flush=True)
        report['status'] = 'PASS'
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        destination = Path(__file__).resolve().parents[1] / 'docs/evidence/gazebo_visible_motion.json'
        destination.write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
