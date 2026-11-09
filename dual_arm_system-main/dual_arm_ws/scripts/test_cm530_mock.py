"""End-to-end check of hardware.launch.py against the firmware-in-the-loop mock.

No board or motor is involved. PASS here proves protocol and software behavior
only; it is not hardware acceptance.
"""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'docs/evidence'
FIXTURES = ROOT / 'src/cm530_hardware/test/fixtures'
MOCK_CAL = str(FIXTURES / 'calibration_mock.yaml')
REFUSED_CAL = str(FIXTURES / 'calibration_not_authorized.yaml')
CONTROLLERS = str(FIXTURES / 'controllers_mock.yaml')
ARM1 = [f'arm1_arm_{name}' for name in ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint', 'wrist_flex_joint')]
ARM2 = [name.replace('arm1_', 'arm2_') for name in ARM1]
TARGET = [0.2, -0.15, 0.1, 0.25]
report = {'status': 'NOT_RUN', 'scope': 'Mock serial with v17 firmware parser; no hardware', 'phases': {}}


class Launch:
    def __init__(self, name, *args):
        self.log = EVIDENCE / f'cm530_mock_{name}.log'
        self.handle = self.log.open('w')
        command = ['ros2', 'launch', 'dual_arm_bringup', 'hardware.launch.py', 'use_mock_serial:=true', *args]
        self.process = subprocess.Popen(command, stdout=self.handle, stderr=subprocess.STDOUT, start_new_session=True)

    def text(self):
        return self.log.read_text(errors='replace')

    def wait_for(self, needle, timeout=120):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if needle in self.text():
                return True
            if self.process.poll() is not None:
                return needle in self.text()
            time.sleep(0.5)
        return False

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                pass
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.process.wait()
        self.handle.close()


def cli(*args, timeout=60):
    done = subprocess.run(['ros2', 'control', *args], capture_output=True, text=True, timeout=timeout)
    return done.returncode, done.stdout + done.stderr


def wait_cli(predicate, *args, timeout=120):
    deadline = time.monotonic() + timeout
    output = ''
    while time.monotonic() < deadline:
        try:
            _, output = cli(*args)
        except subprocess.TimeoutExpired:
            continue
        if predicate(output):
            return True, output
        time.sleep(1)
    return False, output


class Probe(Node):
    def __init__(self):
        super().__init__('cm530_mock_probe')
        self.states = {}
        self.stamp = 0.0
        self.create_subscription(JointState, '/joint_states', self.on_state, qos_profile_sensor_data)
        self.client = ActionClient(self, FollowJointTrajectory, '/arm1_controller/follow_joint_trajectory')

    def on_state(self, message):
        self.states = dict(zip(message.name, message.position))
        self.stamp = time.monotonic()

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_states(self, timeout=60):
        deadline = time.monotonic() + timeout
        self.states = {}
        while time.monotonic() < deadline and len(self.states) != 8:
            rclpy.spin_once(self, timeout_sec=0.1)
        return len(self.states) == 8

    def spin_until(self, future, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def move_arm1(self):
        if not self.client.wait_for_server(timeout_sec=30):
            return 'action server unavailable'
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM1
        goal.trajectory.points = [JointTrajectoryPoint(positions=TARGET, time_from_start=Duration(sec=3))]
        sent = self.client.send_goal_async(goal)
        if not self.spin_until(sent, 20) or not sent.result().accepted:
            return 'goal rejected'
        result = sent.result().get_result_async()
        if not self.spin_until(result, 30):
            return 'result timeout'
        return result.result().result.error_code


def phase(name, function):
    try:
        passed, detail = function()
    except Exception as error:
        passed, detail = False, f'{type(error).__name__}: {error}'
    report['phases'][name] = {'status': 'PASS' if passed else 'FAIL', 'detail': detail}
    print(name, report['phases'][name], flush=True)
    return passed


def read_only(probe):
    launch = Launch('read_only', f'calibration_file:={MOCK_CAL}')
    try:
        if not launch.wait_for('read-only'):
            return False, 'driver did not report read-only connection'
        if not probe.wait_states():
            return False, 'no joint states'
        centered = all(abs(probe.states[name]) < 1e-6 for name in ARM1 + ARM2)
        _, controllers = wait_cli(lambda text: 'joint_state_broadcaster' in text, 'list_controllers')
        no_arm_controllers = 'arm1_controller' not in controllers and 'arm2_controller' not in controllers
        return centered and no_arm_controllers, {'centered': centered, 'no_arm_controllers': no_arm_controllers}
    finally:
        launch.stop()


def refused_calibration(_):
    launch = Launch('refused', f'calibration_file:={REFUSED_CAL}', 'enable_motion:=true',
                    f'controllers_file:={CONTROLLERS}', 'use_moveit:=false')
    try:
        return launch.wait_for('Motion refused', 90), 'NOT_RUN calibration blocks motion'
    finally:
        launch.stop()


def template_refused(_):
    launch = Launch('template')
    try:
        return launch.wait_for('Configuration refused', 90), 'null calibration template blocks configure'
    finally:
        launch.stop()


def controllers_required(_):
    launch = Launch('no_controllers', f'calibration_file:={MOCK_CAL}', 'enable_motion:=true')
    try:
        return launch.wait_for('requires controllers_file', 60), 'motion without goal tolerances is refused'
    finally:
        launch.stop()


def motion_and_fault(probe):
    launch = Launch('motion', f'calibration_file:={MOCK_CAL}', 'enable_motion:=true',
                    f'controllers_file:={CONTROLLERS}', 'use_moveit:=false')
    try:
        ready, controllers = wait_cli(
            lambda text: 'arm1_controller' in text and 'arm2_controller' in text and text.count('inactive') >= 2,
            'list_controllers')
        if not ready:
            return False, f'controllers not ready: {controllers}'
        if 'motion authorized' not in launch.text():
            return False, 'driver did not report motion authorization'
        if 'TORQUE' in launch.text():
            return False, 'torque failure logged before activation'
        code, output = cli('set_hardware_component_state', 'DualArmSystem', 'active')
        if code != 0:
            return False, f'hardware activation failed: {output}'
        code, output = cli('switch_controllers', '--activate', 'arm1_controller', 'arm2_controller')
        if code != 0:
            return False, f'controller activation failed: {output}'
        if not probe.wait_states():
            return False, 'no joint states after activation'
        error_code = probe.move_arm1()
        probe.spin_for(1.0)
        reached = [abs(probe.states[name] - value) for name, value in zip(ARM1, TARGET)]
        arm2_still = max(abs(probe.states[name]) for name in ARM2)
        moved = error_code == FollowJointTrajectory.Result.SUCCESSFUL and max(reached) < 0.02 and arm2_still < 0.01
        detail = {'error_code': error_code, 'max_arm1_error_rad': max(reached), 'max_arm2_motion_rad': arm2_still}
        if not moved:
            return False, detail
        subprocess.run(['pkill', '-TERM', '-f', 'lib/cm530_hardware/cm530_mock_serial'], check=False)
        detail['fault_detected'] = launch.wait_for('CM-530 fault', 30)
        return detail['fault_detected'], detail
    finally:
        launch.stop()


def main():
    os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 100)
    prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    os.environ['AMENT_PREFIX_PATH'] = '/opt/ros/jazzy:' + prefix
    rclpy.init()
    probe = Probe()
    try:
        results = [phase(name, lambda f=function: f(probe)) for name, function in [
            ('read_only', read_only),
            ('refused_calibration', refused_calibration),
            ('template_refused', template_refused),
            ('controllers_required', controllers_required),
            ('motion_and_fault', motion_and_fault),
        ]]
        report['status'] = 'PASS' if all(results) else 'FAIL'
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        (EVIDENCE / 'cm530_mock_integration.json').write_text(json.dumps(report, indent=2) + '\n')
    print(report['status'], flush=True)
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
