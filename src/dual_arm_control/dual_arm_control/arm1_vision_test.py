"""Home_A -> selected camera target hover; remain at the hover pose."""
import json
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_interfaces.srv import GetPickTarget

from .coordinator import DEFAULTS, validate_config
from .moveit_client import MoveItClient
from .vision_target import target_from_pick


class Arm1VisionTest(Node):
    def __init__(self):
        super().__init__('arm1_vision_test')
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        validate_config(self.cfg)
        self.group = ReentrantCallbackGroup()
        self.cancel = threading.Event()
        self.lock = threading.Lock()
        self.busy = False
        self.motion = MoveItClient(self, self.cfg, self.group, self.cancel)
        self.pub = self.create_publisher(String, '/arm1/vision_test/status', 10)
        self.pick_client = self.create_client(GetPickTarget, self.cfg['pick_service'],
                                              callback_group=self.group)
        self.create_service(Trigger, '/arm1/vision_test/start', self.start, callback_group=self.group)
        self.create_service(Trigger, '/arm1/vision_test/cancel', self.stop, callback_group=self.group)
        self.report('IDLE')

    def report(self, state, message='', target=None):
        self.pub.publish(String(data=json.dumps({'state': state, 'message': message,
                                                   'target_above_a': target}, ensure_ascii=False)))
        self.get_logger().info(state + (': ' + message if message else ''))

    def start(self, request, response):
        with self.lock:
            if self.busy:
                response.success, response.message = False, 'Motion test already running'
                return response
            self.busy = True
            self.cancel.clear()
        threading.Thread(target=self.run_test, daemon=True).start()
        response.success, response.message = True, 'Accepted; observe /arm1/vision_test/status'
        return response

    def stop(self, request, response):
        with self.lock:
            response.success = self.busy
            if self.busy:
                self.cancel.set()
                self.motion.stop()
        response.message = 'Cancellation requested' if response.success else 'No active test'
        return response

    def run_test(self):
        try:
            self.report('MOVING_HOME_A')
            self.motion.setup_scene()
            self.motion.move_joints(self.cfg['home_joints'])
            if self.cancel.wait(self.cfg['home_settle']):
                raise InterruptedError('Canceled at Home_A')
            self.report('WAITING_FOR_VISION')
            result = self.motion.service(self.pick_client, GetPickTarget.Request(
                target_rank=0, peek_only=True))
            if self.cancel.is_set():
                raise InterruptedError('Motion test canceled during target query')
            target = target_from_pick(result, self.cfg)
            self.report('MOVING_ABOVE_A', target=target)
            self.motion.move_pose(target)
            if self.cancel.is_set():
                raise InterruptedError('Motion test canceled during hover motion')
            self.report('SUCCEEDED', 'At selected object hover pose; staying here', target)
        except Exception as exc:
            self.motion.stop()
            self.report('FAULT', str(exc))
        finally:
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = Arm1VisionTest()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel.set()
        node.motion.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
