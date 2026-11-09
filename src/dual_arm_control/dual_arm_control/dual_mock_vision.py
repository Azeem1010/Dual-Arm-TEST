"""Shared synthetic A/B/C scene for integrated demo; never load in hardware."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from vision_interfaces.srv import CaptureRanking, GetPickTarget, GetZoneState, GetGridState, VerifyPlace
from .arm2_mock_vision import Arm2MockVision


class DualMockVision(Node):
    def __init__(self):
        super().__init__('dual_mock_vision')
        self.xyz = self.declare_parameter('pick_xyz', [0.15, 0.04, 0.02]).value
        self.c_poses = self.declare_parameter('c_poses', [0.0]*56).value
        self.supply = self.declare_parameter('supply', 8).value
        self.b = self.declare_parameter('initial_b', False).value
        self.occupied = list(self.declare_parameter('initial_occupied', [False]*8).value)
        self.count, self.clear = sum(self.occupied), True
        self.fail_grid = self.grid_mismatch = False
        self.fail_count = self.declare_parameter('fail_count', False).value
        self.freeze_on_grasp = self.declare_parameter('freeze_safety_on_grasp', False).value
        self.frozen = False
        self.previous = {}
        self.create_service(GetPickTarget, '/vision/arm1/get_pick_target', self.pick)
        self.create_service(CaptureRanking, '/vision/env/capture_ranking', self.capture)
        self.create_service(GetZoneState, '/vision/env/get_zone_state', self.zone)
        self.create_service(GetGridState, '/vision/arm2/get_grid_state', self.grid)
        self.create_service(VerifyPlace, '/vision/arm2/verify_place', self.verify)
        self.create_service(Trigger, '/dual_mock/clear_table', self.clear_table)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for arm in ('arm1', 'arm2'):
            self.create_subscription(String, f'/{arm}/status', lambda m, a=arm: self.status(a, m), qos)
        self.stop_pub = self.create_publisher(Bool, '/safety/estop', 10)
        self.create_timer(0.1, self.heartbeat)

    grid = Arm2MockVision.grid
    verify = Arm2MockVision.verify

    def heartbeat(self):
        if not self.frozen:
            self.stop_pub.publish(Bool(data=False))

    def pick(self, req, res):
        res.success, res.message, res.rank = self.supply > 0, 'synthetic target', 1
        res.x, res.y, res.z = self.xyz
        return res

    def capture(self, req, res):
        res.success = self.supply == 8 and not self.b and self.count == 0
        res.message = 'synthetic ranking ready' if res.success else 'synthetic scene is not a fresh eight-object batch'
        res.count = self.supply
        res.low_confidence = False
        return res

    def zone(self, req, res):
        res.success, res.message = True, 'synthetic shared A/B/C scene'
        res.a_has_object, res.b_has_object, res.b_arm_clear = self.supply > 0, self.b, self.clear
        res.c_object_count = self.count
        return res

    def status(self, arm, msg):
        value = json.loads(msg.data)
        state = value['state']
        if self.previous.get(arm) == state:
            return
        self.previous[arm] = state
        if arm == 'arm1':
            if state == 'ARM1_STATE1_ATTACH':
                self.supply -= 1
            if state == 'ARM1_STATE2_B_RESERVED':
                self.clear = False
            if state == 'ARM1_STATE2_DETACH':
                self.b = True
            if state == 'ARM1_STATE3_VERIFY_B':
                self.clear = True
        else:
            if state == 'B_RESERVED':
                self.clear = False
            if state == 'GRASP_B' and self.freeze_on_grasp:
                self.frozen = True
            if state == 'ATTACH':
                self.b = False
            if state == 'VERIFY_B_EMPTY':
                self.clear = True
            if state == 'DETACH' and not self.fail_count:
                self.occupied[value['slot'] - 1] = True
                self.count = sum(self.occupied)

    def clear_table(self, req, res):
        self.b, self.clear, self.occupied, self.count = False, True, [False]*8, 0
        res.success, res.message = True, 'Synthetic B/C cleared'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = DualMockVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
