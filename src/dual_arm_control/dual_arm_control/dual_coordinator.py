"""Integrated ROS 2 A→B→C coordinator, one journal and one B-zone owner."""
import fcntl
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from dual_arm_interfaces.srv import ConfirmStep
from vision_interfaces.srv import CaptureRanking, GetPickTarget, GetZoneState, GetGridState, VerifyPlace
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene

from .coordinator import Coordinator, DEFAULTS, validate_config
from .arm2_coordinator import ARM2_DEFAULTS, Arm2Coordinator
from .arm2_config import validate_arm2
from .moveit_client import MoveItClient
from .dual_workflow import BReservation, run_transfer


class ArmIO:
    """Existing workflow API backed by a shared owner, cancel token and scene."""
    def __init__(self, owner, arm, cfg):
        self.owner, self.arm, self.cfg = owner, arm, cfg
        self.cancel = owner.cancel
        self.group = owner.group
        self.motion = MoveItClient(owner, cfg, owner.group, owner.cancel)
        for attr, kind, key in [('zone_client', GetZoneState, 'zone_service'),
                                ('pick_client', GetPickTarget, 'pick_service'),
                                ('grid_client', GetGridState, 'grid_service'),
                                ('verify_client', VerifyPlace, 'verify_service')]:
            if key in cfg:
                setattr(self, attr, owner.create_client(kind, cfg[key], callback_group=owner.group))
        if arm == 'arm1':
            self.capture_client = owner.create_client(
                CaptureRanking, '/vision/env/capture_ranking', callback_group=owner.group)

    def guard(self):
        self.owner.guard()

    def vision_call(self, client, request):
        self.guard()
        return self.motion.service(client, request)

    zone = Coordinator.zone
    pick = Coordinator.pick

    def capture_ranking(self):
        result = self.vision_call(self.capture_client, CaptureRanking.Request())
        if not result.success or result.low_confidence:
            raise RuntimeError('A ranking snapshot invalid: ' + result.message)
        return result
    check_grid = Arm2Coordinator.check_grid
    verify_layout = Arm2Coordinator.verify_layout
    verify_slot = Arm2Coordinator.verify_slot
    record_slot = Arm2Coordinator.record_slot

    def check_home(self):
        js = self.motion.state().joint_state
        if max(abs(a-b) for a,b in zip(js.position, self.cfg['home_joints'])) > 0.04:
            raise RuntimeError(f'{self.arm} measured pose is not its calibrated HOME')

    def suction(self, operation):
        self.guard()
        self.owner.manual(self.arm, operation)

    def setup_scene(self):
        self.guard()
        if self.arm == 'arm2':
            # Reuse measured obstacle and occupied-slot construction.
            Arm2Coordinator.setup_scene(self)
        else:
            self.motion.setup_scene()

    @property
    def status(self):
        return {'occupied': self.owner.status['occupied']}

    def __getattr__(self, name):
        if name in ('move_joints', 'move_pose', 'check_ik', 'attach_disk', 'detach_disk',
                    'record_placed_disk', 'clear_placed_disk'):
            def guarded(*args, **kwargs):
                self.guard()
                return getattr(self.motion, name)(*args, **kwargs)
            return guarded
        raise AttributeError(name)


class DualCoordinator(Coordinator):
    def __init__(self):
        Node.__init__(self, 'dual_coordinator')
        self.group, self.lock = ReentrantCallbackGroup(), threading.RLock()
        self.cancel, self.confirm_event = threading.Event(), threading.Event()
        self.confirm_result, self.worker, self.busy = None, None, False
        self.estop_latched, self.safety = False, (None, 0.0)
        self.bridge_health = ('', 0.0)
        self.limit = self.declare_parameter('cycles', 1).value
        if type(self.limit) is not int or not 1 <= self.limit <= 8:
            raise ValueError('cycles must be in 1..8')
        self.safety_age = self.declare_parameter('safety_max_age', 2.0).value
        self.cfg = {'simulation': self.declare_parameter('simulation', True).value,
                    'auto_confirm_demo': self.declare_parameter('auto_confirm_demo', False).value,
                    'manual_timeout': self.declare_parameter('manual_timeout', 600.0).value}
        if any(not math.isfinite(v) or v <= 0 for v in (self.safety_age, self.cfg['manual_timeout'])):
            raise ValueError('Safety age and manual timeout must be finite and positive')
        if self.cfg['auto_confirm_demo'] and not self.cfg['simulation']:
            raise ValueError('Automatic confirmation is demo-only')
        configs = {}
        for arm, defaults in [('arm1', {**DEFAULTS, 'model_signature': ''}), ('arm2', ARM2_DEFAULTS)]:
            configs[arm] = {k: self.declare_parameter(arm + '.' + k, v).value
                            for k, v in defaults.items() if k not in ('simulation', 'auto_confirm_demo')}
            configs[arm].update(self.cfg)
            configs[arm]['integrated'] = True
            configs[arm]['object_prefix'] = arm + '_'
            validate_config(configs[arm])
        configs['arm2']['c_slots'] = validate_arm2(configs['arm2'])
        self.journal = Path(self.declare_parameter('journal_path', '~/.ros/dual_arm/integrated.json').value).expanduser()
        self.journal.parent.mkdir(parents=True, exist_ok=True)
        self.file_lock = self.journal.with_suffix('.lock').open('a')
        fcntl.flock(self.file_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.fingerprint = hashlib.sha256(json.dumps(configs, sort_keys=True).encode()).hexdigest()
        self.status = dict(state='IDLE', cycle_id='', active_arm='', holding='empty', b_locked=False,
            b_owner='', occupied=[False]*8, slot=0, completed=0, arms={}, message='',
            confirmation_token='', confirmation_operation='', config_fingerprint=self.fingerprint)
        if self.journal.exists():
            old = json.loads(self.journal.read_text())
            if (not isinstance(old.get('occupied'), list) or len(old['occupied']) != 8
                    or any(type(v) is not bool for v in old['occupied'])):
                raise ValueError('Invalid integrated journal; inspect physical layout')
            self.status.update(old)
            if (old.get('state') not in ('IDLE', 'SUCCEEDED', 'FULL') or old.get('b_owner')
                    or old.get('holding') != 'empty' or old.get('config_fingerprint') != self.fingerprint):
                self.status.update(state='FAULT', message='Previous run incomplete or configuration changed; inspect and recover')
        self.status.update(confirmation_token='', confirmation_operation='')
        if self.status['state'] in ('FAULT', 'FULL'):
            self.status.update(confirmation_token=uuid.uuid4().hex, confirmation_operation='recovery')
        self.reservation = BReservation(self.status['b_owner'])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/dual_arm/status', qos)
        self.arm_publishers = {a: self.create_publisher(String, '/' + a + '/status', qos) for a in configs}
        self.handoff = self.create_publisher(String, '/handoff/b_state', qos)
        self.permit = self.create_publisher(Bool, '/dual_arm/permit', 10)
        self.arm1 = ArmIO(self, 'arm1', configs['arm1'])
        self.arm2 = ArmIO(self, 'arm2', configs['arm2'])
        self.create_subscription(Bool, '/safety/estop', self.on_safety, 10, callback_group=self.group)
        self.create_subscription(String, '/cm530/status', self.on_bridge, 10, callback_group=self.group)
        self.create_service(Trigger, '/dual_arm/start', self.start, callback_group=self.group)
        self.create_service(Trigger, '/dual_arm/cancel', self.stop, callback_group=self.group)
        self.create_service(ConfirmStep, '/dual_arm/confirm', self.confirm, callback_group=self.group)
        self.create_service(ConfirmStep, '/dual_arm/recover', self.recover, callback_group=self.group)
        self.create_timer(0.1, self.watchdog, callback_group=self.group)
        self.create_timer(0.5, self.publish)
        self.update({})

    def on_safety(self, msg):
        self.safety = (msg.data, time.monotonic())
        self.estop_latched |= msg.data

    def on_bridge(self, msg):
        self.bridge_health = (msg.data, time.monotonic())

    def guard(self):
        if self.cancel.is_set():
            raise InterruptedError('Integrated task canceled')
        value, stamp = self.safety
        if self.estop_latched or value is not False or time.monotonic() - stamp > self.safety_age:
            raise RuntimeError('Emergency-stop input asserted, missing or stale')
        health, stamp = self.bridge_health
        if health not in ('ready', 'mock_ready', 'executing') or time.monotonic() - stamp > max(5.0, self.safety_age):
            raise RuntimeError('CM530 unavailable or faulted: ' + health)

    def watchdog(self):
        try:
            self.guard()
            allowed = self.status['state'] != 'FAULT'
        except Exception as exc:
            allowed = False
            if self.busy:
                if not self.cancel.is_set():
                    self.get_logger().error('Safety watchdog canceled integrated task: ' + str(exc))
                self.cancel.set()
                self.confirm_event.set()
                self.arm1.motion.stop()
                self.arm2.motion.stop()
        self.permit.publish(Bool(data=allowed))

    def publish(self):
        super().publish()
        with self.lock:
            self.handoff.publish(String(data=json.dumps(dict(owner=self.status['b_owner'],
                locked=self.status['b_locked'], state=self.status['state'], standalone=False))))

    def start(self, req, res):
        with self.lock:
            if self.busy or self.status['state'] in ('FAULT', 'FULL'):
                res.success, res.message = False, 'Busy, faulted or full; inspect /dual_arm/status'
                return res
            self.cancel.clear()
            try:
                self.guard()
            except Exception as e:
                res.success, res.message = False, str(e)
                return res
            self.busy = True
            self.update(dict(state='STARTING', cycle_id=uuid.uuid4().hex, completed=0, message='',
                             confirmation_token='', confirmation_operation=''))
            self.worker = threading.Thread(target=self.run_cycle, daemon=True)
            self.worker.start()
            res.success, res.message = True, 'A→B→C accepted; completion on /dual_arm/status'
            return res

    def initialize(self):
        for arm in (self.arm1, self.arm2):
            self.update(dict(active_arm=arm.arm, state='INITIALIZING'))
            arm.setup_scene()
            arm.move_joints(arm.cfg['home_joints'])
        if self.cancel.wait(max(self.arm1.cfg['home_settle'], self.arm2.cfg['home_settle'])):
            raise InterruptedError('Canceled during initial settling')

    def occupied(self):
        return list(self.status['occupied'])

    def completed(self):
        self.update(dict(completed=self.status['completed'] + 1))

    def arm_state(self, arm, fields):
        with self.lock:
            if fields['b_locked']:
                self.reservation.acquire(arm)
            elif self.reservation.owner == arm and fields['state'] != 'FAULT':
                # Only the existing workflow's successful departure may release B.
                self.reservation.release(arm, fields['state'] in ('SUCCEEDED', 'B_EMPTY'))
            arms = {**self.status['arms'], arm: dict(fields)}
            update = dict(active_arm=arm, state=arm.upper() + ':' + fields['state'], arms=arms,
                          holding=fields['holding'], b_owner=self.reservation.owner,
                          b_locked=bool(self.reservation.owner))
            for key in ('occupied', 'slot'):
                if key in fields:
                    update[key] = fields[key]
            self.update(update)
            self.arm_publishers[arm].publish(String(data=json.dumps(fields)))

    def manual(self, arm, operation):
        self.update(dict(active_arm=arm))
        Coordinator.suction(self, operation)

    confirm = Arm2Coordinator.confirm

    def run_cycle(self):
        try:
            state = run_transfer(self, self.limit)
            self.update(dict(state=state, active_arm='', holding='empty', message='Verified A→B→C transfer complete',
                confirmation_token=uuid.uuid4().hex if state == 'FULL' else '',
                confirmation_operation='recovery' if state == 'FULL' else ''))
        except Exception as e:
            self.cancel.set()
            self.arm1.motion.stop()
            self.arm2.motion.stop()
            self.update(dict(state='FAULT', message=str(e), confirmation_token=uuid.uuid4().hex,
                             confirmation_operation='recovery'))
            self.get_logger().error(str(e))
        finally:
            with self.lock:
                self.busy = False
            self.publish()

    def stop(self, req, res):
        with self.lock:
            res.success = self.busy
            if self.busy:
                self.cancel.set()
                self.confirm_event.set()
                self.arm1.motion.stop()
                self.arm2.motion.stop()
            res.message = 'Cancellation requested; B reservation retained until verified recovery'
        return res

    def recover(self, req, res):
        with self.lock:
            if (self.busy or self.status['state'] not in ('FAULT', 'FULL') or not req.success
                    or not req.token or req.token != self.status['confirmation_token']):
                res.accepted, res.message = False, 'Requires current recovery token and explicit cleared-scene confirmation'
                return res
            self.busy = True
        try:
            self.cancel.clear()
            self.guard()
            for arm in (self.arm1, self.arm2):
                arm.check_home()
            z = self.arm1.zone()
            if not z.success or z.b_has_object or not z.b_arm_clear or z.c_object_count:
                raise RuntimeError('Recovery requires B/C empty and both tools empty, both arms at HOME')
            self.arm2.check_grid([False]*8)
            for arm in (self.arm1, self.arm2):
                arm.motion.detach_disk()
                arm.motion.clear_placed_disk()
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            # Explicit IDs also clear a retained MoveIt scene after a coordinator
            # restart, when this process has no local attached-object flag.
            scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                link_name=arm.cfg['tcp_link'], object=CollisionObject(
                    id=arm.arm+'_carried_disk', operation=CollisionObject.REMOVE)) for arm in (self.arm1,self.arm2)]
            scene.world.collision_objects = [CollisionObject(id=f'c_slot_{i}', operation=CollisionObject.REMOVE)
                                             for i in range(1, 9)]
            scene.world.collision_objects.extend(CollisionObject(id=name, operation=CollisionObject.REMOVE)
                for name in ('placed_disk_b','arm1_carried_disk','arm2_carried_disk'))
            self.arm2.motion.apply_scene(scene)
            self.reservation = BReservation()
            self.update(dict(state='IDLE', active_arm='', holding='empty', b_owner='', b_locked=False,
                occupied=[False]*8, slot=0, arms={}, confirmation_token='', confirmation_operation='',
                message='Scene cleared and verified; explicit new start required', config_fingerprint=self.fingerprint))
            res.accepted, res.message = True, 'Recovered without motion'
        except Exception as e:
            res.accepted, res.message = False, str(e)
        finally:
            self.busy = False
        return res


def main(args=None):
    rclpy.init(args=args)
    node = DualCoordinator()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel.set()
        node.confirm_event.set()
        node.arm1.motion.stop()
        node.arm2.motion.stop()
        if node.worker:
            node.worker.join(timeout=5)
        executor.shutdown()
        node.file_lock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
