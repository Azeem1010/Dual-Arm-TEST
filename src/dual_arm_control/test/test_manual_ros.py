"""ROS message-level checks; run after sourcing the built workspace."""
from concurrent.futures import Future
import json
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip('moveit_msgs')
pytest.importorskip('dual_arm_interfaces.srv._manual_xy')
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState, RobotTrajectory
from std_srvs.srv import Trigger
from dual_arm_control.moveit_client import MoveItClient
from dual_arm_control.manual_xy import ManualServer
from dual_arm_control.xy_calibration import calibration
from dual_arm_control.bridge_node import CM530Bridge


@pytest.mark.parametrize('fraction', [.5, .999, float('nan'), float('inf')])
def test_partial_and_nonfinite_cartesian_paths_are_never_returned(fraction):
    client = MoveItClient.__new__(MoveItClient)
    client.cfg = dict(base_frame='arm1_base', planning_group='arm1', tcp_link='arm1_suction_tcp',
                      velocity_scale=.1, acceleration_scale=.1, planning_timeout=5.)
    client.cartesian = object()
    client.state = lambda: RobotState()
    client.pose = lambda xyz: Pose()
    client.service = lambda *args: SimpleNamespace(error_code=SimpleNamespace(val=1), fraction=fraction,
                                                   solution=RobotTrajectory())
    with pytest.raises(RuntimeError, match='Incomplete Cartesian'):
        client.plan_cartesian([.15, .04, .04])


def test_current_uses_fresh_read_independently_of_canceled_motion():
    future = Future()
    future.set_result(Trigger.Response(success=True, message='[563,630,676,844]'))
    server = SimpleNamespace(cfg=dict(service_timeout=1., simulation=True, pick_z=.02, travel_z=.04),
        read_client=SimpleNamespace(wait_for_service=lambda **kw: True, call_async=lambda request: future),
        cal=calibration({}), model=SimpleNamespace(forward=lambda q: ([.15, .04, .04], [0., 0., -1.])))
    # No motion.state or cancel event exists: current must get its own fresh READ.
    response = ManualServer.current(server, Trigger.Request(), Trigger.Response())
    assert response.success
    assert json.loads(response.message)['measured_raw'] == [563, 630, 676, 844]


@pytest.mark.parametrize('busy', [False, True])
def test_bridge_stop_latches_before_late_execution_and_holds_when_idle(busy):
    holds = []
    bridge = SimpleNamespace(busy_lock=threading.Lock(), busy=busy, fault='',
                             hardware=SimpleNamespace(healthy=True, hold=holds.append))
    result = CM530Bridge.stop_arm(bridge, 'arm1', Trigger.Response())
    assert result.success and bridge.fault
    assert holds == ([] if busy else ['arm1'])
    with pytest.raises(RuntimeError, match='fault latched'):
        CM530Bridge.check_safety(bridge)


def test_bridge_stop_does_not_write_to_desynchronized_serial():
    bridge = SimpleNamespace(busy_lock=threading.Lock(), busy=False, fault='', hardware=SimpleNamespace(healthy=False))
    result = CM530Bridge.stop_arm(bridge, 'arm1', Trigger.Response())
    assert not result.success and bridge.fault and not bridge.busy


@pytest.mark.parametrize('execute', [False, True])
def test_execution_failure_stops_bridge_but_preview_never_executes(execute):
    stopped, dispatched = [], []
    def fail(trajectory):
        dispatched.append(True)
        raise TimeoutError('Lost execution response')
    q = [0., 0., 0., 0.]
    server = SimpleNamespace(lock=threading.Lock(), cancel=threading.Event(),
        config=dict(task=dict(travel_z=.04), manual=dict(xy_min=[.08, -.12], xy_max=[.20, .12])),
        cfg=dict(travel_z=.04, simulation=True), scene=lambda: None,
        measured=lambda: (None, q, [.15, .04, .04], [0., 0., -1.]),
        motion=SimpleNamespace(plan_cartesian=lambda *args: object(), execute=fail),
        validate_plan=lambda *args: ([1.], [q]), cal=calibration({}),
        stop=lambda *args: (stopped.append(True) or Trigger.Response(success=True)),
        get_logger=lambda: SimpleNamespace(info=lambda msg: None, warning=lambda msg: None))
    from dual_arm_interfaces.srv import ManualXY
    result = ManualServer.request(server, ManualXY.Request(x=.15, y=.04, execute=execute), ManualXY.Response())
    assert result.success is not execute
    assert dispatched == stopped == ([True] if execute else [])
