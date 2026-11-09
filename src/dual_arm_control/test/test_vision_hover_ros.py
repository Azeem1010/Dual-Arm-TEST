"""Exercise the actual hover runner and ROS service contract without moving hardware."""
from concurrent.futures import Future
import threading
from types import SimpleNamespace as S

import pytest

pytest.importorskip('moveit_msgs')
pytest.importorskip('vision_interfaces.srv')
from vision_interfaces.srv import GetPickTarget
from dual_arm_control.arm1_vision_test import Arm1VisionTest
from dual_arm_control.moveit_client import MoveItClient
from dual_arm_control.mock_vision import MockVision

CFG = dict(home_joints=[0., 0., 1.57, 1.57], home_settle=0.001,
           service_timeout=0.01, a_min=[.08, 0., .017], a_max=[.20, .10, .023],
           pick_z=.02, height_tolerance=.003, approach_height=.02)


def runner(failure=None, cancel_stage=None, result=None):
    events, states = [], []
    cancel = threading.Event()
    class Motion:
        def setup_scene(self): events.append('scene')
        def move_joints(self, q): events.append('home')
        def service(self, client, request):
            events.append('query')
            assert request.target_rank == 0 and request.peek_only
            if cancel_stage == 'query': cancel.set()
            if failure: raise failure
            return result if result is not None else GetPickTarget.Response(success=True, message='selected among caps',
                                          rank=0, x=.15, y=.04, z=.02)
        def move_pose(self, xyz):
            events.append(('hover', xyz))
            if cancel_stage == 'motion': cancel.set()
        def stop(self): events.append('stop')
    def report(state, message='', target=None):
        states.append((state, target, message))
        if state == 'WAITING_FOR_VISION': events.append('settled')
    node = S(cfg=CFG, cancel=cancel, lock=threading.Lock(), busy=True,
             motion=Motion(), pick_client=object(), report=report)
    if cancel_stage == 'home': cancel.set()
    Arm1VisionTest.run_test(node)
    return events, states, node


def test_home_then_query_then_hover_stays_at_target():
    events, states, node = runner()
    assert events == ['scene', 'home', 'settled', 'query', ('hover', [.15, .04, .04])]
    assert states[-1][0:2] == ('SUCCEEDED', [.15, .04, .04])
    assert not node.busy


@pytest.mark.parametrize('stage', ['home', 'query', 'motion'])
def test_cancel_never_reports_success_or_returns_home(stage):
    events, states, node = runner(cancel_stage=stage)
    assert states[-1][0] == 'FAULT' and events[-1] == 'stop'
    assert events.count('home') == 1 and not node.busy
    if stage != 'motion': assert not any(isinstance(e, tuple) for e in events)


@pytest.mark.parametrize('failure', [TimeoutError('service timed out'), ValueError('no target')])
def test_query_failure_stops_before_target_motion(failure):
    events, states, _ = runner(failure=failure)
    assert states[-1][0] == 'FAULT'
    assert not any(isinstance(e, tuple) for e in events)
    assert events[-1] == 'stop'


def test_service_response_timeout_is_bounded():
    client = S(wait_for_service=lambda **kw: True, call_async=lambda request: Future())
    motion = S(cfg=CFG, cancel=threading.Event())
    with pytest.raises(TimeoutError):
        MoveItClient.service(motion, client, GetPickTarget.Request())


def test_mock_echoes_rank_and_peek_does_not_consume_target():
    mock = S(a=True, xyz=[.15, .04, .02])
    for peek in (True, False):
        result = MockVision.pick(mock, GetPickTarget.Request(target_rank=0, peek_only=peek),
                                 GetPickTarget.Response())
        assert result.success and result.rank == 0 and mock.a


@pytest.mark.parametrize('response', [
    GetPickTarget.Response(success=False, message='no caps'),
    GetPickTarget.Response(success=True, x=float('nan'), y=.04, z=.02),
    GetPickTarget.Response(success=True, x=.5, y=.04, z=.02),
])
def test_failed_or_invalid_service_response_never_moves_above_a(response):
    events, states, _ = runner(result=response)
    assert states[-1][0] == 'FAULT' and events[-1] == 'stop'
    assert not any(isinstance(e, tuple) for e in events)
