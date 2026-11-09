import threading
from types import SimpleNamespace as S
import pytest
from dual_arm_control.dual_workflow import BReservation, run_transfer
from test_workflow import FakeIO, CFG as A_CFG
from test_arm2_workflow import IO, CFG as B_CFG


class Transfer:
    def __init__(self, occupied=None, b=False, failure=''):
        self.cancel=threading.Event();self.map=list(occupied or [False]*8)
        self.reservation=BReservation();self.states=[];self.count=0
        self.arm1=FakeIO();self.arm1.cfg=A_CFG
        self.arm1.capture_ranking=lambda: self.arm1.events.append('capture_ranking')
        self.arm2=IO(sum(self.map),failure);self.arm2.cfg=B_CFG
        self.arm1.b_occupied=b
        original_zone=self.arm1.zone
        def zone():
            z=original_zone();z.c_object_count=self.arm2.count;return z
        self.arm1.zone=zone
    def initialize(self): pass
    def guard(self):
        if self.cancel.is_set():raise InterruptedError('cancel')
    def occupied(self):return list(self.map)
    def completed(self):self.count+=1
    def arm_state(self,arm,s):
        self.states.append((arm,dict(s)))
        if s['b_locked']:self.reservation.acquire(arm)
        elif self.reservation.owner==arm and s['state']!='FAULT':
            self.reservation.release(arm,s['state'] in ('SUCCEEDED','B_EMPTY'))
        if arm=='arm1' and s['state']=='SUCCEEDED':self.arm2.b=True
        if arm=='arm2':
            self.map=list(s['occupied'])
            if s['state']=='B_EMPTY':self.arm1.b_occupied=False


def test_full_eight_transfers_one_shared_owner():
    t=Transfer()
    assert run_transfer(t,8)=='FULL'
    assert t.count==8 and all(t.map) and t.reservation.owner==''
    assert t.arm1.events.count('capture_ranking')==1
    completed=[a for a,s in t.states if s['state']=='SUCCEEDED']
    assert completed==['arm1','arm2']*7+['arm1']
    assert [s['slot'] for a,s in t.states if a=='arm2' and s['state']=='B_RESERVED']==list(range(1,9))


def test_full_c_preserves_b_and_never_picks_a():
    t=Transfer([True]*8,b=True)
    assert run_transfer(t,8)=='FULL'
    assert t.arm1.b_occupied and t.states==[] and 'pick_service' not in t.arm1.events


def test_existing_b_consumed_before_a_and_smallest_hole():
    t=Transfer([True,False,True,False,False,False,False,False],b=True)
    assert run_transfer(t,1)=='SUCCEEDED'
    assert 'pick_service' not in t.arm1.events
    assert 'capture_ranking' not in t.arm1.events
    assert t.map[:3]==[True]*3 and t.count==1


def test_failed_arm2_grasp_retains_shared_b_owner_and_blocks_new_flow():
    t=Transfer(failure='grasp')
    with pytest.raises(RuntimeError,match='grasp'):run_transfer(t,2)
    assert t.reservation.owner=='arm2' and t.count==0 and not any(t.map)
    assert t.states[-1][1]['holding']=='unknown'
    with pytest.raises(RuntimeError):t.reservation.acquire('arm1')


def test_failed_ranking_stops_before_any_motion():
    t=Transfer()
    def fail():
        raise RuntimeError('snapshot unavailable')
    t.arm1.capture_ranking=fail
    with pytest.raises(RuntimeError, match='snapshot unavailable'):
        run_transfer(t, 1)
    assert t.states==[] and t.count==0


def test_b_owner_cannot_be_stolen_or_released_without_observation():
    b=BReservation();b.acquire('arm1')
    with pytest.raises(RuntimeError):b.acquire('arm2')
    with pytest.raises(RuntimeError):b.release('arm1',False)
    with pytest.raises(RuntimeError):b.release('arm2',True)
    assert b.owner=='arm1'
    b.release('arm1',True);b.acquire('arm2');assert b.owner=='arm2'
