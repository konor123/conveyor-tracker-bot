import csv
import os

import pytest
import yaml

from orchestrator.abort_policy import (AbortCtx, AbortParams, AbortTracker, Event, HARD_FAULTS,
                                       decide, escalate)
from orchestrator.common import AbortLevel, State
from orchestrator.watchdogs import ClockMonitor, FreshnessMonitor, GripperMonitor, HeartbeatMonitor

DATA = os.path.join(os.path.dirname(__file__), 'data')
L = AbortLevel


def ctx(state=State.WAIT_CAP, holding=False, released=False):
    return AbortCtx(state=state, holding=holding, released=released)


def params(cfg):
    return AbortParams.from_dict(cfg['abort'])


def _decide_rows():
    with open(os.path.join(DATA, 'decide_table.csv')) as fh:
        rows = [r for r in csv.DictReader(line for line in fh if not line.startswith('#'))]
    return rows


def test_decide_matches_table():
    event_of = {'HARD_FAULT': Event.PROTECTIVE_STOP, 'STEP_FAILED': Event.STEP_FAILED,
                'USER_CANCEL': Event.USER_CANCEL}
    rows = _decide_rows()
    assert len(rows) == 144
    mismatches = []
    for r in rows:
        c = ctx(state=State[r['state']], holding=r['holding'] == '1', released=r['released'] == '1')
        got = decide(event_of[r['event']], c)
        if got != L[r['expected']]:
            mismatches.append((r, got.name))
    assert mismatches == []


@pytest.mark.parametrize("event", sorted(HARD_FAULTS, key=lambda e: e.name))
def test_every_hard_fault_is_l4_even_after_release(event):
    assert decide(event, ctx(state=State.RETREAT, holding=False, released=True)) == L.L4


@pytest.mark.parametrize("state", [State.DETECT_STONE, State.PICK_STONE])
@pytest.mark.parametrize("event", [Event.STEP_FAILED, Event.STEP_TIMEOUT])
def test_detect_and_pick_failures_are_l2_without_retry(state, event):
    assert decide(event, ctx(state=state)) == L.L2
    assert 1 not in {int(v) for v in L}


@pytest.mark.parametrize("current, new, expected", [
    (L.L3, L.L4, L.L4),
    (L.L4, L.L2, L.L4),
    (L.L2, L.L3, L.L3),
    (L.L3, L.L2, L.L3),
    (L.NONE, L.L2, L.L2),
    (L.CONTINUE_TO_RETREAT_VERIFY, L.L4, L.L4),
    (L.L4, L.CONTINUE_TO_RETREAT_VERIFY, L.L4),
])
def test_escalate_keeps_higher(current, new, expected):
    assert escalate(current, new) == expected


@pytest.mark.parametrize("elapsed, expected", [(10.000, L.L2), (10.001, L.L4)])
def test_l2_path_timeout(fake_clock, orch_cfg, elapsed, expected):
    t = AbortTracker(params(orch_cfg), fake_clock)
    assert t.on_event(Event.STEP_FAILED, ctx(state=State.MOVE_STANDBY)) == L.L2
    fake_clock.advance(elapsed)
    assert t.tick() == expected


@pytest.mark.parametrize("elapsed, expected", [(15.000, L.L3), (15.001, L.L4)])
def test_l3_path_timeout(fake_clock, orch_cfg, elapsed, expected):
    t = AbortTracker(params(orch_cfg), fake_clock)
    assert t.on_event(Event.STEP_FAILED, ctx(state=State.SYNCHRONIZE, holding=True)) == L.L3
    fake_clock.advance(elapsed)
    assert t.tick() == expected


def test_path_done_stops_path_timer(fake_clock, orch_cfg):
    t = AbortTracker(params(orch_cfg), fake_clock)
    t.on_event(Event.STEP_FAILED, ctx(state=State.SYNCHRONIZE, holding=True))
    fake_clock.advance(14.0)
    t.on_path_done()
    fake_clock.advance(10.0)
    assert t.tick() == L.L3
    assert t.waiting_operator


@pytest.mark.parametrize("start, after", [(L.L2, L.L3), (L.L3, L.L4)])
def test_cancel_without_response_escalates_one_level(fake_clock, orch_cfg, start, after):
    t = AbortTracker(params(orch_cfg), fake_clock)
    c = ctx(state=State.MOVE_STANDBY) if start == L.L2 else ctx(holding=True)
    assert t.on_event(Event.STEP_TIMEOUT, c) == start
    t.on_cancel_sent()
    fake_clock.advance(0.499)
    assert t.tick() == start
    fake_clock.advance(0.001)
    assert t.tick() == after


def test_cancel_ack_prevents_escalation(fake_clock, orch_cfg):
    t = AbortTracker(params(orch_cfg), fake_clock)
    t.on_event(Event.STEP_TIMEOUT, ctx(state=State.MOVE_STANDBY))
    t.on_cancel_sent()
    fake_clock.advance(0.2)
    t.on_cancel_ack()
    fake_clock.advance(1.0)
    assert t.tick() == L.L2


@pytest.mark.parametrize("state", [State.MOVE_STANDBY, State.WAIT_CAP, State.PLAN_INTERCEPT])
def test_user_cancel_while_holding_is_l3(state):
    assert decide(Event.USER_CANCEL, ctx(state=state, holding=True)) == L.L3


@pytest.mark.parametrize("released, expected", [(False, L.L3), (True, L.CONTINUE_TO_RETREAT_VERIFY)])
def test_belt_unsteady(released, expected):
    c = ctx(state=State.RELEASE if released else State.SYNCHRONIZE, holding=not released,
            released=released)
    assert decide(Event.BELT_UNSTEADY, c) == expected


def test_wait_operator_has_no_timeout(fake_clock, orch_cfg):
    t = AbortTracker(params(orch_cfg), fake_clock)
    t.on_event(Event.STEP_FAILED, ctx(state=State.MOVE_STANDBY))
    t.on_path_done()
    fake_clock.advance(3600.0)
    assert t.tick() == L.L2
    assert t.waiting_operator


# ---------------------------------------------------------------- 장애 10종 이벤트 열
with open(os.path.join(DATA, 'fault_expectations.yaml')) as _fh:
    FAULTS = yaml.safe_load(_fh)

CTX_KEYS = {'state', 'holding', 'released'}


def _run_fault(spec, clock, cfg):
    tracker = AbortTracker(params(cfg), clock)
    w = cfg['watchdog']
    hb = HeartbeatMonitor(w['heartbeat_timeout_s'], clock)
    fresh = FreshnessMonitor(w['obs_freshness_s'])
    clk = ClockMonitor()
    clk.update(clock.ros_now())
    grip = GripperMonitor(w['gripper_response_max_s'], w['gripper_alive_stall_s'], clock)
    base = dict(spec['ctx'])
    t0 = clock.steady_now()
    trace = []
    for step in spec['steps']:
        clock.advance(t0 + step['t'] - clock.steady_now())
        cur = dict(base, **{k: v for k, v in step.items() if k in CTX_KEYS})
        now = clock.ros_now()
        c = AbortCtx(state=State[cur['state']], holding=cur['holding'], released=cur['released'])
        level = tracker.level
        if 'event' in step:
            level = tracker.on_event(Event[step['event']], c)
        elif 'beat' in step:
            hb.beat(step['beat'])
        elif 'check_heartbeat' in step:
            node = step['check_heartbeat']
            if hb.lost(node):
                ev = Event.CONTROLLER_HEARTBEAT_LOST if node == 'robot_controller' else Event.HEARTBEAT_LOST
                level = tracker.on_event(ev, c)
        elif 'obs' in step:
            if fresh.stale(now, now - step['obs']):
                level = tracker.on_event(Event.OBS_STALE, c)
        elif 'ros_jump' in step:
            clock.jump_ros(step['ros_jump'])
            if clk.update(clock.ros_now()):
                level = tracker.on_event(Event.CLOCK_JUMP, c)
        elif 'gripper' in step:
            g = step['gripper']
            grip.update(g['error_bits'], g['response_s'], alive_counter=None)
            if grip.fault():
                level = tracker.on_event(Event.GRIPPER_FAULT, c)
        elif step.get('cancel_sent'):
            tracker.on_cancel_sent()
        elif step.get('cancel_ack'):
            tracker.on_cancel_ack()
        elif step.get('tick'):
            level = tracker.tick()
        trace.append(level)
        if 'expect' in step:
            assert level == L[step['expect']], (step, level.name)
    return tracker.level, trace


@pytest.mark.parametrize("name", list(FAULTS))
def test_fault_event_sequences(fake_clock, orch_cfg, name):
    final, _ = _run_fault(FAULTS[name], fake_clock, orch_cfg)
    assert final == L[FAULTS[name]['expected']]


def test_fault_list_covers_plan_7_3():
    assert set(FAULTS) == {'robot_hang', 'protective_stop', 'heartbeat_loss', 'time_insufficient',
                           'gripper_miss', 'gripper_cmd_delay', 'occlusion_long', 'speed_change',
                           'frame_drop_10pct', 'clock_jump'}
