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


def ctx(state=State.WAIT_CAP, holding=False, released=False, retry_left=0, retryable=True,
        now=100.0, est_duration_s=1.0, deadline=200.0):
    return AbortCtx(state=state, holding=holding, released=released, retry_left=retry_left,
                    retryable=retryable, now=now, est_duration_s=est_duration_s, deadline=deadline)


def params(cfg):
    return AbortParams.from_dict(cfg['abort'])


def _decide_rows():
    with open(os.path.join(DATA, 'decide_table.csv')) as fh:
        rows = [r for r in csv.DictReader(line for line in fh if not line.startswith('#'))]
    return rows


@pytest.mark.req("REQ-ABT-001")
@pytest.mark.tc("TC-ABT-001-01")
@pytest.mark.level("U")
def test_decide_matches_table():
    event_of = {'HARD_FAULT': Event.PROTECTIVE_STOP, 'STEP_FAILED': Event.STEP_FAILED,
                'USER_CANCEL': Event.USER_CANCEL}
    rows = _decide_rows()
    assert len(rows) == 288
    mismatches = []
    for r in rows:
        c = ctx(state=State[r['state']], holding=r['holding'] == '1', released=r['released'] == '1',
                retry_left=1 if r['retry'] == 'left' else 0)
        got = decide(event_of[r['event']], c)
        if got != L[r['expected']]:
            mismatches.append((r, got.name))
    assert mismatches == []


@pytest.mark.req("REQ-ABT-001")
@pytest.mark.tc("TC-ABT-001-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("event", sorted(HARD_FAULTS, key=lambda e: e.name))
def test_every_hard_fault_is_l4_even_after_release(event):
    assert decide(event, ctx(state=State.RETREAT, holding=False, released=True)) == L.L4


@pytest.mark.req("REQ-ABT-001")
@pytest.mark.tc("TC-ABT-001-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("est, expected", [(0.999, L.L1), (1.0, L.L2), (1.001, L.L2)])
def test_retry_needs_time_before_deadline(est, expected):
    # now + est < deadline 일 때만 재시도
    c = ctx(state=State.DETECT_STONE, retry_left=1, now=100.0, est_duration_s=est, deadline=101.0)
    assert decide(Event.STEP_FAILED, c) == expected


@pytest.mark.req("REQ-ABT-001")
@pytest.mark.tc("TC-ABT-001-01")
@pytest.mark.level("U")
def test_non_retryable_failure_is_l2():
    c = ctx(state=State.PICK_STONE, retry_left=2, retryable=False)
    assert decide(Event.STEP_FAILED, c) == L.L2


@pytest.mark.req("REQ-ABT-002")
@pytest.mark.tc("TC-ABT-002-01")
@pytest.mark.level("U")
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


@pytest.mark.req("REQ-ABT-003")
@pytest.mark.tc("TC-ABT-003-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("elapsed, expected", [(10.000, L.L2), (10.001, L.L4)])
def test_l2_path_timeout(fake_clock, orch_cfg, elapsed, expected):
    t = AbortTracker(params(orch_cfg), fake_clock)
    assert t.on_event(Event.STEP_FAILED, ctx(state=State.MOVE_STANDBY, retryable=False)) == L.L2
    fake_clock.advance(elapsed)
    assert t.tick() == expected


@pytest.mark.req("REQ-ABT-003")
@pytest.mark.tc("TC-ABT-003-02")
@pytest.mark.level("U")
@pytest.mark.parametrize("elapsed, expected", [(15.000, L.L3), (15.001, L.L4)])
def test_l3_path_timeout(fake_clock, orch_cfg, elapsed, expected):
    t = AbortTracker(params(orch_cfg), fake_clock)
    assert t.on_event(Event.STEP_FAILED, ctx(state=State.SYNCHRONIZE, holding=True)) == L.L3
    fake_clock.advance(elapsed)
    assert t.tick() == expected


@pytest.mark.req("REQ-ABT-003")
@pytest.mark.tc("TC-ABT-003-02")
@pytest.mark.level("U")
def test_path_done_stops_path_timer(fake_clock, orch_cfg):
    t = AbortTracker(params(orch_cfg), fake_clock)
    t.on_event(Event.STEP_FAILED, ctx(state=State.SYNCHRONIZE, holding=True))
    fake_clock.advance(14.0)
    t.on_path_done()
    fake_clock.advance(10.0)
    assert t.tick() == L.L3
    assert t.waiting_operator


@pytest.mark.req("REQ-ABT-004")
@pytest.mark.tc("TC-ABT-004-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("start, after", [(L.L2, L.L3), (L.L3, L.L4)])
def test_cancel_without_response_escalates_one_level(fake_clock, orch_cfg, start, after):
    t = AbortTracker(params(orch_cfg), fake_clock)
    c = ctx(state=State.MOVE_STANDBY, retryable=False) if start == L.L2 else ctx(holding=True)
    assert t.on_event(Event.STEP_TIMEOUT, c) == start
    t.on_cancel_sent()
    fake_clock.advance(0.499)
    assert t.tick() == start
    fake_clock.advance(0.001)
    assert t.tick() == after


@pytest.mark.req("REQ-ABT-004")
@pytest.mark.tc("TC-ABT-004-01")
@pytest.mark.level("U")
def test_cancel_ack_prevents_escalation(fake_clock, orch_cfg):
    t = AbortTracker(params(orch_cfg), fake_clock)
    t.on_event(Event.STEP_TIMEOUT, ctx(state=State.MOVE_STANDBY, retryable=False))
    t.on_cancel_sent()
    fake_clock.advance(0.2)
    t.on_cancel_ack()
    fake_clock.advance(1.0)
    assert t.tick() == L.L2


@pytest.mark.req("REQ-ABT-005")
@pytest.mark.tc("TC-ABT-005-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("state", [State.MOVE_STANDBY, State.WAIT_CAP, State.PLAN_INTERCEPT])
def test_user_cancel_while_holding_is_l3(state):
    assert decide(Event.USER_CANCEL, ctx(state=state, holding=True)) == L.L3


@pytest.mark.req("REQ-ABT-013")
@pytest.mark.tc("TC-ABT-013-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("released, expected", [(False, L.L3), (True, L.CONTINUE_TO_RETREAT_VERIFY)])
def test_belt_unsteady(released, expected):
    c = ctx(state=State.RELEASE if released else State.SYNCHRONIZE, holding=not released,
            released=released)
    assert decide(Event.BELT_UNSTEADY, c) == expected


@pytest.mark.req("REQ-ABT-016")
@pytest.mark.tc("TC-ABT-016-01")
@pytest.mark.level("U")
def test_wait_operator_has_no_timeout(fake_clock, orch_cfg):
    t = AbortTracker(params(orch_cfg), fake_clock)
    t.on_event(Event.STEP_FAILED, ctx(state=State.MOVE_STANDBY, retryable=False))
    t.on_path_done()
    fake_clock.advance(3600.0)
    assert t.tick() == L.L2
    assert t.waiting_operator


# ---------------------------------------------------------------- 장애 10종 이벤트 열
with open(os.path.join(DATA, 'fault_expectations.yaml')) as _fh:
    FAULTS = yaml.safe_load(_fh)

CTX_KEYS = {'state', 'holding', 'released', 'retry_left', 'retryable'}


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
        # deadline_s는 시나리오 시작 기준 [s]
        deadline = now - step['t'] + step['deadline_s'] if 'deadline_s' in step else now + 1e6
        c = AbortCtx(state=State[cur['state']], holding=cur['holding'], released=cur['released'],
                     retry_left=cur['retry_left'], retryable=cur['retryable'], now=now,
                     est_duration_s=step.get('est_duration_s', 1.0), deadline=deadline)
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


@pytest.mark.req("REQ-ABT-015")
@pytest.mark.tc("TC-ABT-015-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("name", list(FAULTS))
def test_fault_event_sequences(fake_clock, orch_cfg, name):
    final, _ = _run_fault(FAULTS[name], fake_clock, orch_cfg)
    assert final == L[FAULTS[name]['expected']]


@pytest.mark.req("REQ-ABT-015")
@pytest.mark.tc("TC-ABT-015-01")
@pytest.mark.level("U")
def test_fault_list_covers_plan_7_3():
    assert set(FAULTS) == {'robot_hang', 'protective_stop', 'heartbeat_loss', 'time_insufficient',
                           'gripper_miss', 'gripper_cmd_delay', 'occlusion_long', 'speed_change',
                           'frame_drop_10pct', 'clock_jump'}
