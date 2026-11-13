import pytest

from orchestrator.abort_policy import AbortCtx, Event, decide
from orchestrator.common import AbortLevel, State
from orchestrator.watchdogs import (WATCH_STATES, BeltSteadyMonitor, ClockMonitor, CycleTimer, FreshnessMonitor,
                                    GripperMonitor, HeartbeatMonitor, cycle_watch_events)

L = AbortLevel


def ctx(state, holding=False, released=False):
    return AbortCtx(state=state, holding=holding, released=released)


@pytest.mark.req("REQ-ABT-008")
@pytest.mark.tc("TC-ABT-008-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("holding, expected", [(False, L.L2), (True, L.L3)])
def test_cycle_ends_when_ros_clock_frozen(fake_clock, holding, expected):
    timeout = 90.0
    timer = CycleTimer(fake_clock, ros_deadline=fake_clock.ros_now() + timeout,
                       steady_limit_s=timeout * 2.0)
    fake_clock.freeze_ros()
    fake_clock.advance(timeout * 2.0 - 0.001)
    assert not timer.expired()
    fake_clock.advance(0.001)
    assert timer.expired()
    state = State.WAIT_CAP if holding else State.DETECT_STONE
    assert decide(Event.CYCLE_TIMEOUT, ctx(state, holding=holding)) == expected


@pytest.mark.req("REQ-ABT-008")
@pytest.mark.tc("TC-ABT-008-01")
@pytest.mark.level("U")
def test_cycle_timer_uses_ros_deadline_when_clock_runs(fake_clock):
    timer = CycleTimer(fake_clock, ros_deadline=fake_clock.ros_now() + 90.0, steady_limit_s=180.0)
    fake_clock.advance(89.999)
    assert not timer.expired()
    fake_clock.advance(0.001)
    assert timer.expired()


@pytest.mark.req("REQ-ABT-009")
@pytest.mark.tc("TC-ABT-009-01")
@pytest.mark.level("U")
def test_heartbeat_ignores_ros_jump(fake_clock):
    hb = HeartbeatMonitor(0.3, fake_clock)
    for _ in range(20):
        hb.beat('vision')
        fake_clock.advance(0.1)
        fake_clock.jump_ros(-5.0)
        assert not hb.lost('vision')


@pytest.mark.req("REQ-ABT-009")
@pytest.mark.tc("TC-ABT-009-01")
@pytest.mark.level("U")
def test_heartbeat_detected_with_frozen_ros(fake_clock):
    hb = HeartbeatMonitor(0.3, fake_clock)
    hb.beat('vision')
    fake_clock.freeze_ros()
    fake_clock.advance(0.3)
    assert hb.lost('vision')


@pytest.mark.req("REQ-ABT-010")
@pytest.mark.tc("TC-ABT-010-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("gap, lost", [(0.299, False), (0.300, True)])
def test_heartbeat_boundary(fake_clock, gap, lost):
    hb = HeartbeatMonitor(0.3, fake_clock)
    hb.beat('tracker')
    fake_clock.advance(gap)
    assert hb.lost('tracker') is lost


@pytest.mark.req("REQ-ABT-010")
@pytest.mark.tc("TC-ABT-010-01")
@pytest.mark.level("U")
def test_heartbeat_unknown_node_is_lost(fake_clock):
    assert HeartbeatMonitor(0.3, fake_clock).lost('vision')


@pytest.mark.req("REQ-ABT-010")
@pytest.mark.tc("TC-ABT-010-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("event, state, holding, expected", [
    (Event.HEARTBEAT_LOST, State.SYNCHRONIZE, True, L.L3),           # vision, 동적 구간
    (Event.HEARTBEAT_LOST, State.DETECT_STONE, False, L.L2),         # vision, 비동적
    (Event.CONTROLLER_HEARTBEAT_LOST, State.DETECT_STONE, False, L.L4),
])
def test_heartbeat_loss_levels(event, state, holding, expected):
    assert decide(event, ctx(state, holding=holding)) == expected


@pytest.mark.req("REQ-ABT-011")
@pytest.mark.tc("TC-ABT-011-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("age, stale", [(0.300, False), (0.301, True)])
def test_observation_freshness(age, stale):
    fm = FreshnessMonitor(0.3)
    t_now = 1234.5
    assert fm.stale(t_now, t_now - age) is stale
    if stale:
        assert decide(Event.OBS_STALE, ctx(State.DESCEND, holding=True)) == L.L3


@pytest.mark.req("REQ-ABT-012")
@pytest.mark.tc("TC-ABT-012-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("holding, expected", [(True, L.L3), (False, L.L2)])
def test_clock_regression(fake_clock, holding, expected):
    cm = ClockMonitor()
    assert not cm.update(fake_clock.ros_now())
    fake_clock.advance(0.1)
    assert not cm.update(fake_clock.ros_now())
    fake_clock.freeze_ros()
    fake_clock.advance(0.1)
    assert not cm.update(fake_clock.ros_now())          # 멈춤은 역행이 아니다
    fake_clock.jump_ros(-0.001)
    assert cm.update(fake_clock.ros_now())
    state = State.WAIT_CAP if holding else State.DETECT_STONE
    assert decide(Event.CLOCK_JUMP, ctx(state, holding=holding)) == expected


@pytest.mark.req("REQ-ABT-014")
@pytest.mark.tc("TC-ABT-014-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("error_bits, response_s, fault", [
    (0, 0.500, False),
    (0, 0.501, True),
    (0x01, 0.100, True),
])
def test_gripper_monitor(fake_clock, error_bits, response_s, fault):
    gm = GripperMonitor(0.5, 0.1, fake_clock)
    gm.update(error_bits, response_s, alive_counter=None)
    assert gm.fault() is fault


@pytest.mark.req("REQ-ABT-014")
@pytest.mark.tc("TC-ABT-014-01")
@pytest.mark.level("U")
def test_gripper_alive_counter_stall(fake_clock):
    gm = GripperMonitor(0.5, 0.1, fake_clock)
    gm.update(0, 0.05, alive_counter=1)
    fake_clock.advance(0.1)
    gm.update(0, 0.05, alive_counter=1)
    assert not gm.fault()
    fake_clock.advance(0.001)
    gm.update(0, 0.05, alive_counter=1)
    assert gm.fault()
    gm.update(0, 0.05, alive_counter=2)
    assert not gm.fault()


@pytest.mark.req("REQ-ABT-014")
@pytest.mark.tc("TC-ABT-014-01")
@pytest.mark.level("U")
def test_gripper_alive_flag_from_robot_status(fake_clock):
    """RobotStatus는 카운터 대신 controller가 판정한 gripper_alive를 준다."""
    gm = GripperMonitor(0.5, 0.1, fake_clock)
    gm.update(0, 0.1, alive=True)
    assert not gm.fault()
    gm.update(0, 0.1, alive=False)
    assert gm.fault() and gm.reason == 'GRIPPER_DEAD'


@pytest.mark.req("REQ-ABT-014")
@pytest.mark.tc("TC-ABT-014-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("holding, expected", [(True, L.L3), (False, L.L2)])
def test_gripper_fault_levels(holding, expected):
    state = State.MOVE_STANDBY if holding else State.PRECHECK
    assert decide(Event.GRIPPER_FAULT, ctx(state, holding=holding)) == expected


# ---------------------------------------------------------------- 벨트 불안정 지속 시간(REQ-ABT-013)
@pytest.mark.req("REQ-ABT-013")
@pytest.mark.tc("TC-ABT-013-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("duration, unsteady", [(0.999, False), (1.000, False), (1.001, True)])
def test_belt_unsteady_needs_persistence(duration, unsteady):
    bm = BeltSteadyMonitor(1.0)
    t0 = 1000.0
    assert not bm.update(True, t0 - 0.1)
    assert not bm.update(False, t0)
    assert bm.update(False, t0 + duration) is unsteady
    assert bm.unsteady() is unsteady


@pytest.mark.req("REQ-ABT-013")
@pytest.mark.tc("TC-ABT-013-01")
@pytest.mark.level("U")
def test_belt_pulses_shorter_than_hold_never_accumulate():
    """0.5 s 맥동 경보가 0.1 s 간격으로 되풀이돼도 지속 시간은 매번 처음부터 센다."""
    bm = BeltSteadyMonitor(1.0)
    t = 1000.0
    for _ in range(5):
        for k in range(16):                       # 0.5 s 동안 false(30 Hz)
            assert not bm.update(False, t + k / 30)
        t += 0.6
        assert not bm.update(True, t)
        t += 0.1


@pytest.mark.req("REQ-ABT-013")
@pytest.mark.tc("TC-ABT-013-01")
@pytest.mark.level("U")
def test_belt_unsteady_restarts_on_stamp_regression():
    bm = BeltSteadyMonitor(1.0)
    bm.update(False, 1000.0)
    assert bm.update(False, 1001.5)
    assert not bm.update(False, 1000.2)           # 시각 역행: 그 stamp부터 다시 센다
    assert bm.update(False, 1001.3)


# ---------------------------------------------------------------- 감시 적용 구간(SPEC-110 4.3절)
S = State
DYN = [S.APPROACH, S.SYNCHRONIZE, S.DESCEND, S.RELEASE]
NOT_WATCHED = [S.IDLE, S.PRECHECK, S.DONE, S.ABORT_CYCLE, S.SAFE_ABORT, S.PROTECTIVE_STOP, S.WAIT_OPERATOR]


@pytest.mark.req("REQ-ABT-013")
@pytest.mark.tc("TC-ABT-013-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("state, released, latched", [
    *[(s, False, True) for s in DYN],
    (S.RETREAT, True, True), (S.VERIFY, True, True),             # 해제 뒤: 이벤트 → CONTINUE_TO_RETREAT_VERIFY
    (S.WAIT_CAP, False, False), (S.MOVE_STANDBY, False, False),  # WAIT_CAP은 판정이 보류한다
    *[(s, False, False) for s in NOT_WATCHED],
])
def test_belt_unsteady_window(state, released, latched):
    events = cycle_watch_events(state, released, belt_unsteady=True)
    assert (Event.BELT_UNSTEADY in events) is latched
    if latched:
        holding = not released
        want = L.CONTINUE_TO_RETREAT_VERIFY if released else L.L3
        assert decide(Event.BELT_UNSTEADY, ctx(state, holding=holding, released=released)) == want


@pytest.mark.req("REQ-ABT-011")
@pytest.mark.tc("TC-ABT-011-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("state, released, latched", [
    *[(s, False, True) for s in DYN],
    (S.RELEASE, True, False), (S.RETREAT, True, False),          # 해제 뒤에는 보지 않는다
    (S.WAIT_CAP, False, False), (S.PLAN_INTERCEPT, False, False),
    *[(s, False, False) for s in NOT_WATCHED],
])
def test_observation_freshness_window(state, released, latched):
    assert (Event.OBS_STALE in cycle_watch_events(state, released, obs_stale=True)) is latched


@pytest.mark.req("REQ-ABT-012", "REQ-ABT-014")
@pytest.mark.tc("TC-ABT-012-01", "TC-ABT-014-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("state", list(State))
def test_clock_and_gripper_window(state):
    """시각 역행·그리퍼는 PRECHECK 다음 정상 열에서만 latch한다. PRECHECK는 자기 판정(로봇 미동작)으로 거부한다."""
    events = cycle_watch_events(state, False, clock_back=True, gripper_fault=True)
    want = [Event.CLOCK_JUMP, Event.GRIPPER_FAULT] if state in WATCH_STATES else []
    assert events == want
    assert (state in WATCH_STATES) is (state not in NOT_WATCHED)
