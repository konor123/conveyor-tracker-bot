import pytest

from orchestrator.abort_policy import AbortCtx, Event, decide
from orchestrator.common import AbortLevel, State
from orchestrator.watchdogs import (ClockMonitor, CycleTimer, FreshnessMonitor, GripperMonitor,
                                    HeartbeatMonitor)

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
@pytest.mark.parametrize("holding, expected", [(True, L.L3), (False, L.L2)])
def test_gripper_fault_levels(holding, expected):
    state = State.MOVE_STANDBY if holding else State.PRECHECK
    assert decide(Event.GRIPPER_FAULT, ctx(state, holding=holding)) == expected
