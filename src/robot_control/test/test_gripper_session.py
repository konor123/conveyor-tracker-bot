"""공통 그리퍼 순서(세션 1회 시작, 이미 열림 생략, 예약 뒤 트리거)."""
from __future__ import annotations

import pytest

from fakes.clock import FakeClock
from fakes.transport import FakeTransport
from robot_control.gripper_adapter import (
    RESULT_OK,
    GripperAdapter,
    GripperCfg,
    PreconditionError,
    PulseTable,
)
from robot_control.gripper_session import GripperSession

PULSE_MAX = 1150
OPEN_M = 0.106            # 선형 가정표에서 0 pulse
RELEASE_M = 0.019


def build(started: bool = True):
    clock = FakeClock(steady=100.0)
    transport = FakeTransport(clock=clock, move_delay_s=0.02)
    table = PulseTable.from_samples([(p, 0.106 * (1.0 - p / PULSE_MAX)) for p in range(0, PULSE_MAX + 1, 50)])
    cfg = GripperCfg(close_pulse=1150, response_timeout_s=0.5, read_period_s=0.01)
    adapter = GripperAdapter(transport, table, cfg, clock, sleep=clock.advance)
    session = GripperSession(adapter, width_tolerance_m=0.001)
    if started:
        session.start(robot_standby=True)
    return session, transport


def test_session_starts_once_and_rejects_use_before_start(monkeypatch):
    session, transport = build(started=False)
    for call in (lambda: session.ensure_open(OPEN_M), lambda: session.arm_open(RELEASE_M),
                 session.fire_open, lambda: session.grasp(100)):
        with pytest.raises(PreconditionError) as err:
            call()
        assert err.value.reason == 'SESSION_NOT_STARTED'
    assert transport.writes == [] and transport.triggers == 0

    with pytest.raises(PreconditionError):
        session.start(robot_standby=False)
    assert transport.start_loop_calls == 0 and not session.started

    checks = []
    original = session.adapter.startup_check
    monkeypatch.setattr(session.adapter, 'startup_check', lambda standby=None: (checks.append(1), original(standby)))
    session.start(robot_standby=True)
    session.start(robot_standby=True)
    assert transport.start_loop_calls == 1 and len(checks) == 1 and session.started


def test_ensure_open_skips_when_already_at_width():
    session, transport = build()
    assert session.ensure_open(OPEN_M) is None                 # 이미 열려 있다
    assert transport.writes == [] and transport.triggers == 0
    assert session.events[-1].skipped

    result = session.ensure_open(RELEASE_M)                    # 폭이 다르면 실제로 보낸다
    assert result.result == RESULT_OK
    assert transport.triggers == 1 and transport.writes
    assert not session.events[-1].skipped

    writes = len(transport.writes)
    assert session.ensure_open(RELEASE_M) is None              # 같은 폭을 다시 요구하면 생략
    assert len(transport.writes) == writes and transport.triggers == 1
    assert session.summary()['commands'] == 1 and session.summary()['skipped'] == 2


def test_arm_then_fire_sends_only_the_trigger():
    session, transport = build()
    session.arm_open(RELEASE_M)
    assert session.armed and transport.triggers == 0           # 예약만으로는 움직이지 않는다
    writes = len(transport.writes)
    assert writes > 0

    result = session.fire_open()
    assert result.result == RESULT_OK
    assert transport.triggers == 1 and len(transport.writes) == writes    # 해제 순간 쓰기는 트리거뿐
    assert result.width_m == pytest.approx(RELEASE_M, abs=0.001)
    assert not session.armed

    with pytest.raises(PreconditionError) as err:              # 예약 없이 쏘지 않는다
        session.fire_open()
    assert err.value.reason == 'NOT_ARMED'
    assert transport.triggers == 1


def test_grasp_invalidates_an_earlier_arm():
    session, transport = build()
    session.arm_open(RELEASE_M)
    session.grasp(100)                                         # 목표 레지스터를 닫힘 값으로 덮어쓴다
    with pytest.raises(PreconditionError):
        session.fire_open()
    assert transport.triggers == 1                             # 닫기 1회뿐

    # adapter 단독으로도 같은 순서에서 닫힘 값을 트리거하지 않는다
    adapter = session.adapter
    adapter.prewrite_open(OPEN_M)
    adapter.close(100)
    adapter.open(OPEN_M, prewritten=True)
    assert transport.goal_position == 0 and transport.present_pulse == 0
