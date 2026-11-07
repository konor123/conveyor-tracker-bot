"""TC-ROB-007·008·009·010·012: 정적 skill 판단, cancel·deadline·Stop, 동시 goal 거부."""
from __future__ import annotations

import math

import pytest

from fakes.backend import FakeStaticBackend
from fakes.clock import FakeClock
from robot_control.skill_logic import (
    DisabledBackend,
    GripOutcome,
    OpenOutcome,
    Result,
    SingleCommandAuthority,
    StaticSkillCfg,
    StaticSkillExecutor,
)

QUAT = [0.0, 0.0, 0.0, 1.0]


def cfg(**kw):
    base = dict(
        pose_tolerance_m=0.001,
        approach_height_m=0.05,
        safe_z_m=0.15,
        open_width_tolerance_m=0.001,
        release_rim_clearance_m=0.002,
        grip_min_width_m=0.016,
        grip_max_width_m=0.019,
        grip_min_current_ma=10.0,
        named_poses={'standby': ((0.2, 0.0, 0.2), QUAT)},
    )
    base.update(kw)
    return StaticSkillCfg(**base)


def executor(backend=None, active=True, **kw):
    clock = FakeClock()
    return StaticSkillExecutor(backend or FakeStaticBackend(), cfg(**kw), clock, active=active), clock


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-01')
@pytest.mark.level('U')
def test_move_to_pose_uses_measured_pose_error():
    ex, _ = executor()
    result = ex.move_to_pose([0.1, 0.0, 0.1], QUAT)
    assert result.result == Result.OK
    assert result.metrics['pose_error_m'] == pytest.approx(0.0005)

    backend = FakeStaticBackend()
    backend.move_error_m = 0.01
    ex, _ = executor(backend=backend)
    result = ex.move_to_pose([0.1, 0.0, 0.1], QUAT)
    assert result.result == Result.FAILED
    assert result.reason_code == 'POSE_ERROR'


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-01')
@pytest.mark.level('U')
def test_named_pose_and_missing_config_precondition():
    ex, _ = executor()
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT, named_pose='standby').result == Result.OK
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT, named_pose='nope').reason_code == 'NAMED_POSE_UNKNOWN'

    ex, _ = executor(pose_tolerance_m=None)
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT).reason_code == 'POSE_TOLERANCE_UNSET'
    assert ex.pick_stone([0.1, 0.0, 0.0], 0.05, 100).reason_code == 'POSE_TOLERANCE_UNSET'
    assert ex.place_stone(([0.1, 0.0, 0.0], QUAT), 0.01, 0.021).reason_code == 'POSE_TOLERANCE_UNSET'


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-02')
@pytest.mark.level('U')
def test_deadline_and_cancel_paths():
    ex, clock = executor()
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT, deadline_s=clock.ros_now()).result == Result.TIMEOUT
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT,
                           cancel_requested=lambda: True).result == Result.CANCELED


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-02')
@pytest.mark.level('U')
def test_stop_requires_confirmed_feedback():
    backend = FakeStaticBackend()
    backend.stopped = False
    ex, _ = executor(backend=backend)
    result = ex.stop()
    assert result.result == Result.FAULT
    assert result.stopped is False
    assert result.reason_code == 'STOP_NOT_CONFIRMED'

    backend = FakeStaticBackend()
    ex, _ = executor(backend=backend)
    result = ex.stop()
    assert result.result == Result.OK
    assert result.stopped is True
    assert backend.stop_sent


@pytest.mark.req('REQ-ROB-005')
@pytest.mark.tc('TC-ROB-005-01')
@pytest.mark.level('U')
def test_concurrent_goal_rejected():
    auth = SingleCommandAuthority()
    assert auth.acquire('first') is True
    assert auth.acquire('second') is False
    auth.release('first')
    assert auth.acquire('second') is True

    ex, _ = executor()
    ex.authority.acquire('other')
    result = ex.move_to_pose([0.0, 0.0, 0.0], QUAT)
    assert result.result == Result.PRECONDITION
    assert result.reason_code == 'BUSY'


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-01')
@pytest.mark.level('U')
def test_inactive_lifecycle_rejects_skills():
    ex, _ = executor(active=False)
    for method in (ex.move_to_pose,):
        result = method([0.0, 0.0, 0.0], QUAT)
        assert result.result == Result.PRECONDITION
        assert result.reason_code == 'LIFECYCLE_INACTIVE'


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-01')
@pytest.mark.level('U')
def test_disabled_backend_never_reports_success():
    clock = FakeClock()
    ex = StaticSkillExecutor(None, cfg(), clock)
    assert isinstance(ex.backend, DisabledBackend)
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT).reason_code == 'BACKEND_DISABLED'
    assert ex.stop().result == Result.FAULT


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-01')
@pytest.mark.level('U')
def test_pick_success_requires_measured_holding():
    ex, _ = executor()
    result = ex.pick_stone([0.1, 0.0, 0.0], 0.05, 100)
    assert result.result == Result.OK
    assert result.holding is True

    backend = FakeStaticBackend()
    backend.grip = GripOutcome(None, None, None)
    ex, _ = executor(backend=backend)
    result = ex.pick_stone([0.1, 0.0, 0.0], 0.05, 100)
    assert result.result == Result.FAILED
    assert result.reason_code == 'GRIP_MEASUREMENT_MISSING'

    backend = FakeStaticBackend()
    backend.grip = GripOutcome(0.030, 40.0, True)
    ex, _ = executor(backend=backend)
    result = ex.pick_stone([0.1, 0.0, 0.0], 0.05, 100)
    assert result.result == Result.FAILED
    assert result.reason_code == 'GRIP_FAILED'


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-01')
@pytest.mark.level('U')
def test_pick_missing_grip_criteria_precondition():
    ex, _ = executor(grip_min_width_m=None)
    assert ex.pick_stone([0.1, 0.0, 0.0], 0.05, 100).reason_code == 'GRIP_CRITERIA_UNSET'
    ex, _ = executor()
    assert ex.pick_stone([0.1, 0.0, 0.0], None, 100).reason_code == 'APPROACH_HEIGHT_UNSET'
    assert ex.pick_stone([0.1, 0.0, 0.0], 0.05, 101).reason_code == 'CURRENT_OUT_OF_RANGE'


@pytest.mark.req('REQ-ROB-009')
@pytest.mark.tc('TC-ROB-009-01')
@pytest.mark.level('U')
def test_place_reads_measured_open_width():
    ex, _ = executor()
    result = ex.place_stone(([0.1, 0.0, 0.1], QUAT), 0.01, 0.021)
    assert result.result == Result.OK
    assert result.released is True
    assert result.metrics['open_width_m'] == pytest.approx(0.021)

    backend = FakeStaticBackend()
    backend.open = OpenOutcome(True, 0.030)
    ex, _ = executor(backend=backend)
    result = ex.place_stone(([0.1, 0.0, 0.1], QUAT), 0.01, 0.021)
    assert result.result == Result.FAILED
    assert result.reason_code == 'OPEN_WIDTH'

    # 개방 확인이 없으면 성공으로 만들지 않는다.
    backend = FakeStaticBackend()
    backend.open = OpenOutcome(False, None, 'OPEN_FAILED')
    ex, _ = executor(backend=backend)
    assert ex.place_stone(([0.1, 0.0, 0.1], QUAT), 0.01, 0.021).result == Result.FAILED


@pytest.mark.req('REQ-ROB-010')
@pytest.mark.tc('TC-ROB-010-01')
@pytest.mark.level('U')
def test_safe_retreat_unsupported_modes_fail_closed():
    ex, _ = executor()
    assert ex.safe_retreat(keep_sync=True, place_back=False).reason_code == 'SYNC_NOT_AVAILABLE'
    assert ex.safe_retreat(keep_sync=False, place_back=True).reason_code == 'PLACE_BACK_UNSUPPORTED'
    result = ex.safe_retreat(keep_sync=False, place_back=False)
    assert result.result == Result.OK
    assert result.metrics['tcp_z_m'] == pytest.approx(0.15)


@pytest.mark.req('REQ-ROB-008')
@pytest.mark.tc('TC-ROB-008-01')
@pytest.mark.level('U')
def test_dynamic_place_fails_closed_until_ready():
    ex, _ = executor()
    result = ex.dynamic_place()
    assert result.result == Result.PRECONDITION
    assert result.reason_code == 'PLANNER_NOT_READY'
