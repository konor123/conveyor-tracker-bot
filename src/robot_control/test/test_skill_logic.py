"""정적 skill 판단, cancel·deadline·Stop, 동시 goal 거부, transport 없는 DynamicPlace·동기 유지."""
from __future__ import annotations

import math

import numpy as np
import pytest

from fakes.backend import FakeStaticBackend
from fakes.clock import FakeClock
from robot_control.skill_logic import (
    DisabledBackend,
    GripOutcome,
    MoveOutcome,
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


def test_named_pose_and_missing_config_precondition():
    ex, _ = executor()
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT, named_pose='standby').result == Result.OK
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT, named_pose='nope').reason_code == 'NAMED_POSE_UNKNOWN'

    ex, _ = executor(pose_tolerance_m=None)
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT).reason_code == 'POSE_TOLERANCE_UNSET'
    assert ex.pick_stone([0.1, 0.0, 0.0], 0.05, 100).reason_code == 'POSE_TOLERANCE_UNSET'
    assert ex.place_stone(([0.1, 0.0, 0.0], QUAT), 0.01, 0.021).reason_code == 'POSE_TOLERANCE_UNSET'


def test_deadline_and_cancel_paths():
    ex, clock = executor()
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT, deadline_s=clock.ros_now()).result == Result.TIMEOUT
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT,
                           cancel_requested=lambda: True).result == Result.CANCELED


@pytest.mark.parametrize('skill', ['move', 'pick', 'place', 'retreat'])
@pytest.mark.parametrize('why', ['cancel', 'deadline'])
def test_interrupted_move_reports_cancel_or_deadline(skill, why):
    # backend가 cancel·deadline으로 이동을 끊으면(완료 아님) FAILED가 아니라 CANCELED·TIMEOUT이다
    backend = FakeStaticBackend()
    backend.tcp_pos[2] = 0.05                          # SafeRetreat가 실제로 움직이게 안전 높이 아래
    ex, clock = executor(backend=backend)
    flag = {'on': False}

    def interrupted(pos, quat):
        flag['on'] = True
        clock.advance(1.0)
        return MoveOutcome(False, math.inf, 'CANCELED')
    backend.move_line = interrupted
    cancel = (lambda: flag['on']) if why == 'cancel' else None
    deadline = clock.ros_now() + 0.5 if why == 'deadline' else None
    call = {'move': lambda: ex.move_to_pose([0.1, 0.0, 0.1], QUAT, deadline, cancel),
            'pick': lambda: ex.pick_stone([0.1, 0.0, 0.0], 0.05, 50, deadline, cancel),
            'place': lambda: ex.place_stone(([0.1, 0.0, 0.1], QUAT), 0.01, 0.021, deadline, cancel),
            'retreat': lambda: ex.safe_retreat(False, False, None, deadline, cancel)}[skill]
    res = call()
    assert (res.result, res.reason_code) == ((Result.CANCELED, 'CANCELED') if why == 'cancel'
                                             else (Result.TIMEOUT, 'DEADLINE'))
    assert not ex.authority.busy


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


def test_inactive_lifecycle_rejects_skills():
    ex, _ = executor(active=False)
    for method in (ex.move_to_pose,):
        result = method([0.0, 0.0, 0.0], QUAT)
        assert result.result == Result.PRECONDITION
        assert result.reason_code == 'LIFECYCLE_INACTIVE'


def test_disabled_backend_never_reports_success():
    clock = FakeClock()
    ex = StaticSkillExecutor(None, cfg(), clock)
    assert isinstance(ex.backend, DisabledBackend)
    assert ex.move_to_pose([0.0, 0.0, 0.0], QUAT).reason_code == 'BACKEND_DISABLED'
    assert ex.stop().result == Result.FAULT


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


def test_pick_missing_grip_criteria_precondition():
    ex, _ = executor(grip_min_width_m=None)
    assert ex.pick_stone([0.1, 0.0, 0.0], 0.05, 100).reason_code == 'GRIP_CRITERIA_UNSET'
    ex, _ = executor()
    assert ex.pick_stone([0.1, 0.0, 0.0], None, 100).reason_code == 'APPROACH_HEIGHT_UNSET'
    assert ex.pick_stone([0.1, 0.0, 0.0], 0.05, 151).reason_code == 'CURRENT_OUT_OF_RANGE'


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


def test_safe_retreat_without_keep_sync_transport_fails_closed():
    ex, _ = executor()
    assert ex.safe_retreat(keep_sync=True, place_back=False).reason_code == 'SYNC_NOT_AVAILABLE'
    result = ex.safe_retreat(keep_sync=False, place_back=False)
    assert result.result == Result.OK
    assert result.metrics['tcp_z_m'] == pytest.approx(0.15)


def test_dynamic_place_fails_closed_until_ready():
    ex, _ = executor()
    result = ex.dynamic_place()
    assert result.result == Result.PRECONDITION
    assert result.reason_code == 'PLANNER_NOT_READY'


@pytest.mark.parametrize('limit_ma, ok', [(0, False), (1, True), (150, True), (151, False)])
def test_pick_grip_current_limit_range_is_1_to_150(limit_ma, ok):
    """PickStone `grip_current_limit` 허용 범위는 그리퍼 전류 허용 1~150 mA다. 150 mA goal을 거부하지 않는다."""
    backend = FakeStaticBackend()
    ex, _ = executor(backend=backend)
    result = ex.pick_stone([0.1, 0.0, 0.0], 0.05, limit_ma)
    if ok:
        assert (result.result, result.holding) == (Result.OK, True), result
        assert ('gripper_close', limit_ma) in backend.calls
    else:
        assert (result.result, result.reason_code) == (Result.PRECONDITION, 'CURRENT_OUT_OF_RANGE')
        assert backend.calls == []


def test_place_stone_retreat_rises_to_safe_z_and_reports_measured_height():
    """PlaceStone RETREAT는 개방 뒤 측정 XY에서 `safe_z_m`까지 수직으로 올리고, `tcp_z_m`은 측정 TCP z,
    `outside_workspace`는 측정 z ≥ `safe_z_m`일 때만 1이다(RETREAT 판정과 같은 기준)."""
    backend = FakeStaticBackend()
    ex, _ = executor(backend=backend)                      # safe_z_m 0.15
    result = ex.place_stone(([0.1, 0.02, 0.03], QUAT), 0.002, 0.021)
    assert result.result == Result.OK, result
    moves = [v for k, v in backend.calls if k == 'move_line']
    assert moves[-1] == pytest.approx([0.1, 0.02, 0.15])     # 마지막 이동 = 수직 상승
    assert result.metrics['tcp_z_m'] == pytest.approx(0.15)
    assert (result.metrics['outside_workspace'], result.metrics['stone_in_gripper']) == (1.0, 0.0)

    # 상승을 받고도 안전 높이에 못 미치면(측정 z) 작업영역 안으로 보고하고 성공으로 만들지 않는다
    class ShortRise(FakeStaticBackend):
        def move_line(self, target_pos_m, target_quat_xyzw):
            out = super().move_line(target_pos_m, target_quat_xyzw)
            if target_pos_m[2] >= 0.15:
                self.tcp_pos[2] = 0.12
            return out
    backend = ShortRise()
    ex, _ = executor(backend=backend)
    result = ex.place_stone(([0.1, 0.02, 0.03], QUAT), 0.002, 0.021)
    assert (result.result, result.reason_code, result.released) == (Result.FAILED, 'RETREAT_FAILED', True)
    assert result.metrics['tcp_z_m'] == pytest.approx(0.12)
    assert result.metrics['outside_workspace'] == 0.0

    # 안전 높이가 없으면 움직이기 전에 거부한다
    backend = FakeStaticBackend()
    ex, _ = executor(backend=backend, safe_z_m=None)
    result = ex.place_stone(([0.1, 0.02, 0.03], QUAT), 0.002, 0.021)
    assert (result.result, result.reason_code) == (Result.PRECONDITION, 'SAFE_Z_UNSET')
    assert backend.calls == []


def test_place_stone_release_reports_measured_hold_at_target():
    """PlaceStone RELEASE `sync_held`(정지 목표의 동기 유지 = 개방하는 동안 목표 위치 유지)는 개방 전·후 측정 TCP가
    둘 다 목표에서 `pose_tolerance_m` 안일 때만 1이다. 값이 0이어도 개방 뒤이므로 후퇴는 계속한다."""
    backend = FakeStaticBackend()
    ex, _ = executor(backend=backend)
    result = ex.place_stone(([0.1, 0.02, 0.03], QUAT), 0.002, 0.021)
    assert result.result == Result.OK and result.metrics['sync_held'] == 1.0

    class DriftOnOpen(FakeStaticBackend):
        def gripper_open(self, width_m):
            self.tcp_pos = self.tcp_pos + np.array([0.0015, 0.0, 0.0])      # 개방 중 1.5 mm 밀림(허용 1 mm)
            return super().gripper_open(width_m)
    backend = DriftOnOpen()
    ex, _ = executor(backend=backend)
    result = ex.place_stone(([0.1, 0.02, 0.03], QUAT), 0.002, 0.021)
    assert result.metrics['sync_held'] == 0.0
    assert result.released is True and result.metrics['tcp_z_m'] == pytest.approx(0.15)
