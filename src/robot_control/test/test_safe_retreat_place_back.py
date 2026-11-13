"""SafeRetreat place-back(SPEC-500 REQ-ROB-016, TEST_SPEC.md TC-ROB-016-01~04). 정적 실행기 + 가짜 backend.

동기 유지 구간은 실제 `dynamic_place.KeepSyncRetreat`를 `SimulatedPlaceTransport`(가정값)로 FakeClock 위에서 돌린다.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from fakes.backend import FakeStaticBackend
from fakes.clock import FakeClock
from robot_control.dynamic_place import DynamicPlaceCfg, KeepSyncRetreat, MotionRef, run_keep_sync_retreat
from robot_control.fake_place_transport import SimulatedPlaceTransport
from robot_control.skill_logic import (MoveOutcome, OpenOutcome, Result, SkillResult, StaticSkillCfg,
                                       StaticSkillExecutor)

DOWN = (0.0, 1.0, 0.0, 0.0)               # 플랜지 z축이 robot_base −z(static_pick_place.plan.DOWN_QUAT_XYZW)
SAFE_Z = 0.25
MAT = (0.40, -0.15, 0.144)                # 매트 반납 TCP 위치(음수 y 포함)
OPEN_W = 0.022


def cfg(**kw):
    base = dict(pose_tolerance_m=0.001, safe_z_m=SAFE_Z, open_width_m=OPEN_W, open_width_tolerance_m=0.001,
                place_back_xyz_m=MAT)
    base.update(kw)
    return StaticSkillCfg(**base)


def rig(start=(0.50, 0.0, 0.20), stopped=True, **kw):
    clock = FakeClock()
    backend = FakeStaticBackend(tcp_pose=(start, DOWN))
    backend.stopped = stopped
    backend.open = OpenOutcome(True, OPEN_W + 0.0004)
    return StaticSkillExecutor(backend, cfg(**kw), clock), backend, clock


def moves(backend):
    return [np.round(p, 6).tolist() for kind, p in backend.calls if kind == 'move_line']


def phases(fbs):
    return [(p, d) for p, _, d in fbs]


def recorder():
    fbs = []
    return fbs, (lambda phase, metrics, done: fbs.append((phase, dict(metrics), done)))


class GoalPose:
    """geometry_msgs/Pose 모양(기본값 = 원점, quaternion w=1)."""

    def __init__(self, xyz=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0)):
        self.position = type('P', (), dict(zip('xyz', xyz)))()
        self.orientation = type('Q', (), dict(zip('xyzw', quat)))()


# ------------------------------------------------------------------ TC-ROB-016-01 정상 순서
@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-01')
@pytest.mark.level('U')
def test_static_place_back_sequence_uses_measured_values():
    ex, backend, _ = rig()
    fbs, fb = recorder()
    res = ex.safe_retreat(keep_sync=False, place_back=True, feedback=fb)
    assert (res.result, res.reason_code) == (Result.OK, ''), res
    assert res.released is True                                     # stone_placed_back
    # 안전 높이로 올림 → 안전 높이에서 매트 위로 → 내려감 → 열기 → 안전 높이로 올림
    assert moves(backend) == [[0.5, 0.0, SAFE_Z], [MAT[0], MAT[1], SAFE_Z], list(MAT), [MAT[0], MAT[1], SAFE_Z]]
    assert [c for c in backend.calls if c[0] == 'gripper_open'] == [('gripper_open', OPEN_W)]
    assert [c[0] for c in backend.calls].index('gripper_open') == 3
    # 자세는 측정 자세를 유지한다(항등 quaternion은 그리퍼 아래 방향이 아니다)
    assert all(np.allclose(q, DOWN) for q in backend.quats)
    assert phases(fbs) == [('RETREAT', False), ('RETREAT', True), ('PLACE_BACK', False), ('PLACE_BACK', True)]
    assert res.metrics['open_width_m'] == pytest.approx(OPEN_W + 0.0004)
    assert res.metrics['tcp_z_m'] == pytest.approx(SAFE_Z)
    assert not ex.authority.busy


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-01')
@pytest.mark.level('U')
def test_goal_pose_overrides_parameter_only_when_position_is_nonzero():
    ex, backend, _ = rig()
    res = ex.safe_retreat(False, True, GoalPose())                   # 기본 Pose(원점, w=1) = 지정 없음
    assert res.result == Result.OK
    assert moves(backend)[2] == list(MAT)
    assert all(np.allclose(q, DOWN) for q in backend.quats)

    goal_q = (0.0, 0.7071067811865476, 0.7071067811865476, 0.0)
    ex, backend, _ = rig()
    res = ex.safe_retreat(False, True, GoalPose((0.35, 0.10, 0.15), goal_q))
    assert res.result == Result.OK
    assert moves(backend)[1:] == [[0.35, 0.1, SAFE_Z], [0.35, 0.1, 0.15], [0.35, 0.1, SAFE_Z]]
    assert np.allclose(backend.quats[0], DOWN)                       # 첫 상승은 측정 자세
    assert all(np.allclose(q, goal_q) for q in backend.quats[1:])


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-01')
@pytest.mark.level('U')
def test_keep_sync_leg_runs_keep_sync_retreat_then_places_back():
    # 해제 전 abort(keep_sync=true)로 계획 속도를 유지하던 TCP에서 시작한다(TC-ROB-010-01과 같은 상황)
    ex, backend, clock = rig(start=(0.50, 0.0, 0.20), stopped=False)
    transport = SimulatedPlaceTransport(clock, robot=backend)
    v = (0.02, 0.0)
    for _ in range(20):                                              # 추종 중(주기마다 계획 속도 지령)
        transport.update(MotionRef(clock.ros_now(), (0.5, 0.0, 0.2), (*v, 0.0), (*v, 0.0)))
        clock.advance(0.01)
        transport.poll()
    transport.abort(True)
    dyn = DynamicPlaceCfg(safe_z_m=SAFE_Z)
    used = []

    def keep_sync(feedback):
        retreat = KeepSyncRetreat(transport, v, dyn)
        used.append(retreat)
        return run_keep_sync_retreat(retreat, clock, clock.advance, feedback=feedback)

    fbs, fb = recorder()
    res = ex.safe_retreat(keep_sync=True, place_back=True, feedback=fb, keep_sync_retreat=keep_sync)
    assert (res.result, res.reason_code) == (Result.OK, ''), res
    assert used and used[0].errors and used[0].speed_ok()           # 안전 높이까지 표본 XY 속도 ±1 mm/s
    assert transport.updates > 10
    m = moves(backend)
    assert m == [[MAT[0], MAT[1], SAFE_Z], list(MAT), [MAT[0], MAT[1], SAFE_Z]]   # 동기 유지 상승 뒤 정적 이동
    assert phases(fbs) == [('RETREAT', False), ('RETREAT', True), ('PLACE_BACK', False), ('PLACE_BACK', True)]
    assert res.released is True


# ------------------------------------------------------------------ TC-ROB-016-02 시작 조건(지령 없음)
@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('kw,goal,reason', [
    (dict(place_back_xyz_m=None), None, 'PLACE_BACK_POSE_UNSET'),
    (dict(place_back_xyz_m=None), GoalPose(), 'PLACE_BACK_POSE_UNSET'),
    (dict(safe_z_m=None), None, 'SAFE_Z_UNSET'),
    (dict(open_width_m=None), None, 'OPEN_WIDTH_UNSET'),
    (dict(open_width_tolerance_m=None), None, 'OPEN_WIDTH_TOLERANCE_UNSET'),
    (dict(pose_tolerance_m=None), None, 'POSE_TOLERANCE_UNSET'),
])
@pytest.mark.parametrize('keep_sync', [False, True])
def test_missing_settings_precondition_without_motion(kw, goal, reason, keep_sync):
    ex, backend, _ = rig(**kw)
    runs = []
    res = ex.safe_retreat(keep_sync, True, goal, keep_sync_retreat=lambda fb: runs.append(1))
    assert (res.result, res.reason_code) == (Result.PRECONDITION, reason)
    assert backend.calls == [] and runs == []
    assert not ex.authority.busy


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-02')
@pytest.mark.level('U')
def test_keep_sync_without_transport_and_invalid_goal_pose_do_not_move():
    ex, backend, _ = rig()
    res = ex.safe_retreat(keep_sync=True, place_back=True)
    assert (res.result, res.reason_code) == (Result.PRECONDITION, 'SYNC_NOT_AVAILABLE')
    for goal in (GoalPose((0.3, 0.0, 0.1), (0.0, 0.0, 0.0, 0.0)),          # quaternion 0
                 GoalPose((0.3, math.nan, 0.1)),                            # 비유한
                 GoalPose((0.3, 0.0, SAFE_Z), DOWN)):                       # 안전 높이 이상 = 내려가지 않음
        res = ex.safe_retreat(False, True, goal)
        assert (res.result, res.reason_code) == (Result.FAILED, 'RANGE')
    ex2, backend2, _ = rig(place_back_xyz_m=(0.4, 0.0, SAFE_Z + 0.01))
    assert ex2.safe_retreat(False, True).reason_code == 'RANGE'
    assert backend.calls == [] and backend2.calls == []


# ------------------------------------------------------------------ TC-ROB-016-03 측정값 판정
@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-03')
@pytest.mark.level('U')
@pytest.mark.parametrize('opened,reason', [
    (OpenOutcome(True, OPEN_W + 0.0011), 'OPEN_WIDTH'),
    (OpenOutcome(True, None), 'OPEN_FAILED'),
    (OpenOutcome(False, None, 'GRIPPER_TIMEOUT'), 'GRIPPER_TIMEOUT'),
])
def test_open_width_must_be_measured_within_tolerance(opened, reason):
    ex, backend, _ = rig()
    backend.open = opened
    res = ex.safe_retreat(False, True)
    assert (res.result, res.reason_code) == (Result.FAILED, reason)
    assert not res.released                                          # 반납했다고 보고하지 않는다
    assert len(moves(backend)) == 3                                  # 열기 실패 뒤 더 움직이지 않는다


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-03')
@pytest.mark.level('U')
def test_pose_error_over_tolerance_fails_before_opening():
    ex, backend, _ = rig()
    backend.move_error_m = 0.0011
    res = ex.safe_retreat(False, True)
    assert (res.result, res.reason_code) == (Result.FAILED, 'POSE_ERROR')
    assert 'gripper_open' not in [c[0] for c in backend.calls] and not res.released


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-03')
@pytest.mark.level('U')
@pytest.mark.parametrize('leg', [SkillResult(Result.FAILED, 'SYNC_SPEED', {'tcp_z_m': SAFE_Z}),
                                 SkillResult(Result.PRECONDITION, 'SYNC_NOT_AVAILABLE'),
                                 SkillResult(Result.FAILED, 'RETREAT_TIMEOUT')])
def test_keep_sync_failure_stops_before_place_back(leg):
    ex, backend, _ = rig()
    res = ex.safe_retreat(True, True, keep_sync_retreat=lambda fb: leg)
    assert (res.result, res.reason_code) == (leg.result, leg.reason_code)
    assert backend.calls == [] and not res.released


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-03')
@pytest.mark.level('U')
def test_retreat_failure_after_open_keeps_placed_back():
    ex, backend, _ = rig()
    original = backend.move_line

    def fail_last(pos, quat):
        out = original(pos, quat)
        if 'gripper_open' in [c[0] for c in backend.calls]:
            return MoveOutcome(False, math.inf, 'MOVE_FAILED')
        return out
    backend.move_line = fail_last
    res = ex.safe_retreat(False, True)
    assert (res.result, res.reason_code) == (Result.FAILED, 'MOVE_FAILED')
    assert res.released is True                                      # 측정 개방 뒤이므로 매트에 놓았다


# ------------------------------------------------------------------ TC-ROB-016-04 cancel·deadline
STEPS = ('RETREAT', 'TRANSIT', 'DESCEND', 'RELEASE', 'RISE')      # backend 호출 순서: 이동 3회, 열기, 이동
CASES = [(s, w) for s in STEPS for w in ('before', 'during') if (s, w) != ('RELEASE', 'during')]


@pytest.mark.req('REQ-ROB-016', 'REQ-ROB-007')
@pytest.mark.tc('TC-ROB-016-04')
@pytest.mark.level('U')
@pytest.mark.parametrize('step,when', CASES)
@pytest.mark.parametrize('why', ['cancel', 'deadline'])
def test_cancel_and_deadline_at_each_step(step, when, why):
    ex, backend, clock = rig()
    k = STEPS.index(step)
    flag = {'on': False}
    events = {'n': 0}
    move0, open0 = backend.move_line, backend.gripper_open

    def trip():
        flag['on'] = True
        clock.advance(1.0)                                           # deadline(시작 + 0.5 s)도 지난다

    def call(real, *args):
        i = events['n']
        events['n'] += 1
        if i == k and when == 'during':
            trip()
            return MoveOutcome(False, math.inf, 'CANCELED')          # backend가 cancel·deadline으로 이동을 끊음
        out = real(*args)
        if i + 1 == k and when == 'before':
            trip()                                                   # 다음 단계 시작 전에 요청
        return out
    backend.move_line = lambda pos, quat: call(move0, pos, quat)
    backend.gripper_open = lambda width: call(open0, width)
    deadline = clock.ros_now() + 0.5 if why == 'deadline' else None
    if k == 0 and when == 'before':
        trip()                                                       # goal 시작 때 이미 요청·만료
    cancel = (lambda: flag['on']) if why == 'cancel' else None
    res = ex.safe_retreat(False, True, None, deadline, cancel)
    if (step, when) == ('RISE', 'before'):
        # 개방 확인 뒤의 요청은 상승을 막지 않는다(PlaceStone과 같음). 상승을 마치면 OK·반납
        assert (res.result, res.reason_code, res.released) == (Result.OK, '', True)
        assert events['n'] == 5
    else:
        expected = (Result.CANCELED, 'CANCELED') if why == 'cancel' else (Result.TIMEOUT, 'DEADLINE')
        assert (res.result, res.reason_code) == expected, res
        assert bool(res.released) == (k == 4)                        # 개방 확인 뒤에만 반납
        assert events['n'] == (k + 1 if when == 'during' else k)     # 요청 뒤 새 지령 없음
    assert not ex.authority.busy


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-03')
@pytest.mark.level('U')
def test_unreadable_tcp_never_becomes_a_target():
    # 측정 자세를 못 읽으면(NaN) 그 자세로 움직이지 않는다
    ex, backend, _ = rig(start=(0.50, 0.0, SAFE_Z + 0.01))             # 후퇴 이동 없음
    backend.tcp_quat = np.full(4, np.nan)
    res = ex.safe_retreat(False, True)
    assert (res.result, res.reason_code) == (Result.FAILED, 'TCP_POSE_INVALID')
    assert moves(backend) == []
    # 개방 확인 뒤 TCP를 못 읽으면 반납 위치 XY에서 올린다(반납 사실은 유지)
    ex, backend, _ = rig()
    original = backend.gripper_open

    def open_then_lose_tcp(width):
        out = original(width)
        backend.tcp_pos = np.full(3, np.nan)
        return out
    backend.gripper_open = open_then_lose_tcp
    res = ex.safe_retreat(False, True)
    assert (res.result, res.released) == (Result.OK, True)
    assert moves(backend)[-1] == [MAT[0], MAT[1], SAFE_Z]
