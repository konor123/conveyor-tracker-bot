"""정적 skill 실행 판단(순수 Python). ROS 노드는 이 모듈을 감싸기만 한다.

성공 판정은 보낸 지령이 아니라 backend가 돌려준 **측정값**으로만 한다. 설정이
없거나 backend가 비활성·미구현이면 `PRECONDITION`으로 닫고, `DynamicPlace`는
계획기·추종 제어가 준비될 때까지 `PRECONDITION`(`PLANNER_NOT_READY`)이다.

시간은 float 초(ROS 시각)로 받는다. 하드웨어 단위 변환은 `motion_adapter.py`와
`gripper_adapter.py` 밖에서 하지 않는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import IntEnum
from typing import Callable, Protocol

import numpy as np


class Result(IntEnum):
    OK = 0
    FAILED = 1
    TIMEOUT = 2
    PRECONDITION = 3
    CANCELED = 4
    FAULT = 5


@dataclass(frozen=True)
class SkillResult:
    result: Result
    reason_code: str = ''
    metrics: dict[str, float] = field(default_factory=dict)
    stopped: bool | None = None
    holding: bool | None = None
    released: bool | None = None


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name}: 유한하지 않은 값')
    return value


def _vec3(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != 3 or not np.all(np.isfinite(arr)):
        raise ValueError(f'{name}: 유한한 길이 3 필요')
    return arr


# backend 이동 결과 중 cancel·deadline이어도 FAULT로 보고하는 이유(정지 미확인)
FAULT_MOVE_REASONS = frozenset({'STOP_NOT_CONFIRMED'})
# PickStone `grip_current_limit` 허용 범위 [mA]: 그리퍼 전류 허용 1~150
GRIP_CURRENT_LIMIT_MA = (1, 150)


def _deadline_passed(deadline_s: float | None, now_s: float) -> bool:
    return deadline_s is not None and deadline_s > 0.0 and now_s >= deadline_s


# ---------------------------------------------------------------- backend 계약
@dataclass(frozen=True)
class MoveOutcome:
    completed: bool
    pose_error_m: float
    reason_code: str = ''


@dataclass(frozen=True)
class OpenOutcome:
    opened: bool
    width_m: float | None
    reason_code: str = ''


@dataclass(frozen=True)
class GripOutcome:
    width_m: float | None
    current_ma: float | None
    holding: bool | None
    reason_code: str = ''


class StaticBackend(Protocol):
    def move_line(self, target_pos_m, target_quat_xyzw) -> MoveOutcome: ...
    def read_tcp_pose(self) -> tuple[np.ndarray, np.ndarray]: ...
    def gripper_open(self, width_m: float) -> OpenOutcome: ...
    def gripper_close(self, current_limit_ma: int) -> GripOutcome: ...
    def stop(self) -> bool: ...
    def is_stopped(self) -> bool: ...


class DisabledBackend:
    """기본 backend. 실기 연결을 암묵적으로 만들지 않는다."""

    def move_line(self, target_pos_m, target_quat_xyzw) -> MoveOutcome:
        return MoveOutcome(False, math.inf, 'BACKEND_DISABLED')

    def read_tcp_pose(self):
        return np.full(3, np.nan), np.array([0.0, 0.0, 0.0, 1.0])

    def gripper_open(self, width_m: float) -> OpenOutcome:
        return OpenOutcome(False, None, 'BACKEND_DISABLED')

    def gripper_close(self, current_limit_ma: int) -> GripOutcome:
        return GripOutcome(None, None, None, 'BACKEND_DISABLED')

    def stop(self) -> bool:
        return False

    def is_stopped(self) -> bool:
        return False


# ---------------------------------------------------------------- 설정
@dataclass(frozen=True)
class StaticSkillCfg:
    named_poses: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = \
        field(default_factory=dict)
    pose_tolerance_m: float | None = None
    approach_height_m: float | None = None
    safe_z_m: float | None = None
    height_tolerance_m: float | None = None
    open_width_tolerance_m: float | None = None
    release_rim_clearance_m: float | None = None
    grip_min_width_m: float | None = None
    grip_max_width_m: float | None = None
    grip_min_current_ma: float | None = None
    move_v_mps: float = 0.03
    move_a_mps2: float = 0.05
    # SafeRetreat place-back: 매트 반납 TCP 위치(robot_base) [m]와 개방 폭 [m]. None = 미설정
    place_back_xyz_m: tuple[float, float, float] | None = None
    open_width_m: float | None = None


class SingleCommandAuthority:
    """동시 goal 거부. Stop만 선점할 수 있다."""

    def __init__(self):
        self._owner: str | None = None

    @property
    def busy(self) -> bool:
        return self._owner is not None

    @property
    def owner(self) -> str | None:
        return self._owner

    def acquire(self, command_id: str) -> bool:
        if not command_id:
            raise ValueError('command_id: 빈 값')
        if self._owner is not None:
            return False
        self._owner = command_id
        return True

    def release(self, command_id: str) -> None:
        if self._owner == command_id:
            self._owner = None

    def preempt_for_stop(self, command_id: str = 'stop') -> None:
        self._owner = command_id


class StaticSkillExecutor:
    """MoveToPose·PickStone·PlaceStone·SafeRetreat·Stop의 정적 실행 판단."""

    def __init__(self, backend: StaticBackend | None, cfg: StaticSkillCfg,
                 clock, active: bool = True):
        self.backend = backend if backend is not None else DisabledBackend()
        self.cfg = cfg
        self.clock = clock
        self.active = bool(active)
        self.authority = SingleCommandAuthority()

    # ------------------------------------------------------------ 공통
    def _now(self) -> float:
        return float(self.clock.ros_now())

    def _precondition(self, reason: str) -> SkillResult:
        return SkillResult(Result.PRECONDITION, reason)

    def _start(self, command_id: str, deadline_s: float | None,
               cancel_requested: Callable[[], bool] | None):
        if not self.active:
            return self._precondition('LIFECYCLE_INACTIVE')
        if isinstance(self.backend, DisabledBackend):
            return self._precondition('BACKEND_DISABLED')
        if _deadline_passed(deadline_s, self._now()):
            return SkillResult(Result.TIMEOUT, 'DEADLINE')
        if cancel_requested is not None and cancel_requested():
            return SkillResult(Result.CANCELED, 'CANCELED')
        if not self.authority.acquire(command_id):
            return self._precondition('BUSY')
        return None

    def _finish(self, command_id: str, result: SkillResult,
                cancel_requested: Callable[[], bool] | None,
                deadline_s: float | None) -> SkillResult:
        try:
            if result.result == Result.OK:
                if cancel_requested is not None and cancel_requested():
                    return SkillResult(Result.CANCELED, 'CANCELED', result.metrics)
                if _deadline_passed(deadline_s, self._now()):
                    return SkillResult(Result.TIMEOUT, 'DEADLINE', result.metrics)
            return result
        finally:
            self.authority.release(command_id)

    def _interrupted(self, cancel_requested: Callable[[], bool] | None,
                     deadline_s: float | None, out: MoveOutcome | None = None,
                     metrics: dict | None = None) -> SkillResult | None:
        """이동이 끊겼을 때 원인이 cancel·deadline이면 그 결과, 아니면 None.

        backend가 정지를 확인하지 못했다고 알리면(`STOP_NOT_CONFIRMED`) cancel·deadline보다 앞서 `FAULT`다:
        팔이 아직 움직일 수 있는데 깨끗한 취소로 보고하지 않는다.
        """
        metrics = dict(metrics or {})
        if out is not None and out.reason_code in FAULT_MOVE_REASONS:
            return SkillResult(Result.FAULT, out.reason_code, metrics)
        if cancel_requested is not None and cancel_requested():
            return SkillResult(Result.CANCELED, 'CANCELED', metrics)
        if _deadline_passed(deadline_s, self._now()):
            return SkillResult(Result.TIMEOUT, 'DEADLINE', metrics)
        return None

    @staticmethod
    def _emit(feedback, phase: str, metrics: dict[str, float], phase_done: bool) -> None:
        if feedback is not None:
            feedback(phase, dict(metrics), bool(phase_done))

    # ------------------------------------------------------------ MoveToPose
    def move_to_pose(self, target_pos_m, target_quat_xyzw, deadline_s: float | None = None,
                     cancel_requested: Callable[[], bool] | None = None,
                     feedback=None, named_pose: str = '') -> SkillResult:
        early = self._start('move_to_pose', deadline_s, cancel_requested)
        if early is not None:
            return early
        try:
            if self.cfg.pose_tolerance_m is None:
                return self._precondition('POSE_TOLERANCE_UNSET')
            if named_pose:
                pose = self.cfg.named_poses.get(named_pose)
                if pose is None:
                    return self._precondition('NAMED_POSE_UNKNOWN')
                target_pos_m, target_quat_xyzw = pose
            pos = _vec3(target_pos_m, 'target_pos_m')
            quat = np.asarray(target_quat_xyzw, dtype=float).reshape(-1)
            if quat.size != 4 or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) <= 0.0:
                return SkillResult(Result.FAILED, 'RANGE')
            self._emit(feedback, 'MOVE', {}, False)
            if cancel_requested is not None and cancel_requested():
                return SkillResult(Result.CANCELED, 'CANCELED')
            if _deadline_passed(deadline_s, self._now()):
                return SkillResult(Result.TIMEOUT, 'DEADLINE')
            out = self.backend.move_line(pos, quat / np.linalg.norm(quat))
            metrics = {'pose_error_m': float(out.pose_error_m)}
            if not out.completed:
                return (self._interrupted(cancel_requested, deadline_s, out)
                        or SkillResult(Result.FAILED, out.reason_code or 'MOVE_FAILED', metrics))
            if not math.isfinite(out.pose_error_m) or out.pose_error_m > self.cfg.pose_tolerance_m:
                return SkillResult(Result.FAILED, 'POSE_ERROR', metrics)
            self._emit(feedback, 'MOVE', metrics, True)
            return SkillResult(Result.OK, '', metrics)
        finally:
            self.authority.release('move_to_pose')

    # ------------------------------------------------------------ PickStone
    def pick_stone(self, target_position, approach_height_m: float | None,
                   grip_current_limit_ma: int, deadline_s: float | None = None,
                   cancel_requested: Callable[[], bool] | None = None,
                   feedback=None) -> SkillResult:
        early = self._start('pick_stone', deadline_s, cancel_requested)
        if early is not None:
            return early
        try:
            if self.cfg.pose_tolerance_m is None:
                return self._precondition('POSE_TOLERANCE_UNSET')
            if (self.cfg.grip_min_width_m is None or self.cfg.grip_max_width_m is None
                    or self.cfg.grip_min_current_ma is None):
                return self._precondition('GRIP_CRITERIA_UNSET')
            if approach_height_m is None or not math.isfinite(float(approach_height_m)):
                return self._precondition('APPROACH_HEIGHT_UNSET')
            if not (GRIP_CURRENT_LIMIT_MA[0] <= int(grip_current_limit_ma) <= GRIP_CURRENT_LIMIT_MA[1]):
                return SkillResult(Result.PRECONDITION, 'CURRENT_OUT_OF_RANGE')
            target = _vec3(target_position, 'target_position')
            approach = target + np.array([0.0, 0.0, float(approach_height_m)])

            for phase, pose in (('APPROACH', approach), ('DESCEND', target)):
                self._emit(feedback, phase, {}, False)
                if cancel_requested is not None and cancel_requested():
                    return SkillResult(Result.CANCELED, 'CANCELED')
                if _deadline_passed(deadline_s, self._now()):
                    return SkillResult(Result.TIMEOUT, 'DEADLINE')
                out = self.backend.move_line(pose, np.array([0.0, 0.0, 0.0, 1.0]))
                if not out.completed or not math.isfinite(out.pose_error_m):
                    return (self._interrupted(cancel_requested, deadline_s, out) if not out.completed else None) \
                        or SkillResult(Result.FAILED, out.reason_code or 'MOVE_FAILED')

            self._emit(feedback, 'GRIP', {}, False)
            grip = self.backend.gripper_close(int(grip_current_limit_ma))
            metrics = {
                'grip_width_m': float(grip.width_m) if grip.width_m is not None else math.nan,
                'grip_current_ma': float(grip.current_ma) if grip.current_ma is not None else math.nan,
            }
            if grip.width_m is None or grip.current_ma is None:
                return SkillResult(Result.FAILED, grip.reason_code or 'GRIP_MEASUREMENT_MISSING', metrics)
            holding = self._holding(grip.width_m, grip.current_ma)
            if grip.holding is False:
                holding = False
            metrics['holding'] = 1.0 if holding else 0.0
            if not holding:
                return SkillResult(Result.FAILED, 'GRIP_FAILED', metrics, holding=False)
            self._emit(feedback, 'GRIP', metrics, True)

            self._emit(feedback, 'LIFT', {}, False)
            if cancel_requested is not None and cancel_requested():
                return SkillResult(Result.CANCELED, 'CANCELED', metrics)
            if _deadline_passed(deadline_s, self._now()):
                return SkillResult(Result.TIMEOUT, 'DEADLINE', metrics)
            out = self.backend.move_line(approach, np.array([0.0, 0.0, 0.0, 1.0]))
            if not out.completed or out.pose_error_m > self.cfg.pose_tolerance_m:
                interrupted = self._interrupted(cancel_requested, deadline_s, out) if not out.completed else None
                if interrupted is not None:
                    return SkillResult(interrupted.result, interrupted.reason_code, metrics, holding=holding)
                return SkillResult(Result.FAILED, out.reason_code or 'LIFT_FAILED', metrics,
                                   holding=holding)
            metrics['pose_error_m'] = float(out.pose_error_m)
            self._emit(feedback, 'LIFT', metrics, True)
            return SkillResult(Result.OK, '', metrics, holding=True)
        finally:
            self.authority.release('pick_stone')

    def _near(self, measured, target) -> bool:
        p = np.asarray(measured, dtype=float).reshape(-1)
        return (p.size == 3 and bool(np.all(np.isfinite(p)))
                and float(np.linalg.norm(p - target)) <= self.cfg.pose_tolerance_m + 1e-12)

    def _holding(self, width_m: float, current_ma: float) -> bool:
        return (self.cfg.grip_min_width_m - 1e-12 <= width_m <= self.cfg.grip_max_width_m + 1e-12
                and current_ma >= self.cfg.grip_min_current_ma - 1e-12)

    # ------------------------------------------------------------ PlaceStone
    def place_stone(self, target_pose, release_height_m: float, open_width_m: float,
                    deadline_s: float | None = None,
                    cancel_requested: Callable[[], bool] | None = None,
                    feedback=None) -> SkillResult:
        early = self._start('place_stone', deadline_s, cancel_requested)
        if early is not None:
            return early
        metrics: dict[str, float] = {}
        try:
            if self.cfg.pose_tolerance_m is None:
                return self._precondition('POSE_TOLERANCE_UNSET')
            if self.cfg.open_width_tolerance_m is None:
                return self._precondition('OPEN_WIDTH_TOLERANCE_UNSET')
            if self.cfg.safe_z_m is None:
                return self._precondition('SAFE_Z_UNSET')        # RETREAT 목표 높이(움직이기 전에 확인)
            if release_height_m is None or not math.isfinite(float(release_height_m)):
                return self._precondition('RELEASE_HEIGHT_UNSET')
            if open_width_m is None or not math.isfinite(float(open_width_m)):
                return self._precondition('OPEN_WIDTH_UNSET')
            target_pos_m, target_quat = _pose_parts(target_pose)
            target = _vec3(target_pos_m, 'target_pose.position')
            quat = np.asarray(target_quat, dtype=float).reshape(-1)
            if quat.size != 4 or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) <= 0.0:
                return SkillResult(Result.FAILED, 'RANGE')

            self._emit(feedback, 'APPROACH', {'plan_valid': 1.0, 'pose_error_m': math.nan}, False)
            if cancel_requested is not None and cancel_requested():
                return SkillResult(Result.CANCELED, 'CANCELED')
            if _deadline_passed(deadline_s, self._now()):
                return SkillResult(Result.TIMEOUT, 'DEADLINE')
            approach = target + np.array([0.0, 0.0, float(release_height_m)])
            out = self.backend.move_line(approach, quat / np.linalg.norm(quat))
            metrics['pose_error_m'] = float(out.pose_error_m)
            if not out.completed or out.pose_error_m > self.cfg.pose_tolerance_m:
                return (self._interrupted(cancel_requested, deadline_s, out) if not out.completed else None) \
                    or SkillResult(Result.FAILED, out.reason_code or 'APPROACH_FAILED', metrics)
            metrics['plan_valid'] = 1.0
            self._emit(feedback, 'APPROACH', metrics, True)

            self._emit(feedback, 'DESCEND', {}, False)
            if cancel_requested is not None and cancel_requested():
                return SkillResult(Result.CANCELED, 'CANCELED')
            out = self.backend.move_line(target, quat / np.linalg.norm(quat))
            metrics['target_error_m'] = float(out.pose_error_m)
            metrics['rim_clearance_m'] = float(release_height_m)
            metrics['height_error_m'] = float(abs(out.pose_error_m)) if math.isfinite(out.pose_error_m) else math.nan
            if not out.completed or out.pose_error_m > self.cfg.pose_tolerance_m:
                return (self._interrupted(cancel_requested, deadline_s, out) if not out.completed else None) \
                    or SkillResult(Result.FAILED, out.reason_code or 'DESCEND_FAILED', metrics)
            self._emit(feedback, 'DESCEND', metrics, True)

            self._emit(feedback, 'RELEASE', {}, False)
            if cancel_requested is not None and cancel_requested():
                return SkillResult(Result.CANCELED, 'CANCELED')
            if _deadline_passed(deadline_s, self._now()):
                return SkillResult(Result.TIMEOUT, 'DEADLINE')
            before, _ = self.backend.read_tcp_pose()
            opened = self.backend.gripper_open(float(open_width_m))
            after, _ = self.backend.read_tcp_pose()
            metrics['open_width_m'] = (float(opened.width_m)
                                       if opened.width_m is not None else math.nan)
            # 정지 목표의 동기 유지: 개방하는 동안 목표 위치에 머물렀는가(개방 전·후 측정 TCP)
            metrics['sync_held'] = 1.0 if all(self._near(p, target) for p in (before, after)) else 0.0
            if not opened.opened or opened.width_m is None:
                return SkillResult(Result.FAILED, opened.reason_code or 'OPEN_FAILED', metrics)
            if abs(opened.width_m - float(open_width_m)) > self.cfg.open_width_tolerance_m:
                return SkillResult(Result.FAILED, 'OPEN_WIDTH', metrics)
            self._emit(feedback, 'RELEASE', metrics, True)

            # RETREAT: 측정 XY에서 안전 높이까지 수직 상승. 판정은 지령이 아니라 상승 뒤 측정 TCP z로 한다
            # (RETREAT: z ≥ 안전 높이 = 투입 작업영역 밖, 개방 확인 뒤라 공깃돌 없음)
            self._emit(feedback, 'RETREAT', {}, False)
            pos_now, _ = self.backend.read_tcp_pose()
            pos_now = _vec3(pos_now, 'tcp_pose')
            safe_z = float(self.cfg.safe_z_m)
            retreat = np.array([pos_now[0], pos_now[1], max(safe_z, float(pos_now[2]))])
            out = self.backend.move_line(retreat, quat / np.linalg.norm(quat))
            pos_after, _ = self.backend.read_tcp_pose()
            z = float(np.asarray(pos_after, dtype=float).reshape(-1)[2])
            metrics['tcp_z_m'] = z
            metrics['outside_workspace'] = 1.0 if math.isfinite(z) and z >= safe_z - 1e-9 else 0.0
            metrics['stone_in_gripper'] = 0.0
            if not out.completed:
                interrupted = self._interrupted(cancel_requested, deadline_s, out)
                if interrupted is not None:          # 개방 뒤이므로 released=True를 유지한다
                    return SkillResult(interrupted.result, interrupted.reason_code, metrics, released=True)
                return SkillResult(Result.FAILED, out.reason_code or 'RETREAT_FAILED', metrics, released=True)
            if not metrics['outside_workspace']:
                return SkillResult(Result.FAILED, 'RETREAT_FAILED', metrics, released=True)
            self._emit(feedback, 'RETREAT', metrics, True)
            return SkillResult(Result.OK, '', metrics, released=True)
        finally:
            self.authority.release('place_stone')

    # ------------------------------------------------------------ SafeRetreat
    def safe_retreat(self, keep_sync: bool, place_back: bool, place_back_pose=None,
                     deadline_s: float | None = None,
                     cancel_requested: Callable[[], bool] | None = None,
                     feedback=None, keep_sync_retreat=None) -> SkillResult:
        """안전 높이로 후퇴하고, `place_back`이면 매트에 공깃돌을 내려놓는다.

        `keep_sync_retreat(feedback) -> SkillResult`는 동기 유지 후퇴(`dynamic_place.KeepSyncRetreat`)를 실행하는
        주입 함수다. 없으면 `keep_sync`는 움직이지 않고 `PRECONDITION`(`SYNC_NOT_AVAILABLE`)이다. 시작 조건은 어느
        구간도 움직이기 전에 모두 확인한다.
        """
        early = self._start('safe_retreat', deadline_s, cancel_requested)
        if early is not None:
            return early
        try:
            target = None
            if place_back:
                target = self._place_back_target(place_back_pose)
                if isinstance(target, SkillResult):
                    return target
            if keep_sync:
                if keep_sync_retreat is None:
                    return self._precondition('SYNC_NOT_AVAILABLE')
                res = keep_sync_retreat(
                    None if feedback is None else (lambda ph, m, done: self._emit(feedback, ph, m, done)))
            else:
                res = self._retreat_static(deadline_s, cancel_requested, feedback)
            if not place_back or res.result != Result.OK:
                return res             # 후퇴 실패(SYNC_SPEED 등)는 반납하지 않고 그대로 낸다
            return self._place_back(target, deadline_s, cancel_requested, feedback)
        finally:
            self.authority.release('safe_retreat')

    def _retreat_static(self, deadline_s, cancel_requested, feedback) -> SkillResult:
        pos, quat = self.backend.read_tcp_pose()
        pos = _vec3(pos, 'tcp_pose')
        if self.cfg.safe_z_m is not None and pos[2] >= self.cfg.safe_z_m:
            self._emit(feedback, 'RETREAT', {}, False)
            metrics = {'tcp_z_m': float(pos[2])}
            self._emit(feedback, 'RETREAT', metrics, True)
            return SkillResult(Result.OK, '', metrics)
        if self.cfg.safe_z_m is None:
            safe = self.cfg.named_poses.get('safe')
            if safe is None:
                return self._precondition('SAFE_POSE_UNSET')
            target = _vec3(safe[0], 'safe_pose')
        else:
            target = np.array([pos[0], pos[1], float(self.cfg.safe_z_m)])
        self._emit(feedback, 'RETREAT', {}, False)
        if cancel_requested is not None and cancel_requested():
            return SkillResult(Result.CANCELED, 'CANCELED')
        if _deadline_passed(deadline_s, self._now()):
            return SkillResult(Result.TIMEOUT, 'DEADLINE')
        out = self.backend.move_line(target, quat)
        metrics = {'tcp_z_m': float(target[2]), 'pose_error_m': float(out.pose_error_m)}
        if not out.completed:
            return (self._interrupted(cancel_requested, deadline_s, out)
                    or SkillResult(Result.FAILED, out.reason_code or 'RETREAT_FAILED', metrics))
        self._emit(feedback, 'RETREAT', metrics, True)
        return SkillResult(Result.OK, '', metrics)

    def _place_back_target(self, place_back_pose):
        """반납 목표 (위치, 자세 또는 None=측정 자세) 또는 시작 조건 실패. goal 위치가 0이 아니면 goal 자세."""
        c = self.cfg
        if c.pose_tolerance_m is None:
            return self._precondition('POSE_TOLERANCE_UNSET')
        if c.safe_z_m is None:
            return self._precondition('SAFE_Z_UNSET')
        pos = quat = None
        if place_back_pose is not None:
            goal_pos, goal_quat = _pose_parts(place_back_pose)
            goal_pos = np.asarray(goal_pos, dtype=float).reshape(-1)
            if goal_pos.size != 3 or np.any(goal_pos != 0.0):      # 기본 Pose(원점)는 지정 없음
                pos, quat = goal_pos, np.asarray(goal_quat, dtype=float).reshape(-1)
        if pos is None:
            if c.place_back_xyz_m is None:
                return self._precondition('PLACE_BACK_POSE_UNSET')
            pos = np.asarray(c.place_back_xyz_m, dtype=float).reshape(-1)
        if c.open_width_m is None:
            return self._precondition('OPEN_WIDTH_UNSET')
        if c.open_width_tolerance_m is None:
            return self._precondition('OPEN_WIDTH_TOLERANCE_UNSET')
        if pos.size != 3 or not np.all(np.isfinite(pos)) or pos[2] >= c.safe_z_m:
            return SkillResult(Result.FAILED, 'RANGE')             # 안전 높이 이상이면 내려갈 곳이 아니다
        if quat is not None:
            if quat.size != 4 or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) <= 0.0:
                return SkillResult(Result.FAILED, 'RANGE')
            quat = quat / np.linalg.norm(quat)
        return pos, quat

    def _place_back(self, target, deadline_s, cancel_requested, feedback) -> SkillResult:
        """안전 높이에서 매트 위로 → 내려감 → 개방(측정 폭 확인) → 안전 높이로 올림. 옆 이동은 안전 높이에서만."""
        pos, quat = target
        safe_z = float(self.cfg.safe_z_m)
        if quat is None:
            _, quat = self.backend.read_tcp_pose()                 # 측정 자세 유지
            quat = np.asarray(quat, dtype=float).reshape(-1)
            if quat.size != 4 or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) <= 0.0:
                return SkillResult(Result.FAILED, 'TCP_POSE_INVALID')
            quat = quat / np.linalg.norm(quat)
        metrics: dict[str, float] = {}
        self._emit(feedback, 'PLACE_BACK', {}, False)
        for name, goal in (('TRANSIT', np.array([pos[0], pos[1], safe_z])), ('DESCEND', pos)):
            if cancel_requested is not None and cancel_requested():
                return SkillResult(Result.CANCELED, 'CANCELED', metrics)
            if _deadline_passed(deadline_s, self._now()):
                return SkillResult(Result.TIMEOUT, 'DEADLINE', metrics)
            out = self.backend.move_line(goal, quat)
            metrics['pose_error_m'] = float(out.pose_error_m)
            if not out.completed:
                return (self._interrupted(cancel_requested, deadline_s, out, metrics)
                        or SkillResult(Result.FAILED, out.reason_code or f'{name}_FAILED', metrics))
            if not math.isfinite(out.pose_error_m) or out.pose_error_m > self.cfg.pose_tolerance_m:
                return SkillResult(Result.FAILED, 'POSE_ERROR', metrics)
        if cancel_requested is not None and cancel_requested():
            return SkillResult(Result.CANCELED, 'CANCELED', metrics)
        if _deadline_passed(deadline_s, self._now()):
            return SkillResult(Result.TIMEOUT, 'DEADLINE', metrics)
        width = float(self.cfg.open_width_m)
        opened = self.backend.gripper_open(width)
        metrics['open_width_m'] = float(opened.width_m) if opened.width_m is not None else math.nan
        if not opened.opened or opened.width_m is None:
            return SkillResult(Result.FAILED, opened.reason_code or 'OPEN_FAILED', metrics)
        if abs(opened.width_m - width) > self.cfg.open_width_tolerance_m:
            return SkillResult(Result.FAILED, 'OPEN_WIDTH', metrics)
        # 측정 개방 뒤: 공깃돌은 매트 위다(released). 상승은 cancel·deadline으로 미리 막지 않는다(PlaceStone과 같음)
        now_pos, _ = self.backend.read_tcp_pose()
        now_pos = np.asarray(now_pos, dtype=float).reshape(-1)
        xy = now_pos[:2] if now_pos.size == 3 and np.all(np.isfinite(now_pos)) else pos[:2]   # 못 읽으면 반납 XY
        rise = np.array([xy[0], xy[1], safe_z])
        out = self.backend.move_line(rise, quat)
        metrics['pose_error_m'] = float(out.pose_error_m)
        metrics['tcp_z_m'] = safe_z
        if not out.completed:
            interrupted = self._interrupted(cancel_requested, deadline_s, out, metrics)
            if interrupted is not None:
                return replace(interrupted, released=True)
            return SkillResult(Result.FAILED, out.reason_code or 'RISE_FAILED', metrics, released=True)
        self._emit(feedback, 'PLACE_BACK', metrics, True)
        return SkillResult(Result.OK, '', metrics, released=True)

    # ------------------------------------------------------------ DynamicPlace
    def dynamic_place(self, *args, **kwargs) -> SkillResult:
        """계획기·추종 제어가 준비될 때까지 no-op 성공으로 보고하지 않는다."""
        return self._precondition('PLANNER_NOT_READY')

    # ------------------------------------------------------------ Stop
    def stop(self, reason_code: str = '') -> SkillResult:
        """정지 확인 뒤에만 stopped=True. 지령 송신만으로 성공을 만들지 않는다."""
        self.authority.preempt_for_stop()
        sent = bool(self.backend.stop())
        confirmed = bool(self.backend.is_stopped())
        stopped = sent and confirmed
        result = Result.OK if stopped else Result.FAULT
        reason = '' if stopped else ('STOP_NOT_CONFIRMED' if sent else 'STOP_SEND_FAILED')
        return SkillResult(result, reason, {}, stopped=stopped)


def _pose_parts(pose) -> tuple:
    """geometry_msgs/Pose 또는 (position, orientation)을 순수 값으로 받는다."""
    if hasattr(pose, 'position'):
        p = pose.position
        q = pose.orientation
        return np.array([p.x, p.y, p.z], dtype=float), np.array([q.x, q.y, q.z, q.w], dtype=float)
    return pose[0], pose[1]
