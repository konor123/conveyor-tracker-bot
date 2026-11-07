"""정적 skill 실행 판단(순수 Python). ROS 노드는 이 모듈을 감싸기만 한다.

성공 판정은 보낸 지령이 아니라 backend가 돌려준 **측정값**으로만 한다. 설정이
없거나 backend가 비활성·미구현이면 `PRECONDITION`으로 닫고, `DynamicPlace`는
계획기·추종 제어가 준비될 때까지 `PRECONDITION`(`PLANNER_NOT_READY`)이다.

시간은 float 초(ROS 시각)로 받는다. 하드웨어 단위 변환은 `motion_adapter.py`와
`gripper_adapter.py` 밖에서 하지 않는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
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


class SingleCommandAuthority:
    """동시 goal 거부(TC-ROB-005-01). Stop만 선점할 수 있다."""

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
                return SkillResult(Result.FAILED, out.reason_code or 'MOVE_FAILED', metrics)
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
            if not (1 <= int(grip_current_limit_ma) <= 100):
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
                    return SkillResult(Result.FAILED, out.reason_code or 'MOVE_FAILED')

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
                return SkillResult(Result.FAILED, out.reason_code or 'LIFT_FAILED', metrics,
                                   holding=holding)
            metrics['pose_error_m'] = float(out.pose_error_m)
            self._emit(feedback, 'LIFT', metrics, True)
            return SkillResult(Result.OK, '', metrics, holding=True)
        finally:
            self.authority.release('pick_stone')

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
                return SkillResult(Result.FAILED, out.reason_code or 'APPROACH_FAILED', metrics)
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
                return SkillResult(Result.FAILED, out.reason_code or 'DESCEND_FAILED', metrics)
            self._emit(feedback, 'DESCEND', metrics, True)

            self._emit(feedback, 'RELEASE', {}, False)
            if cancel_requested is not None and cancel_requested():
                return SkillResult(Result.CANCELED, 'CANCELED')
            if _deadline_passed(deadline_s, self._now()):
                return SkillResult(Result.TIMEOUT, 'DEADLINE')
            opened = self.backend.gripper_open(float(open_width_m))
            metrics['open_width_m'] = (float(opened.width_m)
                                       if opened.width_m is not None else math.nan)
            metrics['sync_held'] = 0.0
            if not opened.opened or opened.width_m is None:
                return SkillResult(Result.FAILED, opened.reason_code or 'OPEN_FAILED', metrics)
            if abs(opened.width_m - float(open_width_m)) > self.cfg.open_width_tolerance_m:
                return SkillResult(Result.FAILED, 'OPEN_WIDTH', metrics)
            self._emit(feedback, 'RELEASE', metrics, True)

            self._emit(feedback, 'RETREAT', {}, False)
            pos_now, _ = self.backend.read_tcp_pose()
            retreat = _vec3(pos_now, 'tcp_pose') + np.array([0.0, 0.0, float(release_height_m)])
            out = self.backend.move_line(retreat, quat / np.linalg.norm(quat))
            metrics['tcp_z_m'] = float(retreat[2])
            metrics['outside_workspace'] = 1.0 if not out.completed else 0.0
            metrics['stone_in_gripper'] = 0.0
            if not out.completed:
                return SkillResult(Result.FAILED, out.reason_code or 'RETREAT_FAILED', metrics)
            self._emit(feedback, 'RETREAT', metrics, True)
            return SkillResult(Result.OK, '', metrics, released=True)
        finally:
            self.authority.release('place_stone')

    # ------------------------------------------------------------ SafeRetreat
    def safe_retreat(self, keep_sync: bool, place_back: bool, place_back_pose=None,
                     deadline_s: float | None = None,
                     cancel_requested: Callable[[], bool] | None = None,
                     feedback=None) -> SkillResult:
        early = self._start('safe_retreat', deadline_s, cancel_requested)
        if early is not None:
            return early
        try:
            if keep_sync:
                return self._precondition('SYNC_NOT_AVAILABLE')
            if place_back:
                return self._precondition('PLACE_BACK_UNSUPPORTED')
            pos, quat = self.backend.read_tcp_pose()
            pos = _vec3(pos, 'tcp_pose')
            if self.cfg.safe_z_m is not None and pos[2] >= self.cfg.safe_z_m:
                return SkillResult(Result.OK, '', {'tcp_z_m': float(pos[2])})
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
                return SkillResult(Result.FAILED, out.reason_code or 'RETREAT_FAILED', metrics)
            self._emit(feedback, 'RETREAT', metrics, True)
            return SkillResult(Result.OK, '', metrics)
        finally:
            self.authority.release('safe_retreat')

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
