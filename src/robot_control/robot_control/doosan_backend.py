"""Doosan E0509 정적 skill backend 어댑터(`skill_logic.StaticBackend`, REQ-ROB-017). **실기 미검증.**

`robot_controller_node`의 `backend` 선택지에 연결하지 않는다. 연결 방법과 OFF-2에서 확인할 것은 SPEC-500 4.6절이다.

- 팔은 좁은 client 계약(`DoosanMotionClient`, Doosan 단위)으로만 다룬다: 비동기 직선 이동(`move_line_async`,
  드라이버 `motion/move_line`의 `sync_type=1` 자리), `move_stop`, TCP 읽기(`current_posx`), 로봇·DRL 상태.
  SI ↔ Doosan 변환은 `motion_adapter`가 한다.
- DRL 프로그램이 STOP·LAST가 아니면(PLAY·HOLD) 이동을 보내지 않는다(`DRL_BUSY`). 2026-11-09 현장에서 DRL이 PLAY인
  동안 보낸 move_line은 움직이지 않고 끝났다. 그리퍼 상주 루프도 PLAY인 DRL이므로, 루프가 도는 동안은 이 규칙으로
  이동을 거부한다(상주 루프 중 이동 수락 여부는 OFF-2 확인 항목).
- 이동은 측정 TCP가 목표 위치 `reach_tol_m`·자세 `reach_tol_rad` 안이고 로봇 상태가 STANDBY일 때만 완료다.
  지령을 받고도 `start_grace_s` 안에 MOVING이 보이지 않고 도착하지 않았으면 `MOVE_NOT_STARTED`, 움직였다가
  목표 밖에서 멈췄으면 `MOVE_NOT_REACHED`, `거리/속도 + move_timeout_margin_s`를 넘으면 `MOVE_TIMEOUT`이다.
- cancel·deadline(`cancel_requested`)·시간 초과·읽기 실패는 `move_stop`을 보내고 정지 상태를 `cancel_stop_timeout_s`
  (steady) 동안 기다린다. 확인하지 못하면 `STOP_NOT_CONFIRMED`(실행기는 FAULT로 보고). `stop()`은 정지 유지를 걸고
  `move_stop` 뒤 `stop_timeout_s`까지 기다린다. `is_stopped()`는 매번 로봇 상태를 새로 읽는다.
- 그리퍼는 `GripperSession`(상주 루프 transport + `DsrRegisterIO`)으로만 다룬다. 결과는 측정 폭·전류만 보고한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Protocol

import numpy as np

from .dsr_register_io import DRL_IDLE_STATES, ROBOT_STATE_STANDBY, DsrError
from .gripper_adapter import RESULT_OK, GripperError
from .motion_adapter import (MotionError, accel_si_to_doosan, pose_doosan_to_si, pose_si_to_doosan,
                             quat_to_matrix, rotation_error, speed_si_to_doosan)
from .skill_logic import GripOutcome, MoveOutcome, OpenOutcome

ROBOT_STATE_MOVING = 2
# 움직이지 않는 로봇 상태: STANDBY 1, SAFE_OFF 3, SAFE_STOP 5(tools/robot/e0509_ctl.py 이름표). 실기 전이는 OFF-2 확인
STOPPED_ROBOT_STATES = frozenset({1, 3, 5})


class DoosanMotionClient(Protocol):
    """Doosan 단위 client. posx [mm×3, deg ZYZ×3], vel [mm/s, deg/s], acc [mm/s², deg/s²]."""

    def move_line_async(self, posx: list[float], vel: list[float], acc: list[float]) -> bool: ...
    def move_stop(self) -> bool: ...
    def current_posx(self) -> list[float]: ...
    def robot_state(self) -> int: ...
    def drl_state(self) -> int: ...


@dataclass(frozen=True)
class DoosanBackendCfg:
    v_mps: float = 0.03                 # move_v_mps 기본값
    a_mps2: float = 0.05                # move_a_mps2 기본값
    ang_v_radps: float = 0.35           # static_pick_place.doosan_client.move_line 기본값(2026-11-09 현장 사용)
    ang_a_radps2: float = 0.7           # 같음
    reach_tol_m: float = 0.001          # doosan_client.move_line 도달 확인 1 mm(2026-11-09 현장 사용)
    reach_tol_rad: float = 0.0175       # 1°, 제안(TBD OFF-2)
    poll_s: float = 0.05                # doosan_client DRL 감시 주기와 같음
    start_grace_s: float = 1.0          # 제안(TBD OFF-2): 지령 수락 → MOVING이 보여야 하는 시간
    move_timeout_margin_s: float = 2.0  # 제안(TBD OFF-2): 거리/속도에 더하는 여유(가감속·서비스 지연)
    cancel_stop_timeout_s: float = 0.4  # REQ-ROB-007 cancel 0.5 s − 폴링 1주기 이상 여유
    stop_timeout_s: float = 2.0         # SPEC-500 5절 Stop 확인 대기 2 s
    gripper_current_ma: int | None = None   # None = 그리퍼 adapter 기본 전류

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if name == 'gripper_current_ma':
                continue
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f'{name}: 유한한 양수 필요')
        if self.poll_s + self.cancel_stop_timeout_s >= 0.5:
            raise ValueError('poll_s + cancel_stop_timeout_s는 cancel 응답 0.5 s보다 작아야 한다')


def _nan_pose():
    return np.full(3, np.nan), np.full(4, np.nan)


class DoosanBackend:
    def __init__(self, client: DoosanMotionClient, gripper, clock, sleep: Callable[[float], None],
                 cfg: DoosanBackendCfg = DoosanBackendCfg(), cancel_requested: Callable[[], bool] | None = None):
        self.client = client
        self.gripper = gripper              # GripperSession 또는 None
        self.clock = clock
        self._sleep = sleep
        self.cfg = cfg
        self.cancel_requested = cancel_requested or (lambda: False)
        self._stopped = False

    # ------------------------------------------------------------ 이동
    def move_line(self, target_pos_m, target_quat_xyzw) -> MoveOutcome:
        if self._stopped:
            return MoveOutcome(False, math.inf, 'STOPPED')
        c = self.cfg
        try:
            target = np.asarray(target_pos_m, dtype=float).reshape(-1)
            target_rot = quat_to_matrix(target_quat_xyzw)
            posx = pose_si_to_doosan(target, target_quat_xyzw)
            vel = speed_si_to_doosan([c.v_mps, 0.0, 0.0], [c.ang_v_radps, 0.0, 0.0])
            acc = accel_si_to_doosan(c.a_mps2, c.ang_a_radps2)
        except MotionError:
            return MoveOutcome(False, math.inf, 'RANGE')
        try:
            if int(self.client.drl_state()) not in DRL_IDLE_STATES:
                return MoveOutcome(False, math.inf, 'DRL_BUSY')
            if int(self.client.robot_state()) != ROBOT_STATE_STANDBY:
                return MoveOutcome(False, math.inf, 'NOT_STANDBY')
            start, start_rot = self._measure()
            err, rot_err = self._errors(start, start_rot, target, target_rot)
            if not self.client.move_line_async(posx, [vel[0], vel[3]], list(acc)):
                return MoveOutcome(False, err, 'MOVE_REJECTED')
        except DsrError as exc:
            return MoveOutcome(False, math.inf, exc.reason)
        t0 = self.clock.steady_now()
        limit = t0 + max(err / c.v_mps, rot_err / c.ang_v_radps) + c.move_timeout_margin_s
        seen_moving = False
        while True:
            if self._stopped:
                return MoveOutcome(False, err, 'STOPPED')       # stop()이 move_stop·정지 확인을 맡는다
            if self.cancel_requested():
                return self._halt(target, err, 'CANCELED')
            if self.clock.steady_now() > limit:
                return self._halt(target, err, 'MOVE_TIMEOUT')
            self._sleep(c.poll_s)
            try:
                pos, rot = self._measure()
                state = int(self.client.robot_state())
            except (DsrError, MotionError) as exc:
                return self._halt(target, math.inf, getattr(exc, 'reason', 'TCP_READ_FAILED'))
            err, rot_err = self._errors(pos, rot, target, target_rot)
            if self._stopped:
                return MoveOutcome(False, err, 'STOPPED')       # 읽는 사이 Stop이 들어왔다
            if state == ROBOT_STATE_MOVING:
                seen_moving = True
                continue
            if state != ROBOT_STATE_STANDBY:
                return MoveOutcome(False, err, f'ROBOT_STATE_{state}')
            if err <= c.reach_tol_m and rot_err <= c.reach_tol_rad:
                return MoveOutcome(True, err)
            if seen_moving:
                return MoveOutcome(False, err, 'MOVE_NOT_REACHED')
            if self.clock.steady_now() - t0 >= c.start_grace_s:
                return MoveOutcome(False, err, 'MOVE_NOT_STARTED')

    def _halt(self, target, err: float, reason: str) -> MoveOutcome:
        """move_stop → 정지 상태 확인 → 멈춘 자리의 측정 오차. 확인하지 못하면 STOP_NOT_CONFIRMED."""
        try:
            sent = bool(self.client.move_stop())
        except DsrError:
            sent = False
        if not sent or not self._wait_stopped(self.cfg.cancel_stop_timeout_s):
            return MoveOutcome(False, err, 'STOP_NOT_CONFIRMED')
        try:
            pos, _ = self._measure()
            err = float(np.linalg.norm(pos - target))
        except (DsrError, MotionError):
            err = math.inf
        return MoveOutcome(False, err, reason)

    @staticmethod
    def _errors(pos, rot, target, target_rot) -> tuple[float, float]:
        return float(np.linalg.norm(pos - target)), float(rotation_error(rot, target_rot))

    def _measure(self):
        pos, quat = pose_doosan_to_si(self.client.current_posx())
        return np.asarray(pos, dtype=float), quat_to_matrix(quat)

    def _wait_stopped(self, timeout_s: float) -> bool:
        deadline = self.clock.steady_now() + timeout_s
        while True:
            if self.is_stopped():
                return True
            remaining = deadline - self.clock.steady_now()
            if remaining <= 0.0:
                return False
            self._sleep(min(self.cfg.poll_s, remaining))

    def read_tcp_pose(self):
        try:
            pos, quat = pose_doosan_to_si(self.client.current_posx())
        except (DsrError, MotionError):
            return _nan_pose()                                   # 읽지 못하면 값을 만들지 않는다
        return np.asarray(pos, dtype=float), np.asarray(quat, dtype=float)

    # ------------------------------------------------------------ 정지
    def stop(self) -> bool:
        """정지 유지를 걸고 move_stop을 보낸 뒤 정지 상태를 `stop_timeout_s`까지 기다린다. 반환 = 송신 성공."""
        self._stopped = True
        try:
            sent = bool(self.client.move_stop())
        except DsrError:
            sent = False
        if sent:
            self._wait_stopped(self.cfg.stop_timeout_s)
        return sent

    def is_stopped(self) -> bool:
        try:
            return int(self.client.robot_state()) in STOPPED_ROBOT_STATES
        except DsrError:
            return False

    def reset_stop(self) -> None:
        """작업자 해제(`stop_release_reason`) 뒤 노드가 부른다."""
        self._stopped = False

    # ------------------------------------------------------------ 그리퍼
    def start_gripper(self) -> None:
        """상주 루프 기동 + 시작 확인(세션당 1회). STANDBY가 아니면 adapter가 `PRECONDITION`으로 거부한다."""
        if self.gripper is None:
            raise GripperError('GRIPPER_UNAVAILABLE')
        standby = int(self.client.robot_state()) == ROBOT_STATE_STANDBY
        self.gripper.start(standby)

    def gripper_open(self, width_m: float) -> OpenOutcome:
        if self._stopped:
            return OpenOutcome(False, None, 'STOPPED')
        if self.gripper is None:
            return OpenOutcome(False, None, 'GRIPPER_UNAVAILABLE')
        try:
            res = self.gripper.ensure_open(float(width_m), self.cfg.gripper_current_ma)
        except (GripperError, DsrError) as exc:
            return OpenOutcome(False, None, exc.reason)
        if res is None:                                          # 방금 읽은 폭이 이미 목표 ± 허용오차
            return OpenOutcome(True, self.gripper.events[-1].width_m)
        if res.result != RESULT_OK or res.width_m is None:
            return OpenOutcome(False, None, res.reason_code or 'OPEN_FAILED')
        return OpenOutcome(True, float(res.width_m))

    def gripper_close(self, current_limit_ma: int) -> GripOutcome:
        if self._stopped:
            return GripOutcome(None, None, None, 'STOPPED')
        if self.gripper is None:
            return GripOutcome(None, None, None, 'GRIPPER_UNAVAILABLE')
        try:
            res = self.gripper.grasp(int(current_limit_ma))
        except (GripperError, DsrError) as exc:
            return GripOutcome(None, None, None, exc.reason)
        if res.result != RESULT_OK:
            return GripOutcome(None, None, None, res.reason_code or 'GRIP_FAILED')
        return GripOutcome(res.width_m, res.current_ma, res.holding, '')
