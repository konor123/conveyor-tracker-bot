"""Doosan 팔 대역(`doosan_backend.DoosanMotionClient` 모양). 실제 로봇·`/dsr01`에 연결하지 않는다.

값은 Doosan 단위(posx mm·deg ZYZ, 속도 mm/s·deg/s)다. 시간은 FakeClock steady 시각이다. 모델(실기 측정 아님):
`move_line_async`를 받으면 `start_delay_s` 뒤 MOVING(2)이 되어 지령 속도로 직선 보간하고, 도착하면 STANDBY(1)다.
`move_stop`을 받으면 `stop_delay_s` 뒤 그 자리에서 멈춘다. 주입: 거절, 받고도 움직이지 않음(2026-11-09 DRL PLAY 중
move_line 관찰), 목표 앞에서 멈춤, 정지 무시, 읽기 실패, DRL 상태.
"""
from __future__ import annotations

import math

import numpy as np

from robot_control.dsr_register_io import DsrError

STANDBY, MOVING = 1, 2


class FakeDoosanArm:
    def __init__(self, clock, posx=(500.0, 0.0, 200.0, 0.0, 180.0, 0.0), start_delay_s: float = 0.1,
                 stop_delay_s: float = 0.15, drl_state: int = 3):
        self.clock = clock
        self._posx = np.asarray(posx, dtype=float)
        self.start_delay_s = float(start_delay_s)
        self.stop_delay_s = float(stop_delay_s)
        self.drl_state_value = drl_state
        self.state_override: int | None = None        # 보호정지 등
        self.accept = True
        self.ignore_motion = False
        self.stop_short_mm = 0.0
        self.stop_ignored = False
        self.speed_scale = 1.0                         # 실제 속도 = 지령 × 이 값
        self.fail_reads = False
        self.moves: list[tuple[list, list, list]] = []
        self.stops: list[float] = []
        self.hooks: list = []                          # (steady 시각, fn): 읽기 때 시각이 지나면 1회 호출
        self._seg = None                               # (t_start, t_end, p0, p1)

    # ------------------------------------------------------------ 모델
    def _now(self) -> float:
        return float(self.clock.steady_now())

    def _run_hooks(self) -> None:
        due = [h for h in self.hooks if self._now() >= h[0]]
        for h in due:
            self.hooks.remove(h)
            h[1]()

    def _pose_at(self, t: float) -> np.ndarray:
        if self._seg is None:
            return self._posx.copy()
        t0, t1, p0, p1 = self._seg
        if t <= t0:
            return p0.copy()
        if t >= t1:
            return p1.copy()
        return p0 + (p1 - p0) * ((t - t0) / (t1 - t0))

    def _settle(self) -> None:
        if self._seg is not None and self._now() >= self._seg[1]:
            self._posx = self._seg[3].copy()
            self._seg = None

    # ------------------------------------------------------------ client 계약
    def move_line_async(self, posx, vel, acc) -> bool:
        self.moves.append((list(posx), list(vel), list(acc)))
        if not self.accept:
            return False
        self._settle()
        if self.ignore_motion:
            return True
        start = self._pose_at(self._now())
        target = np.asarray(posx, dtype=float)
        if self.stop_short_mm > 0.0:
            d = target[:3] - start[:3]
            n = float(np.linalg.norm(d))
            if n > self.stop_short_mm:
                target = target.copy()
                target[:3] = start[:3] + d * (1.0 - self.stop_short_mm / n)
        lin = float(np.linalg.norm(target[:3] - start[:3])) / max(float(vel[0]) * self.speed_scale, 1e-9)
        ang = float(np.max(np.abs(target[3:] - start[3:]))) / max(float(vel[1]) * self.speed_scale, 1e-9)
        t0 = self._now() + self.start_delay_s
        self._seg = (t0, t0 + max(lin, ang, 1e-6), start, target)
        return True

    def move_stop(self) -> bool:
        self.stops.append(self._now())
        if self._seg is not None and not self.stop_ignored:
            t_end = max(self._now(), self._seg[0]) + self.stop_delay_s
            if t_end < self._seg[1]:
                self._seg = (self._seg[0], t_end, self._seg[2], self._pose_at(t_end))
        return True

    def current_posx(self) -> list[float]:
        self._run_hooks()
        if self.fail_reads:
            raise DsrError('SERVICE_TIMEOUT', 'aux_control/get_current_posx')
        self._settle()
        return [float(v) for v in self._pose_at(self._now())]

    def robot_state(self) -> int:
        self._run_hooks()
        if self.fail_reads:
            raise DsrError('SERVICE_TIMEOUT', 'system/get_robot_state')
        if self.state_override is not None:
            return self.state_override
        self._settle()
        if self._seg is not None and self._seg[0] <= self._now() < self._seg[1]:
            return MOVING
        return STANDBY

    def drl_state(self) -> int:
        return self.drl_state_value

    @property
    def posx(self) -> np.ndarray:
        return self._pose_at(self._now())


def posx_distance_mm(a, b) -> float:
    return math.dist(list(a)[:3], list(b)[:3])
