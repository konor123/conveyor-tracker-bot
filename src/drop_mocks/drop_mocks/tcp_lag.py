"""mock_robot_controller TCP 1차 지연 모델. ROS와 무관한 순수 모듈.

대역은 동적 phase(APPROACH~RELEASE)에서 TCP가 계획 목표 궤적 `release_xyz + v·(t − release_time)`을 따르되
1차 지연 `dp/dt = (r − p) / τ`로 뒤처진다고 본다. τ 기본값 29 ms는 2026-11-07 OFF-2 참조 부하 speedl 100 Hz에서 잰
지령→위치 지연을 시정수로 옮긴 값이다. 측정 지연에 근거하지만
동특성을 식별한 것은 아니므로 모델(가정)이다. 등속 기준에서는 정상 상태 오차 `v·τ`(20 mm/s에서 0.58 mm), 상대 속도 0이다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

DEFAULT_TCP_LAG_S = 0.029


@dataclass(frozen=True)
class PlanRef:
    """계획 목표 궤적(robot_base, m·s). `InterceptPlan`의 해제 시각·목표 TCP 위치·목표 속도."""
    release_t: float
    release_xyz: tuple
    vel_xyz: tuple

    def at(self, t: float) -> np.ndarray:
        return np.asarray(self.release_xyz, float) + np.asarray(self.vel_xyz, float) * (t - self.release_t)


class TcpLag:
    """위치 기준 1차 지연. 기준이 구간마다 등속이면 진행 간격과 무관하게 정확하다(닫힌 해)."""

    def __init__(self, tau_s: float, pos, t: float):
        if not math.isfinite(tau_s) or tau_s <= 0.0:
            raise ValueError('tau_s: 유한한 양수 필요')
        self.tau_s = float(tau_s)
        self.pos = np.asarray(pos, float).copy()
        self.t = float(t)

    def advance(self, t: float, ref: PlanRef) -> None:
        """t까지 진행한다. 과거 시각이면 아무것도 하지 않는다."""
        h = float(t) - self.t
        if h <= 0.0:
            return
        v = np.asarray(ref.vel_xyz, float)
        offset = self.pos - ref.at(self.t) + v * self.tau_s          # 정상 상태 뒤처짐(−v·τ) 대비 남은 과도
        self.pos = ref.at(float(t)) - v * self.tau_s + offset * math.exp(-h / self.tau_s)
        self.t = float(t)

    def velocity(self, ref: PlanRef) -> np.ndarray:
        return (ref.at(self.t) - self.pos) / self.tau_s

    def errors(self, ref: PlanRef) -> tuple[float, float]:
        """(XY 위치 오차 [m], XY 상대 속도 [m/s]) = DynamicPlace SYNCHRONIZE feedback `e_xy`·`v_rel`."""
        e = ref.at(self.t)[:2] - self.pos[:2]
        dv = np.asarray(ref.vel_xyz, float)[:2] - self.velocity(ref)[:2]
        return float(np.linalg.norm(e)), float(np.linalg.norm(dv))


def make_lag(tau_s: float, pos, t: float) -> TcpLag | None:
    """`tau_s` 0이면 모델을 쓰지 않는다(None). 음수·비유한 값은 예외."""
    tau_s = float(tau_s)
    if tau_s == 0.0:
        return None
    return TcpLag(tau_s, pos, t)
