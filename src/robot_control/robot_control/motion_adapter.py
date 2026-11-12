"""Doosan SI ↔ native 변환, 지령 신선도·범위 검사, 추종 지령(TEST_SPEC.md 2절 ROB).

단위 변환은 이 파일에서만 한다(TEST_SPEC TC-SYS-004-01). 모든 공개 함수는
길이 m, 각도 rad, 속도 m/s·rad/s를 받고 Doosan native 값(mm, deg)은
`pose_si_to_doosan`·`speed_si_to_doosan` 경계에서만 만든다.

ZYZ 오일러 규약은 Doosan move_line/stream과 같이 `R = Rz(a) Ry(b) Rz(c)`다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

MM_PER_M = 1000.0
DEG_PER_RAD = 180.0 / math.pi
SINGULAR_EPS = 1e-9
FINITE_EPS = 1e-12


class MotionError(ValueError):
    """변환·지령 입력이 계약을 어김."""


class WorkspaceError(MotionError):
    """작업영역 또는 속도 한도를 벗어남(reason=RANGE)."""


class _SteadyClock(Protocol):
    def steady_now(self) -> float: ...


def _vector(value, size: int, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != size:
        raise MotionError(f'{name}: 길이 {size} 필요, 받은 값 {arr.size}')
    if not np.all(np.isfinite(arr)):
        raise MotionError(f'{name}: 유한하지 않은 값')
    return arr


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise MotionError(f'{name}: 유한하지 않은 값')
    return value


def quat_to_matrix(quat_xyzw) -> np.ndarray:
    """단위 quaternion(x, y, z, w) → 3×3 회전행렬. 영 quaternion은 거부한다."""
    q = _vector(quat_xyzw, 4, 'quat_xyzw')
    norm = float(np.linalg.norm(q))
    if norm < FINITE_EPS:
        raise MotionError('quat_xyzw: 영 quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def matrix_to_quat(rot: np.ndarray) -> np.ndarray:
    """3×3 회전행렬 → 단위 quaternion(x, y, z, w). 특이 자세를 포함해 왕복 안정."""
    m = np.asarray(rot, dtype=float)
    if m.shape != (3, 3) or not np.all(np.isfinite(m)):
        raise MotionError('rotation: 3×3 유한 행렬 필요')
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


def euler_zyz_from_matrix(rot: np.ndarray) -> tuple[float, float, float]:
    """R = Rz(a) Ry(b) Rz(c), b∈[0, π]. β≈0·π 특이 자세에서 c=0으로 고정한다."""
    m = np.asarray(rot, dtype=float)
    if m.shape != (3, 3) or not np.all(np.isfinite(m)):
        raise MotionError('rotation: 3×3 유한 행렬 필요')
    sin_b = math.hypot(float(m[2, 0]), float(m[2, 1]))
    if sin_b > SINGULAR_EPS:
        beta = math.atan2(sin_b, float(m[2, 2]))
        alpha = math.atan2(float(m[1, 2]), float(m[0, 2]))
        gamma = math.atan2(float(m[2, 1]), -float(m[2, 0]))
    elif float(m[2, 2]) >= 0.0:
        beta = 0.0
        alpha = math.atan2(float(m[1, 0]), float(m[0, 0]))
        gamma = 0.0
    else:
        beta = math.pi
        alpha = math.atan2(-float(m[0, 1]), -float(m[0, 0]))
        gamma = 0.0
    return alpha, beta, gamma


def matrix_from_euler_zyz(alpha: float, beta: float, gamma: float) -> np.ndarray:
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    cg, sg = math.cos(gamma), math.sin(gamma)
    return np.array([
        [ca * cb * cg - sa * sg, -ca * cb * sg - sa * cg, ca * sb],
        [sa * cb * cg + ca * sg, -sa * cb * sg + ca * cg, sa * sb],
        [-sb * cg, sb * sg, cb],
    ])


def rotation_error(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    """두 회전행렬 사이 회전각(rad)."""
    rel = np.asarray(rot_a, dtype=float).T @ np.asarray(rot_b, dtype=float)
    cos_angle = max(-1.0, min(1.0, (float(np.trace(rel)) - 1.0) / 2.0))
    sin_angle = 0.5 * math.sqrt(
        (rel[2, 1] - rel[1, 2]) ** 2 + (rel[0, 2] - rel[2, 0]) ** 2 + (rel[1, 0] - rel[0, 1]) ** 2)
    return math.atan2(sin_angle, cos_angle)


def pose_si_to_doosan(pos_m, quat_xyzw) -> list[float]:
    """위치 m·quaternion(xyzw) → [mm×3, deg ZYZ×3]."""
    pos = _vector(pos_m, 3, 'pos_m')
    rot = quat_to_matrix(quat_xyzw)
    alpha, beta, gamma = euler_zyz_from_matrix(rot)
    return [
        float(pos[0] * MM_PER_M),
        float(pos[1] * MM_PER_M),
        float(pos[2] * MM_PER_M),
        float(alpha * DEG_PER_RAD),
        float(beta * DEG_PER_RAD),
        float(gamma * DEG_PER_RAD),
    ]


def pose_doosan_to_si(p) -> tuple[np.ndarray, np.ndarray]:
    """[mm×3, deg ZYZ×3] → 위치 m, quaternion(xyzw)."""
    values = _vector(p, 6, 'pose_doosan')
    pos = values[:3] / MM_PER_M
    alpha, beta, gamma = (float(v / DEG_PER_RAD) for v in values[3:])
    return pos, matrix_to_quat(matrix_from_euler_zyz(alpha, beta, gamma))


def speed_si_to_doosan(lin_mps, ang_radps) -> list[float]:
    """선속도 m/s·각속도 rad/s → [mm/s×3, deg/s×3]."""
    lin = _vector(lin_mps, 3, 'lin_mps')
    ang = _vector(ang_radps, 3, 'ang_radps')
    return [float(v * MM_PER_M) for v in lin] + [float(v * DEG_PER_RAD) for v in ang]


def accel_si_to_doosan(lin_mps2: float, ang_radps2: float) -> list[float]:
    """선가속도 m/s²·각가속도 rad/s² → Doosan `acc` [mm/s², deg/s²](speedl·servol 스트림)."""
    return [_finite(lin_mps2, 'lin_mps2') * MM_PER_M, _finite(ang_radps2, 'ang_radps2') * DEG_PER_RAD]


def joints_doosan_to_si(posj) -> np.ndarray:
    """관절각 deg×6 → rad×6."""
    return _vector(posj, 6, 'posj') / DEG_PER_RAD


def speed_doosan_to_si(speed) -> tuple[np.ndarray, np.ndarray]:
    """[mm/s×3, deg/s×3] → 선속도 m/s, 각속도 rad/s."""
    values = _vector(speed, 6, 'speed_doosan')
    return values[:3] / MM_PER_M, values[3:] / DEG_PER_RAD


class FreshnessWatchdog:
    """지령 신선도(TC-ROB-003-01). 주기 2배를 넘으면 래치하고 재무장 전까지 거부한다.

    경계는 `age == 2·period`까지 유지, `age > 2·period`에서 L0다.
    """

    def __init__(self, period: float, clock: _SteadyClock, factor: float = 2.0):
        period = _finite(period, 'period')
        factor = _finite(factor, 'factor')
        if period <= 0.0 or factor <= 0.0:
            raise MotionError('period·factor는 0보다 커야 한다')
        self.period = period
        self.factor = factor
        self.max_gap_s = period * factor
        self._clock = clock
        self._last_s: float | None = None
        self._latched = False

    @property
    def latched(self) -> bool:
        return self._latched

    def on_command(self) -> bool:
        """지령 직전 호출. False면 보내지 않는다(래치 또는 간격 초과)."""
        if self._latched:
            return False
        now = float(self._clock.steady_now())
        if self._last_s is not None and now - self._last_s > self.max_gap_s + FINITE_EPS:
            self._latched = True
            return False
        self._last_s = now
        return True

    def check(self) -> bool:
        """주기적 감시. False = L0(속도 0 지령 1회 후 거부)."""
        if self._latched:
            return False
        if self._last_s is None:
            return True
        if float(self._clock.steady_now()) - self._last_s > self.max_gap_s + FINITE_EPS:
            self._latched = True
            return False
        return True

    def rearm(self) -> None:
        """명시적 재무장. 작업자 확인 뒤에만 호출한다."""
        self._last_s = float(self._clock.steady_now())
        self._latched = False


@dataclass(frozen=True)
class Workspace:
    """robot_base 기준 직육면체 작업영역. 값이 없으면 범위 검사를 하지 않는다."""

    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_m: float
    z_max_m: float

    def contains(self, pos_m) -> bool:
        pos = _vector(pos_m, 3, 'pos_m')
        return (self.x_min_m <= pos[0] <= self.x_max_m
                and self.y_min_m <= pos[1] <= self.y_max_m
                and self.z_min_m <= pos[2] <= self.z_max_m)


class CommandAuthority:
    """단일 지령 권한 + 범위·비유한 값 검사(TC-ROB-004-01, TC-ROB-005-01).

    실제 송신은 하지 않는다. 호출자가 `validate_*`가 None일 때만 transport로 보낸다.
    """

    def __init__(self, workspace: Workspace | None = None,
                 v_max_mps: float | None = None,
                 a_max_mps2: float | None = None):
        self.workspace = workspace
        self.v_max_mps = _finite(v_max_mps, 'v_max_mps') if v_max_mps is not None else None
        self.a_max_mps2 = _finite(a_max_mps2, 'a_max_mps2') if a_max_mps2 is not None else None
        self._owner: str | None = None

    @property
    def busy(self) -> bool:
        return self._owner is not None

    @property
    def owner(self) -> str | None:
        return self._owner

    def acquire(self, command_id: str) -> bool:
        if not command_id:
            raise MotionError('command_id: 빈 값')
        if self._owner is not None:
            return False
        self._owner = command_id
        return True

    def release(self, command_id: str) -> None:
        if self._owner == command_id:
            self._owner = None

    def validate_pose(self, pos_m, quat_xyzw=None) -> str | None:
        try:
            pos = _vector(pos_m, 3, 'pos_m')
            if quat_xyzw is not None:
                quat_to_matrix(quat_xyzw)
        except MotionError:
            return 'NONFINITE'
        if self.workspace is not None and not self.workspace.contains(pos):
            return 'RANGE'
        return None

    def validate_speed(self, v_mps) -> str | None:
        try:
            v = _vector(v_mps, 3, 'v_mps')
        except MotionError:
            return 'NONFINITE'
        if self.v_max_mps is not None and float(np.linalg.norm(v)) > self.v_max_mps + FINITE_EPS:
            return 'RANGE'
        return None


def tracking_command(p_target, v_ff, p_tcp, kp: float, v_max: float, a_max: float,
                     v_prev, dt: float) -> np.ndarray:
    """v = v_ff + Kp·(p_target − p_tcp), |v| ≤ v_max, |v − v_prev| ≤ a_max·dt."""
    target = _vector(p_target, 3, 'p_target')
    feed = _vector(v_ff, 3, 'v_ff')
    tcp = _vector(p_tcp, 3, 'p_tcp')
    prev = _vector(v_prev, 3, 'v_prev')
    kp = _finite(kp, 'kp')
    v_max = _finite(v_max, 'v_max')
    a_max = _finite(a_max, 'a_max')
    dt = _finite(dt, 'dt')
    if kp < 0.0 or v_max < 0.0 or a_max < 0.0:
        raise MotionError('kp·v_max·a_max는 음수일 수 없다')
    if dt <= 0.0:
        raise MotionError('dt는 0보다 커야 한다')

    cmd = feed + kp * (target - tcp)
    norm = float(np.linalg.norm(cmd))
    if norm > v_max and norm > 0.0:
        cmd = cmd * (v_max / norm)

    delta = cmd - prev
    delta_norm = float(np.linalg.norm(delta))
    limit = a_max * dt
    if delta_norm > limit and delta_norm > 0.0:
        cmd = prev + delta * (limit / delta_norm)
        # 가속 제한 뒤에도 속도 상한을 넘지 않게 한 번 더 제한한다.
        norm = float(np.linalg.norm(cmd))
        if norm > v_max and norm > 0.0:
            cmd = cmd * (v_max / norm)
    return cmd
