"""보정 공통 기하(SPEC-600 4절). 길이 m, 각도 rad, 4x4 동차 변환."""
from __future__ import annotations

import numpy as np

from .errors import CalibrationError

# 퇴화 판정: 중심화한 점의 둘째 특이값 / 첫째 특이값
COLLINEAR_RATIO = 1e-3


def check_spread(points: np.ndarray, min_points: int, what: str) -> None:
    """점이 min_points 이상이고 한 직선 위에 있지 않은지 확인한다."""
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < min_points:
        raise CalibrationError('DEGENERATE', f'{what}: 3차원 점 {min_points}개 이상이 필요하다(받음 {len(points)})')
    s = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    if s[0] <= 0.0 or s[1] / s[0] < COLLINEAR_RATIO:
        raise CalibrationError('DEGENERATE', f'{what}: 점이 한 직선 위에 있다(σ2/σ1 = {s[1] / max(s[0], 1e-300):.2e})')


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def apply(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    return np.atleast_2d(points) @ T[:3, :3].T + T[:3, 3]


def quat_from_R(R: np.ndarray) -> np.ndarray:
    """회전 행렬 → quaternion (x, y, z, w), w ≥ 0."""
    m = R
    tr = np.trace(m)
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return -q if q[3] < 0.0 else q


def R_from_quat(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def base_to_camera_link(T_base_opt: np.ndarray, T_link_opt: np.ndarray) -> np.ndarray:
    """보정 결과(base ← 컬러 광학 프레임)와 드라이버 TF(camera_link ← 광학)로 base ← camera_link를 낸다."""
    return T_base_opt @ invert(T_link_opt)
