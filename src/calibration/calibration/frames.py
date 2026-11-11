"""평면 보정 결과 → 고정 좌표계(SPEC-600 4.9절). ROS와 무관한 순수 모듈."""
from __future__ import annotations

import numpy as np

from .geometry import make_T


def plane_frame(normal, offset_m: float, x_hint, origin_xy) -> np.ndarray:
    """robot_base ← 평면 좌표계 4×4.

    평면 `normal · p = offset_m`. z축 = 위쪽 법선, x축 = x_hint의 평면 투영,
    원점 = robot_base (origin_xy)를 지나는 수직선과 평면의 교점.
    """
    n = np.asarray(normal, float)
    scale = np.linalg.norm(n)
    n, d = n / scale, float(offset_m) / scale
    if n[2] < 0:                                   # 같은 평면, 위쪽 법선
        n, d = -n, -d
    if n[2] <= 0:
        raise ValueError('수평 법선(z 성분 0)으로는 수직선과 만나는 원점을 정할 수 없다')
    h = np.asarray(x_hint, float)
    x = h - (h @ n) * n
    if np.linalg.norm(x) < 1e-6:
        raise ValueError('x_hint가 평면 법선과 평행하다')
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    ox, oy = (float(v) for v in origin_xy)
    oz = (d - n[0] * ox - n[1] * oy) / n[2]
    return make_T(np.column_stack([x, y, n]), np.array([ox, oy, oz]))
