"""파지한 공깃돌 중심 ↔ TCP 오프셋.

TCP 자세 (R_i, t_i)(robot_base ← tcp)와 관측한 공깃돌 중심 p_i(robot_base)로 p_i = R_i·o + t_i 의 o를 최소제곱으로 푼다.
관측하지 않은 성분은 p에 NaN을 넣는다(카메라 평면 교차 결과의 z 등).
"""
from __future__ import annotations

import numpy as np

from .errors import CalibrationError

# 정규방정식 조건수 상한. 넘으면 그 성분은 관측되지 않은 것으로 본다.
MAX_COND = 1e6


def fit_tcp_offset(R: np.ndarray, t: np.ndarray, p: np.ndarray, fixed: dict | None = None
                   ) -> tuple[np.ndarray, float]:
    """반환 (o [m] tcp 프레임, 사용한 성분 잔차 rms [m]). fixed={축: 값}은 해당 성분을 고정한다."""
    R = np.asarray(R, float).reshape(-1, 3, 3)
    t = np.asarray(t, float).reshape(-1, 3)
    p = np.asarray(p, float).reshape(-1, 3)
    fixed = dict(fixed or {})
    free = [k for k in range(3) if k not in fixed]
    A = R.reshape(-1, 3)                              # 행 = (자세 i, 성분 r)
    b = (p - t).reshape(-1)
    keep = ~np.isnan(b)
    A, b = A[keep], b[keep]
    o = np.zeros(3)
    for k, v in fixed.items():
        o[k] = float(v)
    b = b - A @ o
    A = A[:, free]
    if len(b) < len(free) or (free and np.linalg.cond(A.T @ A) > MAX_COND):
        raise CalibrationError('DEGENERATE', f'fit_tcp_offset: 성분 {free}을 풀 수 있는 자세 변화가 없다')
    if free:
        x, *_ = np.linalg.lstsq(A, b, rcond=None)
        o[free] = x
        b = b - A @ x
    return o, float(np.sqrt(np.mean(b ** 2))) if len(b) else 0.0
