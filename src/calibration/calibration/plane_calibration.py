"""매트·벨트 평면 맞춤과 벨트 진행 방향(SPEC-600 4.1, 4.2절). 입력 점은 robot_base 기준 [m]."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .errors import CalibrationError
from .geometry import check_spread

RANSAC_ITERATIONS = 300
MIN_INLIER_RATIO = 0.5


@dataclass(frozen=True)
class Plane:
    """n·p = offset. normal은 단위 벡터이고 up 쪽(기본 robot_base +z)을 향한다."""
    normal: np.ndarray
    offset: float
    rms_m: float
    inlier_ratio: float
    n: int

    def distance(self, points: np.ndarray) -> np.ndarray:
        return np.atleast_2d(points) @ self.normal - self.offset

    def height_at(self, x: float, y: float) -> float:
        """(x, y)에서 평면의 z [m]."""
        return float((self.offset - self.normal[0] * x - self.normal[1] * y) / self.normal[2])

    def project(self, points: np.ndarray) -> np.ndarray:
        p = np.atleast_2d(points)
        return p - np.outer(self.distance(p), self.normal)


def _lsq_plane(points: np.ndarray):
    c = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - c)
    n = vt[2]
    return n, float(n @ c)


def fit_plane(points: np.ndarray, ransac_thresh: float, up=(0.0, 0.0, 1.0), rng=None) -> Plane:
    """RANSAC(3점) → 인라이어 최소제곱. ransac_thresh: 인라이어 거리 [m]."""
    pts = np.asarray(points, float)
    check_spread(pts, 3, 'fit_plane')
    rng = np.random.default_rng(0) if rng is None else rng
    best = None
    for _ in range(RANSAC_ITERATIONS):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-12:
            continue
        n /= norm
        inl = np.abs(pts @ n - n @ a) <= ransac_thresh
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.mean() < MIN_INLIER_RATIO:
        ratio = 0.0 if best is None else best.mean()
        raise CalibrationError('NO_CONSENSUS', f'fit_plane: 인라이어 비율 {ratio:.2f} < {MIN_INLIER_RATIO}')
    inl = best
    for _ in range(3):                        # 맞춘 평면으로 인라이어를 다시 고른다
        n, d = _lsq_plane(pts[inl])
        new = np.abs(pts @ n - d) <= ransac_thresh
        if new.sum() < 3 or np.array_equal(new, inl):
            break
        inl = new
    n, d = _lsq_plane(pts[inl])
    if n @ np.asarray(up, float) < 0.0:
        n, d = -n, -d
    res = pts[inl] @ n - d
    return Plane(normal=n, offset=d, rms_m=float(np.sqrt(np.mean(res ** 2))),
                 inlier_ratio=float(inl.mean()), n=int(len(pts)))


def fit_direction(points: np.ndarray, plane: Plane | None = None) -> np.ndarray:
    """시간 순서로 정렬된 이동 점 → 진행 방향 단위 벡터. plane이 있으면 평면 위로 투영한다."""
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        raise CalibrationError('DEGENERATE', f'fit_direction: 3차원 점 2개 이상이 필요하다(받음 {len(pts)})')
    if plane is not None:
        pts = plane.project(pts)
    c = pts.mean(axis=0)
    _, s, vt = np.linalg.svd(pts - c)
    if s[0] <= 1e-9:
        raise CalibrationError('DEGENERATE', 'fit_direction: 점이 움직이지 않았다')
    d = vt[0]
    if plane is not None:
        d = d - (d @ plane.normal) * plane.normal
    d /= np.linalg.norm(d)
    if (pts[-1] - pts[0]) @ d < 0.0:
        d = -d
    return d
