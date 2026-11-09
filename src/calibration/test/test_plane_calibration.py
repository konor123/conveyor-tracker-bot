import numpy as np
import pytest

from calibration.errors import CalibrationError
from calibration.plane_calibration import Plane, fit_direction, fit_plane

SEEDS = range(20)


def _tilted_plane(rng, n, noise, outlier_ratio):
    normal = np.array([0.02, -0.015, 1.0])
    normal /= np.linalg.norm(normal)
    offset = 0.050
    xy = rng.uniform([0.20, -0.30], [0.70, 0.30], size=(n, 2))
    # n·p = offset 을 z에 대해 푼다
    z = (offset - xy @ normal[:2]) / normal[2]
    pts = np.column_stack([xy, z]) + rng.normal(0.0, noise, size=(n, 3))
    k = int(round(n * outlier_ratio))
    idx = rng.choice(n, k, replace=False)
    pts[idx, 2] += rng.uniform(0.005, 0.040, size=k) * rng.choice([-1, 1], size=k)
    return pts, normal, offset


@pytest.mark.req("REQ-CAL-001")
@pytest.mark.tc("TC-CAL-001-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("seed", SEEDS)
def test_fit_plane_with_outliers(seed):
    rng = np.random.default_rng(seed)
    pts, normal, offset = _tilted_plane(rng, 200, 0.0002, 0.10)
    plane = fit_plane(pts, ransac_thresh=0.001, rng=np.random.default_rng(seed))
    assert isinstance(plane, Plane)
    ang = np.degrees(np.arccos(np.clip(plane.normal @ normal, -1.0, 1.0)))
    assert ang <= 0.1
    assert plane.offset == pytest.approx(offset, abs=0.0001)
    assert plane.normal[2] > 0.0                      # robot_base +z 쪽을 향한다
    assert plane.inlier_ratio == pytest.approx(0.90, abs=0.03)
    assert plane.rms_m < 0.0004


@pytest.mark.req("REQ-CAL-001")
@pytest.mark.tc("TC-CAL-001-01")
@pytest.mark.level("U")
def test_fit_plane_rejects_collinear_points():
    t = np.linspace(0.0, 0.3, 50)
    pts = np.column_stack([0.3 + t, 0.1 + 0.5 * t, np.full_like(t, 0.05)])
    with pytest.raises(CalibrationError) as e:
        fit_plane(pts, ransac_thresh=0.001)
    assert e.value.code == 'DEGENERATE'


@pytest.mark.req("REQ-CAL-002")
@pytest.mark.tc("TC-CAL-002-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("seed", SEEDS)
def test_fit_direction_straight_track(seed):
    rng = np.random.default_rng(seed)
    yaw = np.deg2rad(rng.uniform(-180.0, 180.0))
    d = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    s = np.linspace(0.0, 0.30, 90)                    # 30 fps × 3 s, 관측 구간 0.3 m
    pts = np.array([0.45, 0.0, 0.05]) + s[:, None] * d + rng.normal(0.0, 0.0003, size=(len(s), 3))
    est = fit_direction(pts)
    assert np.linalg.norm(est) == pytest.approx(1.0, abs=1e-12)
    ang = np.degrees(np.arccos(np.clip(est @ d, -1.0, 1.0)))
    assert ang <= 0.1                                 # 부호까지 맞아야 한다(시간 순서 = 진행 방향)


@pytest.mark.req("REQ-CAL-002")
@pytest.mark.tc("TC-CAL-002-01")
@pytest.mark.level("U")
def test_fit_direction_projects_onto_plane():
    rng = np.random.default_rng(0)
    plane = Plane(normal=np.array([0.0, 0.0, 1.0]), offset=0.05, rms_m=0.0, inlier_ratio=1.0, n=0)
    s = np.linspace(0.0, 0.3, 90)
    # 높이가 서서히 변하는 궤적이어도 평면 위 방향만 낸다
    pts = np.column_stack([0.3 + s, np.zeros_like(s), 0.05 + 0.02 * s]) + rng.normal(0, 0.0003, (90, 3))
    est = fit_direction(pts, plane=plane)
    assert abs(est[2]) < 1e-12
    assert np.degrees(np.arccos(est[0])) <= 0.1


@pytest.mark.level('U')
@pytest.mark.req('REQ-CAL-001')
@pytest.mark.tc('TC-CAL-001-91')
def test_fit_plane_and_direction_use_thin_svd(monkeypatch):
    """영상 평면 점(수만 개)에서 전체 SVD는 N×N 행렬(3만 점 7 GB)을 만든다(2026-11-09 LEGION 메모리 부족)."""
    calls = []
    svd = np.linalg.svd

    def thin_only(a, *args, **kwargs):
        if kwargs.get('compute_uv', True):
            calls.append(kwargs.get('full_matrices', True))
        return svd(a, *args, **kwargs)

    monkeypatch.setattr(np.linalg, 'svd', thin_only)
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(0, 0.3, 5000), rng.uniform(0, 0.3, 5000), np.full(5000, 0.05)])
    fit_plane(pts, 0.001, rng=rng)
    fit_direction(pts)
    assert calls and not any(calls)
