import numpy as np
import pytest

import synth
from calibration.errors import CalibrationError
from calibration.eye_to_hand_calibration import (pixel_to_plane, solve_eye_to_hand, solve_eye_to_hand_pnp,
                                                 verify_xy)
from calibration.plane_calibration import Plane

SEEDS = range(20)
BELT = Plane(normal=np.array([0.0, 0.0, 1.0]), offset=synth.BELT_Z, rms_m=0.0, inlier_ratio=1.0, n=0)


@pytest.mark.req("REQ-CAL-003")
@pytest.mark.tc("TC-CAL-003-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("seed", SEEDS)
def test_rigid_known_transform(seed):
    rng = np.random.default_rng(seed)
    T = synth.camera_pose()
    base = synth.work_points(rng, 20)
    cam = synth.transform(synth.inv(T), base) + rng.normal(0.0, 0.0003, size=base.shape)
    T_est, rms = solve_eye_to_hand(cam, base)
    # 판정은 작업면 점 오차. 카메라 원점 이동 오차는 0.35 m 지렛대로 회전 오차가 커지므로 참고 기준만 둔다
    check = synth.work_points(rng, 30, z_values=(synth.BELT_Z,))
    est = synth.transform(T_est, synth.transform(synth.inv(T), check))
    assert np.linalg.norm(est - check, axis=1).max() <= 0.0005
    assert np.linalg.norm(T_est[:3, 3] - T[:3, 3]) <= 0.001
    assert np.degrees(synth.rot_err_rad(T_est[:3, :3], T[:3, :3])) <= 0.1
    assert 0.0 < rms < 0.001
    assert np.linalg.det(T_est[:3, :3]) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.req("REQ-CAL-003")
@pytest.mark.tc("TC-CAL-003-02")
@pytest.mark.level("U")
def test_rigid_rejects_collinear():
    t = np.linspace(0.0, 0.3, 20)[:, None]
    base = np.array([0.3, 0.0, 0.05]) + t * np.array([1.0, 0.2, 0.0])
    cam = synth.transform(synth.inv(synth.camera_pose()), base)
    with pytest.raises(CalibrationError) as e:
        solve_eye_to_hand(cam, base)
    assert e.value.code == 'DEGENERATE'


@pytest.mark.req("REQ-CAL-003")
@pytest.mark.tc("TC-CAL-003-02")
@pytest.mark.level("U")
def test_pnp_rejects_collinear_and_too_few():
    T = synth.camera_pose()
    t = np.linspace(0.0, 0.3, 20)[:, None]
    base = np.array([0.3, -0.1, 0.05]) + t * np.array([1.0, 0.5, 0.0])
    uv = synth.project(T, base)
    with pytest.raises(CalibrationError) as e:
        solve_eye_to_hand_pnp(base, uv, synth.K, synth.DIST)
    assert e.value.code == 'DEGENERATE'
    good = synth.work_points(np.random.default_rng(0), 5)
    with pytest.raises(CalibrationError) as e:
        solve_eye_to_hand_pnp(good, synth.project(T, good), synth.K, synth.DIST)
    assert e.value.code == 'DEGENERATE'


@pytest.mark.req("REQ-CAL-003")
@pytest.mark.tc("TC-CAL-003-03")
@pytest.mark.level("U")
@pytest.mark.parametrize("seed", SEEDS)
def test_pnp_touch_points(seed):
    """TCP 접촉 좌표(robot_base) + 표식 픽셀 중심 → 카메라 자세. 판정은 보정에 쓰지 않은 점의 벨트면 XY 오차."""
    rng = np.random.default_rng(seed)
    T = synth.camera_pose()
    base = synth.work_points(rng, 24)
    uv = synth.project(T, base) + rng.normal(0.0, 0.3, size=(24, 2))
    T_est, rms_px = solve_eye_to_hand_pnp(base, uv, synth.K, synth.DIST)
    assert 0.0 < rms_px < 0.5
    check = synth.work_points(rng, 30, z_values=(synth.BELT_Z,))
    est = pixel_to_plane(synth.project(T, check), synth.K, synth.DIST, T_est, BELT)
    stats = verify_xy(check, est)
    assert stats['n'] == 30
    assert stats['xy_max_m'] <= 0.0005


@pytest.mark.req("REQ-CAL-006")
@pytest.mark.tc("TC-CAL-006-01")
@pytest.mark.level("U")
def test_pixel_to_plane_roundtrip():
    rng = np.random.default_rng(1)
    T = synth.camera_pose()
    pts = synth.work_points(rng, 50, z_values=(synth.BELT_Z,), margin_px=5)
    est = pixel_to_plane(synth.project(T, pts), synth.K, synth.DIST, T, BELT)
    assert np.abs(est - pts).max() <= 1e-6
    assert np.abs(est[:, 2] - synth.BELT_Z).max() <= 1e-9


@pytest.mark.req("REQ-CAL-006")
@pytest.mark.tc("TC-CAL-006-01")
@pytest.mark.level("U")
def test_verify_xy_stats_and_gate():
    true = np.zeros((20, 3))
    est = np.zeros((20, 3))
    est[:, 0] = np.linspace(0.0, 0.0019, 20)          # 0 ~ 1.9 mm
    est[:, 2] = 0.01                                  # z 차이는 XY 오차에 넣지 않는다
    s = verify_xy(true, est)
    assert s['n'] == 20
    assert s['xy_max_m'] == pytest.approx(0.0019, abs=1e-12)
    assert s['xy_p95_m'] == pytest.approx(np.percentile(est[:, 0], 95), abs=1e-12)
    assert s['pass'] is False                         # p95 1.8 mm > 1 mm (TC-BEN-012-01)
    s = verify_xy(true[:20], est[:20] * 0.5)
    assert s['pass'] is True                          # p95 0.9 mm, 최대 0.95 mm
    s = verify_xy(true[:10], est[:10] * 0.0)
    assert s['pass'] is False                         # 20점 미만은 판정 불가
