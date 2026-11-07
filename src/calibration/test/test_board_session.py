import numpy as np
import pytest

import synth
from calibration.charuco import BoardSpec, board_points
from calibration.eye_to_hand_calibration import BoardView, board_track_direction, solve_from_board_views

SPEC = BoardSpec(squares_x=5, squares_y=7, square_m=0.030, marker_m=0.022, dictionary='DICT_4X4_50')
TOUCH_IDS = np.array([0, 3, 20, 23])        # 안쪽 코너 네 귀퉁이
ALL_IDS = np.arange(SPEC.n_corners)


def _board_pose(rng, z, center_xy=None):
    """robot_base ← board. 보드면 = z, 보드 z축은 아래(-z_base)."""
    yaw = rng.uniform(-np.pi, np.pi)
    R = synth.rot([0, 0, 1], yaw) @ synth.rot([1, 0, 0], np.pi)
    c = center_xy if center_xy is not None else [rng.uniform(0.33, 0.57), rng.uniform(-0.20, 0.25)]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([c[0], c[1], z]) - R @ np.array([0.075, 0.105, 0.0])
    return T


def _view(rng, T_base_board, role, surface, touch_noise=0.0002, px_noise=0.2):
    T_cam = synth.camera_pose()
    base = synth.transform(T_base_board, board_points(SPEC, ALL_IDS))
    uv = synth.project(T_cam, base) + rng.normal(0.0, px_noise, size=(len(ALL_IDS), 2))
    touch = synth.transform(T_base_board, board_points(SPEC, TOUCH_IDS)) + rng.normal(0.0, touch_noise, (len(TOUCH_IDS), 3))
    return BoardView(ids=ALL_IDS, uv=uv, touch_ids=TOUCH_IDS, touch_base=touch, role=role, surface=surface,
                     name=f'{surface}-{role}')


def _session(rng):
    """SPEC-600 4.4절 배치: 면(벨트·매트·받침)마다 fit 4장면, 접촉 σ 0.2 mm, 벨트 위 verify 5장면."""
    views = []
    for z, surface in ((synth.BELT_Z, 'belt'), (synth.MAT_Z, 'mat'), (synth.SPACER_Z, 'spacer')):
        for _ in range(4):
            views.append(_view(rng, _board_pose(rng, z), 'fit', surface))
    for _ in range(5):                                     # 검증 4점 × 5장면 = 20점
        views.append(_view(rng, _board_pose(rng, synth.BELT_Z), 'verify', 'belt'))
    return views


@pytest.mark.req("REQ-CAL-003")
@pytest.mark.tc("TC-CAL-003-04")
@pytest.mark.level("U")
@pytest.mark.parametrize("seed", range(20))
def test_board_session_meets_verify_gate(seed):
    rng = np.random.default_rng(seed)
    T, rep = solve_from_board_views(_session(rng), SPEC, synth.K, synth.DIST)
    T_true = synth.camera_pose()
    assert np.degrees(synth.rot_err_rad(T[:3, :3], T_true[:3, :3])) <= 0.2   # 참고 기준
    assert rep['verify']['n'] == 20
    assert rep['verify']['pass'] is True
    assert rep['verify']['xy_max_m'] <= 0.001
    assert rep['n_fit_points'] == 12 * 24
    assert rep['touch_rms_max_m'] < 0.001
    assert rep['planes']['belt'].height_at(0.45, 0.02) == pytest.approx(synth.BELT_Z, abs=0.0003)
    assert rep['planes']['mat'].height_at(0.45, 0.02) == pytest.approx(synth.MAT_Z, abs=0.0003)


@pytest.mark.req("REQ-CAL-003")
@pytest.mark.tc("TC-CAL-003-04")
@pytest.mark.level("U")
def test_bad_touch_is_flagged():
    rng = np.random.default_rng(0)
    views = _session(rng)
    views[0].touch_base[1] += [0.004, 0.0, 0.0]            # 4 mm 잘못 접촉
    _, rep = solve_from_board_views(views, SPEC, synth.K, synth.DIST)
    assert views[0].name in rep['bad_touch_views']


@pytest.mark.req("REQ-CAL-002")
@pytest.mark.tc("TC-CAL-002-02")
@pytest.mark.level("U")
@pytest.mark.parametrize("seed", range(20))
def test_belt_direction_from_board_track(seed):
    """벨트 위 보드를 벨트로 옮기며 찍은 장면 → 진행 방향. 보정된 T로 카메라 자세를 robot_base로 옮긴다."""
    rng = np.random.default_rng(seed)
    yaw = np.deg2rad(rng.uniform(-180, 180))
    d = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    T0 = _board_pose(rng, synth.BELT_Z, center_xy=[0.45, 0.02])
    T_cam = synth.camera_pose()
    frames = []
    for s in np.linspace(-0.08, 0.08, 6):
        Tb = T0.copy()
        Tb[:3, 3] += s * d
        uv = synth.project(T_cam, synth.transform(Tb, board_points(SPEC, ALL_IDS))) + rng.normal(0, 0.2, (24, 2))
        frames.append((ALL_IDS, uv))
    est = board_track_direction(frames, SPEC, synth.K, synth.DIST, T_cam)
    assert np.degrees(np.arccos(np.clip(est @ d, -1.0, 1.0))) <= 0.2
