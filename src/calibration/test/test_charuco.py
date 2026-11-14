import cv2
import numpy as np
import pytest

import synth
from calibration.charuco import BoardSpec, board_points, board_pose, detect_charuco, render_board

SPEC = BoardSpec(squares_x=5, squares_y=7, square_m=0.030, marker_m=0.022, dictionary='DICT_4X4_50')
PPM = 4000.0          # 보드 그림 해상도 [px/m]
SS = 4                # 초표본 배율


def _render(T_cam_board, K, size=(synth.W, synth.H)):
    """왜곡 없는 카메라로 본 보드 영상. SS배 크기로 그린 뒤 INTER_AREA로 줄인다."""
    board = render_board(SPEC, PPM)
    # 보드 그림 픽셀 인덱스(픽셀 중심) → 보드 좌표 [m]. 칸 경계가 인덱스 x·PPM − 1과 x·PPM 사이에 있다
    A = np.array([[1 / PPM, 0, 0.5 / PPM], [0, 1 / PPM, 0.5 / PPM], [0, 0, 1]])
    H = K @ np.column_stack([T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]]) @ A
    S = np.array([[SS, 0, (SS - 1) / 2], [0, SS, (SS - 1) / 2], [0, 0, 1]])
    hi = cv2.warpPerspective(board, S @ H, (size[0] * SS, size[1] * SS), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    return cv2.resize(hi, size, interpolation=cv2.INTER_AREA)


def _pose(rng):
    R = synth.rot(rng.normal(size=3), np.deg2rad(rng.uniform(0, 20))) @ synth.rot([0, 0, 1], rng.uniform(-np.pi, np.pi))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = R @ -np.array([0.075, 0.105, 0.0]) + [rng.uniform(-0.1, 0.1), rng.uniform(-0.05, 0.05), 0.37]
    return T


@pytest.mark.parametrize("seed", range(5))
def test_corners_and_pose_on_rendered_board(seed):
    rng = np.random.default_rng(seed)
    T = _pose(rng)
    img = _render(T, synth.K)
    zero = np.zeros(5)
    ids, uv = detect_charuco(img, SPEC, synth.K, zero)
    assert len(ids) == 24
    truth = _proj(T, board_points(SPEC, ids), zero)
    assert np.abs(uv - truth).max() <= 0.2
    T_est, rms = board_pose(ids, uv, SPEC, synth.K, zero)
    assert rms <= 0.2
    assert np.linalg.norm(T_est[:3, 3] - T[:3, 3]) <= 0.001
    assert np.degrees(synth.rot_err_rad(T_est[:3, :3], T[:3, :3])) <= 0.2


def _proj(T_cam_board, pts, dist):
    rvec, _ = cv2.Rodrigues(T_cam_board[:3, :3])
    uv, _ = cv2.projectPoints(pts, rvec, T_cam_board[:3, 3], synth.K, dist)
    return uv.reshape(-1, 2)


def test_board_points_match_print_layout():
    ids = np.arange(24)
    p = board_points(SPEC, ids)
    assert p.shape == (24, 3)
    assert p[0] == pytest.approx([0.030, 0.030, 0.0])
    assert p[3] == pytest.approx([0.120, 0.030, 0.0])
    assert p[23] == pytest.approx([0.120, 0.180, 0.0])
