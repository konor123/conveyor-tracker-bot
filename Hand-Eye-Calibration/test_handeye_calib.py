"""Hand-Eye-Calibration/handeye_calib.py 단위 시험. 합성 ArUco 영상과 알려진 X·Y로 추정값을 검증한다(ROS·실장비 없음)."""
import csv
import os
import sys

import cv2
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
import handeye_calib as hc  # noqa: E402

W, H = 1280, 800
K = np.array([[640.0, 0.0, 640.0], [0.0, 640.0, 400.0], [0.0, 0.0, 1.0]])
DIST = np.zeros(5)
SIZE = 0.040
MARKER_ID = 7
CELL, QUIET = 20, 1                      # 4x4 표식 = 6x6 칸(테두리 포함), 둘레 여백 1칸
EDGE = 6 * CELL

Y_TRUE = hc.make_T(Rotation.from_euler('xyz', [170.0, 5.0, -80.0], degrees=True).as_matrix(), [0.80, 0.05, 0.60])
X_TRUE = hc.make_T(Rotation.from_euler('xyz', [175.0, -10.0, 95.0], degrees=True).as_matrix(), [0.00, -0.04, 0.15])


def _marker_canvas() -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, EDGE)
    else:
        img = cv2.aruco.drawMarker(dictionary, MARKER_ID, EDGE)
    return cv2.copyMakeBorder(img, QUIET * CELL, QUIET * CELL, QUIET * CELL, QUIET * CELL,
                              cv2.BORDER_CONSTANT, value=255)


def _render(C: np.ndarray) -> np.ndarray:
    """camera <- marker 자세 C에서 본 표식 영상."""
    canvas = _marker_canvas()
    src = np.array([[QUIET * CELL, QUIET * CELL], [QUIET * CELL + EDGE, QUIET * CELL],
                    [QUIET * CELL + EDGE, QUIET * CELL + EDGE], [QUIET * CELL, QUIET * CELL + EDGE]], np.float32)
    dst = cv2.projectPoints(hc.marker_corners_3d(SIZE), cv2.Rodrigues(C[:3, :3])[0], C[:3, 3], K, DIST)[0]
    Hm = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(canvas, Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=255)


def _marker_poses(n, seed=1):
    """카메라 앞 0.35~0.5 m, 표식 면이 카메라를 향한 자세 n개(camera <- marker)."""
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        ang = [180.0 + rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(-180, 180)]
        R = Rotation.from_euler('xyz', ang, degrees=True).as_matrix()
        out.append(hc.make_T(R, [rng.uniform(-0.05, 0.05), rng.uniform(-0.04, 0.04), rng.uniform(0.35, 0.5)]))
    return out


def _write_session(tmp_path, n, seed=1, with_role=False, blank=()):
    """합성 세션 폴더(영상 + poses.csv + camera.yaml). 반환 (poses.csv, camera.yaml)."""
    rows = []
    for i, C in enumerate(_marker_poses(n, seed)):
        B = Y_TRUE @ C @ hc.invert(X_TRUE)       # B·X = Y·C
        img = np.full((H, W), 255, np.uint8) if i in blank else _render(C)
        cv2.imwrite(str(tmp_path / f'p{i:02d}.png'), img)
        row = {'pose_id': f'p{i:02d}', 'images': f'p{i:02d}.png',
               'x': B[0, 3] * 1e3, 'y': B[1, 3] * 1e3, 'z': B[2, 3] * 1e3}
        row.update({f'r{a}{b}': B[a, b] for a in range(3) for b in range(3)})
        if with_role:
            row['role'] = 'fit' if i < n - 5 else 'verify'
        rows.append(row)
    poses = tmp_path / 'poses.csv'
    with open(poses, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    cam = tmp_path / 'camera.yaml'
    cam.write_text(yaml.safe_dump({'width': W, 'height': H, 'fx': 640.0, 'fy': 640.0, 'cx': 640.0, 'cy': 400.0,
                                   'distortion': [0.0] * 5}))
    return poses, cam


def _args(poses, cam, out, *extra):
    return hc.build_parser().parse_args(['--poses', str(poses), '--camera', str(cam), '--out', str(out),
                                         '--marker-id', str(MARKER_ID), '--marker-size', str(SIZE),
                                         '--rot-format', 'matrix', *extra])


def test_doosan_zyz_matches_controller_rotation_matrix():
    # 2026-11-07 pose-00: 제어기가 준 posx(ZYZ)와 같은 시각의 rotm. 오일러 규약이 틀리면 여기서 잡힌다.
    row = {'a': 179.9971923828125, 'b': -165.07630920410156, 'c': 89.99830627441406}
    R_ref = np.array([[-2.0482153558987193e-05, -0.9662696719169617, 0.25753238797187805],
                      [-1.0, 1.7830092474468984e-05, -1.2633272490347736e-05],
                      [7.615321464982117e-06, -0.25753238797187805, -0.9662696719169617]])
    assert np.allclose(hc.rotation_from_row(row, 'euler-zyz'), R_ref, atol=1e-6)


def test_numeric_core_recovers_x_y_exactly_without_noise():
    P = hc.marker_corners_3d(SIZE)
    poses = []
    for C in _marker_poses(25):
        B = Y_TRUE @ C @ hc.invert(X_TRUE)
        uv, _ = hc.reproject(B, X_TRUE, hc.invert(Y_TRUE), P, K, DIST)
        poses.append({'B': B, 'C': C, 'uv': uv})
    res = hc.calibrate(poses[:20], poses[20:], P, K, DIST, 'auto')
    assert np.linalg.norm(res['Y'][:3, 3] - Y_TRUE[:3, 3]) < 1e-6
    assert hc.rot_err_deg(res['Y'][:3, :3], Y_TRUE[:3, :3]) < 1e-4
    assert np.linalg.norm(res['X'][:3, 3] - X_TRUE[:3, 3]) < 1e-6
    assert res['fit']['rms_px'] < 1e-4 and res['holdout']['rms_px'] < 1e-4
    assert res['init'] == 'PARK'                         # 모두 같은 해로 수렴하면 PARK 우선


def test_end_to_end_from_rendered_images(tmp_path):
    poses, cam = _write_session(tmp_path, 30)
    out = tmp_path / 'out'
    res = hc.run(_args(poses, cam, out, '--holdout-last', '8'))
    assert np.linalg.norm(res['Y'][:3, 3] - Y_TRUE[:3, 3]) < 0.003          # 합성 영상 서브픽셀 오차 수준
    assert hc.rot_err_deg(res['Y'][:3, :3], Y_TRUE[:3, :3]) < 0.5
    assert np.linalg.norm(res['X'][:3, 3] - X_TRUE[:3, 3]) < 0.003
    assert res['fit']['rms_px'] < 0.5 and res['holdout']['rms_px'] < 0.5
    assert res['fit']['n_poses'] == 22 and res['holdout']['n_poses'] == 8
    doc = yaml.safe_load((out / 'handeye_result.yaml').read_text())
    T = np.array(doc['Y_base_from_camera']['matrix_4x4'])
    assert np.allclose(T @ np.array(doc['Y_inverse']['matrix_4x4']), np.eye(4), atol=1e-9)
    assert doc['Y_base_from_camera']['matrix_direction'] == 'base_link <- camera_color_optical_frame'
    assert doc['X_tcp_from_marker']['matrix_direction'] == 'controller_tcp <- aruco_marker_7'
    assert len((out / 'per_pose_errors.csv').read_text().splitlines()) == 31


def test_role_column_defines_holdout_and_skips_undetected_pose(tmp_path):
    poses, cam = _write_session(tmp_path, 30, with_role=True, blank={3})
    res = hc.run(_args(poses, cam, tmp_path / 'out'))
    assert res['fit']['n_poses'] == 24 and res['holdout']['n_poses'] == 5     # 25개 fit에서 검출 실패 1개 제외
    doc = yaml.safe_load((tmp_path / 'out' / 'handeye_result.yaml').read_text())
    assert doc['poses']['skipped_no_detection'] == ['p03']


def test_too_few_poses_is_rejected(tmp_path):
    poses, cam = _write_session(tmp_path, 8)
    with pytest.raises(hc.CalibError, match='FIT 자세가'):
        hc.run(_args(poses, cam, tmp_path / 'out'))


def test_missing_rotation_columns_is_reported(tmp_path):
    poses, cam = _write_session(tmp_path, 8)
    a = _args(poses, cam, tmp_path / 'out')
    a.rot_format = 'quat'
    with pytest.raises(hc.CalibError, match='열이 없다'):
        hc.run(a)


def test_image_size_mismatch_is_rejected(tmp_path):
    poses, cam = _write_session(tmp_path, 12)
    doc = yaml.safe_load(cam.read_text())
    doc['width'] = 640
    cam.write_text(yaml.safe_dump(doc))
    with pytest.raises(hc.CalibError, match='영상 크기'):
        hc.run(_args(poses, cam, tmp_path / 'out'))


def test_main_returns_error_code_for_bad_input(tmp_path, capsys):
    poses, cam = _write_session(tmp_path, 8)
    rc = hc.main(['--poses', str(poses), '--camera', str(cam), '--out', str(tmp_path / 'o'),
                  '--marker-id', str(MARKER_ID), '--marker-size', str(SIZE), '--rot-format', 'matrix'])
    assert rc == 2 and '오류' in capsys.readouterr().err
