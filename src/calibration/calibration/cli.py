"""현장 보정 명령(SPEC-600 4.4, 4.6절). ROS 없이 영상 파일과 세션 기록으로 동작한다.

  ros2 run calibration charuco_view <영상.png> [--out 표시.png]
      보드 코너 id·자세·재투영 오차를 출력하고, 접촉할 코너 id를 그린 영상을 쓴다.
  ros2 run calibration calibrate_camera <session.yaml> --out <결과 폴더>
      camera_extrinsics.yaml, belt_plane.yaml, mat_plane.yaml, calib-verify.csv, report.yaml을 쓴다.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import cv2
import numpy as np
import yaml

from .calibration_file import make_extrinsics, plain, save_extrinsics
from .charuco import BoardSpec, board_pose, detect_charuco
from .errors import CalibrationError
from .eye_to_hand_calibration import BoardView, board_track_direction, solve_from_board_views
from .plane_calibration import Plane

try:
    from ament_index_python.packages import get_package_share_directory
    _SHARE = os.path.join(get_package_share_directory('calibration'), 'config')
except Exception:                                # 소스 트리에서 직접 실행
    _SHARE = os.path.join(os.path.dirname(__file__), os.pardir, 'config')

DEFAULT_BOARD = os.path.join(_SHARE, 'charuco_board.yaml')
DEFAULT_PROFILE = os.path.join(_SHARE, 'd455_color_1280x800.yaml')
DEFAULT_PARAMS = os.path.join(_SHARE, 'calibration.yaml')


def _load_yaml(path):
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def _intrinsics(profile: dict):
    K = np.array([[profile['fx'], 0.0, profile['cx']], [0.0, profile['fy'], profile['cy']], [0.0, 0.0, 1.0]])
    return K, np.asarray(profile['distortion'], float)


def _read_image(path, profile):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(path)
    if (img.shape[1], img.shape[0]) != (profile['width'], profile['height']):
        raise ValueError(f'{path}: 영상 크기 {img.shape[1]}x{img.shape[0]} ≠ 프로파일 '
                         f'{profile["width"]}x{profile["height"]}')
    return img


def charuco_view(argv=None):
    ap = argparse.ArgumentParser(description='ChArUco 코너 검출·자세 확인')
    ap.add_argument('image')
    ap.add_argument('--out', help='코너 id를 그린 영상 경로(기본: <영상>-ids.png)')
    ap.add_argument('--board', default=DEFAULT_BOARD)
    ap.add_argument('--profile', default=DEFAULT_PROFILE)
    a = ap.parse_args(argv)
    spec = BoardSpec.from_dict(_load_yaml(a.board))
    profile = _load_yaml(a.profile)
    K, dist = _intrinsics(profile)
    gray = _read_image(a.image, profile)
    ids, uv = detect_charuco(gray, spec, K, dist)
    print(f'코너 {len(ids)}/{spec.n_corners}개: {ids.tolist()}')
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for i, (u, v) in zip(ids, uv):
        cv2.circle(vis, (int(round(u)), int(round(v))), 4, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(vis, str(i), (int(u) + 5, int(v) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
    try:
        T, rms = board_pose(ids, uv, spec, K, dist)
        n = T[:3, 2]
        print(f'재투영 rms {rms:.3f} px')
        print(f'보드 원점(카메라 광학 프레임) [m]: {np.round(T[:3, 3], 4).tolist()}')
        print(f'광학 중심 → 보드면 거리 {abs(T[:3, 3] @ n):.4f} m, 광축 대비 보드 기울기 '
              f'{np.degrees(np.arccos(min(1.0, abs(n[2])))):.2f}°')
        cv2.drawFrameAxes(vis, K, dist, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], 0.03)
    except CalibrationError as e:
        print(f'자세 계산 불가: {e}')
    out = a.out or os.path.splitext(a.image)[0] + '-ids.png'
    cv2.imwrite(out, vis)
    print(f'표시 영상: {out}')
    return 0


def _plane_doc(cal_id, date, plane: Plane, thickness, frame, extra=None):
    # 보드 윗면 → 놓인 면: 법선 방향으로 두께만큼 내린다
    doc = {'calibration_id': cal_id, 'date': str(date), 'frame': frame,
           'normal': plane.normal.tolist(), 'offset_m': float(plane.offset - thickness),
           'board_thickness_m': float(thickness),
           'residuals': {'rms_m': plane.rms_m, 'inlier_ratio': plane.inlier_ratio, 'n_points_count': plane.n}}
    doc.update(extra or {})
    return plain(doc)


def calibrate_camera(argv=None):
    ap = argparse.ArgumentParser(description='ChArUco + TCP 접촉 eye-to-hand 보정')
    ap.add_argument('session', help='세션 YAML(SPEC-600 4.4절 형식)')
    ap.add_argument('--out', required=True, help='결과 폴더')
    ap.add_argument('--params', default=DEFAULT_PARAMS)
    a = ap.parse_args(argv)
    ses = _load_yaml(a.session)
    base_dir = os.path.dirname(os.path.abspath(a.session))
    rel = lambda p: p if os.path.isabs(p) else os.path.join(base_dir, p)   # noqa: E731
    board_doc = _load_yaml(rel(ses.get('board', DEFAULT_BOARD)))
    spec = BoardSpec.from_dict(board_doc)
    thickness = float(ses.get('board_thickness_m', board_doc.get('board_thickness_m', 0.0)))
    profile = _load_yaml(rel(ses.get('camera_profile', DEFAULT_PROFILE)))
    params = _load_yaml(a.params)
    K, dist = _intrinsics(profile)
    parent = ses.get('parent_frame', 'base_link')
    child = ses.get('child_frame', 'camera_color_optical_frame')

    views = []
    for v in ses['views']:
        ids, uv = detect_charuco(_read_image(rel(v['image']), profile), spec, K, dist)
        touches = {int(k): v_ for k, v_ in (v.get('touches_m') or {}).items()}
        missing = [k for k in touches if k not in set(ids.tolist())]
        if missing:
            print(f'경고: {v["name"]} 접촉 코너 {missing}가 영상에서 검출되지 않았다', file=sys.stderr)
        views.append(BoardView(ids=ids, uv=uv, touch_ids=np.array(list(touches), int),
                               touch_base=np.array(list(touches.values()), float).reshape(-1, 3),
                               role=v.get('role', 'fit'), surface=v.get('surface', 'belt'), name=v['name']))
        print(f'{v["name"]}: 코너 {len(ids)}개, 접촉 {len(touches)}개')

    T, rep = solve_from_board_views(
        views, spec, K, dist, ransac_thresh=float(params.get('plane_ransac_thresh_m', 0.001)),
        touch_rms_max=float(params.get('touch_rms_max_m', 0.001)),
        verify_kw={'min_points': int(params.get('verify_min_points_count', 20)),
                   'p95_max_m': float(params.get('verify_xy_p95_max_m', 0.001)),
                   'max_m': float(params.get('verify_xy_max_m', 0.002))})
    cal_id, date = ses['calibration_id'], ses['date']
    os.makedirs(a.out, exist_ok=True)

    belt_extra = {}
    if ses.get('belt_track'):
        frames = [detect_charuco(_read_image(rel(p), profile), spec, K, dist) for p in ses['belt_track']]
        d = board_track_direction(frames, spec, K, dist, T, plane=rep['planes'].get('belt'))
        belt_extra = {'direction': d.tolist(), 'direction_frames_count': len(frames)}
        print(f'벨트 진행 방향(robot_base): {np.round(d, 4).tolist()}')
    for surface, plane in rep['planes'].items():
        extra = belt_extra if surface == 'belt' else None
        with open(os.path.join(a.out, f'{surface}_plane.yaml'), 'w') as fh:
            yaml.safe_dump(_plane_doc(cal_id, date, plane, thickness, parent, extra), fh, sort_keys=False)

    ver = rep['verify']
    residuals = {'method': rep['method'], 'n_fit_views_count': rep['n_fit_views'],
                 'n_fit_points_count': rep['n_fit_points'], 'rms_px': rep['rms_px'],
                 'touch_rms_max_m': rep['touch_rms_max_m'], 'bad_touch_views': rep['bad_touch_views'],
                 'verify': {k: ver[k] for k in ('n', 'xy_p50_m', 'xy_p95_m', 'xy_max_m', 'pass')}}
    save_extrinsics(make_extrinsics(cal_id, date, T, profile, residuals, parent, child),
                    os.path.join(a.out, 'camera_extrinsics.yaml'))
    with open(os.path.join(a.out, f'calib-verify-{date}.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['point', 'xy_error_m'])
        for i, e in enumerate(ver['errors_m']):
            w.writerow([i, f'{e:.6f}'])
    with open(os.path.join(a.out, 'report.yaml'), 'w') as fh:
        yaml.safe_dump(plain({**residuals, 'touch_rms_m': rep['touch_rms_m']}), fh, sort_keys=False,
                       allow_unicode=True)

    print(f'재투영 rms {rep["rms_px"]:.3f} px, 접촉 잔차 최대 {rep["touch_rms_max_m"]:.5f} m')
    if rep['bad_touch_views']:
        print(f'잘못 접촉한 장면(제외됨): {rep["bad_touch_views"]}')
    print(f'검증 {ver["n"]}점: XY p95 {ver["xy_p95_m"]:.5f} m, 최대 {ver["xy_max_m"]:.5f} m → '
          f'{"합격" if ver["pass"] else "불합격"}(TC-BEN-012-01)')
    return 0 if ver['pass'] else 2


if __name__ == '__main__':      # pragma: no cover
    sys.exit(charuco_view())
