"""고정 카메라 ↔ robot_base 보정.

반환 변환 T는 robot_base ← camera_color_optical_frame(카메라 점을 robot_base로 옮긴다).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .charuco import BoardSpec, board_points, board_pose
from .errors import CalibrationError
from .geometry import apply, check_spread, invert, make_T
from .plane_calibration import Plane, fit_direction, fit_plane

PNP_MIN_POINTS = 6
# 합격 기준(제안값)
VERIFY_MIN_POINTS = 20
VERIFY_XY_P95_M = 0.001
VERIFY_XY_MAX_M = 0.002


def solve_eye_to_hand(cam_pts: np.ndarray, base_pts: np.ndarray) -> tuple[np.ndarray, float]:
    """3D-3D 대응점(Kabsch). base ≈ R·cam + t. 반환 (T, 대응점 거리 rms [m])."""
    cam = np.asarray(cam_pts, float)
    base = np.asarray(base_pts, float)
    if cam.shape != base.shape:
        raise CalibrationError('DEGENERATE', f'solve_eye_to_hand: 점 개수 불일치 {cam.shape} {base.shape}')
    check_spread(base, 3, 'solve_eye_to_hand')
    check_spread(cam, 3, 'solve_eye_to_hand')
    cc, cb = cam.mean(axis=0), base.mean(axis=0)
    U, _, Vt = np.linalg.svd((cam - cc).T @ (base - cb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    t = cb - R @ cc
    res = base - (cam @ R.T + t)
    return make_T(R, t), float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))


def solve_eye_to_hand_pnp(base_pts: np.ndarray, uv: np.ndarray, K: np.ndarray, dist: np.ndarray
                          ) -> tuple[np.ndarray, float]:
    """TCP 접촉 좌표(robot_base) + 표식 픽셀 중심(왜곡 보정 전) → (T, 재투영 rms [px]).

    SQPnP로 초기값을 구하고 Levenberg-Marquardt로 재투영 오차를 줄인다. 평면 배치도 받는다.
    """
    base = np.asarray(base_pts, float)
    px = np.asarray(uv, float).reshape(-1, 2)
    if len(base) != len(px):
        raise CalibrationError('DEGENERATE', f'solve_eye_to_hand_pnp: 점 개수 불일치 {len(base)} {len(px)}')
    check_spread(base, PNP_MIN_POINTS, 'solve_eye_to_hand_pnp')
    K = np.asarray(K, float)
    dist = np.asarray(dist, float)
    ok, rvec, tvec = cv2.solvePnP(base, px, K, dist, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise CalibrationError('DEGENERATE', 'solve_eye_to_hand_pnp: solvePnP 실패')
    rvec, tvec = cv2.solvePnPRefineLM(base, px, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(base, rvec, tvec, K, dist)
    res = proj.reshape(-1, 2) - px
    R_cam_base, _ = cv2.Rodrigues(rvec)
    T = invert(make_T(R_cam_base, tvec.ravel()))
    # 모든 점이 카메라 앞(광축 +z)에 있어야 한다
    z_cam = (base - T[:3, 3]) @ T[:3, :3][:, 2]
    if np.any(z_cam <= 0.0):
        raise CalibrationError('DEGENERATE', 'solve_eye_to_hand_pnp: 카메라 뒤에 있는 점이 있다')
    return T, float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))


def pixel_to_plane(uv: np.ndarray, K: np.ndarray, dist: np.ndarray, T: np.ndarray, plane: Plane) -> np.ndarray:
    """픽셀(왜곡 보정 전) → 광선과 평면의 교점(robot_base) [m]."""
    px = np.asarray(uv, float).reshape(-1, 1, 2)
    # 기본 undistortPoints는 5회 반복이라 시야 가장자리에서 부족하다
    xn = cv2.undistortPointsIter(px, np.asarray(K, float), np.asarray(dist, float), None, None,
                                 (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-12)).reshape(-1, 2)
    rays = np.column_stack([xn, np.ones(len(xn))]) @ T[:3, :3].T
    origin = T[:3, 3]
    denom = rays @ plane.normal
    if np.any(np.abs(denom) < 1e-9):
        raise CalibrationError('DEGENERATE', 'pixel_to_plane: 광선이 평면과 평행하다')
    s = (plane.offset - origin @ plane.normal) / denom
    return origin + s[:, None] * rays


def verify_xy(true_pts: np.ndarray, est_pts: np.ndarray, min_points: int = VERIFY_MIN_POINTS,
              p95_max_m: float = VERIFY_XY_P95_M, max_m: float = VERIFY_XY_MAX_M) -> dict:
    """검증 점 XY 오차 통계 판정(기본 n ≥ 20, p95 ≤ 1 mm, 최대 ≤ 2 mm)."""
    e = np.linalg.norm(np.asarray(est_pts, float)[:, :2] - np.asarray(true_pts, float)[:, :2], axis=1)
    n = int(len(e))
    p50 = float(np.percentile(e, 50)) if n else float('nan')
    p95 = float(np.percentile(e, 95)) if n else float('nan')
    mx = float(e.max()) if n else float('nan')
    ok = n >= min_points and p95 <= p95_max_m + 1e-12 and mx <= max_m + 1e-12
    return {'n': n, 'xy_p50_m': p50, 'xy_p95_m': p95, 'xy_max_m': mx, 'pass': bool(ok), 'errors_m': e.tolist()}


# ---------- ChArUco 보드 장면 묶음 ----------

TOUCH_RMS_MAX_M = 0.001       # 장면별 접촉 잔차가 이보다 크면 잘못 접촉한 장면으로 표시한다
MIN_TOUCHES = 3


@dataclass
class BoardView:
    """보드 한 장면. ids·uv: 검출 코너, touch_ids·touch_base: TCP로 접촉한 코너와 robot_base 좌표 [m]."""
    ids: np.ndarray
    uv: np.ndarray
    touch_ids: np.ndarray
    touch_base: np.ndarray
    role: str = 'fit'          # fit | verify
    surface: str = 'belt'      # belt | mat | spacer(받침) 등. 평면 맞춤 묶음 이름
    name: str = ''


def _board_to_base(view: BoardView, spec: BoardSpec) -> tuple[np.ndarray, float]:
    """접촉점으로 robot_base ← board. 반환 (T, 접촉 잔차 rms [m])."""
    if len(view.touch_ids) < MIN_TOUCHES:
        raise CalibrationError('DEGENERATE', f'{view.name}: 접촉 코너 {MIN_TOUCHES}개 이상이 필요하다')
    return solve_eye_to_hand(board_points(spec, view.touch_ids), np.asarray(view.touch_base, float))


def solve_from_board_views(views: list[BoardView], spec: BoardSpec, K: np.ndarray, dist: np.ndarray,
                           ransac_thresh: float = 0.001, touch_rms_max: float = TOUCH_RMS_MAX_M,
                           verify_kw: dict | None = None) -> tuple[np.ndarray, dict]:
    """fit 장면: 접촉으로 보드를 robot_base에 놓고 검출 코너 전체로 PnP.
    verify 장면: 접촉 코너의 픽셀을 접촉 높이의 수평면에 교차시켜 XY 오차를 낸다.
    평면: surface별로 fit 장면 코너(robot_base)를 맞춘다(보드 윗면 기준, 두께 보정은 호출자가 한다).
    """
    fit_base, fit_uv, touch_rms, bad, by_surface = [], [], {}, [], {}
    for v in views:
        if v.role != 'fit':
            continue
        T_bb, rms = _board_to_base(v, spec)
        touch_rms[v.name] = rms
        if rms > touch_rms_max:
            bad.append(v.name)
            continue
        pts = apply(T_bb, board_points(spec, v.ids))
        fit_base.append(pts)
        fit_uv.append(np.asarray(v.uv, float).reshape(-1, 2))
        by_surface.setdefault(v.surface, []).append(pts)
    if not fit_base:
        raise CalibrationError('DEGENERATE', 'solve_from_board_views: 쓸 수 있는 fit 장면이 없다')
    base = np.vstack(fit_base)
    T, rms_px = solve_eye_to_hand_pnp(base, np.vstack(fit_uv), K, dist)
    true_v, est_v = [], []
    for v in views:
        if v.role != 'verify':
            continue
        uv_of = {int(i): u for i, u in zip(np.asarray(v.ids).ravel(), np.asarray(v.uv).reshape(-1, 2))}
        for tid, p in zip(np.asarray(v.touch_ids).ravel(), np.asarray(v.touch_base, float)):
            if int(tid) not in uv_of:
                continue
            plane = Plane(normal=np.array([0.0, 0.0, 1.0]), offset=float(p[2]), rms_m=0.0, inlier_ratio=1.0, n=1)
            est_v.append(pixel_to_plane(uv_of[int(tid)], K, dist, T, plane)[0])
            true_v.append(p)
    planes = {s: fit_plane(np.vstack(p), ransac_thresh) for s, p in by_surface.items()}
    report = {
        'method': 'charuco_touch_pnp',
        'n_fit_views': len(fit_base),
        'n_fit_points': int(len(base)),
        'rms_px': rms_px,
        'touch_rms_m': touch_rms,
        'touch_rms_max_m': max(touch_rms.values()),
        'bad_touch_views': bad,
        'verify': verify_xy(np.array(true_v).reshape(-1, 3), np.array(est_v).reshape(-1, 3), **(verify_kw or {})),
        'planes': planes,
    }
    return T, report


def board_track_direction(frames, spec: BoardSpec, K: np.ndarray, dist: np.ndarray, T: np.ndarray,
                          plane: Plane | None = None) -> np.ndarray:
    """벨트로 옮겨지는 보드를 시간 순서로 찍은 (ids, uv) 장면들 → robot_base 진행 방향 단위 벡터."""
    # 보드 원점(바깥 모서리)보다 보드 중심이 자세 오차의 영향을 덜 받는다. 장면마다 같은 점을 쓴다
    center = np.array([spec.squares_x * spec.square_m / 2.0, spec.squares_y * spec.square_m / 2.0, 0.0])
    centers = []
    for ids, uv in frames:
        T_cb, _ = board_pose(ids, uv, spec, K, dist)
        centers.append(apply(T @ T_cb, center)[0])
    return fit_direction(np.array(centers), plane=plane)
