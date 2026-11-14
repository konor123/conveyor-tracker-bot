#!/usr/bin/env python3
"""고정 카메라(eye-to-hand) Hand-Eye 보정. ROS·저장소 코드 없이 이 파일 하나로 동작한다.

로봇 TCP에 붙인 ArUco 표식을 여러 자세에서 촬영한 영상과 그때의 로봇 자세로 두 변환을 구한다.

  Y  base <- camera   로봇 베이스에서 본 카메라(광학 프레임)
  X  tcp  <- marker   TCP에서 본 표식

관계식은 B_i · X = Y · C_i 이다(B_i = base <- tcp 로봇 자세, C_i = camera <- marker 영상 자세).
카메라가 로봇 손목에 붙은 eye-in-hand 배치는 식이 달라 지원하지 않는다.

방법(2026-11-07 토요일 보정과 같다): 자세마다 표식 PnP → cv2.calibrateHandEye 초기값
→ 회전 고정 병진 최소제곱 → 4코너 픽셀 재투영 번들 정밀화. 초기값 5종을 모두 돌려 FIT 잔차가
가장 작은 것을 고른다(같으면 PARK). HOLDOUT 자세는 학습·선택에 쓰지 않고 검증에만 쓴다.

필요 패키지: numpy, opencv-contrib-python(cv2.aruco), scipy, pyyaml. 선택: matplotlib(--plot).

사용 예:
  python3 handeye_calib.py --poses poses.csv --camera camera.yaml \\
      --marker-id 25 --marker-size 0.040 --out result/

poses.csv (헤더 필수, UTF-8). 한 줄 = 로봇이 멈춘 한 자세.
  pose_id   자세 이름
  images    그 자세에서 찍은 영상 경로. 여러 장이면 ';'로 구분하고 glob(*)도 된다.
            상대 경로는 --image-dir(기본: CSV가 있는 폴더) 기준. 여러 장이면 코너를 평균한다.
  x,y,z     TCP 위치(기본 mm, --length-unit m 으로 변경)
  회전      --rot-format 에 따라 다음 열을 쓴다.
              euler-zyz  a,b,c (deg)  R = Rz(a)·Ry(b)·Rz(c) — Doosan posx 기본값
              euler-zyx / euler-xyz   a,b,c (deg)  회전축 순서대로 적은 내재(intrinsic) 오일러
              rotvec     a,b,c (rad)  회전벡터
              quat       qx,qy,qz,qw
              matrix     r00,r01,r02,r10,...,r22  (제어기가 주는 실제 회전행렬이 있으면 이것을 권장)
  role      선택. fit 또는 verify. 없으면 --holdout-last 로 정한다.

camera.yaml: fx, fy, cx, cy, distortion(k1 k2 p1 p2 k3, plumb_bob) 키. ROS CameraInfo의 k(9개)·d도 받는다.
영상은 이 내부 파라미터를 얻은 해상도와 같아야 한다.

산출물(--out):
  handeye_result.yaml  Y, X(4x4·역행렬·quaternion xyzw), 잔차, 1σ 불확도, 경고, 입력 설정
  per_pose_errors.csv  자세별 검출 장수·코너 지터·재투영 오차
  residuals.png        --plot 지정 시
"""
from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HAND_EYE_METHODS = [
    ('TSAI', cv2.CALIB_HAND_EYE_TSAI),
    ('PARK', cv2.CALIB_HAND_EYE_PARK),
    ('HORAUD', cv2.CALIB_HAND_EYE_HORAUD),
    ('ANDREFF', cv2.CALIB_HAND_EYE_ANDREFF),
    ('DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS),
]
MIN_FIT_POSES = 10                 # 이보다 적으면 12개 자유도(X·Y)를 믿기 어렵다
MIN_ROTATION_SPREAD_DEG = 30.0     # 자세 사이 회전이 이보다 작으면 X·Y가 풀리지 않는다
HOLDOUT_WARN_RATIO = 2.0           # HOLDOUT 오차가 FIT의 이 배수를 넘으면 과적합 의심
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)


class CalibError(Exception):
    pass


# ---------- 기하 ----------

def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def invert(T: np.ndarray) -> np.ndarray:
    return make_T(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def project_so3(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(np.asarray(R, float).reshape(3, 3))
    return U @ np.diag([1.0, 1.0, np.linalg.det(U @ Vt)]) @ Vt


def rot_err_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R1.T @ R2) - 1.0) / 2.0, -1.0, 1.0))))


def se3_to_vec6(T: np.ndarray) -> np.ndarray:
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


def vec6_to_se3(v: np.ndarray) -> np.ndarray:
    return make_T(cv2.Rodrigues(np.asarray(v[:3], float))[0], v[3:6])


def matrix_record(T: np.ndarray, parent: str, child: str) -> dict:
    """parent <- child 변환 한 개를 YAML에 쓸 수 있는 형태로 만든다."""
    return {
        'matrix_direction': f'{parent} <- {child}',
        'parent_frame': parent,
        'child_frame': child,
        'translation_unit': 'm',
        'translation_m': [float(v) for v in T[:3, 3]],
        'rotation_xyzw': [float(v) for v in Rotation.from_matrix(T[:3, :3]).as_quat()],
        'matrix_4x4': [[float(v) for v in row] for row in T],
    }


# ---------- 입력 읽기 ----------

ROT_COLUMNS = {
    'euler-zyz': ['a', 'b', 'c'],
    'euler-zyx': ['a', 'b', 'c'],
    'euler-xyz': ['a', 'b', 'c'],
    'rotvec': ['a', 'b', 'c'],
    'quat': ['qx', 'qy', 'qz', 'qw'],
    'matrix': [f'r{i}{j}' for i in range(3) for j in range(3)],
}


def rotation_from_row(row: dict, rot_format: str) -> np.ndarray:
    v = [float(row[c]) for c in ROT_COLUMNS[rot_format]]
    if rot_format.startswith('euler-'):
        # 대문자 = 내재(intrinsic) 회전: 'ZYZ'면 R = Rz(a)·Ry(b)·Rz(c)
        return Rotation.from_euler(rot_format[-3:].upper(), v, degrees=True).as_matrix()
    if rot_format == 'rotvec':
        return Rotation.from_rotvec(v).as_matrix()
    if rot_format == 'quat':
        return Rotation.from_quat(v).as_matrix()
    return project_so3(np.array(v).reshape(3, 3))


def load_camera(path: str) -> tuple[np.ndarray, np.ndarray, dict]:
    with open(path, encoding='utf-8') as fh:
        cam = yaml.safe_load(fh) or {}
    try:
        if 'fx' in cam:
            K = np.array([[cam['fx'], 0.0, cam['cx']], [0.0, cam['fy'], cam['cy']], [0.0, 0.0, 1.0]], float)
            dist = np.asarray(cam.get('distortion', cam.get('d', [0.0] * 5)), float)
        else:
            K = np.asarray(cam['k'], float).reshape(3, 3)
            dist = np.asarray(cam.get('d', [0.0] * 5), float)
    except KeyError as e:
        raise CalibError(f'{path}: 내부 파라미터 키가 없다({e}). fx/fy/cx/cy/distortion 또는 k/d가 필요하다') from e
    size = (int(cam['width']), int(cam['height'])) if 'width' in cam and 'height' in cam else None
    return K, dist, {'image_size': size}


def load_poses(path: str, rot_format: str, length_unit: str, image_dir: str | None) -> list[dict]:
    base_dir = image_dir or os.path.dirname(os.path.abspath(path))
    scale = 1e-3 if length_unit == 'mm' else 1.0
    poses = []
    with open(path, newline='', encoding='utf-8-sig') as fh:
        reader = csv.DictReader(fh)
        need = ['pose_id', 'images', 'x', 'y', 'z'] + ROT_COLUMNS[rot_format]
        missing = [c for c in need if c not in (reader.fieldnames or [])]
        if missing:
            raise CalibError(f'{path}: 열이 없다 {missing} (--rot-format {rot_format})')
        for n, row in enumerate(reader, start=2):
            try:
                t = np.array([float(row['x']), float(row['y']), float(row['z'])]) * scale
                R = rotation_from_row(row, rot_format)
            except ValueError as e:
                raise CalibError(f'{path}:{n}: 숫자를 읽을 수 없다({e})') from e
            files = []
            for pat in row['images'].split(';'):
                pat = pat.strip()
                if not pat:
                    continue
                pat = pat if os.path.isabs(pat) else os.path.join(base_dir, pat)
                files += sorted(glob.glob(pat)) or [pat]
            role = (row.get('role') or '').strip().lower()
            if role not in ('', 'fit', 'verify'):
                raise CalibError(f'{path}:{n}: role은 fit 또는 verify여야 한다(받음 {role!r})')
            poses.append({'id': row['pose_id'], 'files': files, 'role': role, 'B': make_T(R, t)})
    if not poses:
        raise CalibError(f'{path}: 자세가 없다')
    return poses


# ---------- 표식 검출·PnP ----------

def marker_corners_3d(size_m: float) -> np.ndarray:
    """ArUco 코너 순서(좌상·우상·우하·좌하)의 표식 좌표. 표식 중심 원점, z는 표식 면의 법선."""
    h = size_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], float)


def make_detector(dict_name: str):
    if not hasattr(cv2.aruco, dict_name):
        raise CalibError(f'알 수 없는 ArUco 사전 {dict_name} (예: DICT_4X4_50)')
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    # OpenCV 4.6의 DetectorParameters()는 세그폴트가 나므로 _create를 먼저 쓴다(4.7+는 _create가 없다)
    params = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create')
              else cv2.aruco.DetectorParameters())
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE      # 서브픽셀은 아래에서 따로 한다
    if hasattr(cv2.aruco, 'ArucoDetector'):                           # OpenCV >= 4.7
        det = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda gray: det.detectMarkers(gray)[:2]
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


def detect_marker(gray: np.ndarray, detect, marker_id: int) -> np.ndarray | None:
    corners, ids = detect(gray)
    if ids is None or marker_id not in ids.ravel().tolist():
        return None
    c = corners[ids.ravel().tolist().index(marker_id)].reshape(4, 2).astype(np.float32)
    return cv2.cornerSubPix(gray, c.copy(), (3, 3), (-1, -1), SUBPIX_CRITERIA).astype(float)


def pnp_marker(uv: np.ndarray, P: np.ndarray, K: np.ndarray, dist: np.ndarray) -> tuple[np.ndarray, float]:
    """평면 표식 PnP(IPPE_SQUARE 후보 0 + LM). 반환 (camera <- marker, 재투영 rms [px])."""
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(P, uv, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or len(rvecs) == 0:
        raise CalibError('solvePnP(IPPE_SQUARE) 실패')
    rvec, tvec = cv2.solvePnPRefineLM(P, uv, K, dist, rvecs[0].copy(), tvecs[0].copy())
    proj = cv2.projectPoints(P, rvec, tvec, K, dist)[0].reshape(4, 2)
    rms = float(np.sqrt(np.mean(np.sum((proj - uv) ** 2, axis=1))))
    return make_T(cv2.Rodrigues(rvec)[0], tvec.ravel()), rms


def observe_poses(poses: list[dict], a, K, dist, expect_size) -> list[dict]:
    """각 자세의 영상에서 코너를 검출·평균해 pose['uv'], pose['C']를 채운다. 검출 실패 자세는 제외한다."""
    detect = make_detector(a.dict)
    P = marker_corners_3d(a.marker_size)
    kept, skipped = [], []
    for p in poses:
        stack = []
        for f in p['files']:
            gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise CalibError(f'{p["id"]}: 영상을 읽을 수 없다 {f}')
            if expect_size and (gray.shape[1], gray.shape[0]) != expect_size:
                raise CalibError(f'{f}: 영상 크기 {gray.shape[1]}x{gray.shape[0]} ≠ camera.yaml {expect_size[0]}x{expect_size[1]}')
            c = detect_marker(gray, detect, a.marker_id)
            if c is not None:
                stack.append(c)
        p['n_images'], p['n_detected'] = len(p['files']), len(stack)
        if len(stack) < a.min_detections:
            skipped.append(p['id'])
            continue
        arr = np.stack(stack)
        p['uv'] = arr.mean(axis=0)
        p['jitter_rms_px'] = float(np.sqrt(np.mean(np.sum((arr - p['uv']) ** 2, axis=-1))))
        p['C'], p['pnp_rms_px'] = pnp_marker(p['uv'], P, K, dist)
        if p['C'][2, 3] <= 0:
            raise CalibError(f'{p["id"]}: 표식이 카메라 뒤에 있다. 표식 크기·코너 순서·내부 파라미터를 확인한다')
        kept.append(p)
    if skipped:
        print(f'경고: 표식 id {a.marker_id} 검출 실패로 제외한 자세 {len(skipped)}개: {skipped}', file=sys.stderr)
    return kept


# ---------- 추정 ----------

def solve_plain_handeye(B_list, C_list, method) -> tuple[np.ndarray, np.ndarray]:
    """cv2.calibrateHandEye로 Y를 구하고, X는 자세별 후보 inv(B)·Y·C의 평균으로 만든다."""
    B_inv = [invert(B) for B in B_list]
    R_Y, t_Y = cv2.calibrateHandEye([T[:3, :3] for T in B_inv], [T[:3, 3].reshape(3, 1) for T in B_inv],
                                    [C[:3, :3] for C in C_list], [C[:3, 3].reshape(3, 1) for C in C_list],
                                    method=method)
    Y = make_T(project_so3(R_Y), t_Y.ravel())
    cands = [invert(B) @ Y @ C for B, C in zip(B_list, C_list)]
    X = make_T(project_so3(sum(c[:3, :3] for c in cands)), np.mean([c[:3, 3] for c in cands], axis=0))
    return X, Y


def solve_decoupled_ls(B_list, C_list, R_Y_init) -> tuple[np.ndarray, np.ndarray, dict]:
    """회전을 고정하고 [R_B, -I]·[t_X; t_Y] = R_Y·t_C − t_B 를 선형 최소제곱으로 푼다."""
    R_Y = project_so3(R_Y_init)
    R_X = project_so3(sum(B[:3, :3].T @ R_Y @ C[:3, :3] for B, C in zip(B_list, C_list)))
    A = np.vstack([np.hstack([B[:3, :3], -np.eye(3)]) for B in B_list])
    b = np.concatenate([R_Y @ C[:3, 3] - B[:3, 3] for B, C in zip(B_list, C_list)])
    sol, _, _, s = np.linalg.lstsq(A, b, rcond=None)
    return make_T(R_X, sol[:3]), make_T(R_Y, sol[3:]), {'translation_cond': float(s[0] / s[-1])}


def reproject(B, X, Y_inv, P, K, dist) -> tuple[np.ndarray, np.ndarray]:
    C_pred = Y_inv @ B @ X
    uv = cv2.projectPoints(P, cv2.Rodrigues(C_pred[:3, :3])[0], C_pred[:3, 3], K, dist)[0]
    return uv.reshape(4, 2), C_pred


def bundle_refine(B_fit, uv_fit, X0, Y0, P, K, dist) -> tuple[np.ndarray, np.ndarray, dict]:
    """FIT 코너의 재투영 오차를 줄이도록 X·Y 12개 파라미터를 LM으로 정밀화하고 1σ를 계산한다."""
    def residuals(p12: np.ndarray) -> np.ndarray:
        X, Y_inv = vec6_to_se3(p12[:6]), vec6_to_se3(p12[6:])
        return np.concatenate([(reproject(B, X, Y_inv, P, K, dist)[0] - uv).ravel() for B, uv in zip(B_fit, uv_fit)])

    opt = least_squares(residuals, np.concatenate([se3_to_vec6(X0), se3_to_vec6(invert(Y0))]),
                        method='lm', ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=5000)
    X, Y = vec6_to_se3(opt.x[:6]), invert(vec6_to_se3(opt.x[6:]))

    # 해석하기 쉽게 (X, Y) 파라미터에 대한 수치 자코비안으로 공분산을 낸다
    def residuals_xy(p12: np.ndarray) -> np.ndarray:
        Xc, Yc = vec6_to_se3(p12[:6]), vec6_to_se3(p12[6:])
        return np.concatenate([(reproject(B, Xc, invert(Yc), P, K, dist)[0] - uv).ravel()
                               for B, uv in zip(B_fit, uv_fit)])

    p_xy = np.concatenate([se3_to_vec6(X), se3_to_vec6(Y)])
    r0 = residuals_xy(p_xy)
    eps = 1e-7
    J = np.zeros((len(r0), 12))
    for k in range(12):
        dp = np.zeros(12)
        dp[k] = eps
        J[:, k] = (residuals_xy(p_xy + dp) - r0) / eps
    s_J = np.linalg.svd(J, compute_uv=False)
    dof = max(1, len(r0) - 12)
    cov = float(np.sum(r0 ** 2) / dof) * np.linalg.inv(J.T @ J)
    std = np.sqrt(np.diag(cov))
    return X, Y, {
        'converged': bool(opt.success), 'dof': int(dof), 'jacobian_condition': float(s_J[0] / s_J[-1]),
        'X_rot_1sigma_deg': [float(np.degrees(v)) for v in std[:3]],
        'X_trans_1sigma_mm': [float(v * 1e3) for v in std[3:6]],
        'Y_rot_1sigma_deg': [float(np.degrees(v)) for v in std[6:9]],
        'Y_trans_1sigma_mm': [float(v * 1e3) for v in std[9:]],
    }


def evaluate(poses: list[dict], X, Y, P, K, dist) -> dict:
    """코너별 유클리드 재투영 오차 통계(4코너 평균 영상 기준)."""
    Y_inv = invert(Y)
    errs = []
    for p in poses:
        uv_pred, C_pred = reproject(p['B'], X, Y_inv, P, K, dist)
        e = np.linalg.norm(uv_pred - p['uv'], axis=1)
        p['err_px'], p['depth_m'] = e, float(C_pred[2, 3])
        errs.append(e)
    if not errs:
        return {'n_poses': 0}
    all_e = np.concatenate(errs)
    pose_rms = np.array([np.sqrt(np.mean(e ** 2)) for e in errs])
    return {
        'n_poses': len(poses), 'n_corners': int(len(all_e)),
        'rms_px': float(np.sqrt(np.mean(all_e ** 2))), 'p95_px': float(np.percentile(all_e, 95)),
        'max_px': float(all_e.max()), 'pose_rms_median_px': float(np.median(pose_rms)),
        'pose_rms_max_px': float(pose_rms.max()),
        'marker_depth_min_m': float(min(p['depth_m'] for p in poses)),
        'marker_depth_max_m': float(max(p['depth_m'] for p in poses)),
    }


def rotation_spread_deg(B_list) -> float:
    """첫 자세 대비 가장 큰 TCP 회전각. 작으면 hand-eye가 퇴화한다."""
    return max(rot_err_deg(B_list[0][:3, :3], B[:3, :3]) for B in B_list)


def calibrate(fit: list[dict], hold: list[dict], P, K, dist, init: str) -> dict:
    B_fit = [p['B'] for p in fit]
    C_fit = [p['C'] for p in fit]
    uv_fit = [p['uv'] for p in fit]
    methods = HAND_EYE_METHODS if init == 'auto' else [m for m in HAND_EYE_METHODS if m[0] == init]
    tried, best = {}, None
    for name, code in methods:
        try:
            _, Y_he = solve_plain_handeye(B_fit, C_fit, code)
            X_ls, Y_ls, ls_info = solve_decoupled_ls(B_fit, C_fit, Y_he[:3, :3])
            X, Y, info = bundle_refine(B_fit, uv_fit, X_ls, Y_ls, P, K, dist)
        except (cv2.error, np.linalg.LinAlgError) as e:
            tried[name] = {'failed': str(e).splitlines()[-1]}
            continue
        fit_m = evaluate(fit, X, Y, P, K, dist)
        tried[name] = {'fit_rms_px': fit_m['rms_px'], 'converged': info['converged']}
        # 선택은 FIT 목적함수만 본다. 수치가 같으면(1e-10 px) PARK를 우선한다.
        better = best is None or fit_m['rms_px'] < best['fit']['rms_px'] - 1e-10
        tie_park = name == 'PARK' and abs(fit_m['rms_px'] - best['fit']['rms_px']) <= 1e-10
        if better or tie_park:
            best = {'init': name, 'X': X, 'Y': Y, 'info': info, 'ls': ls_info, 'fit': fit_m}
    if best is None:
        raise CalibError(f'모든 hand-eye 초기화가 실패했다: {tried}')
    best['tried'] = tried
    best['holdout'] = evaluate(hold, best['X'], best['Y'], P, K, dist)
    return best


# ---------- 출력 ----------

def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 16), b''):
            h.update(chunk)
    return h.hexdigest()


def write_outputs(out: str, a, res: dict, fit, hold, skipped_ids, warnings, camera_path) -> None:
    os.makedirs(out, exist_ok=True)
    X, Y = res['X'], res['Y']
    doc = {
        'status': 'CANDIDATE — 영상 재투영 오차만 확인했다. 실제 TCP 접촉 등 독립 물리 검증 전에는 운용에 쓰지 않는다',
        'method': f'aruco_pnp + calibrateHandEye({res["init"]}) + decoupled_ls + pixel_bundle_lm',
        'equation': 'B_i · X = Y · C_i   (B: base<-tcp, C: camera<-marker)',
        'Y_base_from_camera': matrix_record(Y, a.base_frame, a.camera_frame),
        'Y_inverse': matrix_record(invert(Y), a.camera_frame, a.base_frame),
        'X_tcp_from_marker': matrix_record(X, a.tcp_frame, f'aruco_marker_{a.marker_id}'),
        'X_inverse': matrix_record(invert(X), f'aruco_marker_{a.marker_id}', a.tcp_frame),
        'fit_reprojection_px': res['fit'],
        'holdout_reprojection_px': res['holdout'] if res['holdout']['n_poses'] else None,
        'uncertainty_1sigma': {k: res['info'][k] for k in
                               ('Y_rot_1sigma_deg', 'Y_trans_1sigma_mm', 'X_rot_1sigma_deg', 'X_trans_1sigma_mm')},
        'conditioning': {'translation_ls': res['ls']['translation_cond'],
                         'bundle_jacobian': res['info']['jacobian_condition'], 'dof': res['info']['dof']},
        'init_methods_tried': res['tried'],
        'warnings': warnings,
        'poses': {'fit': [p['id'] for p in fit], 'holdout': [p['id'] for p in hold], 'skipped_no_detection': skipped_ids},
        'settings': {
            'marker_id': a.marker_id, 'marker_dictionary': a.dict, 'marker_black_side_m': a.marker_size,
            'rot_format': a.rot_format, 'length_unit': a.length_unit, 'init': a.init,
            'camera_file': os.path.abspath(camera_path), 'camera_sha256': sha256_file(camera_path),
            'poses_file': os.path.abspath(a.poses), 'poses_sha256': sha256_file(a.poses),
            'opencv': cv2.__version__, 'numpy': np.__version__,
        },
    }
    with open(os.path.join(out, 'handeye_result.yaml'), 'w', encoding='utf-8') as fh:
        yaml.safe_dump(doc, fh, sort_keys=False, allow_unicode=True)
    with open(os.path.join(out, 'per_pose_errors.csv'), 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['pose_id', 'role', 'images_detected', 'images_total', 'corner_jitter_rms_px', 'single_pose_pnp_rms_px',
                    'model_rms_px', 'model_max_px', 'c0_px', 'c1_px', 'c2_px', 'c3_px', 'marker_depth_m'])
        for role, group in (('fit', fit), ('holdout', hold)):
            for p in group:
                e = p['err_px']
                w.writerow([p['id'], role, p['n_detected'], p['n_images'], f'{p["jitter_rms_px"]:.4f}',
                            f'{p["pnp_rms_px"]:.4f}', f'{np.sqrt(np.mean(e ** 2)):.4f}', f'{e.max():.4f}',
                            *[f'{v:.4f}' for v in e], f'{p["depth_m"]:.5f}'])


def plot_residuals(out: str, fit, hold) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rms = [np.sqrt(np.mean(p['err_px'] ** 2)) for p in fit + hold]
    colors = ['#2b6cb0'] * len(fit) + ['#dd6b20'] * len(hold)
    fig, ax = plt.subplots(figsize=(9, 4), dpi=130)
    ax.bar(range(len(rms)), rms, color=colors)
    ax.set_xlabel(f'pose order (blue FIT {len(fit)}, orange HOLDOUT {len(hold)})')
    ax.set_ylabel('reprojection RMS [px]')
    ax.grid(True, linestyle=':', alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'residuals.png'))
    plt.close(fig)


# ---------- 실행 ----------

def split_poses(poses: list[dict], holdout_last: int) -> tuple[list[dict], list[dict]]:
    if any(p['role'] for p in poses):
        if not all(p['role'] for p in poses):
            raise CalibError('role 열은 모든 자세에 채우거나 전부 비워야 한다')
        return [p for p in poses if p['role'] == 'fit'], [p for p in poses if p['role'] == 'verify']
    if holdout_last:
        return poses[:-holdout_last], poses[-holdout_last:]
    return poses, []


def run(a) -> dict:
    K, dist, cam_meta = load_camera(a.camera)
    poses = load_poses(a.poses, a.rot_format, a.length_unit, a.image_dir)
    all_ids = [p['id'] for p in poses]
    poses = observe_poses(poses, a, K, dist, cam_meta['image_size'])
    skipped = [i for i in all_ids if i not in {p['id'] for p in poses}]
    fit, hold = split_poses(poses, a.holdout_last)
    if len(fit) < MIN_FIT_POSES:
        raise CalibError(f'FIT 자세가 {len(fit)}개뿐이다(최소 {MIN_FIT_POSES}개, 권장 20~30개 이상)')
    P = marker_corners_3d(a.marker_size)

    warnings = []
    spread = rotation_spread_deg([p['B'] for p in fit])
    if spread < MIN_ROTATION_SPREAD_DEG:
        warnings.append(f'자세 사이 최대 회전이 {spread:.1f}°뿐이다. 손목을 더 다양하게 돌려 다시 촬영한다')
    if not hold:
        warnings.append('HOLDOUT 자세가 없다. FIT 오차만으로는 과적합을 알 수 없다(--holdout-last 또는 role 열 사용)')

    res = calibrate(fit, hold, P, K, dist, a.init)
    if not res['info']['converged']:
        warnings.append('번들 정밀화가 수렴 판정을 받지 못했다')
    if res['info']['jacobian_condition'] > 1e4:
        warnings.append(f'번들 자코비안 조건수 {res["info"]["jacobian_condition"]:.0f}: X·Y가 서로 보상된다. 자세를 더 다양하게 한다')
    if res['holdout'].get('n_poses') and res['holdout']['rms_px'] > HOLDOUT_WARN_RATIO * res['fit']['rms_px']:
        warnings.append(f'HOLDOUT 오차({res["holdout"]["rms_px"]:.2f} px)가 FIT({res["fit"]["rms_px"]:.2f} px)의 '
                        f'{HOLDOUT_WARN_RATIO:.0f}배를 넘는다. 과적합이거나 자세·영상 대응이 어긋났을 수 있다')
    spread_inits = [t['fit_rms_px'] for t in res['tried'].values() if 'fit_rms_px' in t]
    if len(spread_inits) > 1 and max(spread_inits) - min(spread_inits) > 0.05:
        warnings.append('초기화 방법마다 FIT 오차가 0.05 px 넘게 다르다. 국소해일 수 있으니 자세 입력을 점검한다')

    write_outputs(a.out, a, res, fit, hold, skipped, warnings, a.camera)
    if a.plot:
        plot_residuals(a.out, fit, hold)
    res['warnings'] = warnings
    return res


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description='고정 카메라 Hand-Eye 보정(ArUco 표식, eye-to-hand)',
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--poses', required=True, help='자세 CSV(모듈 설명 참고)')
    ap.add_argument('--camera', required=True, help='내부 파라미터 YAML')
    ap.add_argument('--out', required=True, help='결과 폴더')
    ap.add_argument('--marker-id', type=int, required=True)
    ap.add_argument('--marker-size', type=float, required=True, help='표식 검은 테두리 한 변 [m]')
    ap.add_argument('--dict', default='DICT_4X4_50', help='cv2.aruco 사전 이름')
    ap.add_argument('--rot-format', choices=list(ROT_COLUMNS), default='euler-zyz')
    ap.add_argument('--length-unit', choices=['mm', 'm'], default='mm', help='CSV x,y,z 단위')
    ap.add_argument('--image-dir', help='상대 영상 경로의 기준 폴더(기본: CSV 폴더)')
    ap.add_argument('--holdout-last', type=int, default=0, help='role 열이 없을 때 마지막 N개 자세를 검증용으로 쓴다')
    ap.add_argument('--min-detections', type=int, default=1, help='자세당 최소 검출 영상 수')
    ap.add_argument('--init', choices=['auto'] + [m[0] for m in HAND_EYE_METHODS], default='auto')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--tcp-frame', default='controller_tcp')
    ap.add_argument('--camera-frame', default='camera_color_optical_frame')
    ap.add_argument('--plot', action='store_true', help='residuals.png 저장(matplotlib 필요)')
    return ap


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    try:
        res = run(a)
    except (CalibError, FileNotFoundError) as e:
        print(f'오류: {e}', file=sys.stderr)
        return 2
    print(f'초기화 {res["init"]}: FIT {res["fit"]["n_poses"]}자세 RMS {res["fit"]["rms_px"]:.3f} px '
          f'(p95 {res["fit"]["p95_px"]:.3f}, max {res["fit"]["max_px"]:.3f})')
    if res['holdout']['n_poses']:
        h = res['holdout']
        print(f'HOLDOUT {h["n_poses"]}자세 RMS {h["rms_px"]:.3f} px (p95 {h["p95_px"]:.3f}, max {h["max_px"]:.3f})')
    print(f'Y base<-camera  t[m] = {np.round(res["Y"][:3, 3], 5).tolist()}')
    print(f'X tcp<-marker   t[m] = {np.round(res["X"][:3, 3], 5).tolist()}')
    for w in res['warnings']:
        print(f'경고: {w}', file=sys.stderr)
    print(f'결과: {os.path.join(a.out, "handeye_result.yaml")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
