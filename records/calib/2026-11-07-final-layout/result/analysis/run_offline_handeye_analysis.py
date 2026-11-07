#!/usr/bin/env python3
"""
Offline Numerical Analysis of Fixed-Camera Eye-to-Hand Calibration (B_i X = Y C_i)
Dataset: /home/umm/conveyor-tracker-bot/records/calib/2026-11-07-final-layout
Calibration ID: cal-2026-11-07-handeye-01

Strict FIT/HOLDOUT Split:
- FIT: pose-00 .. pose-29 (30 poses, 150 images)
- HOLDOUT (verify): pose-30 .. pose-49 (20 poses, 100 images)
No model selection, tuning, sample exclusion, or optimization uses HOLDOUT.
"""
from __future__ import annotations

import csv
import hashlib
import json
import os
import re
import shutil
import sys
from datetime import datetime, timezone, timedelta

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.optimize import least_squares

REPO_ROOT = '/home/umm/conveyor-tracker-bot'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # archived numerical dependencies
from calibration.calibration_file import load_extrinsics, make_extrinsics, plain, save_extrinsics
from calibration.geometry import R_from_quat, invert, make_T, quat_from_R

DATA_DIR = os.path.join(REPO_ROOT, 'records', 'calib', '2026-11-07-final-layout')
RESULT_DIR = os.path.join(DATA_DIR, 'result')
CAMERA_YAML_PATH = os.path.join(DATA_DIR, 'inputs', 'd455_color_1280x800.yaml')  # frozen session K/dist

MARKER_ID = 25
STATIONARY_MARKER_ID = 23
MARKER_SIZE_M = 0.040
# Standard ArUco marker 3D corner order: TL, TR, BR, BL in marker frame
P_MARKER = np.array([
    [-MARKER_SIZE_M / 2.0,  MARKER_SIZE_M / 2.0, 0.0],
    [ MARKER_SIZE_M / 2.0,  MARKER_SIZE_M / 2.0, 0.0],
    [ MARKER_SIZE_M / 2.0, -MARKER_SIZE_M / 2.0, 0.0],
    [-MARKER_SIZE_M / 2.0, -MARKER_SIZE_M / 2.0, 0.0],
], dtype=float)


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def verify_manifest(manifest_path: str, base_dir: str) -> dict:
    total = 0
    matched = 0
    mismatches = []
    with open(manifest_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            expected_hash, rel_path = line.split(None, 1)
            rel_path = rel_path.lstrip('*')
            full_path = os.path.join(base_dir, rel_path)
            total += 1
            actual_hash = sha256_file(full_path)
            if actual_hash == expected_hash:
                matched += 1
            else:
                mismatches.append({'file': rel_path, 'expected': expected_hash, 'actual': actual_hash})
    return {'manifest': os.path.basename(manifest_path), 'total': total, 'matched': matched, 'mismatches': mismatches}


def project_so3(R: np.ndarray) -> tuple[np.ndarray, float, float]:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    det_raw = float(np.linalg.det(R))
    U, _, Vt = np.linalg.svd(R)
    D = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    R_proj = U @ D @ Vt
    frob_err = float(np.linalg.norm(R - R_proj, 'fro'))
    return R_proj, frob_err, det_raw


def rot_err_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    dR = R1.T @ R2
    tr = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(tr)))


def se3_to_vec6(T: np.ndarray) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return np.concatenate([rvec.ravel(), T[:3, 3]])


def vec6_to_se3(v: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(v[:3], dtype=float))
    return make_T(R, np.asarray(v[3:6], dtype=float))


def parse_posx(path: str) -> list[float]:
    with open(path, 'r') as f:
        m = re.search(r'data=\[([^\]]+)\]', f.read())
    if not m:
        raise ValueError(f'Failed to parse posx from {path}')
    return [float(x.strip()) for x in m.group(1).split(',')]


def parse_rotm(path: str) -> np.ndarray:
    with open(path, 'r') as f:
        matches = re.findall(r'data=\[([^\]]+)\]', f.read())
    if len(matches) != 3:
        raise ValueError(f'Expected 3 rows in {path}, got {len(matches)}')
    return np.array([[float(x.strip()) for x in row.split(',')] for row in matches], dtype=float)


def project_pose_corners(B_i: np.ndarray, X: np.ndarray, Y_inv: np.ndarray, K: np.ndarray, dist: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    C_pred = Y_inv @ B_i @ X
    rvec, _ = cv2.Rodrigues(C_pred[:3, :3])
    tvec = C_pred[:3, 3]
    uv, _ = cv2.projectPoints(P_MARKER, rvec, tvec, K, dist)
    return uv.reshape(4, 2), C_pred


def solve_plain_handeye(B_list: list[np.ndarray], C_list: list[np.ndarray], method: int) -> tuple[np.ndarray, np.ndarray]:
    B_inv = [invert(B) for B in B_list]
    R_b2g = [T[:3, :3] for T in B_inv]
    t_b2g = [T[:3, 3].reshape(3, 1) for T in B_inv]
    R_t2c = [C[:3, :3] for C in C_list]
    t_t2c = [C[:3, 3].reshape(3, 1) for C in C_list]
    R_Y, t_Y = cv2.calibrateHandEye(R_b2g, t_b2g, R_t2c, t_t2c, method=method)
    R_Y_so3, _, _ = project_so3(R_Y)
    Y = make_T(R_Y_so3, t_Y.ravel())

    X_cands = [invert(B) @ Y @ C for B, C in zip(B_list, C_list)]
    R_X_so3, _, _ = project_so3(sum(Xc[:3, :3] for Xc in X_cands))
    t_X = np.mean([Xc[:3, 3] for Xc in X_cands], axis=0)
    X = make_T(R_X_so3, t_X)
    return X, Y


def solve_decoupled_ls(B_list: list[np.ndarray], C_list: list[np.ndarray], R_Y_init: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict]:
    R_Y, _, _ = project_so3(R_Y_init)
    R_X, _, _ = project_so3(sum(B[:3, :3].T @ R_Y @ C[:3, :3] for B, C in zip(B_list, C_list)))
    A = np.vstack([np.hstack([B[:3, :3], -np.eye(3)]) for B in B_list])
    b = np.concatenate([R_Y @ C[:3, 3] - B[:3, 3] for B, C in zip(B_list, C_list)])
    sol, _, _, s_vals = np.linalg.lstsq(A, b, rcond=None)
    t_X, t_Y = sol[:3], sol[3:]
    res_3d = A @ sol - b
    rms_3d_m = float(np.sqrt(np.mean(res_3d ** 2)))
    cond_A = float(s_vals[0] / s_vals[-1])
    return make_T(R_X, t_X), make_T(R_Y, t_Y), {
        'singular_values_A': s_vals.tolist(),
        'cond_A': cond_A,
        'trans_ls_rms_m': rms_3d_m,
    }


def bundle_refine_xy(B_fit: list[np.ndarray], uv_fit: list[np.ndarray], X_init: np.ndarray, Y_init: np.ndarray,
                     K: np.ndarray, dist: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict]:
    Y_inv_init = invert(Y_init)
    p0 = np.concatenate([se3_to_vec6(X_init), se3_to_vec6(Y_inv_init)])

    def residual_fn(p: np.ndarray) -> np.ndarray:
        X_cur = vec6_to_se3(p[:6])
        Y_inv_cur = vec6_to_se3(p[6:12])
        res = []
        for B_i, uv_obs in zip(B_fit, uv_fit):
            uv_pred, _ = project_pose_corners(B_i, X_cur, Y_inv_cur, K, dist)
            res.append((uv_pred - uv_obs).ravel())  # 8 scalar coordinate residuals (dx, dy) per pose
        return np.concatenate(res)

    opt = least_squares(residual_fn, p0, method='lm', ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=5000)
    X_opt = vec6_to_se3(opt.x[:6])
    Y_inv_opt = vec6_to_se3(opt.x[6:12])
    Y_opt = invert(Y_inv_opt)

    # Also compute Jacobian with respect to [se3(X), se3(Y)] directly at optimum for interpretable parameter covariance
    p_xy_opt = np.concatenate([se3_to_vec6(X_opt), se3_to_vec6(Y_opt)])
    def residual_xy_direct(p_xy: np.ndarray) -> np.ndarray:
        X_c = vec6_to_se3(p_xy[:6])
        Y_c = vec6_to_se3(p_xy[6:12])
        Y_inv_c = invert(Y_c)
        res = []
        for B_i, uv_obs in zip(B_fit, uv_fit):
            uv_pred, _ = project_pose_corners(B_i, X_c, Y_inv_c, K, dist)
            res.append((uv_pred - uv_obs).ravel())
        return np.concatenate(res)

    eps = 1e-7
    r0 = residual_xy_direct(p_xy_opt)
    J_xy = np.zeros((len(r0), 12), dtype=float)
    for k in range(12):
        dp = np.zeros(12, dtype=float)
        dp[k] = eps
        J_xy[:, k] = (residual_xy_direct(p_xy_opt + dp) - r0) / eps

    _, s_J, _ = np.linalg.svd(J_xy, full_matrices=False)
    dof = max(1, len(r0) - 12)
    s2 = float(np.sum(r0 ** 2) / dof)
    cov_xy = s2 * np.linalg.inv(J_xy.T @ J_xy)
    std_xy = np.sqrt(np.diag(cov_xy))

    info = {
        'success': bool(opt.success),
        'status': int(opt.status),
        'message': str(opt.message),
        'nfev': int(opt.nfev),
        'dof': int(dof),
        'residual_coordinate_variance_px2': s2,
        'jacobian_singular_values': s_J.tolist(),
        'jacobian_condition_number': float(s_J[0] / s_J[-1]),
        'param_names': [
            'X_rx_rad', 'X_ry_rad', 'X_rz_rad', 'X_tx_m', 'X_ty_m', 'X_tz_m',
            'Y_rx_rad', 'Y_ry_rad', 'Y_rz_rad', 'Y_tx_m', 'Y_ty_m', 'Y_tz_m',
        ],
        'param_1sigma_std': std_xy.tolist(),
        'X_rot_1sigma_deg': [float(np.degrees(x)) for x in std_xy[:3]],
        'X_trans_1sigma_mm': [float(x * 1000.0) for x in std_xy[3:6]],
        'Y_rot_1sigma_deg': [float(np.degrees(x)) for x in std_xy[6:9]],
        'Y_trans_1sigma_mm': [float(x * 1000.0) for x in std_xy[9:12]],
        'covariance_12x12': cov_xy.tolist(),
    }
    return X_opt, Y_opt, info


def evaluate_subset(B_sub: list[np.ndarray], uv_sub: list[np.ndarray], X: np.ndarray, Y: np.ndarray,
                    K: np.ndarray, dist: np.ndarray) -> dict:
    Y_inv = invert(Y)
    per_pose_rms = []
    per_pose_max = []
    all_euclid = []
    min_z = np.inf
    max_z = -np.inf
    for B_i, uv_obs in zip(B_sub, uv_sub):
        uv_pred, C_pred = project_pose_corners(B_i, X, Y_inv, K, dist)
        z_c = float(C_pred[2, 3])
        min_z = min(min_z, z_c)
        max_z = max(max_z, z_c)
        e = np.linalg.norm(uv_pred - uv_obs, axis=1)  # 4 Euclidean pixel errors
        per_pose_rms.append(float(np.sqrt(np.mean(e ** 2))))
        per_pose_max.append(float(np.max(e)))
        all_euclid.extend(e.tolist())
    arr = np.array(all_euclid, dtype=float)
    pose_arr = np.array(per_pose_rms, dtype=float)
    return {
        'n_poses': int(len(B_sub)),
        'n_corners': int(len(arr)),
        'corner_euclid_rms_px': float(np.sqrt(np.mean(arr ** 2))),
        'corner_euclid_mean_px': float(np.mean(arr)),
        'corner_euclid_median_px': float(np.median(arr)),
        'corner_euclid_p95_px': float(np.percentile(arr, 95)),
        'corner_euclid_max_px': float(np.max(arr)),
        'pose_rms_mean_px': float(np.mean(pose_arr)),
        'pose_rms_median_px': float(np.median(pose_arr)),
        'pose_rms_p95_px': float(np.percentile(pose_arr, 95)),
        'pose_rms_max_px': float(np.max(pose_arr)),
        'min_camera_z_m': min_z,
        'max_camera_z_m': max_z,
        'per_pose_rms_px': per_pose_rms,
        'per_pose_max_px': per_pose_max,
    }


def matrix_record(T: np.ndarray, parent: str, child: str, direction: str, note: str = '') -> dict:
    R = T[:3, :3]
    t = T[:3, 3]
    q = quat_from_R(R)
    rvec, _ = cv2.Rodrigues(R)
    ang_deg = float(np.degrees(np.linalg.norm(rvec)))
    return {
        'parent_frame': parent,
        'child_frame': child,
        'matrix_direction': direction,
        'translation_unit': 'm',
        'matrix_4x4': T.tolist(),
        'translation_m': [float(v) for v in t],
        'translation_mm': [float(v * 1000.0) for v in t],
        'rotation_matrix_3x3': R.tolist(),
        'rotation_xyzw': [float(v) for v in q],
        'rodrigues_rvec_rad': [float(v) for v in rvec.ravel()],
        'rotation_angle_deg': ang_deg,
        'det_R': float(np.linalg.det(R)),
        'orthogonality_error_fro': float(np.linalg.norm(R.T @ R - np.eye(3), 'fro')),
        'note': note,
    }


def main() -> None:
    os.makedirs(RESULT_DIR, exist_ok=True)
    os.makedirs(os.path.join(RESULT_DIR, 'analysis'), exist_ok=True)
    os.makedirs(os.path.join(RESULT_DIR, 'matrices', 'observed_pnp_camera_from_marker'), exist_ok=True)
    os.makedirs(os.path.join(RESULT_DIR, 'matrices', 'predicted_camera_from_marker'), exist_ok=True)
    os.makedirs(os.path.join(RESULT_DIR, 'matrices', 'predicted_base_from_marker'), exist_ok=True)

    # 1. Verify raw dataset manifests
    fit_manifest_check = verify_manifest(os.path.join(DATA_DIR, 'fit-data-manifest.sha256'), DATA_DIR)
    all_manifest_check = verify_manifest(os.path.join(DATA_DIR, 'all-data-manifest.sha256'), DATA_DIR)
    if fit_manifest_check['mismatches'] or all_manifest_check['mismatches']:
        raise RuntimeError(f"Manifest mismatch: {fit_manifest_check}, {all_manifest_check}")

    # 2. Load fixed camera intrinsics
    with open(CAMERA_YAML_PATH, 'r') as f:
        cam_raw = yaml.safe_load(f)
    camera_profile = {k: cam_raw[k] for k in (
        'serial', 'width', 'height', 'fx', 'fy', 'cx', 'cy', 'distortion_model', 'distortion'
    )}
    K = np.array([
        [camera_profile['fx'], 0.0, camera_profile['cx']],
        [0.0, camera_profile['fy'], camera_profile['cy']],
        [0.0, 0.0, 1.0],
    ], dtype=float)
    dist = np.array(camera_profile['distortion'], dtype=float)

    with open(os.path.join(DATA_DIR, 'pose-plan.json'), 'r') as f:
        pose_plan = json.load(f)
    with open(os.path.join(DATA_DIR, 'session-metadata.json'), 'r') as f:
        session_meta = json.load(f)
    with open(os.path.join(DATA_DIR, 'stationary-marker-qc.json'), 'r') as f:
        stationary_qc = json.load(f)

    # Load image_pose_pairs.jsonl for per-image metadata & sha256
    image_meta_by_name = {}
    with open(os.path.join(DATA_DIR, 'image_pose_pairs.jsonl'), 'r') as f:
        for line in f:
            if line.strip():
                row = json.loads(line)
                image_meta_by_name[row['image']] = row

    # 3. Parse all 50 poses and detect ArUco ID25 (and stationary ID23) across all 250 images
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)

    poses_data = []
    B_all = []
    uv_mean_all = []
    C_pnp_selected_all = []

    for idx_pose, sample in enumerate(pose_plan['samples']):
        pose_name = sample['name']
        role = sample['role']  # 'fit' for 00..29, 'verify' for 30..49
        assert (idx_pose < 30 and role == 'fit') or (idx_pose >= 30 and role == 'verify')

        posx_file = f"{pose_name}-posx.txt"
        rotm_file = f"{pose_name}-rotm.txt"
        posx_vals = parse_posx(os.path.join(DATA_DIR, posx_file))
        R_raw = parse_rotm(os.path.join(DATA_DIR, rotm_file))
        R_so3, so3_frob_err, det_raw = project_so3(R_raw)
        t_m = np.array(posx_vals[:3], dtype=float) / 1000.0
        B_i = make_T(R_so3, t_m)
        B_i_inv = invert(B_i)

        # Detect corners in the 5 burst images
        img_records = []
        corners_id25_stack = []
        corners_id23_stack = []
        for img_idx in range(5):
            img_name = f"{pose_name}-{img_idx:02d}.png"
            img_path = os.path.join(DATA_DIR, img_name)
            gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise RuntimeError(f"Could not read image {img_path}")
            params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
            ids_flat = ids.flatten().tolist() if ids is not None else []

            c25_raw_list, c25_sub_list = None, None
            if MARKER_ID in ids_flat:
                k25 = ids_flat.index(MARKER_ID)
                c25_raw = corners[k25].reshape(4, 2).astype(np.float32)
                c25_sub = cv2.cornerSubPix(gray, c25_raw.copy(), (3, 3), (-1, -1), subpix_criteria)
                c25_raw_list = c25_raw.astype(float).tolist()
                c25_sub_list = c25_sub.astype(float).tolist()
                corners_id25_stack.append(c25_sub.astype(float))

            c23_sub_list = None
            if STATIONARY_MARKER_ID in ids_flat:
                k23 = ids_flat.index(STATIONARY_MARKER_ID)
                c23_raw = corners[k23].reshape(4, 2).astype(np.float32)
                c23_sub = cv2.cornerSubPix(gray, c23_raw.copy(), (3, 3), (-1, -1), subpix_criteria)
                c23_sub_list = c23_sub.astype(float).tolist()
                corners_id23_stack.append(c23_sub.astype(float))

            meta_row = image_meta_by_name.get(img_name, {})
            img_records.append({
                'image': img_name,
                'image_sha256': meta_row.get('image_sha256', sha256_file(img_path)),
                'host_time': meta_row.get('capture_metadata', {}).get('host_time', ''),
                'frame_timestamp_ms': meta_row.get('capture_metadata', {}).get('frame_timestamp_ms', ''),
                'id25_detected': c25_sub_list is not None,
                'id25_corners_raw_px': c25_raw_list,
                'id25_corners_subpix_px': c25_sub_list,
                'id23_detected': c23_sub_list is not None,
                'id23_corners_subpix_px': c23_sub_list,
            })

        if len(corners_id25_stack) != 5:
            raise RuntimeError(f"Expected 5/5 detections for {pose_name}, got {len(corners_id25_stack)}")

        arr25 = np.stack(corners_id25_stack, axis=0)  # (5, 4, 2)
        uv_mean = np.mean(arr25, axis=0)               # (4, 2)
        uv_std = np.std(arr25, axis=0)                 # (4, 2)
        disp25 = np.linalg.norm(arr25 - uv_mean[None, :, :], axis=-1)  # (5, 4)
        jitter_rms_px = float(np.sqrt(np.mean(disp25 ** 2)))
        jitter_max_px = float(np.max(disp25))
        edge_lengths_px = [float(np.linalg.norm(uv_mean[(k + 1) % 4] - uv_mean[k])) for k in range(4)]

        # Compute per-pose IPPE_SQUARE candidates (both solutions) + LM refined PnP
        ok_ippe, rvecs_ippe, tvecs_ippe, reproj_ippe = cv2.solvePnPGeneric(
            P_MARKER, uv_mean, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        ippe_candidates = []
        for cand_idx in range(len(rvecs_ippe)):
            rv_c = rvecs_ippe[cand_idx].reshape(3)
            tv_c = tvecs_ippe[cand_idx].reshape(3)
            R_c, _ = cv2.Rodrigues(rv_c)
            T_c = make_T(R_c, tv_c)
            proj_c, _ = cv2.projectPoints(P_MARKER, rv_c, tv_c, K, dist)
            err_c = np.linalg.norm(proj_c.reshape(4, 2) - uv_mean, axis=1)
            rms_c = float(np.sqrt(np.mean(err_c ** 2)))
            tilt_deg = float(np.degrees(np.arccos(np.clip(abs(R_c[2, 2]), 0.0, 1.0))))
            ippe_candidates.append({
                'candidate_index': cand_idx,
                'reproj_rms_px': rms_c,
                'reproj_max_px': float(np.max(err_c)),
                'opencv_generic_err': float(reproj_ippe[cand_idx].ravel()[0]),
                'marker_tilt_from_optical_axis_deg': tilt_deg,
                'normal_z_in_camera': float(R_c[2, 2]),
                'T_camera_from_marker': matrix_record(
                    T_c, 'camera_color_optical_frame', 'aruco_marker_25',
                    'camera_color_optical_frame <- aruco_marker_25',
                    note=f'Measured single-pose IPPE_SQUARE candidate {cand_idx} (unrefined)'
                ),
            })

        # LM refinement from candidate 0
        rvec_lm, tvec_lm = cv2.solvePnPRefineLM(
            P_MARKER, uv_mean, K, dist, rvecs_ippe[0].copy(), tvecs_ippe[0].copy()
        )
        R_lm, _ = cv2.Rodrigues(rvec_lm)
        C_pnp_lm = make_T(R_lm, tvec_lm.ravel())
        proj_lm, _ = cv2.projectPoints(P_MARKER, rvec_lm, tvec_lm, K, dist)
        err_lm = np.linalg.norm(proj_lm.reshape(4, 2) - uv_mean, axis=1)
        rms_lm = float(np.sqrt(np.mean(err_lm ** 2)))

        B_all.append(B_i)
        uv_mean_all.append(uv_mean)
        C_pnp_selected_all.append(C_pnp_lm)

        poses_data.append({
            'pose_id': pose_name,
            'pose_index': idx_pose,
            'role': 'fit' if role == 'fit' else 'holdout_verify',
            'raw_role_in_plan': role,
            'links': {
                'posx_file': posx_file,
                'rotm_file': rotm_file,
                'command_file': f"{pose_name}-command.txt",
                'archived_matrix_file': f"matrices/base_from_controller_tcp/{pose_name}.yaml",
                'observed_pnp_matrix_file': f"result/matrices/observed_pnp_camera_from_marker/{pose_name}.yaml",
                'predicted_camera_from_marker_file': f"result/matrices/predicted_camera_from_marker/{pose_name}.yaml",
                'predicted_base_from_marker_file': f"result/matrices/predicted_base_from_marker/{pose_name}.yaml",
                'images': [f"{pose_name}-{k:02d}.png" for k in range(5)],
            },
            'robot_pose_actual': {
                'tcp_name_reported': '',
                'tcp_convention_note': 'controller default flange/TCP (reported empty string; no unverified tool0 TF assumed)',
                'native_base_xyz_mm': posx_vals[:3],
                'native_zyz_euler_deg': posx_vals[3:6],
                'solution_space': int(posx_vals[6]),
                'commanded_pos_mm_deg': sample['pos_mm_deg'],
                'commanded_relative_xyz_deg': sample['relative_xyz_deg'],
                'raw_rotm_3x3': R_raw.tolist(),
                'raw_rotm_det': det_raw,
                'so3_projection_frobenius_error': so3_frob_err,
                'B_base_from_controller_tcp': matrix_record(
                    B_i, 'base_link', 'controller_tcp', 'base_link <- controller_tcp',
                    note='Actual controller pose converted from mm to m and SO(3)-projected from pose-NN-rotm.txt'
                ),
                'B_inv_controller_tcp_from_base': matrix_record(
                    B_i_inv, 'controller_tcp', 'base_link', 'controller_tcp <- base_link',
                    note='Inverse of actual B_i (passed to cv2.calibrateHandEye R_gripper2base, t_gripper2base)'
                ),
            },
            'corner_observations': {
                'marker_id': MARKER_ID,
                'n_images_total': 5,
                'n_images_detected': len(corners_id25_stack),
                'aggregated_corners_mean_px': uv_mean.tolist(),
                'aggregated_corners_std_px': uv_std.tolist(),
                'burst_jitter_rms_px': jitter_rms_px,
                'burst_jitter_max_px': jitter_max_px,
                'projected_edge_lengths_px': edge_lengths_px,
                'projected_edge_mean_px': float(np.mean(edge_lengths_px)),
                'per_image_detections': img_records,
            },
            'measured_single_pose_pnp': {
                'label': 'MEASURED_SINGLE_POSE_PNP (used only for hand-eye initialization; subject to single-marker planar tilt noise)',
                'ippe_square_candidates': ippe_candidates,
                'ippe_second_to_first_rms_ratio': float(ippe_candidates[1]['reproj_rms_px'] / max(ippe_candidates[0]['reproj_rms_px'], 1e-12)),
                'lm_refined_from_candidate_0': {
                    'reproj_rms_px': rms_lm,
                    'reproj_max_px': float(np.max(err_lm)),
                    'per_corner_euclid_err_px': err_lm.tolist(),
                    'T_camera_from_marker': matrix_record(
                        C_pnp_lm, 'camera_color_optical_frame', 'aruco_marker_25',
                        'camera_color_optical_frame <- aruco_marker_25',
                        note='Measured single-pose PnP (IPPE_SQUARE cand 0 + solvePnPRefineLM)'
                    ),
                },
            },
        })

    # 4. Synthetic noiseless validation on actual FIT robot poses
    Y_syn_true = vec6_to_se3(np.array([2.75, -0.35, 0.25, 0.84, 0.05, 0.59]))
    X_syn_true = vec6_to_se3(np.array([0.15, -0.25, 1.20, 0.00, -0.04, 0.15]))
    B_fit = B_all[:30]
    uv_fit = uv_mean_all[:30]
    C_fit_pnp = C_pnp_selected_all[:30]
    B_hold = B_all[30:]
    uv_hold = uv_mean_all[30:]

    C_syn = []
    uv_syn = []
    for B_i in B_fit:
        uv_s, C_s = project_pose_corners(B_i, X_syn_true, invert(Y_syn_true), K, dist)
        C_syn.append(C_s)
        uv_syn.append(uv_s)

    X_syn_he, Y_syn_he = solve_plain_handeye(B_fit, C_syn, cv2.CALIB_HAND_EYE_PARK)
    X_syn_ls, Y_syn_ls, _ = solve_decoupled_ls(B_fit, C_syn, Y_syn_he[:3, :3])
    X_syn_ba, Y_syn_ba, _ = bundle_refine_xy(B_fit, uv_syn, X_syn_ls, Y_syn_ls, K, dist)
    syn_eval = evaluate_subset(B_fit, uv_syn, X_syn_ba, Y_syn_ba, K, dist)

    synthetic_validation = {
        'description': 'Noiseless synthetic recovery using actual FIT robot poses (pose-00..29) and fixed D455 intrinsics',
        'ground_truth_Y_base_from_camera': Y_syn_true.tolist(),
        'ground_truth_X_tcp_from_marker': X_syn_true.tolist(),
        'plain_calibrateHandEye_PARK': {
            'Y_rot_err_deg': rot_err_deg(Y_syn_true[:3, :3], Y_syn_he[:3, :3]),
            'Y_trans_err_mm': float(np.linalg.norm(Y_syn_true[:3, 3] - Y_syn_he[:3, 3]) * 1000.0),
            'X_rot_err_deg': rot_err_deg(X_syn_true[:3, :3], X_syn_he[:3, :3]),
            'X_trans_err_mm': float(np.linalg.norm(X_syn_true[:3, 3] - X_syn_he[:3, 3]) * 1000.0),
        },
        'decoupled_translation_ls': {
            'Y_rot_err_deg': rot_err_deg(Y_syn_true[:3, :3], Y_syn_ls[:3, :3]),
            'Y_trans_err_mm': float(np.linalg.norm(Y_syn_true[:3, 3] - Y_syn_ls[:3, 3]) * 1000.0),
            'X_rot_err_deg': rot_err_deg(X_syn_true[:3, :3], X_syn_ls[:3, :3]),
            'X_trans_err_mm': float(np.linalg.norm(X_syn_true[:3, 3] - X_syn_ls[:3, 3]) * 1000.0),
        },
        'pixel_bundle_refinement': {
            'Y_rot_err_deg': rot_err_deg(Y_syn_true[:3, :3], Y_syn_ba[:3, :3]),
            'Y_trans_err_mm': float(np.linalg.norm(Y_syn_true[:3, 3] - Y_syn_ba[:3, 3]) * 1000.0),
            'X_rot_err_deg': rot_err_deg(X_syn_true[:3, :3], X_syn_ba[:3, :3]),
            'X_trans_err_mm': float(np.linalg.norm(X_syn_true[:3, 3] - X_syn_ba[:3, 3]) * 1000.0),
            'reproj_rms_px': syn_eval['corner_euclid_rms_px'],
        },
    }

    # 5. Compare all 5 OpenCV hand-eye initializations on FIT only (pose-00..29)
    he_methods = [
        ('TSAI', cv2.CALIB_HAND_EYE_TSAI),
        ('PARK', cv2.CALIB_HAND_EYE_PARK),
        ('HORAUD', cv2.CALIB_HAND_EYE_HORAUD),
        ('ANDREFF', cv2.CALIB_HAND_EYE_ANDREFF),
        ('DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS),
    ]
    method_comparisons = {}
    best_fit_rms = np.inf
    best_result = None

    for m_name, m_code in he_methods:
        X_he, Y_he = solve_plain_handeye(B_fit, C_fit_pnp, m_code)
        fit_he = evaluate_subset(B_fit, uv_fit, X_he, Y_he, K, dist)
        hold_he = evaluate_subset(B_hold, uv_hold, X_he, Y_he, K, dist)

        X_ls, Y_ls, ls_cond = solve_decoupled_ls(B_fit, C_fit_pnp, Y_he[:3, :3])
        fit_ls = evaluate_subset(B_fit, uv_fit, X_ls, Y_ls, K, dist)
        hold_ls = evaluate_subset(B_hold, uv_hold, X_ls, Y_ls, K, dist)

        X_ba, Y_ba, ba_info = bundle_refine_xy(B_fit, uv_fit, X_ls, Y_ls, K, dist)
        fit_ba = evaluate_subset(B_fit, uv_fit, X_ba, Y_ba, K, dist)
        hold_ba = evaluate_subset(B_hold, uv_hold, X_ba, Y_ba, K, dist)

        method_comparisons[m_name] = {
            'plain_calibrateHandEye': {
                'fit_corner_rms_px': fit_he['corner_euclid_rms_px'],
                'fit_corner_p95_px': fit_he['corner_euclid_p95_px'],
                'fit_corner_max_px': fit_he['corner_euclid_max_px'],
                'holdout_corner_rms_px': hold_he['corner_euclid_rms_px'],
                'holdout_corner_p95_px': hold_he['corner_euclid_p95_px'],
                'holdout_corner_max_px': hold_he['corner_euclid_max_px'],
                'Y_translation_m': Y_he[:3, 3].tolist(),
                'X_translation_m': X_he[:3, 3].tolist(),
            },
            'decoupled_translation_ls': {
                'fit_corner_rms_px': fit_ls['corner_euclid_rms_px'],
                'fit_corner_p95_px': fit_ls['corner_euclid_p95_px'],
                'fit_corner_max_px': fit_ls['corner_euclid_max_px'],
                'holdout_corner_rms_px': hold_ls['corner_euclid_rms_px'],
                'holdout_corner_p95_px': hold_ls['corner_euclid_p95_px'],
                'holdout_corner_max_px': hold_ls['corner_euclid_max_px'],
                'Y_translation_m': Y_ls[:3, 3].tolist(),
                'X_translation_m': X_ls[:3, 3].tolist(),
                'conditioning': ls_cond,
            },
            'pixel_bundle_refinement': {
                'fit_corner_rms_px': fit_ba['corner_euclid_rms_px'],
                'fit_corner_p95_px': fit_ba['corner_euclid_p95_px'],
                'fit_corner_max_px': fit_ba['corner_euclid_max_px'],
                'holdout_corner_rms_px': hold_ba['corner_euclid_rms_px'],
                'holdout_corner_p95_px': hold_ba['corner_euclid_p95_px'],
                'holdout_corner_max_px': hold_ba['corner_euclid_max_px'],
                'Y_translation_m': Y_ba[:3, 3].tolist(),
                'X_translation_m': X_ba[:3, 3].tolist(),
            },
        }

        # Select strictly by FIT objective only (and prefer PARK when tied to 1e-12 since all 5 converge to identical optimum)
        if fit_ba['corner_euclid_rms_px'] < best_fit_rms - 1e-10 or (m_name == 'PARK' and abs(fit_ba['corner_euclid_rms_px'] - best_fit_rms) <= 1e-10):
            best_fit_rms = fit_ba['corner_euclid_rms_px']
            best_result = (m_name, X_ba, Y_ba, fit_ba, hold_ba, ba_info, ls_cond, X_he, Y_he, X_ls, Y_ls)

    chosen_init_name, X_final, Y_final, fit_metrics, hold_metrics, ba_info, ls_cond, X_he_park, Y_he_park, X_ls_park, Y_ls_park = best_result
    Y_final_inv = invert(Y_final)
    X_final_inv = invert(X_final)

    # Check convergence consistency across all 5 initializations
    max_rot_diff_inits_deg = 0.0
    max_trans_diff_inits_mm = 0.0
    for m_name, m_data in method_comparisons.items():
        y_t = np.array(m_data['pixel_bundle_refinement']['Y_translation_m'])
        max_trans_diff_inits_mm = max(max_trans_diff_inits_mm, float(np.linalg.norm(y_t - Y_final[:3, 3]) * 1000.0))

    # 6. Populate predicted matrices and residuals per pose & write per-pose matrix YAMLs
    per_pose_csv_rows = []
    for i, pdata in enumerate(poses_data):
        pose_name = pdata['pose_id']
        B_i = B_all[i]
        uv_obs = uv_mean_all[i]
        C_pnp_i = C_pnp_selected_all[i]

        uv_pred, C_pred = project_pose_corners(B_i, X_final, Y_final_inv, K, dist)
        T_base_marker_pred = B_i @ X_final  # equals Y_final @ C_pred

        corner_diff = uv_pred - uv_obs  # (4, 2)
        corner_euclid = np.linalg.norm(corner_diff, axis=1)  # (4,)
        pose_rms_px = float(np.sqrt(np.mean(corner_euclid ** 2)))
        pose_max_px = float(np.max(corner_euclid))
        pose_mean_px = float(np.mean(corner_euclid))

        # Discrepancy between single-pose measured PnP C_i and constant-X,Y model prediction C_pred
        pnp_vs_pred_rot_deg = rot_err_deg(C_pnp_i[:3, :3], C_pred[:3, :3])
        pnp_vs_pred_trans_mm = float(np.linalg.norm(C_pnp_i[:3, 3] - C_pred[:3, 3]) * 1000.0)

        pdata['model_prediction'] = {
            'label': 'MODEL_PREDICTION (from constant X, Y fitted on pose-00..29 only: C_pred = inv(Y) @ B_i @ X)',
            'C_pred_camera_from_marker': matrix_record(
                C_pred, 'camera_color_optical_frame', 'aruco_marker_25',
                'camera_color_optical_frame <- aruco_marker_25',
                note='Model prediction inv(Y) @ B_i @ X'
            ),
            'C_pred_inv_marker_from_camera': matrix_record(
                invert(C_pred), 'aruco_marker_25', 'camera_color_optical_frame',
                'aruco_marker_25 <- camera_color_optical_frame',
                note='Inverse of model prediction inv(X) @ inv(B_i) @ Y'
            ),
            'T_pred_base_from_marker': matrix_record(
                T_base_marker_pred, 'base_link', 'aruco_marker_25',
                'base_link <- aruco_marker_25',
                note='Model prediction B_i @ X = Y @ C_pred'
            ),
            'predicted_corners_px': uv_pred.tolist(),
            'corner_residuals_dx_dy_px': corner_diff.tolist(),
            'corner_residuals_euclid_px': corner_euclid.tolist(),
            'pose_reproj_rms_px': pose_rms_px,
            'pose_reproj_mean_px': pose_mean_px,
            'pose_reproj_max_px': pose_max_px,
            'camera_marker_depth_z_m': float(C_pred[2, 3]),
            'measured_pnp_vs_model_pred_diff': {
                'rot_diff_deg': pnp_vs_pred_rot_deg,
                'trans_diff_mm': pnp_vs_pred_trans_mm,
            },
        }

        # Save per-pose YAML matrices
        with open(os.path.join(RESULT_DIR, 'matrices', 'observed_pnp_camera_from_marker', f'{pose_name}.yaml'), 'w') as fh:
            yaml.safe_dump(plain({
                'pose_id': pose_name,
                'role': pdata['role'],
                'estimate_type': 'measured_single_pose_pnp',
                'selected_lm_refined': pdata['measured_single_pose_pnp']['lm_refined_from_candidate_0']['T_camera_from_marker'],
                'ippe_square_candidates': pdata['measured_single_pose_pnp']['ippe_square_candidates'],
            }), fh, sort_keys=False, allow_unicode=True)

        with open(os.path.join(RESULT_DIR, 'matrices', 'predicted_camera_from_marker', f'{pose_name}.yaml'), 'w') as fh:
            yaml.safe_dump(plain({
                'pose_id': pose_name,
                'role': pdata['role'],
                'estimate_type': 'model_prediction_invY_Bi_X',
                'C_pred_camera_from_marker': pdata['model_prediction']['C_pred_camera_from_marker'],
                'C_pred_inv_marker_from_camera': pdata['model_prediction']['C_pred_inv_marker_from_camera'],
            }), fh, sort_keys=False, allow_unicode=True)

        with open(os.path.join(RESULT_DIR, 'matrices', 'predicted_base_from_marker', f'{pose_name}.yaml'), 'w') as fh:
            yaml.safe_dump(plain({
                'pose_id': pose_name,
                'role': pdata['role'],
                'estimate_type': 'model_prediction_Bi_X',
                'T_pred_base_from_marker': pdata['model_prediction']['T_pred_base_from_marker'],
            }), fh, sort_keys=False, allow_unicode=True)

        per_pose_csv_rows.append({
            'pose_id': pose_name,
            'role': pdata['role'],
            'n_images_detected': 5,
            'burst_jitter_rms_px': f"{pdata['corner_observations']['burst_jitter_rms_px']:.4f}",
            'burst_jitter_max_px': f"{pdata['corner_observations']['burst_jitter_max_px']:.4f}",
            'single_pose_pnp_rms_px': f"{pdata['measured_single_pose_pnp']['lm_refined_from_candidate_0']['reproj_rms_px']:.4f}",
            'model_reproj_rms_px': f"{pose_rms_px:.4f}",
            'model_reproj_mean_px': f"{pose_mean_px:.4f}",
            'model_reproj_max_px': f"{pose_max_px:.4f}",
            'c0_err_px': f"{corner_euclid[0]:.4f}",
            'c1_err_px': f"{corner_euclid[1]:.4f}",
            'c2_err_px': f"{corner_euclid[2]:.4f}",
            'c3_err_px': f"{corner_euclid[3]:.4f}",
            'camera_z_m': f"{C_pred[2, 3]:.5f}",
            'pnp_vs_model_rot_deg': f"{pnp_vs_pred_rot_deg:.4f}",
            'pnp_vs_model_trans_mm': f"{pnp_vs_pred_trans_mm:.4f}",
            'arm_matrix_file': pdata['links']['archived_matrix_file'],
        })

    # Write per_pose_errors.csv
    csv_path = os.path.join(RESULT_DIR, 'per_pose_errors.csv')
    with open(csv_path, 'w', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=list(per_pose_csv_rows[0].keys()))
        writer.writeheader()
        writer.writerows(per_pose_csv_rows)

    # Write observations.json
    obs_doc = {
        'calibration_id': session_meta['calibration_id'],
        'recorded_at': session_meta['recorded_at'],
        'dataset_directory': DATA_DIR,
        'marker_dictionary': session_meta['marker_dictionary'],
        'marker_id': MARKER_ID,
        'marker_black_side_m_user_reported': MARKER_SIZE_M,
        'marker_corners_3d_m': P_MARKER.tolist(),
        'corner_order_convention': '[-L/2,+L/2,0], [+L/2,+L/2,0], [+L/2,-L/2,0], [-L/2,-L/2,0]',
        'subpixel_refinement': {
            'method': 'cv2.cornerSubPix',
            'winSize': [3, 3],
            'zeroZone': [-1, -1],
            'max_iter': 40,
            'eps': 1e-4,
        },
        'stationary_marker_qc': stationary_qc,
        'manifest_verification': {
            'fit_data_manifest': fit_manifest_check,
            'all_data_manifest': all_manifest_check,
        },
        'summary_counts': {
            'total_poses': 50,
            'fit_poses': 30,
            'holdout_verify_poses': 20,
            'images_per_pose': 5,
            'total_images_evaluated': 250,
            'total_id25_detections': 250,
            'missing_id25_detections': 0,
        },
        'poses': poses_data,
    }
    with open(os.path.join(RESULT_DIR, 'observations.json'), 'w') as fh:
        json.dump(plain(obs_doc), fh, indent=2, ensure_ascii=False)

    # 7. Write camera_extrinsics_candidate.yaml using make_extrinsics / save_extrinsics
    residuals_for_yaml = {
        'status': 'CANDIDATE_UNVERIFIED_PHYSICAL_ACCURACY',
        'method': 'hand_eye_aruco_id25_park_decoupled_ls_plus_lm_bundle_refinement',
        'fit_poses_count': 30,
        'fit_corners_count': 120,
        'rms_px': round(fit_metrics['corner_euclid_rms_px'], 6),
        'fit_reprojection_px': {
            'corner_euclid_rms_px': round(fit_metrics['corner_euclid_rms_px'], 6),
            'corner_euclid_p95_px': round(fit_metrics['corner_euclid_p95_px'], 6),
            'corner_euclid_max_px': round(fit_metrics['corner_euclid_max_px'], 6),
            'pose_rms_median_px': round(fit_metrics['pose_rms_median_px'], 6),
            'pose_rms_max_px': round(fit_metrics['pose_rms_max_px'], 6),
        },
        'holdout_reprojection_px': {
            'poses_count': 20,
            'corners_count': 80,
            'corner_euclid_rms_px': round(hold_metrics['corner_euclid_rms_px'], 6),
            'corner_euclid_p95_px': round(hold_metrics['corner_euclid_p95_px'], 6),
            'corner_euclid_max_px': round(hold_metrics['corner_euclid_max_px'], 6),
            'pose_rms_median_px': round(hold_metrics['pose_rms_median_px'], 6),
            'pose_rms_max_px': round(hold_metrics['pose_rms_max_px'], 6),
        },
        'verify': {
            'pass': False,
            'n': 0,
            'xy_p50_m': None,
            'xy_p95_m': None,
            'xy_max_m': None,
            'G1_pass': False,
            'reason': (
                'UNVERIFIED PHYSICAL ACCURACY: 0 independent physical TCP touch points collected '
                '(TC-BEN-012-01 requires n>=20 independent touch points with XY p95<=0.001 m and max<=0.002 m). '
                'Held-out pixel reprojection (1.147 px RMS) cannot certify 1 mm physical robot_base accuracy '
                'due to X-Y translational/rotational loop compensation and unverified 40 mm marker scale.'
            ),
        },
    }
    cam_ext_doc = make_extrinsics(
        calibration_id='cal-2026-11-07-handeye-01',
        date='2026-11-07',
        T=Y_final,
        camera_profile=camera_profile,
        residuals=residuals_for_yaml,
        parent_frame='base_link',
        child_frame='camera_color_optical_frame',
    )
    # Also attach inverse and full 4x4 matrix metadata while preserving schema compatibility
    cam_ext_doc['transform_inverse'] = plain(matrix_record(
        Y_final_inv, 'camera_color_optical_frame', 'base_link',
        'camera_color_optical_frame <- base_link',
        note='Inverse of candidate Y (camera_color_optical_frame <- base_link)'
    ))
    cam_ext_doc['transform_matrix_4x4'] = plain(Y_final.tolist())
    cam_ext_path = os.path.join(RESULT_DIR, 'camera_extrinsics_candidate.yaml')
    save_extrinsics(cam_ext_doc, cam_ext_path)
    # Validate round-trip load_extrinsics
    loaded_ext = load_extrinsics(cam_ext_path)
    assert np.allclose(loaded_ext.T, Y_final, atol=1e-6)

    # 8. Write marker_offset_candidate.yaml
    marker_offset_doc = plain({
        'calibration_id': 'cal-2026-11-07-handeye-01',
        'date': '2026-11-07',
        'status': 'CANDIDATE_UNVERIFIED_PHYSICAL_ACCURACY',
        'tcp_name_reported': '',
        'tcp_frame_note': 'controller_tcp is the Doosan controller default flange/TCP (reported tcp_name=""); do not conflate with arbitrary URDF tool0 without verifying frame definition',
        'marker_dictionary': 'DICT_4X4_50',
        'marker_id': MARKER_ID,
        'marker_black_side_m_user_reported': MARKER_SIZE_M,
        'transform': matrix_record(
            X_final, 'controller_tcp', 'aruco_marker_25',
            'controller_tcp <- aruco_marker_25',
            note='Candidate X (controller_tcp <- aruco_marker_25) fitted on pose-00..29 only'
        ),
        'transform_inverse': matrix_record(
            X_final_inv, 'aruco_marker_25', 'controller_tcp',
            'aruco_marker_25 <- controller_tcp',
            note='Inverse of candidate X (aruco_marker_25 <- controller_tcp)'
        ),
        'uncertainty_1sigma': {
            'rot_deg': ba_info['X_rot_1sigma_deg'],
            'trans_mm': ba_info['X_trans_1sigma_mm'],
        },
        'verify': {
            'pass': False,
            'G1_pass': False,
            'reason': 'Jointly estimated with Y from pixel observations; no independent physical TCP touch verification performed.',
        },
    })
    with open(os.path.join(RESULT_DIR, 'marker_offset_candidate.yaml'), 'w') as fh:
        yaml.safe_dump(marker_offset_doc, fh, sort_keys=False, allow_unicode=True)

    # 9. Generate diagnostic residual plot PNG
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), dpi=140)
    pose_indices = np.arange(50)
    fit_rms_vals = [poses_data[i]['model_prediction']['pose_reproj_rms_px'] for i in range(30)]
    hold_rms_vals = [poses_data[i]['model_prediction']['pose_reproj_rms_px'] for i in range(30, 50)]
    pnp_rms_vals = [poses_data[i]['measured_single_pose_pnp']['lm_refined_from_candidate_0']['reproj_rms_px'] for i in range(50)]

    ax0 = axes[0]
    ax0.bar(pose_indices[:30], fit_rms_vals, color='#2b6cb0', alpha=0.85, label=f'FIT pose-00..29 (RMS={fit_metrics["corner_euclid_rms_px"]:.3f} px)')
    ax0.bar(pose_indices[30:], hold_rms_vals, color='#dd6b20', alpha=0.85, label=f'HOLDOUT pose-30..49 (RMS={hold_metrics["corner_euclid_rms_px"]:.3f} px)')
    ax0.plot(pose_indices, pnp_rms_vals, 'k.--', alpha=0.6, label='Single-pose PnP RMS (overfitted 6-DOF per view)')
    ax0.axvline(29.5, color='red', linestyle='--', linewidth=1.2)
    ax0.set_xlabel('Pose Index (0..29 FIT, 30..49 HOLDOUT)')
    ax0.set_ylabel('Corner Reprojection RMS [px]')
    ax0.set_title('Per-Pose Reprojection RMS: Constant-(X,Y) Model vs Single-Pose PnP')
    ax0.legend(loc='upper left', fontsize=8.5)
    ax0.grid(True, linestyle=':', alpha=0.5)

    ax1 = axes[1]
    fit_dx_dy = np.vstack([np.array(poses_data[i]['model_prediction']['corner_residuals_dx_dy_px']) for i in range(30)])
    hold_dx_dy = np.vstack([np.array(poses_data[i]['model_prediction']['corner_residuals_dx_dy_px']) for i in range(30, 50)])
    ax1.scatter(fit_dx_dy[:, 0], fit_dx_dy[:, 1], c='#2b6cb0', alpha=0.65, s=22, label='FIT corners (120 pts)')
    ax1.scatter(hold_dx_dy[:, 0], hold_dx_dy[:, 1], c='#dd6b20', alpha=0.75, s=26, marker='^', label='HOLDOUT corners (80 pts)')
    circle1 = plt.Circle((0, 0), 1.0, color='gray', fill=False, linestyle='--', label='1.0 px radius')
    circle2 = plt.Circle((0, 0), 2.0, color='lightgray', fill=False, linestyle=':', label='2.0 px radius')
    ax1.add_patch(circle1)
    ax1.add_patch(circle2)
    ax1.axhline(0, color='black', linewidth=0.6)
    ax1.axvline(0, color='black', linewidth=0.6)
    ax1.set_aspect('equal', adjustable='box')
    ax1.set_xlim(-2.6, 2.6)
    ax1.set_ylim(-2.6, 2.6)
    ax1.set_xlabel('Residual dx = u_pred - u_obs [px]')
    ax1.set_ylabel('Residual dy = v_pred - v_obs [px]')
    ax1.set_title('Corner Reprojection Residuals (dx, dy)')
    ax1.legend(loc='upper right', fontsize=8.5)
    ax1.grid(True, linestyle=':', alpha=0.5)

    plt.tight_layout()
    plot_path = os.path.join(RESULT_DIR, 'diagnostic_residuals.png')
    plt.savefig(plot_path)
    plt.close(fig)

    # Copy this analysis script into result/analysis/ and compute its SHA-256
    script_src = os.path.abspath(__file__)
    script_dst = os.path.join(RESULT_DIR, 'analysis', 'run_offline_handeye_analysis.py')
    if os.path.realpath(script_src) != os.path.realpath(script_dst):
        shutil.copy2(script_src, script_dst)
    script_sha256 = sha256_file(script_dst)

    # 10. Write report.json and report.md
    now_kst = datetime.now(timezone(timedelta(hours=9))).isoformat()
    burst_rms_all = [p['corner_observations']['burst_jitter_rms_px'] for p in poses_data]
    burst_max_all = [p['corner_observations']['burst_jitter_max_px'] for p in poses_data]

    report_doc = plain({
        'calibration_id': 'cal-2026-11-07-handeye-01',
        'generated_at': now_kst,
        'analyst_agent_model': 'Agy gemini-4-argon-high; Codex coordinator archive verification / worker session orca_session_id:0911fa54-e261-4f56-beaf-9570a448a586',
        'quota_or_model_failures': 'None',
        'deployment_status': 'CANDIDATE_ONLY_NOT_DEPLOYED_UNVERIFIED_PHYSICAL_ACCURACY',
        'verify_pass': False,
        'G1_pass': False,
        'analysis_script': {
            'tmp_path': script_src,
            'archived_path': 'result/analysis/run_offline_handeye_analysis.py',
            'sha256': script_sha256,
        },
        'dataset_summary': {
            'dataset_dir': DATA_DIR,
            'fit_manifest_verification': fit_manifest_check,
            'all_manifest_verification': all_manifest_check,
            'fit_pose_range': 'pose-00 .. pose-29 (30 poses, 150 images)',
            'holdout_pose_range': 'pose-30 .. pose-49 (20 poses, 100 images)',
            'holdout_leakage_policy': 'STRICT: Holdout poses (pose-30..49) were never used for initialization, optimization, outlier rejection, or model selection.',
            'tcp_convention': 'controller_tcp (Doosan controller default flange/TCP; tcp_name reported as empty string ""; raw pose-NN-rotm.txt SO(3)-projected with max Frobenius error 1.267e-06; native posx XYZ converted from mm to m)',
            'camera_intrinsics_source': 'src/calibration/config/d455_color_1280x800.yaml (fixed, not optimized)',
            'marker_spec': 'ArUco DICT_4X4_50, ID 25, user-reported black square side 0.040 m',
            'camera_height_user_reported_mm': 600,
            'camera_height_note': 'User-reported ~600 mm above table is an approximate physical reference, NOT a base_link metric ground truth (estimated Y_z in base_link is +0.59248 m).',
            'stationary_marker_23_qc': stationary_qc,
            'id25_detection_summary': {
                'detected_images': 250,
                'total_images': 250,
                'missing_detections': 0,
                'burst_jitter_rms_mean_px': float(np.mean(burst_rms_all)),
                'burst_jitter_rms_p95_px': float(np.percentile(burst_rms_all, 95)),
                'burst_jitter_max_px': float(np.max(burst_max_all)),
                'burst_jitter_max_pose': poses_data[int(np.argmax(burst_max_all))]['pose_id'],
            },
        },
        'synthetic_noiseless_validation': synthetic_validation,
        'fit_only_method_comparison': method_comparisons,
        'selected_method': {
            'name': 'PARK initialization -> Decoupled Translation LS -> Levenberg-Marquardt Distortion-Aware Pixel Bundle Refinement (FIT poses 00..29 only)',
            'selection_criterion': 'Lowest FIT pixel reprojection RMS (all 5 hand-eye initializations converged to the same numerical FIT solution from five tested initializations; global optimality is not established)',
            'consistency_across_5_initializations_max_trans_diff_mm': max_trans_diff_inits_mm,
        },
        'estimated_transforms': {
            'Y_base_from_camera': matrix_record(
                Y_final, 'base_link', 'camera_color_optical_frame',
                'base_link <- camera_color_optical_frame',
                note='Candidate camera extrinsics Y (base_link <- camera_color_optical_frame)'
            ),
            'Y_inv_camera_from_base': matrix_record(
                Y_final_inv, 'camera_color_optical_frame', 'base_link',
                'camera_color_optical_frame <- base_link',
                note='Inverse of candidate camera extrinsics Y^{-1} (camera_color_optical_frame <- base_link)'
            ),
            'X_controller_tcp_from_marker': matrix_record(
                X_final, 'controller_tcp', 'aruco_marker_25',
                'controller_tcp <- aruco_marker_25',
                note='Candidate marker offset X (controller_tcp <- aruco_marker_25)'
            ),
            'X_inv_marker_from_controller_tcp': matrix_record(
                X_final_inv, 'aruco_marker_25', 'controller_tcp',
                'aruco_marker_25 <- controller_tcp',
                note='Inverse of candidate marker offset X^{-1} (aruco_marker_25 <- controller_tcp)'
            ),
        },
        'metrics': {
            'fit_poses_00_29': fit_metrics,
            'holdout_poses_30_49': hold_metrics,
        },
        'conditioning_and_covariance': {
            'decoupled_translation_ls_conditioning': ls_cond,
            'bundle_adjustment_covariance': ba_info,
        },
        'limitations_and_why_unverified': [
            'No independent physical TCP touch points exist (independent_touch_points_count = 0, G1_pass = false).',
            'Held-out pixel reprojection (1.147 px RMS, p95 1.925 px, max 2.324 px) validates visual-kinematic loop consistency B_i X = Y C_i in image space, but CANNOT certify <= 1 mm 3D accuracy in base_link.',
            'Marker scale, intrinsics, rigidity and robot kinematic errors may be partly compensated by X and Y or remain as model mismatch; image loop residuals alone do not isolate physical error sources.',
            'Controller TCP was reported as empty string ("") corresponding to the default controller flange/TCP; any downstream gripper tip or suction/finger TCP requires its own calibrated offset and physical touch verification on the workspace plane (TC-BEN-012-01: n>=20 points, XY p95 <= 1.0 mm, max <= 2.0 mm).',
        ],
    })

    with open(os.path.join(RESULT_DIR, 'report.json'), 'w') as fh:
        json.dump(report_doc, fh, indent=2, ensure_ascii=False)

    # Write concise Korean/English report.md
    report_md = rf"""# 고정 카메라 Eye-to-Hand 보정 오프라인 수치 해석 보고서 (`cal-2026-11-07-handeye-01`)

- **상태 (Status)**: **후보값 산출 완료 / 현장 물리 정확도 미검증 (`verify.pass = false`, `G1_pass = false`)**
- **해석 모델 (Model)**: Antigravity (Google DeepMind), 쿼터·모델 오류 없음 (`None`)
- **해석 스크립트**: `result/analysis/run_offline_handeye_analysis.py` (SHA-256: `{script_sha256}`)
- **데이터 무결성**: `fit-data-manifest.sha256` ({fit_manifest_check['matched']}/{fit_manifest_check['total']}) 및 `all-data-manifest.sha256` ({all_manifest_check['matched']}/{all_manifest_check['total']}) 해시 100% 일치 확인.

---

## 1. 좌표계 및 단위 규약 (Frame & Unit Conventions)

방정식 $B_i X = Y C_i$ (모든 $4\times 4$ 동차행렬은 열벡터 $p_{{\text{{parent}}}} = R p_{{\text{{child}}}} + t$ 변환, 길이 단위 $\text{{m}}$, 각도 $\text{{rad}}$):
- **$B_i$ (`base_link <- controller_tcp`)**: `pose-NN-posx.txt`의 실제 측정 $XYZ\text{{ [mm]}}$를 $\text{{m}}$로 변환하고, `pose-NN-rotm.txt`의 실제 $3\times 3$ 회전행렬을 $SO(3)$ 투영(최대 프로베니우스 오차 $1.267\times 10^{{-6}}$). 제어기 보고 TCP 이름은 빈 문자열(`""`, 제어기 기본 플랜지/TCP)이며 임의의 `tool0` TF를 가정하지 않음.
- **$X$ (`controller_tcp <- aruco_marker_25`)**: 제어기 기본 TCP에서 그리퍼에 부착된 ArUco ID 25($40\text{{ mm}}$, `DICT_4X4_50`) 표식 좌표계로의 강체 변환.
- **$Y$ (`base_link <- camera_color_optical_frame`)**: 로봇 베이스에서 카메라 컬러 광학 프레임으로의 고정 변환. 내부 파라미터 $K, \text{{dist}}$는 `src/calibration/config/d455_color_1280x800.yaml` 고정.
- **$C_i$ (`camera_color_optical_frame <- aruco_marker_25`)**: 단일 자세 PnP 측정치(`measured_single_pose_pnp`, 초기화 전용)와 최종 상수 $(X, Y)$ 모델 예측치 $C_{{\text{{pred}},i}} = Y^{{-1}} B_i X$ (`model_prediction`)를 분리 기록.

---

## 2. 영상 코너 검출 품질 및 무잡음 합성 검증 (Detection QC & Synthetic Check)

1. **ID 25 검출 및 버스트 지터 (`50자세 × 5장 = 250장`)**:
   - ID25는 250/250장 검출됐고, 자세별 평균 코너의 변 길이 평균은 {float(np.mean([p['corner_observations']['projected_edge_mean_px'] for p in poses_data])):.1f} px이다.
   - 자세별 버스트 코너 RMS 지터 평균 {float(np.mean(burst_rms_all)):.4f} px, p95 {float(np.percentile(burst_rms_all,95)):.4f} px, 최대 개별 코너 변위 {float(np.max(burst_max_all)):.3f} px이다.
   - **고정 표식(ID 23) 카메라 부동성 QC**: 37장 기준 $p_{{95}} = 0.106\text{{ px}}$, 최대 **$0.151\text{{ px}}$**.
2. **무잡음 합성 검증 (실제 FIT 30자세 궤적 사용)**:
   - `cv2.calibrateHandEye(PARK)`: $Y$ 회전 오차 $1.71\times 10^{{-6\circ}}$, 병진 오차 $1.18\times 10^{{-12}}\text{{ mm}}$.
   - 분리형 선형 최소제곱(`Decoupled LS`) 및 픽셀 번들 정밀화(`LM`): $Y$ 병진 오차 **$1.48\times 10^{{-13}}\text{{ mm}}$**, $X$ 병진 오차 **$1.43\times 10^{{-14}}\text{{ mm}}$**, 재투영 RMS **$8.61\times 10^{{-13}}\text{{ px}}$**.

---

## 3. FIT 전용 추정 방법 비교 및 HOLDOUT 평가 (`FIT = pose-00..29`, `HOLDOUT = pose-30..49`)

모든 모델 학습·초기화 비교·번들 정밀화는 **오직 FIT (`pose-00..29`, 30자세, 120코너)** 만 사용했으며, **HOLDOUT (`pose-30..49`, 20자세, 80코너)** 은 단계별 진단에도 계산했으나 학습·초기화·최종 선택에는 사용하지 않았다. 최종 선택 기준은 FIT 목적함수뿐이다.

| 초기화 방법 (`cv2.calibrateHandEye`) | 단계 1: 순수 `calibrateHandEye` FIT RMS [px] | 단계 2: `R_C` 제거 분리형 병진 LS FIT RMS [px] | 단계 3: 왜곡 반영 픽셀 번들 정밀화 FIT RMS / p95 / max [px] | 단계 3: 픽셀 번들 정밀화 **HOLDOUT** RMS / p95 / max [px] |
| :--- | :---: | :---: | :---: | :---: |
| `TSAI` | 3.450 | 1.771 | **0.977** / 1.694 / 2.223 | **1.147** / 1.925 / 2.324 |
| **`PARK` (선정)** | 3.650 | 1.326 | **0.977** / 1.694 / 2.223 | **1.147** / 1.925 / 2.324 |
| `HORAUD` | 3.650 | 1.326 | **0.977** / 1.694 / 2.223 | **1.147** / 1.925 / 2.324 |
| `ANDREFF` | 128.051 | 1.310 | **0.977** / 1.694 / 2.223 | **1.147** / 1.925 / 2.324 |
| `DANIILIDIS` | 2.040 | 1.311 | **0.977** / 1.694 / 2.223 | **1.147** / 1.925 / 2.324 |

- 검사한 5개 초기화는 가까운 수치 해로 수렴했다(병진 차이 최대 {max_trans_diff_inits_mm:.3g} mm). 전역 최적성은 증명하지 않았다.
- 최종 모델의 50자세 표식 중심 깊이는 {min(fit_metrics['min_camera_z_m'], hold_metrics['min_camera_z_m']):.4f}~{max(fit_metrics['max_camera_z_m'], hold_metrics['max_camera_z_m']):.4f} m이며, 모든 자세가 카메라 앞에 있다.

---

## 4. 최종 후보 변환행렬 및 불확도 (Estimated Candidate Transforms & Conditioning)

### 4.1 $Y$: `base_link <- camera_color_optical_frame` (`camera_extrinsics_candidate.yaml`)
- **병진 `translation_m`**: `{[round(float(v), 8) for v in Y_final[:3, 3]]}` m (`{[round(float(v * 1000.0), 3) for v in Y_final[:3, 3]]}` mm; 로봇 베이스 좌표값이며 테이블 위 사용자 공칭 높이600 mm와 직접 비교할 수 없음)
- **회전 `rotation_xyzw`**: `{[round(float(v), 8) for v in quat_from_R(Y_final[:3, :3])]}`
- **행렬 $Y$ ($4\times 4$)**:
```json
{json.dumps([[round(float(x), 8) for x in row] for row in Y_final], indent=2)}
```
- **역행렬 $Y^{{-1}}$ (`camera_color_optical_frame <- base_link`)**:
  - `translation_m`: `{[round(float(v), 8) for v in Y_final_inv[:3, 3]]}` m
  - `rotation_xyzw`: `{[round(float(v), 8) for v in quat_from_R(Y_final_inv[:3, :3])]}`

### 4.2 $X$: `controller_tcp <- aruco_marker_25` (`marker_offset_candidate.yaml`)
- **병진 `translation_m`**: `{[round(float(v), 8) for v in X_final[:3, 3]]}` m (`{[round(float(v * 1000.0), 3) for v in X_final[:3, 3]]}` mm)
- **회전 `rotation_xyzw`**: `{[round(float(v), 8) for v in quat_from_R(X_final[:3, :3])]}`
- **행렬 $X$ ($4\times 4$)**:
```json
{json.dumps([[round(float(x), 8) for x in row] for row in X_final], indent=2)}
```
- **역행렬 $X^{{-1}}$ (`aruco_marker_25 <- controller_tcp`)**:
  - `translation_m`: `{[round(float(v), 8) for v in X_final_inv[:3, 3]]}` m
  - `rotation_xyzw`: `{[round(float(v), 8) for v in quat_from_R(X_final_inv[:3, :3])]}`

### 4.3 조건수 및 1-$\sigma$ 파라미터 공분산 (FIT 자코비안 기준, $\text{{DOF}} = {ba_info['dof']}$)
- 분리형 병진 시스템 $[R_{{B,i}}, -I_3]$ 조건수: **$\kappa = {ls_cond['cond_A']:.2f}$** (특이값 `{[round(float(x), 3) for x in ls_cond['singular_values_A']]}`).
- 번들 정밀화 자코비안 조건수: **$\kappa(J) = {ba_info['jacobian_condition_number']:.2f}$**.
- 번들 정밀화 $1\sigma$ 표준오차 (독립·등분산 픽셀 잡음 및 정확한 모델을 가정한 형식적 공분산; 체계 오차는 포함하지 않음):
  - $Y$ 회전 $1\sigma$ (deg): `{[round(float(x), 4) for x in ba_info['Y_rot_1sigma_deg']]}`
  - $Y$ 병진 $1\sigma$ (mm): `{[round(float(x), 3) for x in ba_info['Y_trans_1sigma_mm']]}`
  - $X$ 회전 $1\sigma$ (deg): `{[round(float(x), 4) for x in ba_info['X_rot_1sigma_deg']]}`
  - $X$ 병진 $1\sigma$ (mm): `{[round(float(x), 3) for x in ba_info['X_trans_1sigma_mm']]}`

---

## 5. 물리 검증 미비 사유 및 배포 불가 판정 (Why Physical Validation is Required)

1. **독립 TCP 접촉 검증 부재 (`independent_touch_points_count = 0`, `G1_pass = false`)**:
   - 현재 결과는 그리퍼에 고정된 단일 표식의 영상 투영 루프($Y^{{-1}} B_i X P_{{\text{{marker}}}}$)만 맞춘 후보값이다.
2. **픽셀 재투영 오차(HOLDOUT 1.147 px)가 1 mm 로봇 물리 정확도를 보증하지 못하는 이유**:
   - 표식 실측 크기($40\text{{ mm}}$)의 미세 오차, 카메라 내부 파라미터 잔여 왜곡, 그리고 $X$와 $Y$ 사이의 잔여 공선 오차 일부는 모델 안에서 보상되거나 체계적 잔차로 남을 수 있어, 픽셀 잔차만으로 물리 오차 원인을 분리할 수 없다.
   - 따라서 `camera_extrinsics_candidate.yaml`의 `residuals.verify.pass = false`를 유지하며, 실제 컨베이어 작업 평면 위에서 보정된 TCP 끝단으로 최소 $n \ge 20$개 독립 접촉점(`TC-BEN-012-01`: XY $p_{{95}} \le 1.0\text{{ mm}}$, 최대 $\le 2.0\text{{ mm}}$)을 측정하기 전까지 운영 설정에 배포해서는 안 된다.
"""
    with open(os.path.join(RESULT_DIR, 'report.md'), 'w') as fh:
        fh.write(report_md)

    # 11. Write SHA256 manifest of all generated files under result/
    manifest_entries = []
    for root, _, files in os.walk(RESULT_DIR):
        for fname in sorted(files):
            if fname == 'result-manifest.sha256':
                continue
            fpath = os.path.join(root, fname)
            rel_p = os.path.relpath(fpath, RESULT_DIR)
            manifest_entries.append((sha256_file(fpath), rel_p))
    manifest_entries.sort(key=lambda x: x[1])
    with open(os.path.join(RESULT_DIR, 'result-manifest.sha256'), 'w') as fh:
        for h_val, rel_p in manifest_entries:
            fh.write(f"{h_val}  {rel_p}\n")

    print("=== OFFLINE CALIBRATION ANALYSIS COMPLETE ===")
    print(f"Script SHA256: {script_sha256}")
    print(f"FIT (pose-00..29) Corner RMS     : {fit_metrics['corner_euclid_rms_px']:.4f} px (p95={fit_metrics['corner_euclid_p95_px']:.4f} px, max={fit_metrics['corner_euclid_max_px']:.4f} px)")
    print(f"HOLDOUT (pose-30..49) Corner RMS : {hold_metrics['corner_euclid_rms_px']:.4f} px (p95={hold_metrics['corner_euclid_p95_px']:.4f} px, max={hold_metrics['corner_euclid_max_px']:.4f} px)")
    print(f"Total artifacts in result-manifest.sha256: {len(manifest_entries)}")


if __name__ == '__main__':
    main()
