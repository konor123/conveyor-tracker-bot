"""드라이버 CameraInfo ↔ 보정에 쓴 카메라 프로파일 비교(SPEC-600 4.8절)."""
from __future__ import annotations

import cv2
import numpy as np

TOL_PX = 0.5                 # K 항목·왜곡 투영 차이 허용 [px]
GRID_COUNT = 9               # 왜곡 비교 격자(한 변)


def _grid(width, height):
    """영상 전체를 덮는 GRID_COUNT × GRID_COUNT 픽셀 격자."""
    u, v = np.meshgrid(np.linspace(0, width - 1, GRID_COUNT), np.linspace(0, height - 1, GRID_COUNT))
    return u.ravel(), v.ravel()


def compare_camera_info(info: dict, profile: dict, tol_px: float = TOL_PX, check_focal: bool = True) -> list[str]:
    """info: width, height, distortion_model, k(9), d. 불일치 설명 목록(비어 있으면 일치).

    D455 펌웨어가 스트림 시작마다 보고하는 fx·fy는 0.3% 정도 흔들리지만 실제 영상 배율 변화는 0.04%였다
    (SPEC-600 4.8절). check_focal=False면 fx·fy는 비교하지 않는다.
    """
    out = []
    for key in ('width', 'height', 'distortion_model'):
        if info[key] != profile[key]:
            out.append(f'{key}: CameraInfo {info[key]} ≠ 프로파일 {profile[key]}')
    if out:
        return out
    k = np.asarray(info['k'], float).reshape(3, 3)
    names = (('fx', k[0, 0]), ('fy', k[1, 1])) if check_focal else ()
    for name, val in names + (('cx', k[0, 2]), ('cy', k[1, 2])):
        if abs(val - float(profile[name])) > tol_px:
            out.append(f'{name}: CameraInfo {val:.3f} ≠ 프로파일 {float(profile[name]):.3f} (허용 {tol_px} px)')
    d_info = np.asarray(info['d'], float)
    d_prof = np.asarray(profile['distortion'], float)
    if d_info.shape != d_prof.shape:
        return out + [f'distortion: 계수 개수 {len(d_info)} ≠ {len(d_prof)}']
    # 계수 차이 대신 같은 광선을 두 계수로 투영한 픽셀 차이로 비교한다
    K_prof = np.array([[profile['fx'], 0.0, profile['cx']], [0.0, profile['fy'], profile['cy']], [0.0, 0.0, 1.0]])
    u, v = _grid(info['width'], info['height'])
    xn = cv2.undistortPointsIter(np.column_stack([u, v]).reshape(-1, 1, 2), K_prof, d_prof, None, None,
                                 (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-12)).reshape(-1, 2)
    pts = np.column_stack([xn, np.ones(len(xn))])
    a, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K_prof, d_prof)
    b, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K_prof, d_info)
    diff = float(np.abs(a - b).max())
    if diff > tol_px:
        out.append(f'distortion: 같은 광선의 투영 차이 최대 {diff:.2f} px (허용 {tol_px} px)')
    return out
