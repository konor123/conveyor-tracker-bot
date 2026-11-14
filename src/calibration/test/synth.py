"""보정 시험용 합성 기하. D455 공장 내부 파라미터(1280x800)와 350 mm 수직 설치를 기본으로 한다."""
import numpy as np

# D455 공장 내부 파라미터(2026-11-07), Color 1280x800
K = np.array([[644.14013671875, 0.0, 640.61962890625],
              [0.0, 643.2152709960938, 405.6260681152344],
              [0.0, 0.0, 1.0]])
DIST = np.array([-0.05499815568327904, 0.06443826854228973, -0.0008744244114495814,
                 0.0007429665420204401, -0.021013302728533745])
W, H = 1280, 800

BELT_Z = 0.050      # robot_base 기준 벨트면 높이 [m](가정)
MAT_Z = 0.070       # 매트면 높이 [m](가정)
SPACER_Z = BELT_Z + 0.050  # 벨트 위 50 mm 받침에 올린 보드면 [m]
CAM_HEIGHT = 0.350  # 벨트면 → 렌즈 [m](site-survey L01)


def rot(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(ang), np.sin(ang)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def camera_pose(tilt_rad=np.deg2rad(3.0)):
    """robot_base ← camera_color_optical_frame 4x4. 광축이 아래(-z)를 보고 tilt만큼 기운다."""
    # 광학 프레임: x 오른쪽, y 아래, z 앞. 아래를 보는 기본 자세: x_c = +y_b, y_c = +x_b, z_c = -z_b
    R0 = np.array([[0.0, 1.0, 0.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = rot([1.0, 0.3, 0.0], tilt_rad) @ R0
    T[:3, 3] = [0.45, 0.02, BELT_Z + CAM_HEIGHT]
    return T


def transform(T, p):
    p = np.atleast_2d(p)
    return p @ T[:3, :3].T + T[:3, 3]


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def project(T_base_cam, p_base, K=K, dist=DIST):
    """robot_base 점 → 왜곡 포함 픽셀(OpenCV plumb_bob = RealSense inverse_brown_conrady)."""
    import cv2
    T_cam_base = inv(T_base_cam)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    uv, _ = cv2.projectPoints(np.atleast_2d(p_base).astype(float), rvec, T_cam_base[:3, 3], K, dist)
    return uv.reshape(-1, 2)


def work_points(rng, n, z_values=(BELT_Z, MAT_Z, SPACER_Z), margin_px=60):
    """시야 안(가장자리 margin_px 제외)에 들어오는 작업면 점 n개."""
    T = camera_pose()
    out = []
    while len(out) < n:
        p = np.array([rng.uniform(0.20, 0.70), rng.uniform(-0.35, 0.40), z_values[len(out) % len(z_values)]])
        u, v = project(T, p)[0]
        if margin_px <= u <= W - margin_px and margin_px <= v <= H - margin_px:
            out.append(p)
    return np.array(out)


def rot_err_rad(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))
