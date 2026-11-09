"""왜곡 포함 투영만 제공하는 합성 카메라(render는 별도 범위)."""
from dataclasses import dataclass, field

import cv2
import numpy as np

from vision.localizer import CameraModel


def _pose():
    pose = np.eye(4)
    pose[:3, :3] = [[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]]
    pose[:3, 3] = [.45, .02, .40]
    return pose


@dataclass
class SyntheticCamera:
    """K·dist는 plumb_bob, T_base_cam은 base ← optical 좌표 변환(SI)."""
    K: np.ndarray = field(default_factory=lambda: CameraModel().K.copy())
    dist: np.ndarray = field(default_factory=lambda: CameraModel().dist.copy())
    T_base_cam: np.ndarray = field(default_factory=_pose)

    def project(self, point):
        """base의 점 (3,) 또는 (N,3)을 픽셀 (2,) 또는 (N,2)로 투영한다."""
        points = np.asarray(point, dtype=float)
        single = points.shape == (3,)
        if not single and (points.ndim != 2 or points.shape[1] != 3):
            raise ValueError('point must have shape (3,) or (N,3)')
        points = points.reshape(-1, 3)
        R = self.T_base_cam[:3, :3].T
        t = -R @ self.T_base_cam[:3, 3]
        if not np.isfinite(points).all() or np.any((points @ R.T + t)[:, 2] <= 0):
            raise ValueError('point must be finite and in front of camera')
        rvec, _ = cv2.Rodrigues(R)
        uv, _ = cv2.projectPoints(points, rvec, t, self.K, self.dist)
        uv = uv.reshape(-1, 2)
        return uv[0] if single else uv
