"""왜곡 보정 RGB 광선과 대상물 평면의 교차, 픽셀 공분산 전파.

모든 공간 값은 robot_base 기준 m. calibration·ROS를 import하지 않는다.
공분산은 픽셀 중심의 불확실성만 포함하며 K·외부 자세·평면 오차는 포함하지 않는다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


def _default_K():
    # src/calibration/config/d455_color_1280x800.yaml (2026-11-07 기록).
    return np.array([[646.3336181640625, 0., 640.61962890625],
                     [0., 645.4055175781250, 405.6260681152344], [0., 0., 1.]])


def _default_dist():
    return np.array([-.05499815568327904, .06443826854228973,
                     -.0008744244114495814, .0007429665420204401, -.021013302728533745])


def _finite_array(value, shape, name):
    array = np.array(value, dtype=float, copy=True)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f'{name} must be finite with shape {shape}')
    return array


@dataclass(frozen=True)
class CameraModel:
    """OpenCV pinhole K와 plumb_bob (k1,k2,p1,p2,k3), 왜곡 전 픽셀 입력."""
    K: np.ndarray = field(default_factory=_default_K)
    dist: np.ndarray = field(default_factory=_default_dist)

    def __post_init__(self):
        K = _finite_array(self.K, (3, 3), 'K')
        dist = _finite_array(self.dist, (5,), 'dist')
        if (K[0, 0] <= 0 or K[1, 1] <= 0 or K[0, 1] != 0 or K[1, 0] != 0
                or not np.array_equal(K[2], [0, 0, 1])):
            raise ValueError('K must be an OpenCV pinhole intrinsic matrix')
        object.__setattr__(self, 'K', K)
        object.__setattr__(self, 'dist', dist)


@dataclass(frozen=True)
class Plane:
    """normal·point = offset, calibration.Plane과 같은 속성 및 height_at.

    입력 법선의 길이는 임의이며 offset과 함께 정규화한다.
    """
    normal: np.ndarray
    offset: float

    def __post_init__(self):
        normal = _finite_array(self.normal, (3,), 'normal')
        norm = np.linalg.norm(normal)
        if norm <= 1e-12 or not np.isfinite(self.offset):
            raise ValueError('plane must have a nonzero normal and finite offset')
        object.__setattr__(self, 'normal', normal / norm)
        object.__setattr__(self, 'offset', float(self.offset) / norm)

    def height_at(self, x: float, y: float) -> float:
        if abs(self.normal[2]) <= 1e-12:
            raise ValueError('vertical plane has no unique height')
        return float((self.offset - self.normal[:2] @ [x, y]) / self.normal[2])


@dataclass(frozen=True)
class HeightParams:
    """관측면 높이 [m], 모두 호출자가 전달하는 TBD 제안값(실측 확정 전).

    h_stone_offset은 매트 또는 뚜껑 바닥부터 관측 중심면까지,
    h_rim은 벨트부터 테두리까지, floor_thickness는 뚜껑 바닥 두께다.
    """
    h_stone_offset: float
    h_rim: float
    floor_thickness: float

    def __post_init__(self):
        values = np.array([self.h_stone_offset, self.h_rim, self.floor_thickness])
        if not np.isfinite(values).all() or np.any(values < 0):
            raise ValueError('object heights must be finite and nonnegative')


def select_plane(object_class, mat, belt, params: HeightParams, *, inside_cap=False) -> Plane:
    """STONE/1: 매트, CAP/2: 벨트, inside_cap STONE: 벨트+바닥.

    mat/belt는 normal과 offset 속성이 있는 객체(calibration.Plane 포함).
    높이는 평면 법선 방향 거리가 아닌 robot_base +z 방향 이동이다.
    """
    if object_class in ('STONE', 1):
        base = belt if inside_cap else mat
        height = params.h_stone_offset + (params.floor_thickness if inside_cap else 0.)
    elif object_class in ('CAP', 2) and not inside_cap:
        base, height = belt, params.h_rim
    else:
        raise ValueError('unsupported object class or inside_cap context')
    plane = Plane(base.normal, base.offset)
    return Plane(plane.normal, plane.offset + plane.normal[2] * height)


def pixel_to_plane(u: float, v: float, cam: CameraModel, T_base_cam: np.ndarray,
                   plane: Plane, *, pixel_cov=None) -> tuple[np.ndarray, np.ndarray]:
    """왜곡된 중심 픽셀 → (base 위치[3], 위치 공분산[3,3]).

    pixel_cov는 중심 픽셀 공분산[2,2], 기본 diag(0.25,0.25) px²(TBD).
    왜곡 보정의 중심 차분(0.001 px)과 교차의 해석 야코비안으로 전파한다.
    광선과 평면이 평행하거나 교차가 카메라 뒤/원점이면 ValueError.
    """
    uv = _finite_array([u, v], (2,), 'pixel')
    T = _finite_array(T_base_cam, (4, 4), 'T_base_cam')
    R, origin = T[:3, :3], T[:3, 3]
    if (not np.allclose(T[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(R), 1., atol=1e-8, rtol=0)):
        raise ValueError('T_base_cam must be a rigid homogeneous transform')
    surface = Plane(plane.normal, plane.offset)
    covariance = _finite_array(np.diag([.25, .25]) if pixel_cov is None else pixel_cov,
                               (2, 2), 'pixel_cov')
    if (not np.allclose(covariance, covariance.T, rtol=0, atol=1e-12)
            or np.linalg.eigvalsh(covariance).min() < -1e-12):
        raise ValueError('pixel_cov must be symmetric positive semidefinite')

    step = .001
    samples = np.array([uv, uv + [step, 0], uv - [step, 0],
                        uv + [0, step], uv - [0, step]])
    xy = cv2.undistortPoints(samples.reshape(-1, 1, 2), cam.K, cam.dist).reshape(-1, 2)
    if not np.isfinite(xy).all():
        raise ValueError('undistortion produced a nonfinite ray')
    direction = R @ np.r_[xy[0], 1.]
    n = surface.normal
    denominator = n @ direction
    if abs(denominator) <= 1e-10 * np.linalg.norm(direction):
        raise ValueError('ray is parallel to plane')
    scale = (surface.offset - n @ origin) / denominator
    if scale <= 0:
        raise ValueError('plane intersection must be in front of camera')
    point = origin + scale * direction
    J_xy = np.column_stack([(xy[1] - xy[2]) / (2 * step),
                            (xy[3] - xy[4]) / (2 * step)])
    J_ray = R @ np.vstack([J_xy, np.zeros((1, 2))])
    J = scale * (np.eye(3) - np.outer(direction, n) / denominator) @ J_ray
    cov = J @ covariance @ J.T
    return point, (cov + cov.T) * .5
