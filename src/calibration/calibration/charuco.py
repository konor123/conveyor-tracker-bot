"""ChArUco 보드 검출과 자세(SPEC-600 4.4절). OpenCV 4.6 aruco API.

보드 좌표계: 원점은 보드 왼쪽 위 바깥 모서리, x는 squares_x 방향, y는 squares_y 방향, z는 보드 뒤쪽.
코너 id는 왼쪽 위 안쪽 코너부터 행 우선(한 행에 squares_x − 1개).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .errors import CalibrationError
from .geometry import make_T

POSE_MIN_CORNERS = 6


@dataclass(frozen=True)
class BoardSpec:
    squares_x: int
    squares_y: int
    square_m: float
    marker_m: float
    dictionary: str

    @classmethod
    def from_dict(cls, d: dict) -> 'BoardSpec':
        return cls(squares_x=int(d['squares_x_count']), squares_y=int(d['squares_y_count']),
                   square_m=float(d['square_m']), marker_m=float(d['marker_m']), dictionary=str(d['dictionary']))

    @property
    def n_corners(self) -> int:
        return (self.squares_x - 1) * (self.squares_y - 1)


def _dictionary(spec: BoardSpec):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))


def _board(spec: BoardSpec):
    return cv2.aruco.CharucoBoard_create(spec.squares_x, spec.squares_y, spec.square_m, spec.marker_m,
                                         _dictionary(spec))


def board_points(spec: BoardSpec, ids) -> np.ndarray:
    """코너 id → 보드 좌표 [m] (N, 3)."""
    ids = np.asarray(ids, int).ravel()
    if np.any(ids < 0) or np.any(ids >= spec.n_corners):
        raise ValueError(f'코너 id 범위 밖: {ids}')
    nx = spec.squares_x - 1
    return np.column_stack([(ids % nx + 1) * spec.square_m, (ids // nx + 1) * spec.square_m, np.zeros(len(ids))])


def render_board(spec: BoardSpec, px_per_m: float) -> np.ndarray:
    """보드 그림(여백 없음, 흰 255 / 검정 0). 시험용."""
    size = (int(round(spec.squares_x * spec.square_m * px_per_m)), int(round(spec.squares_y * spec.square_m * px_per_m)))
    return _board(spec).draw(size, marginSize=0, borderBits=1)


def detect_charuco(image: np.ndarray, spec: BoardSpec, K: np.ndarray, dist: np.ndarray
                   ) -> tuple[np.ndarray, np.ndarray]:
    """영상 → (코너 id (N,), 픽셀 (N, 2), 왜곡 보정 전). 마커가 없으면 빈 배열."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE   # 코너는 체커 교차점에서 따로 정밀화한다
    corners, ids, _ = cv2.aruco.detectMarkers(gray, _dictionary(spec), parameters=params)
    if ids is None or len(ids) == 0:
        return np.zeros(0, int), np.zeros((0, 2))
    n, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, _board(spec),
                                                    cameraMatrix=np.asarray(K, float),
                                                    distCoeffs=np.asarray(dist, float))
    if not n:
        return np.zeros(0, int), np.zeros((0, 2))
    # interpolateCornersCharuco의 정밀화 창은 이웃 마커의 검은 테두리까지 넣어 코너를 0.3 px 정도 끌어당긴다.
    # 칸 모서리 ~ 마커 사이 여백보다 작은 창으로 다시 정밀화한다
    side_px = float(np.mean([np.linalg.norm(np.roll(q[0], 1, axis=0) - q[0], axis=1).mean() for q in corners]))
    margin_px = side_px * (spec.square_m - spec.marker_m) / (2.0 * spec.marker_m)
    win = max(2, int(0.8 * margin_px))
    cc = cv2.cornerSubPix(gray, cc.astype(np.float32), (win, win), (-1, -1),
                          (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-4))
    ids_out = ci.ravel().astype(int)
    order = np.argsort(ids_out)
    return ids_out[order], cc.reshape(-1, 2)[order].astype(float)


def board_pose(ids, uv, spec: BoardSpec, K: np.ndarray, dist: np.ndarray) -> tuple[np.ndarray, float]:
    """코너 → (camera_color_optical_frame ← board 4x4, 재투영 rms [px])."""
    ids = np.asarray(ids, int).ravel()
    px = np.asarray(uv, float).reshape(-1, 2)
    if len(ids) < POSE_MIN_CORNERS:
        raise CalibrationError('DEGENERATE', f'board_pose: 코너 {POSE_MIN_CORNERS}개 이상이 필요하다(받음 {len(ids)})')
    obj = board_points(spec, ids)
    if np.linalg.matrix_rank(obj[:, :2] - obj[:, :2].mean(axis=0), tol=1e-6) < 2:
        raise CalibrationError('DEGENERATE', 'board_pose: 코너가 한 직선 위에 있다')
    K = np.asarray(K, float)
    dist = np.asarray(dist, float)
    ok, rvec, tvec = cv2.solvePnP(obj, px, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise CalibrationError('DEGENERATE', 'board_pose: solvePnP 실패')
    rvec, tvec = cv2.solvePnPRefineLM(obj, px, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - px) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    return make_T(R, tvec.ravel()), rms
