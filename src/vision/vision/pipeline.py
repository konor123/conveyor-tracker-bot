"""한 프레임 처리: ROI별 검출 → stone_inside → 대상물별 평면 교차(SPEC-200 4.4절). ROS와 무관한 순수 모듈."""
from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from .detector import (CLASS_CAP, CLASS_STONE, REGION_BELT, REGION_MAT, Blob, ColorModel, DetectParams,
                       InsideParams, Roi, SceneParams, default_classes, detect, find_belt_roi, find_mat_roi,
                       stone_inside)
from .localizer import CameraModel, HeightParams, Plane, pixel_to_plane, select_plane


@dataclass(frozen=True)
class SceneConfig:
    """mat_roi/belt_roi가 None이면 프레임마다 자동 탐색한다(보정 전 확인용)."""
    mat_roi: Roi | None = None
    belt_roi: Roi | None = None
    classes: dict[int, ColorModel] = field(default_factory=default_classes)
    detect: DetectParams = field(default_factory=DetectParams)
    inside: InsideParams = field(default_factory=InsideParams)
    scene: SceneParams = field(default_factory=SceneParams)
    # ROI 사각형 + 이 여유[px]만 잘라 검출한다(REQ-VIS-011). None이면 끈다(전체 프레임, 정적 투입 경로).
    # 실시간 vision_node만 켠다
    crop_margin_px: int | None = None


@dataclass(frozen=True)
class Geometry:
    """보정 결과. 없으면 position_valid=false로 픽셀 검출만 낸다."""
    calibration_id: str
    cam: CameraModel
    T_base_cam: np.ndarray
    mat: Plane
    belt: Plane
    heights: HeightParams


@dataclass(frozen=True)
class Detection:
    blob: Blob
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    position_cov: tuple[float, ...] = (0.0,) * 9
    position_valid: bool = False
    stone_inside: bool = False
    stone_inside_valid: bool = False
    inside_cap: bool = False          # STONE 전용: 벨트 위 뚜껑 원 안에 있다

    @property
    def region(self) -> int:
        return self.blob.region

    @property
    def object_class(self) -> int:
        return self.blob.object_class


def _rois(image_bgr, cfg: SceneConfig) -> list[Roi]:
    mat = cfg.mat_roi or find_mat_roi(image_bgr, cfg.scene)
    belt = cfg.belt_roi or find_belt_roi(image_bgr, cfg.scene)
    return [r for r in (mat, belt) if r is not None]


CROP_MARGIN_PX = 64     # ROI 끝에 중심이 걸친 뚜껑(지름 ≤ 52 px)과 형태 연산 커널이 잘리지 않는 여유


def _detect_in_crop(image_bgr, roi: Roi, classes, robot_mask, params, margin: int = CROP_MARGIN_PX) -> list[Blob]:
    """ROI를 감싸는 사각형(+ 여유)만 잘라 detect를 부르고 픽셀 좌표를 원래 프레임으로 되돌린다(REQ-VIS-011).

    detect는 넘긴 영상 전체를 계산한다. 1280×800 전체를 넘기면 프레임당 약 160 ms였다(2026-11-13 현장).
    - 꼭짓점은 `Roi.mask`와 같이 먼저 반올림한 정수로 옮긴다(짝수 반올림은 평행이동에 대해 불변이 아니다).
    - 범위는 영상 안으로 자르고, 영상과 겹치지 않으면 검출 없음(전체 프레임에서도 ROI 마스크가 비어 없음).
    - 영상 경계가 아닌 crop 경계에 닿은 검출은 버린다. 중심이 ROI 안인 유효 대상물은 ROI 사각형 밖으로 `margin`까지
      닿지 못하므로, 닿은 것은 crop에 잘린 큰 성분(팔·손 등)이다. 전체 프레임에서는 크기·원형도로 빠지는 것이다.
    """
    poly = np.round(np.asarray(roi.polygon, np.float64)).astype(np.int64)
    H, W = image_bgr.shape[:2]
    x0 = min(max(int(poly[:, 0].min()) - margin, 0), W)
    y0 = min(max(int(poly[:, 1].min()) - margin, 0), H)
    x1 = max(min(int(poly[:, 0].max()) + margin + 1, W), 0)
    y1 = max(min(int(poly[:, 1].max()) + margin + 1, H), 0)
    if x1 <= x0 or y1 <= y0:
        return []
    crop_roi = Roi(roi.region, tuple((int(u) - x0, int(v) - y0) for u, v in poly))
    mask = None if robot_mask is None else robot_mask[y0:y1, x0:x1]
    blobs = detect(image_bgr[y0:y1, x0:x1], crop_roi, classes, robot_mask=mask, params=params)
    w, h = x1 - x0, y1 - y0
    out = []
    for b in blobs:
        if b.contour is not None:
            c = b.contour.reshape(-1, 2)
            if ((x0 > 0 and c[:, 0].min() <= 0) or (y0 > 0 and c[:, 1].min() <= 0)
                    or (x1 < W and c[:, 0].max() >= w - 1) or (y1 < H and c[:, 1].max() >= h - 1)):
                continue
        out.append(replace(b, u=b.u + x0, v=b.v + y0,
                           contour=None if b.contour is None else b.contour + np.array([x0, y0], b.contour.dtype)))
    return out


def process_frame(image_bgr: np.ndarray, cfg: SceneConfig, geom: Geometry | None = None,
                  robot_mask: np.ndarray | None = None) -> list[Detection]:
    """매트 ROI는 공깃돌만, 벨트 ROI는 병뚜껑·공깃돌을 찾는다."""
    out: list[Detection] = []
    for roi in _rois(image_bgr, cfg):
        classes = cfg.classes if roi.region == REGION_BELT else {
            c: m for c, m in cfg.classes.items() if c == CLASS_STONE}
        if cfg.crop_margin_px is None:
            blobs = detect(image_bgr, roi, classes, robot_mask=robot_mask, params=cfg.detect)
        else:
            blobs = _detect_in_crop(image_bgr, roi, classes, robot_mask, cfg.detect, cfg.crop_margin_px)
        stones = [b for b in blobs if b.object_class == CLASS_STONE]
        caps = [b for b in blobs if b.object_class == CLASS_CAP]
        r_in = {id(c): 0.5 * c.diameter_px * cfg.inside.inner_diameter_ratio for c in caps}
        for b in blobs:
            inside_cap = (b.object_class == CLASS_STONE and roi.region == REGION_BELT
                          and any(np.hypot(b.u - c.u, b.v - c.v) < r_in[id(c)] for c in caps))
            si, si_valid = False, False
            if b.object_class == CLASS_CAP:
                verdict = stone_inside(b, stones, cfg.inside)
                si, si_valid = bool(verdict), verdict is not None
            pos, cov, valid = (0.0, 0.0, 0.0), (0.0,) * 9, False
            if geom is not None:
                # 뚜껑 밖 벨트 위 공깃돌은 벨트면 + h_stone_offset(매트 자리에 벨트 평면을 넣는다)
                on_belt = b.object_class == CLASS_STONE and roi.region == REGION_BELT and not inside_cap
                plane = select_plane(b.object_class, geom.belt if on_belt else geom.mat, geom.belt,
                                     geom.heights, inside_cap=inside_cap)
                try:
                    p, c = pixel_to_plane(b.u, b.v, geom.cam, geom.T_base_cam, plane,
                                          pixel_cov=np.reshape(b.center_cov_px, (2, 2)))
                    pos, cov, valid = tuple(float(x) for x in p), tuple(float(x) for x in c.ravel()), True
                except ValueError:
                    pass
            out.append(Detection(blob=b, position=pos, position_cov=cov, position_valid=valid,
                                 stone_inside=si, stone_inside_valid=si_valid, inside_cap=inside_cap))
    return out


__all__ = ['SceneConfig', 'Geometry', 'Detection', 'process_frame', 'REGION_MAT', 'REGION_BELT']
