"""한 프레임 처리: ROI별 검출 → stone_inside → 대상물별 평면 교차(SPEC-200 4.4절). ROS와 무관한 순수 모듈."""
from __future__ import annotations

from dataclasses import dataclass, field

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


def process_frame(image_bgr: np.ndarray, cfg: SceneConfig, geom: Geometry | None = None,
                  robot_mask: np.ndarray | None = None) -> list[Detection]:
    """매트 ROI는 공깃돌만, 벨트 ROI는 병뚜껑·공깃돌을 찾는다."""
    out: list[Detection] = []
    for roi in _rois(image_bgr, cfg):
        classes = cfg.classes if roi.region == REGION_BELT else {
            c: m for c, m in cfg.classes.items() if c == CLASS_STONE}
        blobs = detect(image_bgr, roi, classes, robot_mask=robot_mask, params=cfg.detect)
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
