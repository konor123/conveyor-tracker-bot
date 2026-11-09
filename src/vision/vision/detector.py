"""공깃돌·병뚜껑 픽셀 검출(SPEC-200 4.1~4.3절). ROS와 무관한 순수 모듈.

- 공깃돌: 면마다 색이 달라(노랑·분홍·보라·빨강·파랑) 색 범위 대신 ROI 배경색과의 Lab 거리로 찾는다.
- 병뚜껑: 색 범위(HSV) 마스크 + 구멍 채움 + 원형도 필터. 뚜껑 안 공깃돌은 뚜껑 색이 아닌 전경으로 찾는다.
- 중심은 경계 픽셀에 배경 거리 비율의 연속 가중치를 주어 구한다(서브픽셀).

좌표는 왜곡 보정 전 픽셀 (u 오른쪽, v 아래)이다. 픽셀 단위 값은 `_px` 이름을 쓰고 설정 파일에서는 `unitless`에 둔다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import cv2
import numpy as np

# drop_interfaces/msg/ObjectDetection.msg 상수와 같은 값
REGION_UNKNOWN = 0
REGION_MAT = 1
REGION_BELT = 2
CLASS_UNKNOWN = 0
CLASS_STONE = 1
CLASS_CAP = 2


@dataclass(frozen=True)
class Roi:
    """검출 영역. polygon은 픽셀 꼭짓점 (u, v) 열."""
    region: int
    polygon: tuple[tuple[float, float], ...]

    def mask(self, shape: tuple[int, ...]) -> np.ndarray:
        m = np.zeros(shape[:2], np.uint8)
        cv2.fillPoly(m, [np.round(np.asarray(self.polygon, np.float64)).astype(np.int32)], 1)
        return m


@dataclass(frozen=True)
class ColorModel:
    """대상물 하나의 색·형상 모델.

    hsv_lo/hsv_hi가 있으면 그 범위(OpenCV HSV, H 0~179)를 대상물 색으로 본다.
    없으면 ROI 배경색(ROI 중앙값)과 Lab 거리가 delta_e_min 이상인 픽셀을 대상물로 본다.
    """
    diameter_min_px: float
    diameter_max_px: float
    circularity_min: float = 0.70
    hsv_lo: tuple[int, int, int] | None = None
    hsv_hi: tuple[int, int, int] | None = None
    delta_e_min: float = 30.0      # Lab 거리(CIE76), 이 값 이상이면 전경
    delta_e_soft: float = 15.0     # 경계 가중치가 0이 되는 거리(delta_e_min에서 1)
    ring_coverage_min: float = 0.50  # 끊긴 고리를 원 적합할 때 바깥 경계가 덮어야 하는 각도 비율


@dataclass(frozen=True)
class DetectParams:
    roi_edge_margin_px: int = 4          # ROI 가장자리 제외 폭
    open_kernel_px: int = 3              # 전경 열림 연산 커널
    exclude_dilate_px: int = 6           # 큰 덩어리(손·마커) 주변 제외 폭
    color_dilate_px: int = 1             # 색 범위 대상물 경계 혼합 픽셀 제외 폭
    center_std_px: float = 0.5           # 중심 표준편차 제안값(TBD, OFF-1 정지 영상 반복성)
    occlusion_dilate_px: int = 2         # 로봇 마스크 겹침 판정 여유
    cap_search_ratio: float = 0.85       # 뚜껑 안 공깃돌을 뚜껑 색 기준으로 찾는 원 반지름 / 바깥 반지름(테두리 하이라이트 제외)


@dataclass(frozen=True)
class Blob:
    object_class: int
    region: int
    u: float
    v: float
    diameter_px: float
    area_px: float
    circularity: float
    confidence: float
    center_cov_px: tuple[float, float, float, float]
    occluded_partial: bool = False
    hue_deg: float | None = None
    contour: np.ndarray | None = field(default=None, compare=False, repr=False)


@dataclass(frozen=True)
class InsideParams:
    inner_diameter_ratio: float = 0.80   # 안쪽 지름 / 바깥 지름. TBD(D_in 실측 OFF-1)


def default_classes() -> dict[int, ColorModel]:
    """2026-11-09 OFF-1 정지 영상(D455 1280x800, 공깃돌 약 17 px, 파란 병뚜껑 약 38 px) 기준 제안값."""
    return {
        CLASS_STONE: ColorModel(diameter_min_px=11.0, diameter_max_px=26.0, circularity_min=0.70,
                                delta_e_min=30.0, delta_e_soft=15.0),
        CLASS_CAP: ColorModel(diameter_min_px=28.0, diameter_max_px=52.0, circularity_min=0.75,
                              hsv_lo=(90, 120, 70), hsv_hi=(118, 255, 255)),
    }


def _lab(image_bgr: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    lab[..., 0] *= 100.0 / 255.0
    lab[..., 1:] -= 128.0
    return lab


def _kernel(r: int) -> np.ndarray:
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))


def _fill_holes(mask: np.ndarray) -> np.ndarray:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = np.zeros_like(mask)
    cv2.drawContours(out, contours, -1, 1, thickness=cv2.FILLED)
    return out


def _circularity(comp: np.ndarray) -> tuple[float, np.ndarray | None]:
    contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return 0.0, None
    c = max(contours, key=cv2.contourArea)
    perim = cv2.arcLength(c, True)
    area = float(comp.sum())
    # 픽셀 경계 둘레는 실제보다 길어 작은 원의 원형도를 낮게 잰다. 반 픽셀 보정한 면적을 쓴다.
    return (min(1.0, 4.0 * np.pi * (area + 0.5 * perim) / (perim * perim)) if perim > 0 else 0.0), c


def _weighted_center(weight: np.ndarray, x0: int, y0: int) -> tuple[float, float, float]:
    s = float(weight.sum())
    ys, xs = np.mgrid[0:weight.shape[0], 0:weight.shape[1]]
    return x0 + float((weight * xs).sum()) / s, y0 + float((weight * ys).sum()) / s, s


def _hue_deg(image_bgr: np.ndarray, comp: np.ndarray) -> float:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    h = hsv[..., 0][comp > 0].astype(np.float64) * (np.pi / 90.0)
    return float(np.degrees(np.arctan2(np.sin(h).mean(), np.cos(h).mean())) % 360.0)


def _components(mask: np.ndarray):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        yield i, labels, int(x), int(y), int(w), int(h), int(a)


def _blob(image_bgr, comp_full, weight_full, model, object_class, region, p, robot_mask):
    """comp_full(0/1 전체 영상 크기) 하나를 Blob으로 만든다. 형상 조건을 못 넘으면 None."""
    ys, xs = np.nonzero(comp_full)
    pad = 3 + p.occlusion_dilate_px
    x0, y0 = max(int(xs.min()) - pad, 0), max(int(ys.min()) - pad, 0)
    x1 = min(int(xs.max()) + pad + 1, comp_full.shape[1])
    y1 = min(int(ys.max()) + pad + 1, comp_full.shape[0])
    comp = comp_full[y0:y1, x0:x1]
    circ, contour = _circularity(comp)
    support = cv2.dilate(comp, _kernel(1))
    w = np.where(comp > 0, 1.0, weight_full[y0:y1, x0:x1] * support).astype(np.float64)
    u, v, area = _weighted_center(w, x0, y0)
    diameter = float(np.sqrt(4.0 * area / np.pi))
    if not (model.diameter_min_px <= diameter <= model.diameter_max_px) or circ < model.circularity_min:
        return None
    occluded = False
    if robot_mask is not None:
        near = cv2.dilate(comp, _kernel(p.occlusion_dilate_px))
        occluded = bool((near & (robot_mask[y0:y1, x0:x1] > 0)).any())
    s2 = p.center_std_px ** 2
    if contour is not None:
        contour = contour + np.array([x0, y0], np.int32)
    return Blob(object_class=object_class, region=region, u=u, v=v, diameter_px=diameter, area_px=area,
                circularity=circ, confidence=float(circ), center_cov_px=(s2, 0.0, 0.0, s2),
                occluded_partial=occluded, hue_deg=_hue_deg(image_bgr[y0:y1, x0:x1], comp),
                contour=contour)


def _fit_circle(pts: np.ndarray) -> tuple[float, float, float]:
    """대수적 최소제곱 원 적합(Kasa). pts: N×2 (u, v)."""
    x, y = pts[:, 0].astype(np.float64), pts[:, 1].astype(np.float64)
    A = np.column_stack([x, y, np.ones_like(x)])
    sol, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    cu, cv = sol[0] / 2.0, sol[1] / 2.0
    return cu, cv, float(np.sqrt(sol[2] + cu * cu + cv * cv))


def _ring_blob(image_bgr, comp, model, object_class, region, p, robot_mask):
    """끊긴 고리 모양 성분에서 바깥 원을 구한다. 각도 덮임 비율이 ring_coverage_min 미만이면 None."""
    contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    pts = np.vstack(contours).reshape(-1, 2).astype(np.float64)
    (cu, cv), r = cv2.minEnclosingCircle(pts.astype(np.float32))
    outer = pts[np.abs(np.hypot(pts[:, 0] - cu, pts[:, 1] - cv) - r) < 2.0]
    if len(outer) < 12:
        return None
    cu, cv, r = _fit_circle(outer)
    diameter = 2.0 * r + 1.0          # 경계 픽셀 중심은 실제 경계보다 반 픽셀 안쪽
    ang = np.arctan2(outer[:, 1] - cv, outer[:, 0] - cu)
    coverage = len(np.unique(np.floor((ang + np.pi) / (2 * np.pi) * 36).astype(int) % 36)) / 36.0
    if not (model.diameter_min_px <= diameter <= model.diameter_max_px) or coverage < model.ring_coverage_min:
        return None
    occluded = False
    if robot_mask is not None:
        disk = np.zeros_like(comp)
        cv2.circle(disk, (int(round(cu)), int(round(cv))), int(np.ceil(r)) + p.occlusion_dilate_px, 1, -1)
        occluded = bool((disk & (robot_mask > 0)).any())
    s2 = p.center_std_px ** 2
    return Blob(object_class=object_class, region=region, u=cu, v=cv, diameter_px=diameter,
                area_px=float(np.pi * diameter * diameter / 4.0), circularity=coverage, confidence=coverage,
                center_cov_px=(s2, 0.0, 0.0, s2), occluded_partial=occluded,
                hue_deg=_hue_deg(image_bgr, comp), contour=np.vstack(contours))


def detect(image_bgr: np.ndarray, roi: Roi, classes: dict[int, ColorModel], *,
           robot_mask: np.ndarray | None = None, params: DetectParams | None = None) -> list[Blob]:
    """ROI 하나에서 classes의 대상물을 찾는다. 색 범위 class를 먼저 찾고, 배경 거리 class는 채택된 색 범위
    대상물의 색 픽셀을 뺀 전경에서 찾는다.

    robot_mask(0/1)와 겹치는 검출은 버리지 않고 occluded_partial=True로 표시한다(개수 제외는 호출자 몫).
    """
    p = params or DetectParams()
    roi_mask = roi.mask(image_bgr.shape)
    roi_in = cv2.erode(roi_mask, _kernel(p.roi_edge_margin_px)) if p.roi_edge_margin_px > 0 else roi_mask
    if not roi_in.any():
        return []
    lab = _lab(image_bgr)
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    background = np.median(lab[roi_in > 0], axis=0)
    dist = np.linalg.norm(lab - background, axis=2)

    color_raw = np.zeros(roi_in.shape, np.uint8)
    caps: list[tuple[Blob, np.ndarray]] = []     # (뚜껑, 뚜껑 색 Lab 중앙값)
    out: list[Blob] = []
    ranged = [(c, m) for c, m in classes.items() if m.hsv_lo is not None]
    by_bg = [(c, m) for c, m in classes.items() if m.hsv_lo is None]

    for object_class, m in ranged:
        # ROI 끝에 걸친 뚜껑도 온전히 보도록 색 마스크는 자르지 않고, 중심이 ROI 안인 것만 쓴다
        raw = cv2.inRange(hsv, np.array(m.hsv_lo, np.uint8), np.array(m.hsv_hi, np.uint8)) // 255
        filled = _fill_holes(cv2.morphologyEx(raw, cv2.MORPH_CLOSE, _kernel(2)))
        soft = np.clip((dist - m.delta_e_soft) / max(m.delta_e_min - m.delta_e_soft, 1e-6), 0.0, 1.0)
        for i, labels, *_ in _components(filled):
            comp = (labels == i).astype(np.uint8)
            if not (comp & roi_in).any():
                continue
            b = _blob(image_bgr, comp, soft, m, object_class, roi.region, p, robot_mask)
            if b is None:
                # 뚜껑 안 공깃돌이나 하이라이트로 고리가 끊기면 채움이 안 된다. 바깥 경계 원 적합으로 구한다
                b = _ring_blob(image_bgr, comp, m, object_class, roi.region, p, robot_mask)
            if b is not None and roi_mask[min(int(round(b.v)), roi_mask.shape[0] - 1),
                                          min(int(round(b.u)), roi_mask.shape[1] - 1)]:
                out.append(b)
                # 채택한 대상물의 색 픽셀만 배경 거리 class에서 뺀다(같은 색의 작은 파란 공깃돌은 남긴다)
                color_raw |= raw & comp
                caps.append((b, np.median(lab[(raw & comp) > 0], axis=0)))

    if by_bg:
        exclude_color = cv2.dilate(color_raw, _kernel(p.color_dilate_px)) if p.color_dilate_px > 0 else color_raw
        for object_class, m in by_bg:
            fg = ((dist >= m.delta_e_min) & (roi_in > 0) & (exclude_color == 0)).astype(np.uint8)
            if p.open_kernel_px > 1:
                fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, _kernel(p.open_kernel_px // 2))
            # 손·마커처럼 큰 덩어리와 그 안쪽 조각은 대상물이 아니다
            big = np.zeros_like(fg)
            area_max = np.pi * (m.diameter_max_px / 2.0) ** 2 * 1.5
            for i, labels, x, y, w, h, a in _components(fg):
                if a > area_max:
                    cs, _ = cv2.findContours((labels == i).astype(np.uint8), cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)
                    cv2.fillPoly(big, [cv2.convexHull(np.vstack(cs))], 1)
            if big.any():
                big = cv2.dilate(big, _kernel(p.exclude_dilate_px))
            soft = np.clip((dist - m.delta_e_soft) / max(m.delta_e_min - m.delta_e_soft, 1e-6), 0.0, 1.0)
            soft[exclude_color > 0] = 0.0
            # ROI 경계에 닿은 성분은 일부가 ROI 밖이라 중심을 믿을 수 없다(레일·매트 가장자리 조각)
            edge = roi_in - cv2.erode(roi_in, _kernel(1))
            search = [(c, 0.5 * c.diameter_px * p.cap_search_ratio) for c, _ in caps]
            for i, labels, x, y, w, h, a in _components(fg):
                comp = (labels == i).astype(np.uint8)
                if (comp & big).any() or (comp & edge).any():
                    continue
                b = _blob(image_bgr, comp, soft, m, object_class, roi.region, p, robot_mask)
                if b is not None and not any(np.hypot(b.u - c.u, b.v - c.v) < r for c, r in search):
                    out.append(b)
            # 뚜껑 안은 배경이 벨트가 아니라 뚜껑이다. 뚜껑 색과의 거리로 다시 찾는다(벨트와 색이 비슷한 노란 돌)
            for cap, cap_lab in caps:
                disk = np.zeros_like(roi_in)
                cv2.circle(disk, (int(round(cap.u)), int(round(cap.v))),
                           int(0.5 * cap.diameter_px * p.cap_search_ratio), 1, -1)
                cdist = np.linalg.norm(lab - cap_lab, axis=2)
                cfg = ((cdist >= m.delta_e_min) & (disk > 0) & (exclude_color == 0)).astype(np.uint8)
                if p.open_kernel_px > 1:
                    cfg = cv2.morphologyEx(cfg, cv2.MORPH_OPEN, _kernel(p.open_kernel_px // 2))
                csoft = np.clip((cdist - m.delta_e_soft) / max(m.delta_e_min - m.delta_e_soft, 1e-6), 0.0, 1.0)
                csoft[exclude_color > 0] = 0.0
                for i, labels, *_ in _components(cfg):
                    b = _blob(image_bgr, (labels == i).astype(np.uint8), csoft, m, object_class, roi.region, p,
                              robot_mask)
                    if b is not None:
                        out.append(b)
    return out


def stone_inside(cap: Blob, stones: Sequence[Blob], p: InsideParams | None = None) -> bool | None:
    """뚜껑 원 안에 공깃돌 중심이 있으면 True. 뚜껑이 가려졌으면 None(판정 불가).

    안쪽 테두리에 닿은 돌은 중심 거리 = r_in − r_stone < r_in, 바깥 테두리에 닿은 돌은 r_out + r_stone > r_in이다.
    """
    p = p or InsideParams()
    if cap.occluded_partial:
        return None
    r_in = 0.5 * cap.diameter_px * p.inner_diameter_ratio
    return any(np.hypot(s.u - cap.u, s.v - cap.v) < r_in for s in stones if s.object_class == CLASS_STONE)


@dataclass(frozen=True)
class SceneParams:
    """ROI 자동 탐색(설정 ROI가 없을 때·확인용). 운용 ROI는 보정 결과로 고정한다."""
    mat_v_max: int = 110                 # 매트(검정) HSV 밝기 상한
    mat_area_min_px: int = 15000
    belt_hsv_lo: tuple[int, int, int] = (33, 70, 90)
    belt_hsv_hi: tuple[int, int, int] = (50, 255, 235)
    belt_area_min_px: int = 4000
    belt_end_margin_px: float = 24.0     # 벨트 사각형을 길이 방향 양 끝으로 넓히는 폭(끝에 걸친 뚜껑 포함)
    belt_side_margin_px: float = 0.0     # 폭 방향으로 넓히는 폭


def find_mat_roi(image_bgr: np.ndarray, p: SceneParams | None = None) -> Roi | None:
    """영상 가장자리에 닿지 않는 가장 큰 어두운 영역의 외곽(볼록 껍질)."""
    p = p or SceneParams()
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    dark = (hsv[..., 2] <= p.mat_v_max).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, _kernel(2))
    H, W = dark.shape
    best = None
    for i, labels, x, y, w, h, a in _components(dark):
        if a < p.mat_area_min_px or x == 0 or y == 0 or x + w >= W or y + h >= H:
            continue
        if best is None or a > best[1]:
            best = (i, a, labels)
    if best is None:
        return None
    cs, _ = cv2.findContours((best[2] == best[0]).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hull = cv2.convexHull(np.vstack(cs)).reshape(-1, 2)
    return Roi(REGION_MAT, tuple((float(u), float(v)) for u, v in hull))


def find_belt_roi(image_bgr: np.ndarray, p: SceneParams | None = None) -> Roi | None:
    """가장 큰 벨트색 영역의 최소 외접 사각형을 길이 방향으로 belt_end_margin_px만큼 넓힌 것."""
    p = p or SceneParams()
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    green = cv2.inRange(hsv, np.array(p.belt_hsv_lo, np.uint8), np.array(p.belt_hsv_hi, np.uint8)) // 255
    best = None
    for i, labels, x, y, w, h, a in _components(green):
        if a >= p.belt_area_min_px and (best is None or a > best[1]):
            best = (i, a, labels)
    if best is None:
        return None
    pts = np.column_stack(np.nonzero(best[2] == best[0]))[:, ::-1].astype(np.float32)
    (cu, cv), (w, h), ang = cv2.minAreaRect(pts)
    grow_w, grow_h = ((p.belt_end_margin_px, p.belt_side_margin_px) if w >= h
                      else (p.belt_side_margin_px, p.belt_end_margin_px))
    box = cv2.boxPoints(((cu, cv), (w + 2 * grow_w, h + 2 * grow_h), ang))
    return Roi(REGION_BELT, tuple((float(u), float(v)) for u, v in box))
