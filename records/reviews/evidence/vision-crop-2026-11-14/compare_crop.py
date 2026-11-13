#!/usr/bin/env python3
"""저장한 실제 프레임에서 vision `process_frame`의 crop 켬·끔 결과를 비교한다(REQ-VIS-011).

사용: python3 compare_crop.py <프레임 목록 파일> [저장소 루트]
설정 두 가지: 정적 투입 경로(SceneConfig 기본 + static_pick_place의 inner_diameter_ratio)와 vision_node 경로
(src/vision/config/detector.yaml). 둘 다 ROI는 프레임마다 자동으로 찾는다(현장 설정과 같음).
"""
import sys
import time
from dataclasses import replace
from pathlib import Path

import cv2
import yaml

from vision.detector import ColorModel, DetectParams, InsideParams
from vision.pipeline import CROP_MARGIN_PX, SceneConfig, process_frame

root = Path(sys.argv[2] if len(sys.argv) > 2 else '.')
frames = [l.strip() for l in open(sys.argv[1]) if l.strip()]
static_ratio = yaml.safe_load(open(root / 'src/static_pick_place/config/static_pick_place.yaml'))
static_ratio = static_ratio.get('inner_diameter_ratio') if isinstance(static_ratio, dict) else None
det = yaml.safe_load(open(root / 'src/vision/config/detector.yaml'))


def color(key):
    values = dict(det[key])
    for old, new in (('circularity_min_ratio', 'circularity_min'), ('ring_coverage_min_ratio', 'ring_coverage_min')):
        if old in values:
            values[new] = values.pop(old)
    for name in ('hsv_lo', 'hsv_hi'):
        if name in values:
            values[name] = tuple(values[name])
    return ColorModel(**values)


configs = {
    'static': SceneConfig(inside=InsideParams(float(static_ratio))) if static_ratio else SceneConfig(),
    'vision_node': SceneConfig(classes={1: color('stone'), 2: color('cap')}, detect=DetectParams(**det['detect']),
                               inside=InsideParams(inner_diameter_ratio=det['inner_diameter_ratio'])),
}
key = lambda d: (d.blob.object_class, d.blob.region, round(d.blob.u, 4), round(d.blob.v, 4),
                 round(d.blob.diameter_px, 4))
for name, cfg in configs.items():
    n = same = dets = 0
    t_off = t_on = 0.0
    diffs = []
    for f in frames:
        img = cv2.imread(str(root / f), cv2.IMREAD_COLOR)
        if img is None:
            continue
        a = time.perf_counter(); off = process_frame(img, replace(cfg, crop_margin_px=None)); b = time.perf_counter()
        on = process_frame(img, replace(cfg, crop_margin_px=CROP_MARGIN_PX)); c = time.perf_counter()
        t_off += b - a; t_on += c - b; n += 1; dets += len(off)
        if sorted(map(key, off)) == sorted(map(key, on)):
            same += 1
        else:
            diffs.append((f, sorted(set(map(key, off)) - set(map(key, on))), sorted(set(map(key, on)) - set(map(key, off)))))
    print(f'{name}: frames {n}, same {same}, detections(off) {dets}, mean off {1000*t_off/max(n,1):.1f} ms, '
          f'mean on {1000*t_on/max(n,1):.1f} ms')
    for f, only_off, only_on in diffs:
        print(f'  DIFF {f}: only_off={only_off} only_on={only_on}')
