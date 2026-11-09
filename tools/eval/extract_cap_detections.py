#!/usr/bin/env python3
"""벨트 영상(MP4) → 병뚜껑 검출 CSV. 원래 header stamp는 같은 이름의 `.frames.csv`에서 가져온다.

  ~/.venvs/vision/bin/python tools/eval/extract_cap_detections.py \
      records/media/2026-11-07/<name>.rgb.mp4 [...] --out-dir records/bench/belt-video-2026-11-07

OpenCV가 필요하다(검출기 `vision.pipeline`). 출력 CSV는 numpy만으로 읽는다(`replay_belt_tracking.py`).
보정은 2026-11-07 최종 배치 hand-eye 후보 외부 변환 + 2026-11-09 잠정 벨트 평면(records/calib/2026-11-09-planes)이다.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for pkg in ('vision', 'calibration'):
    sys.path.insert(0, str(ROOT / 'src' / pkg))

CALIB = 'records/calib/2026-11-07-final-layout/result/camera_extrinsics_candidate.yaml'
MAT = 'records/calib/2026-11-09-planes/mat_plane.yaml'
BELT = 'records/calib/2026-11-09-planes/belt_plane.yaml'
# 공깃돌 높이 15 mm, 벨트면 → 테두리 22 mm(records/calib/2026-11-09-planes/vision_node.yaml), 뚜껑 바닥 1.5 mm(vision.yaml)
HEIGHTS = (0.015, 0.022, 0.0015)
FIELDS = ['frame', 'stamp_ns', 'u_px', 'v_px', 'diameter_px', 'x_m', 'y_m', 'z_m',
          'cov_xx_m2', 'cov_xy_m2', 'cov_yy_m2', 'position_valid', 'stone_inside', 'stone_inside_valid']


def frames_csv(video: Path) -> Path:
    return video.with_name(video.name[:-len('.mp4')] + '.frames.csv')


def extract(video: Path, out: Path) -> tuple[int, int]:
    import cv2
    from vision.geometry_files import load_geometry
    from vision.localizer import HeightParams
    from vision.pipeline import SceneConfig, process_frame

    os.environ.setdefault('DROP_REPO_ROOT', str(ROOT))
    geom = load_geometry(CALIB, MAT, BELT, HeightParams(*HEIGHTS))
    with open(frames_csv(video)) as f:
        stamps = [int(r['header_stamp_ns']) for r in csv.DictReader(f)]
    cap = cv2.VideoCapture(str(video))
    cfg = SceneConfig()
    rows, k = [], 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        for d in process_frame(img, cfg, geom):
            if d.object_class != 2:
                continue
            c = d.position_cov
            rows.append([k, stamps[k], f'{d.blob.u:.3f}', f'{d.blob.v:.3f}', f'{d.blob.diameter_px:.2f}',
                         *(f'{x:.6f}' for x in d.position), f'{c[0]:.3e}', f'{c[1]:.3e}', f'{c[4]:.3e}',
                         int(d.position_valid), int(d.stone_inside), int(d.stone_inside_valid)])
        k += 1
    if k != len(stamps):
        raise SystemExit(f'{video.name}: decoded {k} frames but frames.csv has {len(stamps)}')
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(FIELDS)
        w.writerows(rows)
    return k, len(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('videos', nargs='+', type=Path)
    ap.add_argument('--out-dir', type=Path, required=True)
    a = ap.parse_args(argv)
    for v in a.videos:
        name = v.name[:-len('.rgb.mp4')] if v.name.endswith('.rgb.mp4') else v.stem
        n_frames, n_det = extract(v, a.out_dir / f'{name}.caps.csv')
        print(f'{name}: frames {n_frames}, cap detections {n_det}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
