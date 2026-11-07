#!/usr/bin/env python3
"""D455 컬러 1280x800 정지 영상 저장(OFF-3 보정 장면). ROS 드라이버 없이 pyrealsense2로 바로 찍는다.

realsense-ros 노드가 같은 카메라를 열고 있으면 실패한다(장치 하나는 한 프로세스만 연다).

사용 예:
  python3 tools/bench/capture_d455.py records/calib/2026-11-07 belt-fit-1
  python3 tools/bench/capture_d455.py records/calib/2026-11-07 track --burst 8 --interval-s 0.5
      벨트를 돌리며 track-00.png … track-07.png(벨트 진행 방향 측정용)

저장: <폴더>/<이름>.png, <폴더>/capture-log.csv(시각·노출·장치 정보 한 줄씩)
pyrealsense2가 없으면(시스템 pip 제한):
  python3 -m pip install --target ~/.local/share/pyrs pyrealsense2==2.58.4.10922
  PYTHONPATH=~/.local/share/pyrs python3 tools/bench/capture_d455.py ...
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import os
import sys
import time


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('out_dir')
    ap.add_argument('name')
    ap.add_argument('--burst', type=int, default=1, help='연속 장수(1이면 <이름>.png 한 장)')
    ap.add_argument('--interval-s', type=float, default=0.5)
    ap.add_argument('--warmup', type=int, default=45, help='자동 노출 안정화용으로 버리는 프레임 수')
    ap.add_argument('--exposure', type=float,
                    help='수동 노출(RealSense exposure 옵션 원시값, 단위는 펌웨어 문서 확인). 주지 않으면 자동 노출')
    a = ap.parse_args(argv)
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as e:
        print(f'{e}. 파일 머리말의 설치 방법을 보라', file=sys.stderr)
        return 1

    os.makedirs(a.out_dir, exist_ok=True)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    dev = prof.get_device()
    sensor = dev.first_color_sensor()
    if a.exposure is not None:
        sensor.set_option(rs.option.enable_auto_exposure, 0)
        sensor.set_option(rs.option.exposure, a.exposure)
    log_path = os.path.join(a.out_dir, 'capture-log.csv')
    new_log = not os.path.exists(log_path)
    try:
        for _ in range(a.warmup):
            pipe.wait_for_frames()
        with open(log_path, 'a', newline='') as fh:
            w = csv.writer(fh)
            if new_log:
                w.writerow(['file', 'host_time', 'frame_timestamp_ms', 'auto_exposure', 'exposure_option', 'gain',
                            'serial', 'firmware', 'fx', 'fy', 'cx', 'cy'])
            for k in range(a.burst):
                if k:
                    time.sleep(a.interval_s)
                col = pipe.wait_for_frames().get_color_frame()
                # 펌웨어 보고 내부 파라미터는 스트림 시작마다 바뀔 수 있어 장마다 남긴다(SPEC-600 4.8절)
                intr = col.profile.as_video_stream_profile().get_intrinsics()
                img = np.asanyarray(col.get_data()).copy()
                name = a.name if a.burst == 1 else f'{a.name}-{k:02d}'
                path = os.path.join(a.out_dir, f'{name}.png')
                cv2.imwrite(path, img)
                w.writerow([f'{name}.png', _dt.datetime.now().isoformat(timespec='milliseconds'),
                            f'{col.get_timestamp():.3f}', sensor.get_option(rs.option.enable_auto_exposure),
                            sensor.get_option(rs.option.exposure), sensor.get_option(rs.option.gain),
                            dev.get_info(rs.camera_info.serial_number),
                            dev.get_info(rs.camera_info.firmware_version),
                            f'{intr.fx:.4f}', f'{intr.fy:.4f}', f'{intr.ppx:.4f}', f'{intr.ppy:.4f}'])
                print(path)
    finally:
        pipe.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
