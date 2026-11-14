#!/usr/bin/env python3
"""시도 한 번의 상단 카메라 영상(mp4)·프레임 시각(csv)·검출(jsonl)을 남긴다. 로봇 지령 없음.

  python3 tools/robot/run_recorder.py --out logs/run-01

SIGINT·SIGTERM 또는 --max-s가 지나면 파일을 닫고 끝난다. 영상은 작업 구역만 잘라(--crop) 30 fps mp4v로 쓰고,
왼쪽 위에 프레임 stamp(초의 아래 세 자리.밀리초)를 찍는다. 카메라와 같은 큰 SHM FastDDS 프로파일이 필요하다.
"""
from __future__ import annotations

import argparse
import csv
import json
import signal
import sys
import time

import numpy as np


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', required=True, help='출력 접두(.mp4, .frames.csv, .dets.jsonl이 붙는다)')
    ap.add_argument('--max-s', type=float, default=240.0)
    ap.add_argument('--crop', type=int, nargs=4, default=[240, 0, 1060, 520], metavar=('X0', 'Y0', 'X1', 'Y1'))
    ap.add_argument('--image-topic', default='/camera/camera/color/image_raw')
    args = ap.parse_args(argv)

    import cv2
    import rclpy
    from drop_interfaces.msg import ObjectDetectionArray
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    x0, y0, x1, y1 = args.crop
    rclpy.init()
    node = Node('run_recorder')
    writer = cv2.VideoWriter(args.out + '.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30.0, (x1 - x0, y1 - y0))
    frames = open(args.out + '.frames.csv', 'w', newline='')
    fw = csv.writer(frames)
    fw.writerow(['index', 'stamp_s', 'recv_s'])
    dets = open(args.out + '.dets.jsonl', 'w')
    count = [0]

    def on_image(m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            img = img[:, :, ::-1]
        crop = np.ascontiguousarray(img[y0:y1, x0:x1])
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cv2.putText(crop, f'{stamp % 1000:7.3f}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        writer.write(crop)
        fw.writerow([count[0], f'{stamp:.6f}', f'{time.time():.6f}'])
        count[0] += 1

    def on_dets(m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        items = [{'c': d.object_class, 'r': d.region, 'x': round(d.position.x, 5), 'y': round(d.position.y, 5),
                  'u': round(d.u, 1), 'v': round(d.v, 1), 'd': round(d.diameter_px, 1), 'pv': d.position_valid,
                  'si': d.stone_inside, 'sv': d.stone_inside_valid, 'occ': d.occluded_partial}
                 for d in m.detections]
        dets.write(json.dumps({'t': round(stamp, 6), 'recv': round(time.time(), 6), 'items': items}) + '\n')

    node.create_subscription(Image, args.image_topic, on_image, 5)
    node.create_subscription(ObjectDetectionArray, 'objects/detections', on_dets, 20)
    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    t0 = time.monotonic()
    try:
        while not stop[0] and time.monotonic() - t0 < args.max_s:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        writer.release()
        frames.close()
        dets.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'event': 'recorded', 'frames': count[0], 'out': args.out}), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
