"""Export review videos from existing bags; raw bags and exact stamps stay intact.

Run with the ROS Jazzy Python environment and --ffmpeg /path/to/ffmpeg.
MP4 uses a fixed 30 fps presentation timeline; *.frames.csv preserves source time.
Depth previews map uint16 values 0..2000 to grayscale; they are not depth data.
"""
import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def export(bag, root, ffmpeg):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag['path'], storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    available = {t.name for t in reader.get_all_topics_and_types()}
    topics = {'/camera/camera/color/image_raw': 'rgb',
              '/camera/camera/depth/image_rect_raw': 'depth-preview'}
    topics = {k: v for k, v in topics.items() if k in available}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    writers = {}
    try:
        while reader.has_next():
            topic, data, received = reader.read_next()
            msg = deserialize_message(data, Image)
            role = topics[topic]
            if role not in writers:
                depth = role == 'depth-preview'
                if depth:
                    assert msg.encoding in ('16UC1', 'mono16') and not msg.is_bigendian
                    pixel_format, row_bytes = 'gray16le', msg.width * 2
                else:
                    assert msg.encoding in ('rgb8', 'bgr8')
                    pixel_format, row_bytes = ('rgb24' if msg.encoding == 'rgb8' else 'bgr24'), msg.width * 3
                stem = f"{bag['bag_name']}.{role}"
                path = root / f'{stem}.mp4'
                cmd = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin',
                       '-f', 'rawvideo', '-pixel_format', pixel_format,
                       '-video_size', f'{msg.width}x{msg.height}', '-framerate', '30',
                       '-i', 'pipe:0', '-an']
                if depth:
                    cmd += ['-vf', 'format=gray16le,lut=y=val*32.7675,format=yuv420p']
                cmd += ['-c:v', 'libx264', '-preset', 'medium', '-crf', '26',
                        '-pix_fmt', 'yuv420p', '-threads', '2', '-movflags', '+faststart',
                        '-y', str(path)]
                err = (root / f'{stem}.ffmpeg.log').open('w')
                proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=err)
                timing = (root / f'{stem}.frames.csv').open('w', newline='')
                csv_out = csv.writer(timing)
                csv_out.writerow(['frame_index', 'video_time_s', 'header_stamp_ns',
                                  'bag_received_ns', 'received_minus_stamp_s'])
                writers[role] = dict(proc=proc, timing=timing, csv=csv_out, err=err,
                                     frames=0, path=path, row_bytes=row_bytes,
                                     width=msg.width, height=msg.height, command=cmd)
            w = writers[role]
            assert (msg.width, msg.height) == (w['width'], w['height'])
            frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :w['row_bytes']]
            w['proc'].stdin.write(frame.tobytes())
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            index = w['frames']
            w['csv'].writerow([index, f'{index / 30:.9f}', stamp, received,
                              f'{(received - stamp) / 1e9:.9f}'])
            w['frames'] += 1
    finally:
        for w in writers.values():
            w['proc'].stdin.close()
            w['timing'].close()
        for w in writers.values():
            code = w['proc'].wait(timeout=90)
            w['err'].close()
            if code:
                raise RuntimeError(f"ffmpeg failed: {w['path']}, exit {code}")
    result = []
    for role, w in writers.items():
        cap = cv2.VideoCapture(str(w['path']))
        assert cap.isOpened()
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        assert count == w['frames'], (count, w['frames'])
        assert (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) == (w['width'], w['height'])
        assert cap.read()[0]
        cap.set(cv2.CAP_PROP_POS_FRAMES, count - 1)
        assert cap.read()[0]
        cap.release()
        subprocess.run([ffmpeg, '-v', 'error', '-i', str(w['path']), '-f', 'null', '-'],
                       check=True, stdout=subprocess.DEVNULL)
        assert w['path'].stat().st_size < 100 * 1024 * 1024
        result.append(dict(bag_name=bag['bag_name'], source_bag_path=bag['path'],
                           source_bag_manifest_sha256=bag['sha256'], role=role,
                           video=w['path'].name, frames_csv=w['path'].with_suffix('.frames.csv').name,
                           frames=count, width=w['width'], height=w['height'], presentation_fps=30,
                           bytes=w['path'].stat().st_size, sha256=sha256(w['path']),
                           ffmpeg_command=w['command'], full_decode_verified=True))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ffmpeg', required=True)
    parser.add_argument('--index', default='records/bags.csv')
    parser.add_argument('--out', default='records/media/2026-11-07')
    args = parser.parse_args()
    root = Path(args.out)
    root.mkdir(parents=True, exist_ok=True)
    entries = []
    version = subprocess.check_output([args.ffmpeg, '-version'], text=True).splitlines()[0]
    for bag in csv.DictReader(Path(args.index).open()):
        print('Exporting', bag['bag_name'], flush=True)
        entries.extend(export(bag, root, args.ffmpeg))
        (root / 'manifest.json').write_text(json.dumps(dict(ffmpeg_version=version, videos=entries),
                                                     ensure_ascii=False, indent=2) + '\n')
        print('Verified videos', len(entries), 'bytes', sum(x['bytes'] for x in entries), flush=True)
