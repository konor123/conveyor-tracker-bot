"""One-session hardware observation; no robot access or sensor option writes."""
import csv
import datetime
import json
from pathlib import Path
import time

import numpy as np
import pyrealsense2 as rs

out = Path(__file__).parent
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipe.start(cfg)
dev = profile.get_device()
info = {'started': datetime.datetime.now().astimezone().isoformat(),
        'method': 'SDK wait_for_frames arrival; not ROS transport latency',
        'serial': dev.get_info(rs.camera_info.serial_number),
        'firmware': dev.get_info(rs.camera_info.firmware_version),
        'sensors': []}
try:
    for sensor in dev.query_sensors():
        opts = {}
        for name in ['enable_auto_exposure', 'exposure', 'gain',
                     'enable_auto_white_balance', 'white_balance', 'global_time_enabled']:
            option = getattr(rs.option, name)
            if sensor.supports(option):
                opts[name] = {'value': sensor.get_option(option),
                              'description': sensor.get_option_description(option)}
        info['sensors'].append({'name': sensor.get_info(rs.camera_info.name), 'options': opts})
    (out / 'device.json').write_text(json.dumps(info, indent=2))
    warm_end = time.monotonic() + 10
    while time.monotonic() < warm_end:
        pipe.wait_for_frames(5000)
    start = time.monotonic()
    rows = []
    with (out / 'frames.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['elapsed_s', 'stream', 'frame_number', 'timestamp_domain',
                         'frame_timestamp_ms', 'arrival_unix_s', 'arrival_monotonic_s',
                         'actual_exposure_raw', 'white_balance_raw'])
        while time.monotonic() - start < 180:
            frames = pipe.wait_for_frames(5000)
            arrival = time.time()
            mono = time.monotonic()
            for name, frame in [('color', frames.get_color_frame()), ('depth', frames.get_depth_frame())]:
                if not frame:
                    continue
                metadata = []
                for key in [rs.frame_metadata_value.actual_exposure, rs.frame_metadata_value.white_balance]:
                    metadata.append(frame.get_frame_metadata(key) if frame.supports_frame_metadata(key) else '')
                row = [mono-start, name, frame.get_frame_number(), str(frame.get_frame_timestamp_domain()),
                       frame.get_timestamp(), arrival, mono, *metadata]
                writer.writerow(row)
                rows.append(row)
            f.flush()
    summary = {}
    for name in ['color', 'depth']:
        data = [r for r in rows if r[1] == name]
        elapsed = np.array([r[0] for r in data])
        stamp = np.array([r[4] / 1000 for r in data])
        delay = np.array([r[5] for r in data]) - stamp
        # Fit relative values to avoid epoch-related numerical conditioning.
        slope = np.polyfit(elapsed, delay - delay[0], 1)[0]
        exposures = [r[7] for r in data if r[7] != '']
        summary[name] = {
            'frames': len(data), 'span_s': float(stamp[-1]-stamp[0]),
            'domains': sorted(set(r[3] for r in data)),
            'arrival_minus_stamp_ms_p0_p50_p95_p100': (np.percentile(delay, [0,50,95,100])*1000).tolist(),
            'arrival_offset_slope_ms_per_min': float(slope*60000),
            'last10s_minus_first10s_offset_median_ms': float((np.median(delay[elapsed>elapsed[-1]-10])-np.median(delay[elapsed<10]))*1000),
            'stamp_gap_ms_p50_p95_max': (np.percentile(np.diff(stamp), [50,95,100])*1000).tolist(),
            'nonincreasing_stamps': int(np.sum(np.diff(stamp)<=0)),
            'frame_counter_skips': int(sum(max(0,b[2]-a[2]-1) for a,b in zip(data,data[1:]))),
            'actual_exposure_raw_p0_p50_p95_p100': np.percentile(exposures,[0,50,95,100]).tolist() if exposures else None,
            'white_balance_raw_values': sorted(set(r[8] for r in data if r[8] != '')),
        }
    summary['host_wall_minus_monotonic_change_ms'] = float(((rows[-1][5]-rows[0][5])-(rows[-1][6]-rows[0][6]))*1000)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
finally:
    pipe.stop()
