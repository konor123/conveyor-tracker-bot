#!/usr/bin/env python3
"""moving_drop.py 실행 하나의 기록(<접두>.jsonl + .dets.jsonl + .mp4)을 판정한다. 로봇 지령 없음.

  python3 tools/robot/moving_drop.py ... --execute --record logs/run-01 > logs/run-01.jsonl
  python3 tools/robot/drop_analyze.py logs/run-01

판정(검출 기록 기준, 트리거 뒤 뚜껑이 그리퍼 아래를 벗어나 보이는 구간):
  SUCCESS  뚜껑 검출의 stone_inside가 참인 프레임이 5개 이상이고 판정 가능한 프레임의 60% 이상
  MISS     stone_inside 거짓 프레임이 5개 이상이고 참이 2개 미만
  UNKNOWN  그 밖(뚜껑이 보이지 않음 등)
같이 낸다: 뚜껑 안 공깃돌의 중심 차(진행 방향, 양수 = 돌이 뚜껑보다 하류 = 일찍 떨어짐), 벨트 위에 떨어진 돌과
뚜껑의 거리, 그 거리를 벨트 속도로 나눈 시간(다음 시도의 lead 보정값), 트리거 전후 장면을 이어 붙인 사진.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys

import numpy as np

CAP, STONE, BELT = 2, 1, 2
INSIDE_R_M = 0.017          # 뚜껑 중심에서 이 안의 돌은 뚜껑 안(바깥 지름 32 mm)
NEAR_CAP_M = 0.06           # 벨트 위 돌이 뚜껑에서 이보다 가까우면 이번 시도에서 떨어진 돌로 본다


def load_events(path):
    out = {}
    with open(path) as fh:
        for line in fh:
            if line.startswith('{'):
                e = json.loads(line)
                out.setdefault(e['event'], []).append(e)
    return out


def analyze(prefix: str) -> dict:
    ev = load_events(prefix + '.jsonl')
    if 'fired' not in ev or 'plan' not in ev:
        reason = ev['abort'][-1].get('reason') if 'abort' in ev else 'NO_TRIGGER'
        return {'result': 'NOT_ATTEMPTED', 'reason': reason, 'holding': 'park_holding' in
                [d.get('step') for d in ev.get('done', [])]}
    plan, fired = ev['plan'][-1], ev['fired'][-1]
    t_fire = fired.get('t_ros') or fired['t']
    rel = plan['release_m']
    aim_x = plan.get('aim_x_m', rel[0])     # 도착 시각을 맞춘 x(해제 지점을 x로 옮겼으면 다르다)
    v_plan = plan['cap_v_mps'][0]
    t_release = fired.get('t_release_est') or plan.get('t_release') or (plan['t'] + plan['release_in_s'])
    frames = [json.loads(line) for line in open(prefix + '.dets.jsonl')]
    caps_t, si_true, si_false, inside, loose = [], 0, 0, [], []
    for f in frames:
        if not t_fire - 3.0 <= f['t'] <= t_fire + 14.0:
            continue
        caps = [d for d in f['items'] if d['c'] == CAP and d['r'] == BELT and d['pv']]
        stones = [d for d in f['items'] if d['c'] == STONE and d['r'] == BELT and d['pv']]
        for c in caps:
            caps_t.append((f['t'], c['x'], c['y']))
        if f['t'] < t_fire:
            continue
        for c in caps:
            if c['x'] < rel[0] + 0.02:           # 그리퍼 아래·상류의 다른 검출은 뺀다
                continue
            if c['sv']:
                si_true += bool(c['si'])
                si_false += not c['si']
            for s in stones:
                dx, dy = s['x'] - c['x'], s['y'] - c['y']
                dist = float(np.hypot(dx, dy))
                if dist <= INSIDE_R_M:
                    inside.append((f['t'], dx, dy, s['v'] - c['v'], s['u'] - c['u']))
                elif dist <= NEAR_CAP_M:
                    loose.append((f['t'], dx, dy))
    # 실제 뚜껑 속도와 해제 시각의 실제 위치(트리거 전후 검출의 직선 맞춤)
    out = {'t_fire': round(t_fire, 3), 'drop_delay_s': plan.get('drop_delay_s', plan.get('lead_s')), 'release_m': rel,
           'aim_x_m': aim_x, 'x_offset_m': plan.get('x_offset_m', 0.0),
           'v_plan_mmps': round(v_plan * 1000, 1), 'rehearse': bool(fired.get('rehearse')),
           'fired_late_s': fired.get('late_s'), 'si_true': si_true, 'si_false': si_false,
           'inside_frames': len(inside), 'loose_frames': len(loose)}
    pts = np.array([c for c in caps_t if abs(c[0] - t_release) < 4.0])
    if len(pts) >= 8:
        # 한 뚜껑만 있다고 보고 x(t) 직선 맞춤. 가려진 구간은 비어 있다
        a, b = np.polyfit(pts[:, 0] - t_release, pts[:, 1], 1)
        out['v_actual_mmps'] = round(float(a) * 1000, 1)
        out['cap_x_at_release_minus_rel_mm'] = round((float(b) - aim_x) * 1000, 1)    # 양수 = 뚜껑이 이미 지나감
        out['cap_y_minus_rel_mm'] = round((float(np.median(pts[:, 2])) - rel[1]) * 1000, 1)
        if abs(a) > 1e-4:
            out['cap_arrives_minus_est_s'] = round(-(float(b) - aim_x) / float(a), 3)       # 음수 = 예상보다 일찍 옴
            out['cap_arrives_after_fire_s'] = round(t_release - (float(b) - aim_x) / float(a) - t_fire, 3)
    if inside:
        arr = np.array(inside)
        out['stone_in_cap_dx_mm'] = round(float(np.median(arr[:, 1])) * 1000, 1)     # 양수 = 돌이 하류 쪽(일찍 떨어짐)
        out['stone_in_cap_dy_mm'] = round(float(np.median(arr[:, 2])) * 1000, 1)
        out['stone_in_cap_dv_px'] = round(float(np.median(arr[:, 3])), 1)
    if loose:
        arr = np.array(loose)
        out['loose_dx_mm'] = round(float(np.median(arr[:, 1])) * 1000, 1)            # 음수 = 돌이 뚜껑 뒤(늦게 떨어짐)
        out['loose_dy_mm'] = round(float(np.median(arr[:, 2])) * 1000, 1)
    valid = si_true + si_false
    if si_true >= 5 and si_true >= 0.6 * valid:
        out['result'] = 'SUCCESS'
    elif si_false >= 5 and si_true < 2:
        out['result'] = 'MISS'
    else:
        out['result'] = 'UNKNOWN'
    v = out.get('v_actual_mmps', v_plan * 1000) / 1000.0
    dx = out.get('loose_dx_mm') if out['result'] != 'SUCCESS' else out.get('stone_in_cap_dx_mm')
    if dx is not None and abs(v) > 1e-3:
        out['drop_delay_correction_s'] = round(-dx / 1000.0 / v, 3)  # drop_delay에 더할 값(돌이 뒤처졌으면 양수)
    return out


def contact_sheet(prefix: str, t_fire: float, offsets, tile=(330, 20, 560, 430), scale=1.4) -> str | None:
    """트리거 기준 offsets[s] 시각의 프레임을 벨트 구역만 잘라 한 장으로 붙인다(tile = 녹화 영상 좌표 x0 y0 x1 y1)."""
    import cv2
    rows = list(csv.DictReader(open(prefix + '.frames.csv')))
    if not rows:
        return None
    stamps = np.array([float(r['stamp_s']) for r in rows])
    wanted = {int(np.argmin(np.abs(stamps - (t_fire + o)))): o for o in offsets}
    cap = cv2.VideoCapture(prefix + '.mp4')
    tiles, idx = {}, 0
    x0, y0, x1, y1 = tile
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx in wanted:
            t = frame[y0:y1, x0:x1].copy()
            t = cv2.resize(t, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            label = f'{stamps[idx] - t_fire:+.2f}s'
            cv2.putText(t, label, (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(t, label, (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            tiles[idx] = t
        idx += 1
    cap.release()
    if not tiles:
        return None
    ordered = [tiles[k] for k in sorted(tiles)]
    half = (len(ordered) + 1) // 2
    blank = np.zeros_like(ordered[0])
    top = np.hstack(ordered[:half])
    bottom = np.hstack(ordered[half:] + [blank] * (half - len(ordered[half:])))
    path = prefix + '.sheet.jpg'
    cv2.imwrite(path, np.vstack([top, bottom]), [cv2.IMWRITE_JPEG_QUALITY, 85])
    return path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('prefix')
    ap.add_argument('--no-sheet', action='store_true')
    args = ap.parse_args(argv)
    out = analyze(args.prefix)
    if not args.no_sheet and 't_fire' in out:
        out['sheet'] = contact_sheet(args.prefix, out['t_fire'],
                                     [-1.0, -0.5, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.5, 3.0, 4.0, 5.5])
    print(json.dumps(out, ensure_ascii=False))
    with open(args.prefix + '.analysis.json', 'w') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
