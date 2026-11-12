#!/usr/bin/env python3
"""시스템 시계 계단 보정 감시(시연·측정 전 장비 점검).

  python3 tools/bench/check_clock_steps.py --duration 300

ROS 노드는 use_sim_time이 아니면 시스템 시각을 쓴다. 시스템 시각이 뒤로 가면 tracker가 모든 track을 지우고
epoch를 올린다(SPEC-300 REQ-TRK-008). NTP·가상머신 시간 동기가 시계를 계단으로 고치는 장비에서는 추적이
그때마다 끊긴다. 벽시계와 단조 시계의 차이를 짧은 주기로 비교해 계단(기본 0.5 ms 초과 변화)을 센다.

종료 코드: 계단이 없으면 0, 있으면 1. 표준 라이브러리만 쓴다.
"""
from __future__ import annotations

import argparse
import time


def find_steps(offsets, threshold_s: float = 0.0005):
    """(벽시계 − 단조 시계) 표본 열에서 이웃 표본 차이가 threshold를 넘는 곳을 (표본 번호, 변화 [s])로 돌려준다."""
    return [(i, b - a) for i, (a, b) in enumerate(zip(offsets, offsets[1:]), start=1) if abs(b - a) > threshold_s]


def sample(duration_s: float, period_s: float = 0.005, wall=time.time, mono=time.monotonic, sleep=time.sleep):
    """duration 동안 period마다 (단조 시각, 벽시계 − 단조 시계)를 모은다."""
    rows, end = [], mono() + duration_s
    while True:
        m = mono()
        rows.append((m, wall() - m))
        if m >= end:
            return rows
        sleep(period_s)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=300.0, help='감시 시간 [s]')
    ap.add_argument('--period', type=float, default=0.005, help='표본 주기 [s]')
    ap.add_argument('--threshold-ms', type=float, default=0.5, help='계단으로 볼 변화 [ms]')
    a = ap.parse_args(argv)
    rows = sample(a.duration, a.period)
    steps = find_steps([off for _, off in rows], a.threshold_ms * 1e-3)
    t0 = rows[0][0]
    for i, delta in steps:
        print(f'step at +{rows[i][0] - t0:.1f} s: {delta * 1e3:+.2f} ms')
    back = [d for _, d in steps if d < 0]
    drift_ms = (rows[-1][1] - rows[0][1]) * 1e3
    print(f'{a.duration:.0f} s: steps {len(steps)} (backward {len(back)}), net change {drift_ms:+.2f} ms')
    return 1 if steps else 0


if __name__ == '__main__':
    raise SystemExit(main())
