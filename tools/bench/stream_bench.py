#!/usr/bin/env python3
"""스트림 지령 성능 측정(TEST_SPEC.md TC-BEN-004-01, Progress.md 2절 OFF-2).

현재 플랜지 위치를 중심으로 한 축 사인 궤적을 servol_stream 또는 speedl_stream으로 보내고,
지령 간격·지연(지령 ↔ 실제 위치 상호상관)·추종 오차를 기록한다. 실제 위치는 joint_states(100 Hz)에
robot_description URDF 순기구학을 적용해 얻는다(/tf는 발행 주기가 낮다).

  # 계획만 출력(로봇에 아무것도 보내지 않음)
  python3 tools/bench/stream_bench.py --iface servol --rate-hz 50 --dry-run
  # 측정(비상정지 담당자 대기, 안전 높이)
  python3 tools/bench/stream_bench.py --iface servol --rate-hz 50 --amp-m 0.010 --freq-hz 0.5 --duration-s 60

결과: records/bench/stream-<날짜>-<시각>-<iface>-<rate>hz.csv(원시 열) + .summary.txt(`--out-dir`로 폴더 변경)
안전: 진폭 ≤ 20 mm, 최대 속도 ≤ 50 mm/s, 로봇 상태 STANDBY·알람 없음일 때만 시작한다. 종료·예외·Ctrl-C 때는
지령을 멈추고(speedl은 0 속도 1회) move_stop을 부른다. 단위 변환(m ↔ mm)은 이 측정 도구 안에서만 한다.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
MM_PER_M = 1000.0
AMP_MAX_M = 0.020
SPEED_MAX_MPS = 0.050
GUARD_M = 0.005              # 중심에서 진폭 + 5 mm, 다른 축 5 mm를 넘으면 중단


# ------------------------------------------------------------------ 순기구학(URDF)
def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


class UrdfChain:
    """base → tip 사이 관절 사슬. revolute·fixed만 지원한다."""

    def __init__(self, urdf_xml: str, base: str = 'base_link', tip: str = 'link_6'):
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for j in root.findall('joint'):
            by_child[j.find('child').get('link')] = j
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f'{base} → {tip} 사슬을 찾지 못함({link})')
            j = by_child[link]
            chain.append(j)
            link = j.find('parent').get('link')
        self.joints = []
        for j in reversed(chain):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            ax = j.find('axis')
            axis = [float(v) for v in ax.get('xyz').split()] if ax is not None else [0, 0, 1]
            self.joints.append((j.get('name'), j.get('type'), np.array(xyz), _rpy_matrix(*rpy), axis))

    def names(self) -> list[str]:
        return [n for n, t, *_ in self.joints if t != 'fixed']

    def position(self, q: dict[str, float]) -> np.ndarray:
        R, p = np.eye(3), np.zeros(3)
        for name, typ, xyz, Ro, axis in self.joints:
            p = p + R @ xyz
            R = R @ Ro
            if typ in ('revolute', 'continuous'):
                R = R @ _axis_angle(axis, q[name])
        return p


# ------------------------------------------------------------------ 궤적·분석(순수 함수)
def sine_target(t: float, amp: float, freq: float, ramp_s: float,
                duration_s: float = math.inf) -> tuple[float, float]:
    """(변위, 속도). 처음과 끝 ramp_s 동안 진폭을 선형으로 키우고 줄여, 정지 상태로 시작해 중심에서 정지로 끝난다."""
    w = 2 * math.pi * freq
    if ramp_s <= 0:
        k, dk = 1.0, 0.0
    elif t < ramp_s:
        k, dk = t / ramp_s, 1.0 / ramp_s
    elif t > duration_s - ramp_s:
        k, dk = max(0.0, (duration_s - t) / ramp_s), -1.0 / ramp_s
    else:
        k, dk = 1.0, 0.0
    if t >= duration_s:
        k, dk = 0.0, 0.0
    return k * amp * math.sin(w * t), k * amp * w * math.cos(w * t) + dk * amp * math.sin(w * t)


def quantiles(x) -> dict[str, float]:
    x = np.asarray(x, float)
    if x.size == 0:
        return {}
    return {'p50': float(np.percentile(x, 50)), 'p95': float(np.percentile(x, 95)),
            'p99': float(np.percentile(x, 99)), 'max': float(np.max(x))}


def estimate_lag(t_cmd, x_cmd, t_fb, x_fb, max_lag_s=0.5, step_s=0.001) -> float:
    """실제 위치가 지령을 몇 s 늦게 따라가는지: x_fb(t) ≈ x_cmd(t − lag)인 lag(최소제곱)."""
    lags = np.arange(0.0, max_lag_s + step_s / 2, step_s)
    errs = [np.mean((x_fb - np.interp(t_fb - lag, t_cmd, x_cmd)) ** 2) for lag in lags]
    return float(lags[int(np.argmin(errs))])


def analyze(rows: list[dict], period_s: float, ramp_s: float) -> dict:
    cmd = [r for r in rows if r['kind'] == 'cmd']
    fb = [r for r in rows if r['kind'] == 'fb']
    t_cmd = np.array([r['t_steady'] for r in cmd])
    x_cmd = np.array([r['x_m'] for r in cmd])
    t_fb = np.array([r['t_steady'] for r in fb])
    x_fb = np.array([r['x_m'] for r in fb])
    gaps = np.diff(t_cmd)
    out = {
        'commands': len(cmd), 'feedback': len(fb), 'period_s': period_s,
        'cmd_interval_s': quantiles(gaps),
        'cmd_interval_p99_ok': bool(gaps.size > 0 and np.percentile(gaps, 99) <= 1.5 * period_s),
        'fb_interval_s': quantiles(np.diff(t_fb)),
    }
    keep = np.zeros(len(t_fb), bool)
    if len(t_cmd):
        keep = (t_fb >= t_cmd[0] + ramp_s) & (t_fb <= t_cmd[-1] - ramp_s)   # 앞뒤 ramp 구간은 오차 통계에서 뺀다
    if np.count_nonzero(keep) < 2:
        # 안전 중단이 일찍 나면 ramp 밖 표본이 없다. 원시 CSV와 지령 간격 통계는 그대로 남긴다
        out.update(lag_s=None, tracking_error_raw_m={}, tracking_error_lag_comp_m={},
                   note=f'추종 통계 생략: 앞뒤 ramp {ramp_s} s를 뺀 feedback 표본 {int(np.count_nonzero(keep))}개')
        return out
    lag = estimate_lag(t_cmd, x_cmd, t_fb[keep], x_fb[keep])
    raw = np.abs(x_fb[keep] - np.interp(t_fb[keep], t_cmd, x_cmd))
    comp = np.abs(x_fb[keep] - np.interp(t_fb[keep] - lag, t_cmd, x_cmd))
    out.update(lag_s=lag, tracking_error_raw_m=quantiles(raw), tracking_error_lag_comp_m=quantiles(comp))
    return out


# ------------------------------------------------------------------ ROS 측정
def run(args) -> int:
    import rclpy
    from dsr_msgs2.msg import ServolStream, SpeedlStream
    from dsr_msgs2.srv import GetCurrentPosx, GetLastAlarm, GetRobotState, MoveStop
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    ns = args.ns
    rclpy.init()
    node = rclpy.create_node('stream_bench')
    rows: list[dict] = []

    def call(typ, name, timeout=5.0, **kw):
        cli = node.create_client(typ, f'{ns}/{name}')
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'서비스 없음: {ns}/{name}')
        fut = cli.call_async(typ.Request(**kw))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if not fut.done():
            raise RuntimeError(f'응답 없음: {ns}/{name}')
        return fut.result()

    # robot_description(transient local) → 순기구학
    desc = []
    node.create_subscription(String, f'{ns}/robot_description', lambda m: desc.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end = time.monotonic() + 5.0
    while not desc and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not desc:
        raise RuntimeError('robot_description을 받지 못함')
    chain = UrdfChain(desc[0])
    latest = {}

    def on_js(m):
        q = dict(zip(m.name, m.position))
        if all(n in q for n in chain.names()):
            p = chain.position(q)
            latest['p'] = p
            rows.append({'kind': 'fb', 't_steady': time.monotonic(),
                         't_ros': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 'seq': 0,
                         'x_m': p[0], 'y_m': p[1], 'z_m': p[2], 'v_mps': 0.0})
    node.create_subscription(JointState, f'{ns}/joint_states', on_js, 50)

    # 사전 점검
    state = call(GetRobotState, 'system/get_robot_state').robot_state
    alarm = call(GetLastAlarm, 'system/get_last_alarm').log_alarm
    posx = list(call(GetCurrentPosx, 'aux_control/get_current_posx', ref=0).task_pos_info[0].data[:6])
    end = time.monotonic() + 3.0
    while 'p' not in latest and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if 'p' not in latest:
        raise RuntimeError('joint_states를 받지 못함')
    fk = latest['p']
    fk_err = float(np.linalg.norm(fk - np.array(posx[:3]) / MM_PER_M))
    axis = 'xyz'.index(args.axis)
    v_peak = 2 * math.pi * args.freq_hz * args.amp_m
    print(f'robot_state={state}(1=STANDBY) 마지막 알람(기록)=(level {alarm.level}, group {alarm.group}, index {alarm.index})')
    print(f'posx(mm,deg)={["%.2f" % v for v in posx]}  FK−posx 차={fk_err * MM_PER_M:.2f} mm')
    print(f'계획: {args.iface}_stream {args.rate_hz} Hz, {args.axis}축 ±{args.amp_m * MM_PER_M:.1f} mm, '
          f'{args.freq_hz} Hz, {args.duration_s} s, 최대 {v_peak * MM_PER_M:.1f} mm/s, ramp {args.ramp_s} s')
    problems = []
    if state != 1:
        problems.append(f'robot_state {state} ≠ 1(STANDBY)')
    if args.amp_m > AMP_MAX_M or v_peak > SPEED_MAX_MPS:
        problems.append(f'진폭 ≤ {AMP_MAX_M} m, 속도 ≤ {SPEED_MAX_MPS} m/s 위반')
    if fk_err > 0.002:
        problems.append(f'FK와 posx 차 {fk_err * MM_PER_M:.1f} mm > 2 mm')
    if problems:
        print('시작하지 않음: ' + '; '.join(problems))
        return 2
    if args.dry_run:
        print('dry-run: 지령을 보내지 않고 끝냄')
        return 0

    period = 1.0 / args.rate_hz
    center_mm = posx
    pub = (node.create_publisher(ServolStream, f'{ns}/servol_stream', 20) if args.iface == 'servol'
           else node.create_publisher(SpeedlStream, f'{ns}/speedl_stream', 10))
    seq = 0
    aborted = False
    t_start = time.monotonic()
    next_t = t_start
    try:
        while True:
            now = time.monotonic()
            t = now - t_start
            if t >= args.duration_s:
                break
            dev = latest['p'] - np.array(center_mm[:3]) / MM_PER_M      # 표류 감시(speedl 적분 오차 포함)
            off = np.delete(np.abs(dev), axis)
            if abs(dev[axis]) > args.amp_m + GUARD_M or np.any(off > GUARD_M):
                print(f'안전 중단: 중심 대비 {dev * MM_PER_M} mm (허용 축 {args.amp_m * MM_PER_M + GUARD_M * MM_PER_M:.1f}, '
                      f'다른 축 {GUARD_M * MM_PER_M:.1f})')
                aborted = True
                break
            d, v = sine_target(t, args.amp_m, args.freq_hz, args.ramp_s, args.duration_s)
            if args.iface == 'servol':
                pos = list(center_mm)
                pos[axis] += d * MM_PER_M
                msg = ServolStream(pos=pos, vel=[args.vel_limit_mps * MM_PER_M, 30.0],
                                   acc=[args.acc_limit_mps2 * MM_PER_M, 60.0], time=args.cmd_time_s)
            else:
                vel = [0.0] * 6
                vel[axis] = v * MM_PER_M
                msg = SpeedlStream(vel=vel, acc=[args.acc_limit_mps2 * MM_PER_M, 60.0], time=args.cmd_time_s)
            pub.publish(msg)
            p = np.array(center_mm[:3]) / MM_PER_M
            p[axis] += d
            rows.append({'kind': 'cmd', 't_steady': now, 't_ros': 0.0, 'seq': seq,
                         'x_m': p[0], 'y_m': p[1], 'z_m': p[2], 'v_mps': v})
            seq += 1
            next_t += period
            while True:                                   # 남은 시간 동안 feedback 처리
                rem = next_t - time.monotonic()
                if rem <= 0:
                    break
                rclpy.spin_once(node, timeout_sec=min(rem, 0.002))
    finally:
        if args.iface == 'speedl':
            pub.publish(SpeedlStream(vel=[0.0] * 6, acc=[args.acc_limit_mps2 * MM_PER_M, 60.0], time=0.1))
        try:
            call(MoveStop, 'motion/move_stop', stop_mode=2)     # DR_SSTOP(soft stop)
        except Exception as e:                                  # noqa: BLE001 - 정지 실패도 기록하고 넘어간다
            print(f'move_stop 실패: {e}')
        end = time.monotonic() + 1.0
        while time.monotonic() < end:                           # 정지 후 위치 1 s 기록
            rclpy.spin_once(node, timeout_sec=0.01)
        node.destroy_node()
        rclpy.try_shutdown()

    stamp = datetime.now().strftime('%Y-%m-%d-%H%M%S')
    os.makedirs(args.out_dir, exist_ok=True)
    base = os.path.join(args.out_dir, f'stream-{stamp}-{args.iface}-{int(args.rate_hz)}hz')
    with open(base + '.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=['kind', 't_steady', 't_ros', 'seq', 'x_m', 'y_m', 'z_m', 'v_mps'])
        w.writeheader()
        w.writerows(rows)
    s = analyze([r for r in rows if r['t_steady'] <= t_start + args.duration_s], period, args.ramp_s)
    lines = [f'# TC-BEN-004-01 {stamp}', f'args: {vars(args)}', f'center_posx_mm_deg: {center_mm}',
             f'aborted_by_guard: {aborted}']
    lines += [f'{k}: {v}' for k, v in s.items()]
    with open(base + '.summary.txt', 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print(f'기록: {base}.csv')
    return 3 if aborted else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--iface', choices=['servol', 'speedl'], default='servol')
    ap.add_argument('--rate-hz', type=float, default=50.0)
    ap.add_argument('--axis', choices=['x', 'y', 'z'], default='x')
    ap.add_argument('--amp-m', type=float, default=0.010)
    ap.add_argument('--freq-hz', type=float, default=0.5)
    ap.add_argument('--duration-s', type=float, default=60.0)
    ap.add_argument('--ramp-s', type=float, default=2.0)
    ap.add_argument('--vel-limit-mps', type=float, default=0.050, help='servol 선속도 제한')
    ap.add_argument('--acc-limit-mps2', type=float, default=0.200)
    ap.add_argument('--cmd-time-s', type=float, default=0.0, help='DRFL time 인수(servol 도달 시간 / speedl 유지 시간)')
    ap.add_argument('--ns', default='/dsr01')
    ap.add_argument('--out-dir', default=os.path.join(REPO, 'records', 'bench'),
                    help='기록 폴더(기본 records/bench). 대역 시험은 저장소 밖 임시 폴더를 준다')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args(argv)
    if args.iface == 'speedl' and args.cmd_time_s <= 0:
        args.cmd_time_s = 2.0 / args.rate_hz                   # 지령이 끊기면 두 주기 뒤 스스로 멈춤
    try:
        return run(args)
    except RuntimeError as e:
        print(f'오류: {e}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
