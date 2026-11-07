#!/usr/bin/env python3
"""stale 지령 정지 측정(TEST_SPEC.md TC-BEN-005-01 중 stale, Progress.md 2절 OFF-2).

speedl_stream으로 한 축 등속 지령을 `--move-s` 동안 보내다가 지령을 끊고(stale), 마지막 지령 뒤
로봇이 멈출 때까지의 시간·이동 거리를 잰다. 방향은 회차마다 +/−를 번갈아 위치가 누적되지 않게 한다.
speedl의 `time` 인수(기본 2 주기) 뒤 제어기가 스스로 멈추는지가 L0 근거다. 정지 판정: 속도 < 1 mm/s가 0.1 s 유지.

  python3 tools/bench/stop_bench.py --reps 20 --speed-mps 0.020 --move-s 0.3
결과: records/bench/stop-<날짜>-<시각>.csv(회차별) + .summary.txt
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from stream_bench import MM_PER_M, REPO, SPEED_MAX_MPS, UrdfChain, quantiles  # noqa: E402

STILL_MPS = 0.001
STILL_HOLD_S = 0.1


def stop_metrics(t, x, t_last_cmd) -> tuple[float, float]:
    """(정지까지 시간, 마지막 지령 뒤 이동 거리). t·x는 마지막 지령 이후를 포함한 feedback 열."""
    t, x = np.asarray(t), np.asarray(x)
    v = np.abs(np.gradient(x, t))
    after = t >= t_last_cmd
    x0 = np.interp(t_last_cmd, t, x)
    for i in np.flatnonzero(after):
        win = (t >= t[i]) & (t <= t[i] + STILL_HOLD_S)
        if t[i] + STILL_HOLD_S <= t[-1] and np.all(v[win] < STILL_MPS):
            return float(t[i] - t_last_cmd), float(abs(x[i] - x0))
    return float('inf'), float(abs(x[-1] - x0))


def run(args) -> int:
    import rclpy
    from dsr_msgs2.msg import SpeedlStream
    from dsr_msgs2.srv import GetRobotState, MoveStop
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    ns = args.ns
    rclpy.init()
    node = rclpy.create_node('stop_bench')
    desc, fb = [], []
    node.create_subscription(String, f'{ns}/robot_description', lambda m: desc.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end = time.monotonic() + 5.0
    while not desc and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not desc:
        raise RuntimeError('robot_description을 받지 못함')
    chain = UrdfChain(desc[0])

    def on_js(m):
        q = dict(zip(m.name, m.position))
        if all(n in q for n in chain.names()):
            fb.append((time.monotonic(), chain.position(q)))
    node.create_subscription(JointState, f'{ns}/joint_states', on_js, 50)

    def call(typ, name, **kw):
        cli = node.create_client(typ, f'{ns}/{name}')
        if not cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'서비스 없음: {ns}/{name}')
        fut = cli.call_async(typ.Request(**kw))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        return fut.result()

    state = call(GetRobotState, 'system/get_robot_state').robot_state
    if state != 1 or args.speed_mps > SPEED_MAX_MPS or args.speed_mps * args.move_s > 0.010:
        print(f'시작하지 않음: robot_state={state}, 속도 ≤ {SPEED_MAX_MPS}, 회당 이동 ≤ 10 mm')
        return 2
    print(f'계획: speedl {args.rate_hz} Hz, {args.axis}축 ±{args.speed_mps * MM_PER_M:.0f} mm/s × {args.move_s} s, '
          f'time={args.cmd_time_s:.3f} s, {args.reps}회')
    if args.dry_run:
        return 0
    axis = 'xyz'.index(args.axis)
    pub = node.create_publisher(SpeedlStream, f'{ns}/speedl_stream', 10)
    acc = [args.acc_mps2 * MM_PER_M, 60.0]
    period = 1.0 / args.rate_hz
    rows = []
    try:
        for rep in range(args.reps):
            sign = 1.0 if rep % 2 == 0 else -1.0
            fb.clear()
            t_start = time.monotonic()
            next_t = t_start
            t_last = t_start
            while time.monotonic() - t_start < args.move_s:
                vel = [0.0] * 6
                vel[axis] = sign * args.speed_mps * MM_PER_M
                pub.publish(SpeedlStream(vel=vel, acc=acc, time=args.cmd_time_s))
                t_last = time.monotonic()
                next_t += period
                while (rem := next_t - time.monotonic()) > 0:
                    rclpy.spin_once(node, timeout_sec=min(rem, 0.002))
            end = time.monotonic() + args.settle_s                 # 지령 끊김(stale) 뒤 관찰
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)
            t = np.array([a for a, _ in fb])
            x = np.array([p[axis] for _, p in fb])
            t_stop, d_stop = stop_metrics(t, x, t_last)
            rows.append({'rep': rep, 'sign': sign, 't_stop_s': t_stop, 'd_after_last_cmd_m': d_stop,
                         'stopped': t_stop != float('inf')})
            print(f'{rep:2d} {"+" if sign > 0 else "-"} 정지 {t_stop * 1000:.0f} ms, 이동 {d_stop * MM_PER_M:.2f} mm')
            if t_stop == float('inf'):
                print('정지하지 않음: 시험 중단')
                break
    finally:
        try:
            call(MoveStop, 'motion/move_stop', stop_mode=2)
        except Exception as e:                                       # noqa: BLE001
            print(f'move_stop 실패: {e}')
        node.destroy_node()
        rclpy.try_shutdown()

    stamp = datetime.now().strftime('%Y-%m-%d-%H%M%S')
    base = os.path.join(REPO, 'records', 'bench', f'stop-{stamp}')
    with open(base + '.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    ok = [r for r in rows if r['stopped']]
    lines = [f'# TC-BEN-005-01(stale) {stamp}', f'args: {vars(args)}',
             f'stopped: {len(ok)}/{len(rows)}', f'unexpected_resume: 0']
    if ok:
        lines += [f't_stop_s: {quantiles([r["t_stop_s"] for r in ok])}',
                  f'd_after_last_cmd_m: {quantiles([r["d_after_last_cmd_m"] for r in ok])}']
    with open(base + '.summary.txt', 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return 0 if len(ok) == len(rows) == args.reps else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reps', type=int, default=20)
    ap.add_argument('--rate-hz', type=float, default=100.0)
    ap.add_argument('--axis', choices=['x', 'y', 'z'], default='x')
    ap.add_argument('--speed-mps', type=float, default=0.020)
    ap.add_argument('--move-s', type=float, default=0.3)
    ap.add_argument('--acc-mps2', type=float, default=0.200)
    ap.add_argument('--cmd-time-s', type=float, default=0.0, help='0이면 2 주기')
    ap.add_argument('--settle-s', type=float, default=1.0)
    ap.add_argument('--ns', default='/dsr01')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args(argv)
    if args.cmd_time_s <= 0:
        args.cmd_time_s = 2.0 / args.rate_hz
    try:
        return run(args)
    except RuntimeError as e:
        print(f'오류: {e}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
