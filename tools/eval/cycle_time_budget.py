#!/usr/bin/env python3
"""static_pick_place 실행 로그(jsonl)에서 단계별 시간을 읽고, 그리퍼 통신 방식을 바꿨을 때의 사이클 시간을 추정한다.

측정값은 로그의 `ready`·`done` 시각 차다. 정책별 값은 **추정**이다(실기 미확인).
모델: 상주 루프에서 그리퍼 명령 1회 = 목표 쓰기 1왕복 + 확인 읽기 n왕복, 왕복 시간은 `--rtt`.
왕복 0.58 s는 일회성 DRL에서 잰 값(2026-11-09)이며 상주 루프에서도 같은지는 OFF-2 측정 대상이다.

이동 속도를 바꾼 추정(`--vertical-v` 등)은 사다리꼴 속도 모델 `d/v + v/a`에 구간별 잔차(측정 − 모델)를 더한 값이다.
구간을 잇는 곡선 이동(blending)은 모델에 없다.

    python3 tools/eval/cycle_time_budget.py <정지 투입 기록>.jsonl
    python3 tools/eval/cycle_time_budget.py <log> --vertical-v 0.1 --vertical-a 0.3
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

GRIPPER_STEPS = ('open_before_pick', 'close_grasp', 'open_release')


@dataclass(frozen=True)
class Step:
    name: str
    seconds: float
    gripper: bool


@dataclass(frozen=True)
class Policy:
    name: str
    resident_loop: bool = False        # 명령마다 DRL을 띄우지 않고 상주 루프에 쓴다
    skip_open_if_open: bool = False    # 이미 열려 있으면 첫 열기를 생략한다
    overlap_first_open: bool = False   # 첫 열기를 공깃돌 위로 가는 이동과 겹친다


POLICIES = (
    Policy('measured'),
    Policy('skip_first_open', skip_open_if_open=True),
    Policy('resident_loop', resident_loop=True),
    Policy('resident_loop+skip_first_open', resident_loop=True, skip_open_if_open=True),
    Policy('resident_loop+overlap_first_open', resident_loop=True, overlap_first_open=True),
)


def read_steps(path) -> list[Step]:
    """`ready` 뒤 첫 사이클의 `done` 간격. 여러 사이클이면 첫 `cycle_done`까지만 쓴다."""
    steps: list[Step] = []
    prev = None
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line.startswith('{'):
            continue
        row = json.loads(line)
        event = row.get('event')
        if event == 'ready':
            prev, steps = float(row['t']), []
        elif event == 'done' and prev is not None:
            t = float(row['t'])
            steps.append(Step(row['step'], t - prev, row['step'] in GRIPPER_STEPS))
            prev = t
        elif event in ('cycle_done', 'abort') and steps:
            break
    if not steps:
        raise ValueError(f'{path}: ready 뒤 done 단계가 없다')
    return steps


# 로그를 남긴 실행의 속도·가속(static_pick_place.plan.PickPlaceParams, 2026-11-09)
BASE_TRAVEL = (0.100, 0.300)
BASE_VERTICAL = (0.050, 0.150)


def read_plan(path) -> list[tuple[str, tuple[float, float, float]]]:
    """마지막 계획의 이동 단계 이름과 목표 위치 [m]."""
    plan: list[tuple[str, tuple[float, float, float]]] = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line.startswith('{'):
            continue
        row = json.loads(line)
        if row.get('event') == 'targets':
            plan = []
        elif row.get('event') == 'plan' and 'pos_m' in row:
            plan.append((row['step'], tuple(float(x) for x in row['pos_m'])))
        elif row.get('event') == 'ready' and plan:
            break
    return plan


def trapezoid_s(distance_m: float, v_mps: float, a_mps2: float) -> float:
    """정지에서 정지까지 사다리꼴(짧으면 삼각형) 속도 이동 시간."""
    if distance_m <= 0.0:
        return 0.0
    if distance_m >= v_mps * v_mps / a_mps2:
        return distance_m / v_mps + v_mps / a_mps2
    return 2.0 * math.sqrt(distance_m / a_mps2)


def retime_moves(steps: list[Step], plan, travel=BASE_TRAVEL, vertical=BASE_VERTICAL) -> tuple[list[Step], dict]:
    """이동 구간을 새 속도·가속으로 다시 계산한다. 첫 구간은 대기 자세(계획의 마지막 위치)에서 출발한다고 본다."""
    pos = dict(plan)
    prev = plan[-1][1] if plan else None
    out, residuals = [], {}
    for s in steps:
        if s.gripper or s.name not in pos or prev is None:
            out.append(s)
            continue
        cur = pos[s.name]
        dist = math.dist(prev, cur)
        is_vertical = math.hypot(cur[0] - prev[0], cur[1] - prev[1]) < 1e-3
        base = BASE_VERTICAL if is_vertical else BASE_TRAVEL
        new = vertical if is_vertical else travel
        residuals[s.name] = s.seconds - trapezoid_s(dist, *base)
        out.append(Step(s.name, max(0.0, trapezoid_s(dist, *new) + residuals[s.name]), False))
        prev = cur
    return out, residuals


def gripper_seconds(step: str, rtt_s: float, open_polls: int, grasp_polls: int) -> float:
    polls = grasp_polls if step == 'close_grasp' else open_polls
    return rtt_s * (1 + polls)


def apply(steps: list[Step], policy: Policy, rtt_s: float = 0.58, open_polls: int = 1,
          grasp_polls: int = 2) -> list[Step]:
    """정책을 적용한 단계 시간. 이동 단계는 그대로 둔다."""
    out: list[Step] = []
    for i, s in enumerate(steps):
        sec = s.seconds
        if s.gripper and policy.resident_loop:
            sec = gripper_seconds(s.name, rtt_s, open_polls, grasp_polls)
        if s.name == 'open_before_pick':
            if policy.skip_open_if_open:
                sec = 0.0
            elif policy.overlap_first_open and i + 1 < len(steps) and not steps[i + 1].gripper:
                sec = max(0.0, sec - steps[i + 1].seconds)     # 다음 이동보다 길 때만 기다린다
        out.append(Step(s.name, sec, s.gripper))
    return out


def summarize(steps: list[Step]) -> dict:
    grip = sum(s.seconds for s in steps if s.gripper)
    move = sum(s.seconds for s in steps if not s.gripper)
    return {'gripper_s': round(grip, 2), 'move_s': round(move, 2), 'total_s': round(grip + move, 2)}


def table(steps: list[Step], rtt_s: float, open_polls: int, grasp_polls: int) -> list[dict]:
    base = summarize(steps)['total_s']
    rows = []
    for p in POLICIES:
        row = {'policy': p.name, **summarize(apply(steps, p, rtt_s, open_polls, grasp_polls))}
        row['saved_s'] = round(base - row['total_s'], 2)
        row['estimated'] = p.name != 'measured'
        rows.append(row)
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('log', help='static_pick_place 실행 로그(jsonl)')
    ap.add_argument('--rtt', type=float, nargs='+', default=[0.58], help='Modbus 왕복 시간 [s], 여러 값이면 각각 계산')
    ap.add_argument('--open-polls', type=int, default=1, help='열기 확인 읽기 횟수')
    ap.add_argument('--grasp-polls', type=int, default=2, help='파지 확인 읽기 횟수(멈춤 판정에 2회)')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--travel-v', type=float, default=BASE_TRAVEL[0], help='수평 이동 속도 [m/s]')
    ap.add_argument('--travel-a', type=float, default=BASE_TRAVEL[1], help='수평 이동 가속 [m/s²]')
    ap.add_argument('--vertical-v', type=float, default=BASE_VERTICAL[0], help='수직 이동 속도 [m/s]')
    ap.add_argument('--vertical-a', type=float, default=BASE_VERTICAL[1], help='수직 이동 가속 [m/s²]')
    args = ap.parse_args(argv)
    measured = read_steps(args.log)
    steps, residuals = retime_moves(measured, read_plan(args.log), (args.travel_v, args.travel_a),
                                    (args.vertical_v, args.vertical_a))
    retimed = (args.travel_v, args.travel_a, args.vertical_v, args.vertical_a) != BASE_TRAVEL + BASE_VERTICAL
    result = {'steps': [{'step': s.name, 'seconds': round(s.seconds, 2), 'gripper': s.gripper} for s in measured],
              'move_model_residual_s': {k: round(v, 3) for k, v in residuals.items()}, 'moves_retimed': retimed,
              'measured_total_s': summarize(measured)['total_s'],
              'by_rtt': {str(r): table(steps, r, args.open_polls, args.grasp_polls) for r in args.rtt}}
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=1))
        return 0
    print('measured steps [s]: ' + ', '.join(f"{s.name} {s.seconds:.2f}" for s in measured))
    if retimed:
        print(f"moves retimed (추정): travel {args.travel_v} m/s {args.travel_a} m/s², "
              f"vertical {args.vertical_v} m/s {args.vertical_a} m/s² — 첫 줄 'measured'는 그리퍼만 측정값, "
              f"측정 합계 {result['measured_total_s']:.2f} s")
        print('retimed moves [s]: ' + ', '.join(f"{s.name} {s.seconds:.2f}" for s in steps if not s.gripper))
        print(f"move model residual [s]: {result['move_model_residual_s']}")
    for rtt, rows in result['by_rtt'].items():
        print(f'\nrtt {rtt} s (open polls {args.open_polls}, grasp polls {args.grasp_polls})')
        print(f"{'policy':36s} {'gripper':>8s} {'move':>7s} {'total':>7s} {'saved':>7s}")
        for r in rows:
            mark = '' if not r['estimated'] else ' (추정)'
            print(f"{r['policy']:36s} {r['gripper_s']:8.2f} {r['move_s']:7.2f} {r['total_s']:7.2f} {r['saved_s']:7.2f}{mark}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
