#!/usr/bin/env python3
"""움직이는 병뚜껑에 공깃돌 넣기(실제 로봇·그리퍼 지령, 입회 필수).

매트의 공깃돌을 집어 벨트 옆에서 기다리다가, 카메라 검출로 뚜껑이 해제 지점에 오는 시각을 잡아 그보다
조금 앞서 해제 지점으로 들어가고, 그리퍼가 열리는 시간만큼 먼저 개방 신호를 보내 뚜껑 안에 떨어뜨린다.
놓는 순간 벨트를 따라 움직이지 않는다(해제 지점 x 고정).

쓰는 것: 공깃돌 검출·직선 이동(`static_pick_place`), 그리퍼 상주 루프의 예약 → 트리거
(`robot_control.resident_transport`), vision_node의 실시간 뚜껑 검출(`objects/detections`).
tracker·planner는 쓰지 않는다: 2.8 s 앞 예측은 벨트 속도가 바뀌면 1 s 넘게 틀렸고(−1.42~+0.85 s), 빠른 벨트에서
planner의 해제 지점은 팔이 틀어지는 x > 0.70 m에 놓였다. 대신 해제 x를 고정하고 검출로 도착 시각만 맞춘다.

  # 계획만(로봇·그리퍼 지령 없음): 고른 공깃돌·IK, 계획, 도착 시각 예측이 실제 통과와 얼마나 맞는지
  PYTHONPATH=. python3 tools/robot/moving_drop.py --config config/site.yaml
  # 한 개 옮기기(영상·검출 녹화 포함)
  PYTHONPATH=. python3 tools/robot/moving_drop.py --config config/site.yaml --execute --record logs/run-01
  # 매트의 돌을 모두 옮기기(돌 사이에 park로 가지 않는다), park로만 돌아가기
  PYTHONPATH=. python3 tools/robot/moving_drop.py --config config/site.yaml --execute --count 0
  PYTHONPATH=. python3 tools/robot/moving_drop.py --config config/site.yaml --execute --park-only
버튼 창: tools/robot/drop_ui.sh

순서(--execute): 상주 루프 확인·기동 → 공깃돌 위 → 하강 → 닫기(150 mA) → 들기 → 벨트 옆 대기(--wait-y)
→ 지난 통과로 벨트 선·속도를 잡음 → 새 통과가 보이면 개방 예약 → 검출로 도착 시각을 계속 고쳐 잡음
→ (도착 − drop_delay − arrive_before_fire − 이동 시간)에 출발 → (도착 − drop_delay)에 트리거
→ 손가락이 열릴 때까지 높이 유지 → (다음 돌 또는) 들기 → 벨트 옆 → park.
--rehearse: 공깃돌·그리퍼 없이 같은 동작을 하고 트리거 시각만 기록한다(시각 검증용).
멈추는 법: 이동 중에는 비상정지·펜던트. --stop-file(버튼 창의 중지)은 다음 결정 지점에서 멈춘다.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

RELEASE_X_MAX_M = 0.685   # 이보다 멀면 제어기 특이점 처리로 경로가 틀어진다(2026-11-14: x 0.690 상승 1.7 mm 모자람,
                          # x 0.6975 해제 지점 2.4 mm 모자람, x 0.695 수직 상승이 x로 5 mm 밀림). x ≤ 0.676은 4회 모두 정상
MAT_Y_MAX_M = 0.02        # 매트 공깃돌 y 상한. 매트와 벨트 사이 빨간 LED 오검출(y ≈ +0.047 m, 2026-11-14)을 막는다
CLASS_CAP, REGION_BELT = 2, 2


def crossing(samples, x_line: float):
    """(t, x, y) 표본에서 x가 x_line을 처음 넘는 시각과 그때 y를 선형 보간한다. 넘지 않으면 None."""
    s = sorted(samples)
    for (t0, x0, y0), (t1, x1, y1) in zip(s, s[1:]):
        if (x0 - x_line) * (x1 - x_line) <= 0 and x0 != x1:
            a = (x_line - x0) / (x1 - x0)
            return t0 + a * (t1 - t0), y0 + a * (y1 - y0)
    return None


def move_time_s(dist_m: float, v_mps: float, a_mps2: float) -> float:
    """사다리꼴 속도 직선 이동 시간(가감속 포함)."""
    if dist_m <= v_mps * v_mps / a_mps2:
        return 2.0 * np.sqrt(dist_m / a_mps2)
    return dist_m / v_mps + v_mps / a_mps2


def log(event: str, **fields) -> None:
    print(json.dumps({'t': round(time.time(), 3), 'event': event, **fields}, ensure_ascii=False, default=str),
          flush=True)


class SteadyClock:
    def steady_now(self) -> float:
        return time.monotonic()


ROLLER_END_X_M = 0.56     # 이보다 상류에서는 뚜껑이 롤러를 돌아 올라오는 중이라 x 속도가 약 30 mm/s로 보인다
ROLLER_V_MPS = 0.030      # (2026-11-14 통과 9회: 0.545~0.56 구간 24~32 mm/s, 그 뒤 50 mm/s)


def arrival_estimate(points, rel_x: float, rel_y: float, now: float, window_s: float = 1.0, min_points: int = 8,
                     max_age_s: float = 0.35, t_stop: float | None = None, min_span_s: float = 0.3,
                     v_prior: float | None = None):
    """뚜껑 검출 (stamp, x, y, ...)로 x = rel_x 도착 시각을 낸다.

    해제 지점 상류(x < rel_x − 3 mm)이고 벨트 선(|y − rel_y| < 20 mm)에 있는 최근 window_s의 검출만 쓴다. t_stop이
    있으면 그보다 이른 검출만 쓴다(팔이 벨트를 가리기 시작한 뒤의 검출은 중심이 밀린다). 낡았으면(max_age_s) None.

    v_prior(지난 통과 전체의 평균 속도)가 있으면 지금 위치(마지막 다섯 검출의 중앙값)에서 그 속도로 계산한다.
    통과 9회, 도착 1.15 s 전 기준: 이 방법 오차 최대 0.045 s, 최근 1 s 직선 맞춤은 최대 0.092 s였다
    (벨트 속도가 40 mm 구간마다 ±4 mm/s 맥동해서 짧은 구간의 기울기가 흔들린다). 다만 최근 검출의 기울기가
    v_prior와 12 mm/s 넘게 다르면 벨트 속도가 바뀐 것으로 보고 직선 맞춤을 쓴다. v_prior가 없으면 직선 맞춤만 쓴다
    (검출 min_points개·min_span_s 이상, 잔차 3 mm 이하, 속도 8~150 mm/s).
    """
    end = now if t_stop is None else min(now, t_stop)
    pts = [(q[0], q[1]) for q in points
           if end - window_s <= q[0] <= end and q[1] < rel_x - 0.003 and abs(q[2] - rel_y) < 0.02]
    if not pts or end - pts[-1][0] > max_age_s:
        return None
    t = np.array([q[0] for q in pts])
    x = np.array([q[1] for q in pts])
    fit = None
    past = x >= ROLLER_END_X_M + 0.005
    if int(past.sum()) >= min_points and t[past][-1] - t[past][0] >= min_span_s:
        tp, xp = t[past], x[past]
        a, b = np.polyfit(tp - tp[-1], xp, 1)
        if 0.008 <= a <= 0.15 and float(np.sqrt(np.mean((xp - (a * (tp - tp[-1]) + b)) ** 2))) <= 0.003:
            fit = {'t_arrive': float(tp[-1] + (rel_x - b) / a), 'v': float(a), 'n': int(past.sum()),
                   't_last': float(t[-1]), 'x_last': float(b), 'prior': False}
    if v_prior is None:
        return fit
    if fit is not None and abs(fit['v'] - v_prior) > 0.012:
        return fit
    if len(pts) < 3:
        return None
    k = min(5, len(pts))
    t_ref, x_ref = float(np.median(t[-k:])), float(np.median(x[-k:]))
    roller = max(0.0, ROLLER_END_X_M - x_ref)
    t_arrive = t_ref + roller / ROLLER_V_MPS + (rel_x - x_ref - roller) / v_prior
    return {'t_arrive': t_arrive, 'v': float(v_prior), 'n': len(pts), 't_last': float(t[-1]), 'x_last': x_ref,
            'prior': True}


def judge(flags, t_fire: float, x_min: float, min_frames: int = 5, window_s: float = 6.0) -> dict:
    """트리거 뒤 뚜껑이 그리퍼 아래를 벗어난 구간(x > x_min)의 stone_inside 표시로 판정한다.

    flags: (stamp, x, stone_inside, stone_inside_valid). drop_analyze.py와 같은 기준:
    SUCCESS = 참 5개 이상이고 판정 가능한 프레임의 60% 이상, MISS = 거짓 5개 이상이고 참 2개 미만, 그 밖 UNKNOWN.
    트리거 뒤 window_s까지만 센다: 다음 바퀴에는 같은 뚜껑이 빈 채로 다시 지나간다(한 바퀴는 11 s 이상).
    """
    seen = [(si, sv) for t, x, si, sv in flags if t_fire < t <= t_fire + window_s and x > x_min]
    si_true = sum(1 for si, sv in seen if sv and si)
    si_false = sum(1 for si, sv in seen if sv and not si)
    if si_true >= min_frames and si_true >= 0.6 * (si_true + si_false):
        result = 'SUCCESS'
    elif si_false >= min_frames and si_true < 2:
        result = 'MISS'
    else:
        result = 'UNKNOWN'
    return {'result': result, 'si_true': si_true, 'si_false': si_false}


def last_pass_model(points, x_lo: float = 0.56, x_hi: float = 0.76, min_points: int = 20, min_span_m: float = 0.06):
    """가장 최근에 x_lo~x_hi를 지나간 통과의 속도 v, 벨트 선 y, 테두리 높이 z. points: (stamp, x, y, z). 없으면 None."""
    pts = [q for q in points if x_lo <= q[1] <= x_hi]
    if not pts:
        return None
    clusters, cur = [], [pts[-1]]
    for q in reversed(pts[:-1]):
        if cur[-1][0] - q[0] > 1.0:
            clusters.append(cur)
            cur = [q]
        else:
            cur.append(q)
    clusters.append(cur)
    for c in clusters:                               # 최근 통과부터
        c = c[::-1]
        if len(c) < min_points or c[-1][1] - c[0][1] < min_span_m:
            continue
        t = np.array([q[0] for q in c])
        v = float(np.polyfit(t - t[0], np.array([q[1] for q in c]), 1)[0])
        if 0.008 <= v <= 0.15:
            return {'v': v, 'y': float(np.median([q[2] for q in c])), 'z': float(np.median([q[3] for q in c])),
                    'n': len(c), 't': float(c[-1][0])}
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--config', action='append', default=[])
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--rel-x', type=float, default=0.67, help='해제 지점 x [m](고정). 0.676 이하는 팔이 정상 도달했다')
    ap.add_argument('--wait-y', type=float, default=0.16,
                    help='벨트 옆 대기 y [m](x = rel-x). 벨트 선(y≈0.09)에서 약 70 mm: 해제 지점까지 약 1.0 s')
    ap.add_argument('--wait-plan-s', type=float, default=45.0, help='잡을 수 있는 통과를 기다리는 최대 시간(통과마다 다시 센다)')
    ap.add_argument('--rehearse', action='store_true', help='공깃돌·그리퍼 없이 같은 동작과 트리거 시각 기록만 한다')
    ap.add_argument('--passes', type=int, default=4, help='놓지 못하고 보낸 통과를 포함해 시도할 통과 수')
    ap.add_argument('--drop-delay-s', type=float, default=0.6,
                    help='트리거 → 공깃돌이 뚜껑 높이에 닿기까지. 도착 예상 시각보다 이만큼 먼저 트리거한다. 손가락은 명령 '
                         '직후 약 150 pulse/s(틈 14 mm/s)로 열린다(2026-11-14 빈 그리퍼 661→450: 0.7 s에 579). 600 pulse '
                         '목표에서는 감속 구간 끝에서야 돌이 빠져 약 1.3 s가 걸렸다')
    ap.add_argument('--release-open-pulse', type=int, default=450,
                    help='놓을 때 여는 목표 pulse. 600은 돌 폭보다 겨우 넓어 감속 구간에서 떨어졌다. 더 넓게 열어 '
                         '떨어지는 틈을 제 속도로 지나가게 한다')
    ap.add_argument('--arrive-before-fire-s', type=float, default=0.2,
                    help='트리거보다 이만큼 먼저 해제 지점에 도착한다(도착 = 뚜껑 도착 예상 − drop_delay − 이 값)')
    ap.add_argument('--block-dist-m', type=float, default=0.05,
                    help='팔이 해제 지점에서 이 거리 안으로 들어온 뒤의 검출은 쓰지 않는다(그리퍼가 뚜껑을 가린다)')
    ap.add_argument('--x-offset-m', type=float, default=0.005,
                    help='해제 지점을 겨냥점(rel-x)보다 벨트 진행 방향(+x)으로 이만큼 옮긴다. 도착 시각은 겨냥점 기준이라 '
                         '돌은 뚜껑 중심보다 이만큼 하류에 떨어진다(성공 13회: 돌이 중심보다 평균 1.7 mm, '
                         '최대 3.9 mm 상류에 놓였고 뚜껑은 추정보다 평균 0.041 s 일찍 왔다)')
    ap.add_argument('--y-offset-m', type=float, default=0.0,
                    help='해제 지점 y 보정. 돌이 +y 손가락에 붙어 떨어지면 음수로 준다')
    ap.add_argument('--holding', action='store_true',
                    help='이미 공깃돌을 쥐고 있다. 검출·파지를 건너뛴다')
    ap.add_argument('--release-raise-m', type=float, default=0.015, help='해제 높이에 더하는 여유(손가락·테두리 간격)')
    ap.add_argument('--count', type=int, default=1,
                    help='옮길 공깃돌 수. 0이면 매트에 남은 돌이 없을 때까지. 여러 개일 때는 돌 사이에 park로 가지 않고 '
                         '놓은 자리에서 다음 돌로 곧장 간다')
    ap.add_argument('--stop-file', help='이 파일이 생기면 멈춘다(UI의 중지 버튼). 뚜껑으로 들어가는 중이면 그 돌까지 놓고, '
                                        '아직 기다리는 중이면 쥔 돌을 집었던 매트 자리에 내려놓은 뒤 park로 간다')
    ap.add_argument('--park-only', action='store_true', help='이동 높이로 올린 뒤 park로 돌아가기만 한다')
    ap.add_argument('--speed-scale', type=float, default=1.0,
                    help='집기·이동 구간의 속도·가속도 배율(0.5~2.0). 해제 지점으로 들어가는 이동은 바꾸지 않는다')
    ap.add_argument('--hold-after-s', type=float, default=0.6, help='돌이 떨어진 뒤 높이를 유지하는 시간')
    ap.add_argument('--record', help='이 접두로 상단 카메라 영상·검출을 녹화한다(tools/robot/run_recorder.py, 폴더는 미리 만든다)')
    ap.add_argument('--side-watch-s', type=float, default=2.5, help='놓은 뒤 벨트 옆에서 뚜껑이 지나가는 것을 찍는 시간')
    args = ap.parse_args(argv)
    if not 0.5 <= args.speed_scale <= 2.0:
        ap.error('--speed-scale은 0.5~2.0')

    import os
    import threading
    from collections import deque

    import rclpy
    from drop_interfaces.msg import ObjectDetectionArray
    from dsr_msgs2.srv import MoveLine
    from rclpy.executors import SingleThreadedExecutor
    from robot_control.dsr_register_io import DsrRegisterIO, RosServiceCaller
    from robot_control.gripper_adapter import MODBUS
    from robot_control.resident_transport import ResidentLoopTransport, ResidentRegs
    from static_pick_place import doosan_client as dc
    from static_pick_place.doosan_client import DoosanClient, RobotError
    from static_pick_place.pick_place import detect, grab_frame, load_config, load_geometry, make_params
    from static_pick_place.plan import CLASS_STONE, DOWN_QUAT_XYZW, REGION_MAT, _in_workspace, _usable

    cfg = load_config(args.config)
    p = make_params(cfg)
    recorder = None
    if args.record:
        import subprocess
        os.makedirs(os.path.dirname(os.path.abspath(args.record)), exist_ok=True)
        recorder = subprocess.Popen([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                  'run_recorder.py'), '--out', args.record],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('moving_drop')
    caller = RosServiceCaller('moving_drop_dsr_io')
    io = DsrRegisterIO(caller, 'dsr01')
    regs = ResidentRegs()
    loop = ResidentLoopTransport(io, SteadyClock(), time.sleep, regs)
    robot = DoosanClient(node, 'dsr01')

    # 뚜껑 검출을 별도 노드·executor 스레드로 모은다(주 스레드는 서비스 호출로 막혀 있을 때가 많다)
    caps = deque(maxlen=1500)                  # (stamp, x, y, z) robot_base, z = 테두리 윗면
    cap_flags = deque(maxlen=3000)             # (stamp, x, stone_inside, stone_inside_valid) 판정용
    caps_lock = threading.Lock()
    watch_node = rclpy.create_node('moving_drop_watch')

    def on_dets(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with caps_lock:
            for d in msg.detections:
                if d.object_class == CLASS_CAP and d.region == REGION_BELT and d.position_valid:
                    caps.append((stamp, d.position.x, d.position.y, d.position.z))
                    cap_flags.append((stamp, d.position.x, bool(d.stone_inside), bool(d.stone_inside_valid)))

    watch_node.create_subscription(ObjectDetectionArray, 'objects/detections', on_dets, 20)
    watch_exec = SingleThreadedExecutor()
    watch_exec.add_node(watch_node)

    def _watch_spin():
        try:
            watch_exec.spin()
        except Exception:          # noqa: BLE001 - 종료 때 스레드만 끝낸다
            pass

    threading.Thread(target=_watch_spin, daemon=True).start()

    def cap_points():
        with caps_lock:
            return list(caps)

    def now_s():
        return node.get_clock().now().nanoseconds * 1e-9

    # 해제 지점 위에서 미리 기다리지 않는다: 그 자세에서는 팔이 상류 벨트를 가려 뚜껑이 x 0.689부터만 보인다
    tv, ta = p.travel_v_mps * args.speed_scale, p.travel_a_mps2 * args.speed_scale
    vv, va = p.vertical_v_mps * args.speed_scale, p.vertical_a_mps2 * args.speed_scale
    arrive_before_s = args.drop_delay_s + args.arrive_before_fire_s

    def move(name, pos, v=None, a=None, tolerate=False):
        """tolerate: 몇 mm 못 미쳐도(MOVE_NOT_REACHED) 다음 동작으로 넘어간다."""
        try:
            reached = robot.move_line(pos, DOWN_QUAT_XYZW, v or tv, a or ta)
        except RobotError as exc:
            if not (tolerate and exc.reason == 'MOVE_NOT_REACHED'):
                raise
            reached, _ = robot.tcp_pose()
            log('not_reached', step=name, target_m=[round(float(x), 4) for x in pos],
                reached_m=[round(float(x), 4) for x in reached])
            return reached
        log('done', step=name, reached_m=[round(float(x), 4) for x in reached])
        return reached

    def ik_ok(name, pos) -> bool:
        j = robot.ik_joints(pos, DOWN_QUAT_XYZW)
        for _ in range(2):                  # 관절이 모두 0이면 IK 응답 실패로 보고 다시 묻는다
            if np.any(np.abs(np.asarray(j)) > 1e-9):
                break
            j = robot.ik_joints(pos, DOWN_QUAT_XYZW)
        ok = abs(j[2]) >= p.j3_min_rad and p.x_min_m <= pos[0] <= p.x_max_m and p.y_min_m <= pos[1] <= p.y_max_m
        log('check', step=name, pos_m=[round(float(x), 4) for x in pos], j3_rad=round(float(j[2]), 4), ok=ok)
        return ok

    def gripper_to(pulse, current_ma, timeout_s=8.0, tries=3):
        """gripper_once를 Modbus 쓰기 실패(결과 242)면 다시 보낸다."""
        for attempt in range(tries):
            pos, done = gripper_once(pulse, current_ma, timeout_s)
            if not (done and done[0] and done[1] == 242):
                return pos, done
            log('gripper_retry', pulse=pulse, attempt=attempt + 1, cmd_result=242)
        return pos, done

    def gripper_begin(pulse, current_ma):
        """루프에 그리퍼 지령을 보내고 바로 돌아온다. 반환 = 보낸 시각(monotonic)."""
        t0 = time.monotonic()
        if not (loop.write_register(MODBUS.goal_current, current_ma) and loop.write_goal_position(pulse)
                and loop.trigger()):
            raise RuntimeError('GRIPPER_WRITE_FAILED')
        return t0

    def gripper_finish(pulse, t0, timeout_s):
        """보낸 지령이 도착하거나 1 s 멈출 때까지 기다린다. 실행 중에는 상태가 갱신되지 않는다."""
        samples, done = [], None
        while time.monotonic() - t0 < timeout_s:
            done = loop.last_command()
            pos = loop._word(regs.st_pos_hi)
            samples.append(pos)
            if done and done[0] and (done[1] != 0 or abs(pos - pulse) <= 5 or (time.monotonic() - t0 > 3.0
                                                                              and len(set(samples[-4:])) == 1)):
                break
            time.sleep(0.25)
        return loop._word(regs.st_pos_hi), done

    def gripper_once(pulse, current_ma, timeout_s):
        return gripper_finish(pulse, gripper_begin(pulse, current_ma), timeout_s)

    aim_x = args.rel_x          # 뚜껑 중심이 이 x에 올 때 돌이 해제 지점 높이에서 떨어지도록 시각을 맞춘다

    def release_point(model):
        return np.array([aim_x + args.x_offset_m, model['y'] + args.y_offset_m,
                         model['z'] + p.release_above_rim_m + args.release_raise_m])

    def get_pass(start_pos, deadline_mono):
        """벨트 옆에서 잡을 수 있는 통과를 기다린다. 반환 (해제 지점, 도착 예상 시각, 이동 시간, 지난 통과 모델),
        중지 요청이면 'STOP', 시간이 다 되면 None.

        해제 지점 = (rel-x + x-offset, 지난 통과의 벨트 선 y, 테두리 z + release_above_rim + 여유). 새 통과의 검출이 보이고
        도착까지 max(drop_delay, 이동 + arrive_before)보다 시간이 남아 있으면 돌려준다. 너무 가까이 온 통과는 보낸다.
        """
        waited, checked = {}, None
        while time.monotonic() < deadline_mono:
            if stop_requested():
                log('stop_seen', where='wait_cap', waited=waited)
                return 'STOP'
            pts, now = cap_points(), now_s()
            model = last_pass_model(pts)
            if model is None:
                waited['NO_PASS_SEEN'] = waited.get('NO_PASS_SEEN', 0) + 1
                time.sleep(0.05)
                continue
            rel = release_point(model)
            if checked is None or float(np.linalg.norm(checked - rel)) > 0.003:
                if not (rel[2] < p.travel_z_m and rel[0] <= RELEASE_X_MAX_M and ik_ok('release', rel)):
                    log('abort', reason='RELEASE_POSE', release_m=[round(float(v), 4) for v in rel])
                    return None
                checked = rel
            t_move = move_time_s(float(np.linalg.norm(rel - start_pos)), p.travel_v_mps, p.travel_a_mps2)
            e = arrival_estimate(cap_points(), aim_x, model['y'], now, v_prior=model['v'])
            if e is None:
                waited['NO_CAP_UPSTREAM'] = waited.get('NO_CAP_UPSTREAM', 0) + 1
                time.sleep(0.01)
                continue
            if e['t_arrive'] - now < max(args.drop_delay_s, t_move + arrive_before_s) + 0.1:
                waited['TOO_LATE'] = waited.get('TOO_LATE', 0) + 1
                time.sleep(0.01)
                continue
            log('plan', release_m=[round(float(v), 4) for v in rel], cap_v_mps=[round(e['v'], 4), 0.0],
                v_prior_mmps=round(model['v'] * 1000, 1), prior=e['prior'], n=e['n'],
                x_last_m=round(e['x_last'], 4), release_in_s=round(e['t_arrive'] - now, 2), move_s=round(t_move, 2),
                t_release=round(e['t_arrive'], 3), drop_delay_s=args.drop_delay_s,
                aim_x_m=round(aim_x, 4), x_offset_m=args.x_offset_m,
                cap_y_m=round(model['y'], 4), waited=waited)
            return rel, e['t_arrive'], t_move, model
        log('no_plan', waited=waited)
        return None

    def wait_to_go(rel, t_release_plan, t_move, v_prior, y_line):
        """벨트 옆에서 검출로 도착 시각을 고쳐 잡으며, 트리거와 출발 가운데 이른 쪽의 시각을 기다린다.

        반환 (도착 예상 시각, 마지막 추정). 트리거 = 도착 − drop_delay, 출발 = 트리거 − arrive_before_fire − 이동 시간.
        """
        est, t_rel = None, t_release_plan
        lead = max(args.drop_delay_s, t_move + arrive_before_s)
        while True:
            now = now_s()
            e = arrival_estimate(cap_points(), aim_x, y_line, now, v_prior=v_prior)
            if e is not None:
                est, t_rel = e, e['t_arrive']
            if now >= t_rel - lead or now > t_release_plan + 6.0 or stop_requested():
                return t_rel, est
            time.sleep(0.005)

    def timing_check(rel, t_release_plan, t_move, v_prior, y_line):
        """로봇 없이: 트리거 시각에 쓴 도착 예상을 실제 통과 시각과 견준다."""
        t_plan_now = now_s()
        t_rel, est = wait_to_go(rel, t_release_plan, t_move, v_prior, y_line)
        while now_s() < t_rel - args.drop_delay_s:
            e = arrival_estimate(cap_points(), aim_x, y_line, now_s(), v_prior=v_prior)
            if e is not None:
                est, t_rel = e, e['t_arrive']
            time.sleep(0.005)
        t_fire = now_s()
        while now_s() < max(t_release_plan, t_rel) + 2.5:
            time.sleep(0.05)
        seen = [(c[0], c[1], c[2]) for c in cap_points()
                if c[0] >= t_plan_now - 1.0 and abs(c[2] - y_line) < 0.02]
        cross = crossing(seen, aim_x)
        out = {'fire_before_est_arrival_s': round(t_rel - t_fire, 3),
               'v_mmps': None if est is None else round(est['v'] * 1000, 1),
               'x_at_fire_m': None if est is None else round(est['x_last'], 4)}
        if cross is not None:
            out.update(plan_minus_actual_s=round(t_release_plan - cross[0], 3),
                       fire_est_minus_actual_s=round(t_rel - cross[0], 3),
                       y_at_cross_minus_rel_mm=round((cross[1] - rel[1]) * 1000, 1))
        log('timing_check', **out)

    def release(rel, t_release_plan, t_move, start_pos, v_prior, y_line):
        """개방 예약 → 벨트 옆에서 관측 → 해제 지점으로 출발 → (도착 − drop_delay)에 트리거.

        도착 예상은 팔이 해제 지점 block_dist 안으로 들어오기 전까지만 고친다(그 뒤의 검출은 그리퍼에 가려 중심이
        밀린다). 이동은 sync_type 1(비동기)로 보낸다: 블로킹 move_line 서비스는 드라이버의 레지스터 쓰기를 이동이
        끝날 때까지 막아 트리거가 0.4 s 늦었다.
        반환 ('fired'|'skipped'|'stopped', 팔 위치, 트리거 시각). 출발한 뒤에는 중지 요청이 있어도 놓는다.
        """
        if not args.rehearse:        # 쥔 전류 그대로 예약한다(전류를 바꾸면 루프가 Modbus 쓰기를 한 번 더 한다)
            loop.write_register(MODBUS.goal_current, p.grip_current_ma)
            loop.write_goal_position(args.release_open_pulse)
        t_rel, est = wait_to_go(rel, t_release_plan, t_move, v_prior, y_line)
        now = now_s()
        if stop_requested():                         # 아직 출발 전: 놓지 않는다
            log('stop_seen', where='before_approach')
            return 'stopped', start_pos, None
        if t_rel - now < max(args.drop_delay_s, t_move + arrive_before_s) - 0.15 or now > t_release_plan + 6.0:
            log('skip', reason='TOO_LATE', arrive_in_s=round(t_rel - now, 2))
            return 'skipped', start_pos, None
        target = np.asarray(rel, float)
        vel = dc.speed_si_to_doosan([p.travel_v_mps, 0.0, 0.0], [0.35, 0.0, 0.0])
        acc = dc.speed_si_to_doosan([p.travel_a_mps2, 0.0, 0.0], [0.7, 0.0, 0.0])
        reached = np.asarray(start_pos, float)
        dist = float(np.linalg.norm(target - reached))
        fired, t_start, arrived, last, still_since, t_block = None, None, False, None, None, None
        while True:
            now = now_s()
            if fired is None and t_block is None:
                e = arrival_estimate(cap_points(), aim_x, y_line, now, v_prior=v_prior)
                if e is not None:
                    est, t_rel = e, e['t_arrive']
            t_fire = t_rel - args.drop_delay_s
            t_go = t_rel - arrive_before_s - t_move
            if fired is None and now >= t_fire - 0.003:
                t0 = time.monotonic()
                if not args.rehearse:
                    loop.trigger()
                fired = {'mono': time.monotonic(), 'ros': now_s(), 'call_s': round(time.monotonic() - t0, 3),
                         't_fire': t_fire, 'dist_mm': round(dist * 1000, 1)}
                continue
            if t_start is None and now >= t_go - 0.003:
                robot._call(MoveLine, 'motion/move_line', pos=dc.pose_si_to_doosan(target, DOWN_QUAT_XYZW),
                            vel=[vel[0], vel[3]], acc=[acc[0], acc[3]], time=0.0, radius=0.0, ref=0, mode=0,
                            blend_type=0, sync_type=1)
                t_start = time.monotonic()
                log('go', arrive_in_s=round(t_rel - now_s(), 3), fired=fired is not None,
                    v_est_mmps=None if est is None else round(est['v'] * 1000, 1))
                continue
            if fired is None and 0.0 < t_fire - now < 0.05:
                time.sleep(0.002)                   # 트리거 직전에는 다른 서비스를 부르지 않는다
                continue
            if t_start is not None and not arrived:
                reached, _ = robot.tcp_pose()
                dist = float(np.linalg.norm(reached - target))
                if t_block is None and dist <= args.block_dist_m:
                    t_block = now_s()               # 이 뒤의 검출은 그리퍼에 가려 중심이 밀린다
                if dist <= 0.0015 and time.monotonic() - t_start > 0.3:
                    arrived = True
                    log('done', step='to_release', reached_m=[round(float(x), 4) for x in reached],
                        move_s=round(time.monotonic() - t_start, 2), before_cap_s=round(t_rel - now_s(), 3))
                else:
                    moved = last is None or float(np.linalg.norm(reached - last)) > 0.0002
                    last = reached
                    still_since = None if moved else (still_since or time.monotonic())
                    stuck = (still_since is not None and time.monotonic() - still_since > 0.4
                             and time.monotonic() - t_start > 0.8)
                    if stuck or time.monotonic() > t_start + t_move + 1.5:
                        arrived = True
                        log('not_reached', step='to_release', target_m=[round(float(x), 4) for x in target],
                            reached_m=[round(float(x), 4) for x in reached])
            if arrived and fired is not None:
                break
            time.sleep(0.01)
        log('fired', t_ros=round(fired['ros'], 3), late_s=round(fired['ros'] - fired['t_fire'], 3),
            call_s=fired['call_s'], arm_dist_at_fire_mm=fired['dist_mm'], t_release_est=round(t_rel, 3),
            t_release_plan=round(t_release_plan, 3), est_minus_plan_s=round(t_rel - t_release_plan, 3),
            v_est_mmps=None if est is None else round(est['v'] * 1000, 1),
            est_n=None if est is None else est['n'], est_prior=None if est is None else est['prior'],
            est_age_s=None if est is None else round(fired['ros'] - est['t_last'], 2),
            x_at_fire_m=None if est is None else round(est['x_last'], 4), drop_delay_s=args.drop_delay_s,
            open_pulse=args.release_open_pulse, y_offset_m=args.y_offset_m, x_offset_m=args.x_offset_m,
            aim_x_m=round(aim_x, 4),
            t_block=None if t_block is None else round(t_block, 3), rehearse=args.rehearse)
        # 높이를 유지한 채 돌이 떨어질 때까지 기다린 뒤 올린다(열리는 중에 올리면 돌을 끌고 올라간다)
        samples = []
        while time.monotonic() - fired['mono'] < args.drop_delay_s + args.hold_after_s:
            if args.rehearse:
                time.sleep(0.05)
                continue
            samples.append((round(time.monotonic() - fired['mono'], 2), loop._word(regs.st_pos_hi)))
            time.sleep(0.05)
        if not args.rehearse:
            result = loop.last_command()
            log('opened', pulse=loop._word(regs.st_pos_hi), cmd=result,
                pulse_trace=[sm for k, sm in enumerate(samples) if k == 0 or sm[1] != samples[k - 1][1]])
            if result and result[0] and result[1] == 242:      # 해제 쓰기 실패: 늦더라도 다시 연다
                pos, _ = gripper_to(args.release_open_pulse, p.grip_current_ma)
                log('gripper', step='open_retry', pulse=pos)
        return 'fired', reached, fired['ros']

    results = []

    def stop_requested() -> bool:
        return bool(args.stop_file) and os.path.exists(args.stop_file)

    def judge_and_log(index, t_fire, rel_x):
        with caps_lock:
            flags = list(cap_flags)
        verdict = judge(flags, t_fire, rel_x + 0.02)
        results.append(verdict['result'])
        log('verdict', index=index, **verdict)

    unreachable = {'n': 0}

    def find_stone(geom):
        """매트에서 팔이 닿는 공깃돌 가운데 로봇에 가장 가까운 것을 고른다(지령 없음). 반환 위치 또는 None.

        공깃돌 위·파지 자세의 IK가 실패하는 돌(예: x 0.706, y −0.093 m)은 건너뛴다.
        """
        dets = detect(grab_frame(node, cfg['image_topic']), cfg, geom)
        excluded = [[round(v, 4) for v in d.position] for d in dets if d.position[1] > MAT_Y_MAX_M]
        if excluded:
            log('excluded', reason='Y_ABOVE_MAT', pos_m=excluded)
        cands = sorted((d for d in dets if d.position[1] <= MAT_Y_MAX_M and d.region == REGION_MAT
                        and d.object_class == CLASS_STONE and _usable(d) and _in_workspace(d.position, p)),
                       key=lambda d: float(np.hypot(d.position[0], d.position[1])))
        far = []
        for d in cands:
            pos = np.asarray(d.position, float)
            over = np.array([pos[0], pos[1], p.travel_z_m])
            grasp = np.array([pos[0], pos[1], pos[2] + p.grasp_above_stone_top_m])
            if ik_ok('over_stone', over) and ik_ok('grasp', grasp):
                unreachable['n'] = len(far)
                if far:
                    log('unreachable', pos_m=far)
                log('stone', pos_m=[round(float(v), 4) for v in pos], candidates=len(cands))
                return pos
            far.append([round(float(v), 4) for v in pos])
        unreachable['n'] = len(far)
        if far:
            log('unreachable', pos_m=far)
        return None

    def place_back(spot):
        """쥔 돌을 집었던 매트 자리(집을 때 비었다)에 내려놓고 이동 높이로 올라온다."""
        over = np.array([spot[0], spot[1], p.travel_z_m])
        low = np.array([spot[0], spot[1], spot[2] + p.grasp_above_stone_top_m + 0.007])
        move('place_back_over', over, tolerate=True)
        move('place_back_lower', low, vv, va, tolerate=True)
        pos, done = gripper_to(p.open_pulse, 100)
        t_end = time.monotonic() + 3.0
        while abs(pos - p.open_pulse) > 10 and time.monotonic() < t_end:   # 상태 갱신이 늦다: 열린 것을 본 뒤 올린다
            time.sleep(0.2)
            pos = loop._word(regs.st_pos_hi)
        log('gripper', step='place_back_open', pulse=pos, done=done)
        move('place_back_lift', over, vv, va, tolerate=True)

    def pick(stone, preopen_t0):
        """공깃돌 위 → 하강 → 닫기 → 들기. 반환 'held' | 'empty'. preopen_t0: 이동 전에 이미 보낸 열기 지령의 시각."""
        over = np.array([stone[0], stone[1], p.travel_z_m])                 # IK는 find_stone이 확인했다
        grasp = np.array([stone[0], stone[1], stone[2] + p.grasp_above_stone_top_m])
        if preopen_t0 is None:
            pos, _ = gripper_to(p.open_pulse, 100)
            log('gripper', step='open_before_pick', pulse=pos)
            move('over_stone', over, tolerate=True)          # 먼 공깃돌은 이동 높이에서 몇 mm 덜 간다. 하강이 맞춘다
        else:                                                # 열기는 이동하는 동안 끝난다
            move('over_stone', over, tolerate=True)
            pos, done = gripper_finish(p.open_pulse, preopen_t0, 6.0)
            if (done and done[0] and done[1] == 242) or abs(pos - p.open_pulse) > 15:
                pos, _ = gripper_to(p.open_pulse, 100)
            log('gripper', step='open_before_pick', pulse=pos, during_move=True)
        move('descend_grasp', grasp, vv, va)
        pos, done = gripper_to(p.close_pulse, p.grip_current_ma)
        log('gripper', step='close_grasp', pulse=pos, done=done)
        if pos >= p.close_pulse - 10:
            move('lift_empty', over, vv, va, tolerate=True)
            return 'empty'
        move('lift_stone', over, vv, va, tolerate=True)
        return 'held'

    def ensure_loop():
        """그리퍼 상주 루프가 돌고 있는지 확인하고, 멈춰 있을 때만 띄운다.

        시작 표시(init) 하나만 보고 다시 띄우면 안 된다: start_resident_loop는 표시를 0으로 지운 뒤 DRL 상태를 보므로,
        루프가 도는 중에 표시가 한 번 다르게 읽히면 표시만 지워지고 그 뒤 실행이 모두 DRL_BUSY로 막힌다
        (DRL PLAY·생존 카운터 증가·Mode 5인데 init 0으로 읽힌 적이 있다). 루프가 살아 있으면 표시를 다시 세운다.
        """
        init = drl = None
        for _ in range(3):
            init, drl = io.get(regs.init), robot.status()['drl_state']
            if init == 1 and drl == 0:
                return
            time.sleep(0.3)
        log('loop_check', init=init, drl_state=drl)
        if drl == 0:                                     # DRL이 돌고 있다: 우리 루프인지 생존 카운터로 본다
            a0 = io.get(regs.alive)
            time.sleep(1.8)                              # 상태 갱신 주기 약 0.8 s
            a1 = io.get(regs.alive)
            if a1 != a0 and io.get(regs.mode) == 5 and io.get(regs.error) == 0:
                io.set(regs.init, 1)
                log('loop_flag_restored', alive=[a0, a1])
                return
            raise RuntimeError('DRL_BUSY')
        t0 = time.monotonic()
        loop.start_resident_loop()
        log('loop_started', ready_s=round(time.monotonic() - t0, 2))

    moved = 0
    try:
        # ---------------- 상태·검출(지령 없음)
        for attempt in range(4):            # 새 노드가 드라이버 서비스를 찾기 전 첫 호출이 실패할 수 있다
            try:
                st = robot.status()
                break
            except RobotError as exc:
                if exc.reason != 'SERVICE_UNAVAILABLE' or attempt == 3:
                    raise
        log('ready', status=st)
        if st['robot_state'] != 1 or st['robot_mode'] != 1:
            log('abort', reason='NOT_STANDBY_AUTONOMOUS')
            return 2
        start, _ = robot.tcp_pose()
        wait = np.array([args.rel_x, args.wait_y, p.travel_z_m])
        park = [p.park_x_m, p.park_y_m, p.park_z_m]
        if args.park_only:
            if args.execute:
                if start[2] < p.travel_z_m - 0.003:
                    move('retreat', [float(start[0]), float(start[1]), p.travel_z_m], vv, va, tolerate=True)
                move('park', park, tolerate=True)
            log('summary', moved=0, results=[], parked=bool(args.execute))
            return 0
        geom = load_geometry(cfg)
        target = None if args.count <= 0 else args.count
        holding, stone, preopen_t0, last_fire, grip_fail, held_from = False, None, None, None, 0, None
        if not ik_ok('wait', wait):
            log('abort', reason='IK_OR_WORKSPACE')
            return 2
        if args.rehearse:
            pass
        elif args.holding:
            pos = loop._word(regs.st_pos_hi)
            if io.get(regs.init) != 1 or pos >= p.close_pulse - 10 or pos <= p.open_pulse + 10:
                log('abort', reason='NOT_HOLDING', pulse=pos)
                return 2
            log('holding', pulse=pos)
            holding = True
        else:
            stone = find_stone(geom)
            if stone is None:
                log('abort', reason='STONE_OUT_OF_REACH' if unreachable['n'] else 'NO_MAT_STONE',
                    unreachable=unreachable['n'])
                return 2
            if args.execute:
                ensure_loop()

        while True:
            if stop_requested() and not holding:            # 아무것도 쥐지 않았다: 집지 않고 끝낸다
                log('stop_seen', where='before_pick')
                break
            # ---------------- 파지(--execute)
            if args.execute and stone is not None and not holding:
                held_from = None
                if pick(stone, preopen_t0) == 'empty':
                    preopen_t0 = None
                    grip_fail += 1
                    log('grip_empty', count=grip_fail)
                    start = move('wait', wait, tolerate=True)       # 벨트 옆에서는 팔이 매트를 가리지 않는다
                    stone = None if grip_fail >= 3 else find_stone(geom)
                    if stone is None:
                        break
                    continue
                preopen_t0 = None
                held_from = stone
            holding = False
            start = move('wait', wait) if args.execute else wait
            if last_fire is not None:                               # 앞 돌의 뚜껑은 이미 벨트 끝을 지났다
                judge_and_log(*last_fire)
                last_fire = None
            want_more = (target is None or moved + 1 < target) and not args.rehearse and not stop_requested()
            next_stone = find_stone(geom) if (want_more and args.execute) else None

            # ---------------- 통과 잡기 → 해제(통과마다)
            outcome, t_fire = None, None
            for k in range(max(1, args.passes)):
                chosen = get_pass(np.asarray(start, float), time.monotonic() + args.wait_plan_s)
                if chosen == 'STOP':
                    outcome = 'stopped'
                    break
                if chosen is None:
                    break
                rel, t_release_plan, t_move, model = chosen
                if not args.execute:
                    timing_check(rel, t_release_plan, t_move, model['v'], model['y'])
                    log('dry_run_done')
                    return 0
                outcome, at, t_fire = release(rel, t_release_plan, t_move, start, model['v'], model['y'])
                if outcome in ('fired', 'stopped'):
                    break
                start = move('side_again', wait, tolerate=True)   # 놓지 않았다. 벨트 옆에서 다음 바퀴를 기다린다
            if outcome != 'fired':
                stopped = outcome == 'stopped'
                reason = 'STOP_REQUESTED' if stopped else 'NOT_RELEASED'
                log('abort', reason=reason, outcome=outcome)
                still_holding = args.execute and not args.rehearse
                if still_holding and held_from is not None:      # 쥔 채 끝내지 않는다
                    place_back(held_from)
                    still_holding = False
                if args.execute:
                    move('park_holding' if still_holding else 'park', park, tolerate=True)
                log('summary', moved=moved, results=results, reason=reason, holding=still_holding,
                    placed_back=held_from is not None and args.execute and not args.rehearse)
                return 0 if stopped else 2
            moved += 1
            if not args.rehearse:
                last_fire = (moved, t_fire, float(rel[0]))
            log('released', index=moved)
            if next_stone is None or stop_requested():
                break
            preopen_t0 = gripper_begin(p.open_pulse, 100)           # 다음 돌로 가는 동안 손가락을 집는 폭으로 맞춘다
            stone = next_stone

        # ---------------- 마무리: 올리고 벨트 옆으로 뺀 뒤 판정하고 park
        if args.execute:
            at, _ = robot.tcp_pose()
            if at[2] < p.travel_z_m - 0.003:
                move('retreat', [float(at[0]), float(at[1]), p.travel_z_m], vv, va, tolerate=True)
            if moved:
                move('side', wait, tolerate=True)
        if last_fire is not None:                                   # 뚜껑이 벨트 끝으로 가는 동안 가리지 않고 본다
            while now_s() < last_fire[1] + max(4.5, args.side_watch_s):
                time.sleep(0.05)
            judge_and_log(*last_fire)
        if args.execute:
            move('park', park, tolerate=True)
        reason = 'STOP_REQUESTED' if stop_requested() else ('NO_STONE_IN_GRIP' if grip_fail >= 3 else None)
        log('summary', moved=moved, results=results, reason=reason, unreachable=unreachable['n'])
        return 0 if moved else 2
    except (RobotError, RuntimeError) as exc:
        reason = getattr(exc, 'reason', None) or (str(exc) if str(exc).isupper() else type(exc).__name__)
        log('abort', reason=reason, detail=str(exc))
        holding_now = None
        try:                                 # 끝난 뒤 버튼 창이 쥔 돌이 있는지 알도록 그리퍼 위치를 읽는다
            pos = loop._word(regs.st_pos_hi)
            holding_now = p.open_pulse + 10 < pos < p.close_pulse - 10
        except Exception:                    # noqa: BLE001 - 읽지 못하면 모른다고 남긴다
            pass
        log('summary', moved=moved, results=results, reason=reason, holding=holding_now)
        return 2
    finally:
        try:
            watch_exec.shutdown(timeout_sec=1.0)
            watch_node.destroy_node()
        except Exception:               # noqa: BLE001
            pass
        caller.close()
        node.destroy_node()
        rclpy.shutdown()
        if recorder is not None:
            import signal
            time.sleep(1.0)
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=10)
            except Exception:           # noqa: BLE001 - 녹화가 안 끝나면 강제로 닫는다
                recorder.kill()


if __name__ == '__main__':
    sys.exit(main())
