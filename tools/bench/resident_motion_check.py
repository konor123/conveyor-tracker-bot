#!/usr/bin/env python3
"""그리퍼 상주 루프와 팔 이동 동시 동작 확인(실제 로봇, 입회 필수).

프로젝트 client(`DsrRegisterIO`·`ResidentLoopTransport`·`DsrMotionClient`)를 그대로 써서 한 단계씩 확인한다.
단계마다 따로 실행하고, 팔·그리퍼가 움직이는 단계는 실행 전에 사람이 승인한다.

  status       읽기만: robot·DRL 상태, TCP, 상주 루프 레지스터 30~40
  loop-start   상주 루프 DRL 기동(그리퍼 토크가 꺼져 있으면 켠다, 팔 지령 없음) → 시작 상태·생존 카운터 3 s
  grip-hold    지금 그리퍼 위치를 그대로 목표로(전류 100 mA) 써서 명령 경로만 확인(손가락은 움직이지 않아야 한다)
  grip-move    --pulse로 그리퍼만 옮기고 멈출 때까지 기다린다(0~750, 100 mA, 팔 지령 없음). 실측표용
  move         루프가 도는 동안 팔을 z로 --dz-mm(기본 +10, |dz| ≤ 20) 직선 이동했다가 원위치(20 mm/s)
  loop-stop    종료 레지스터(25)를 써서 루프를 끝내고 DRL이 STOP·LAST로 돌아오는지 본다

멈추는 법: move 중에는 비상정지·펜던트. 루프가 끝나지 않으면 펜던트에서 프로그램을 멈춘다.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

DZ_MAX_MM = 20.0
PULSE_WORK_MAX = 750       # 현장 운용 범위
MOVE_V = [20.0, 10.0]          # mm/s, deg/s
MOVE_A = [50.0, 20.0]          # mm/s², deg/s²


def log(event: str, **fields) -> None:
    print(json.dumps({'t': round(time.time(), 3), 'event': event, **fields}, ensure_ascii=False, default=str),
          flush=True)


def offset_z(posx, dz_mm: float) -> list[float]:
    """Doosan posx [mm×3, deg×3]의 z만 dz_mm 옮긴다. |dz| ≤ DZ_MAX_MM."""
    if not abs(dz_mm) <= DZ_MAX_MM:
        raise ValueError(f'|dz| must be <= {DZ_MAX_MM} mm: {dz_mm}')
    out = [float(v) for v in posx[:6]]
    out[2] += float(dz_mm)
    return out


class SteadyClock:
    def steady_now(self) -> float:
        return time.monotonic()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('step', choices=['status', 'loop-start', 'grip-hold', 'grip-move', 'move', 'loop-stop'])
    ap.add_argument('--pulse', type=int, help='grip-move 목표 pulse(0~750, 현장 운용 범위)')
    ap.add_argument('--dz-mm', type=float, default=10.0)
    ap.add_argument('--ns', default='dsr01')
    args = ap.parse_args(argv)

    import rclpy
    from robot_control.dsr_motion_client import DsrMotionClient
    from robot_control.gripper_adapter import CURRENT_DEFAULT_MA, MODBUS
    from robot_control.dsr_register_io import DsrRegisterIO, RosServiceCaller
    from robot_control.resident_transport import ResidentLoopTransport, ResidentRegs

    rclpy.init()
    caller = RosServiceCaller('resident_motion_check')
    io = DsrRegisterIO(caller, args.ns)
    arm = DsrMotionClient(caller, args.ns)
    regs = ResidentRegs()
    loop = ResidentLoopTransport(io, SteadyClock(), time.sleep, regs)

    def snapshot():
        return {'robot_state': arm.robot_state(), 'drl_state': arm.drl_state(),
                'posx': [round(v, 2) for v in arm.current_posx()]}

    def loop_regs():
        names = {'alive': regs.alive, 'pos': None, 'error': regs.error, 'init': regs.init, 'mode': regs.mode,
                 'firmware': regs.firmware, 'done_seq': regs.done_seq, 'cmd_result': regs.cmd_result}
        out = {k: io.get(a) for k, a in names.items() if a is not None}
        out['pos'] = loop._word(regs.st_pos_hi)
        out['cur'] = loop._word(regs.st_cur_hi)
        return out

    try:
        # 새 helper 노드가 드라이버 서비스를 발견하기 전에는 1 s 호출 제한에 걸린다. 첫 호출 전 발견을 기다린다
        from robot_control.dsr_register_io import DsrError
        t_find = time.monotonic()
        while True:
            try:
                arm.robot_state()
                break
            except DsrError as exc:
                if exc.reason != 'SERVICE_UNAVAILABLE' or time.monotonic() - t_find > 10.0:
                    raise
        log('services_found', after_s=round(time.monotonic() - t_find, 2))
        if args.step == 'status':
            log('status', **snapshot(), regs=loop_regs())
        elif args.step == 'loop-start':
            log('before', **snapshot())
            t0 = time.monotonic()
            ok = loop.start_resident_loop()
            log('loop_started', ok=ok, ready_s=round(time.monotonic() - t0, 2), **snapshot())
            samples = []
            for _ in range(7):
                samples.append((round(time.monotonic() - t0, 2), io.get(regs.alive)))
                time.sleep(0.5)
            log('alive', samples=samples, regs=loop_regs(), drl_state=arm.drl_state())
        elif args.step == 'grip-hold':
            before = loop_regs()
            if before['init'] != 1:
                log('abort', reason='LOOP_NOT_RUNNING', regs=before)
                return 2
            pos = before['pos']
            t0 = time.monotonic()
            # 전류를 쓰지 않으면 루프가 0 mA를 그리퍼에 쓴다. 기본 전류(100 mA)를 함께 쓴다
            ok = (loop.write_register(MODBUS.goal_current, CURRENT_DEFAULT_MA) and loop.write_goal_position(pos)
                  and loop.trigger())
            seq = loop._last_seq
            done = None
            while time.monotonic() - t0 < 5.0:
                done = loop.last_command()
                if done and done[0]:
                    break
                time.sleep(0.02)
            log('grip_hold', wrote_pulse=pos, wrote_current_ma=CURRENT_DEFAULT_MA, ok=ok, seq=seq, done=done, done_s=round(time.monotonic() - t0, 3),
                before=before, after=loop_regs())
        elif args.step == 'grip-move':
            if args.pulse is None or not 0 <= args.pulse <= PULSE_WORK_MAX:
                log('abort', reason='BAD_PULSE', pulse=args.pulse, allowed=[0, PULSE_WORK_MAX])
                return 2
            before = loop_regs()
            if before['init'] != 1:
                log('abort', reason='LOOP_NOT_RUNNING', regs=before)
                return 2
            t0 = time.monotonic()
            ok = (loop.write_register(MODBUS.goal_current, CURRENT_DEFAULT_MA) and loop.write_goal_position(args.pulse)
                  and loop.trigger())
            positions = []
            while time.monotonic() - t0 < 8.0:
                done = loop.last_command()
                pos = loop._word(regs.st_pos_hi)
                positions.append((round(time.monotonic() - t0, 2), pos))
                # 예약·실행 중에는 상태가 갱신되지 않아 지난 위치가 보인다(2026-11-14 현장). 목표 근처면 끝,
                # 아니면(공깃돌·맞닿음으로 막힘) 실행 뒤 3 s 지나고 마지막 네 표본(약 1 s)이 같을 때 끝
                if done and done[0] and (abs(pos - args.pulse) <= 5 or (
                        time.monotonic() - t0 > 3.0 and len({p for _, p in positions[-4:]}) == 1)):
                    break
                time.sleep(0.25)
            log('grip_move', cmd_pulse=args.pulse, ok=ok, done=loop.last_command(), positions=positions,
                after=loop_regs())
        elif args.step == 'move':
            start = snapshot()
            log('before', **start)
            target = offset_z(start['posx'], args.dz_mm)
            for name, goal in (('out', target), ('back', start['posx'])):
                t0 = time.monotonic()
                accepted = arm.move_line_async(goal, MOVE_V, MOVE_A)
                trace = []
                while time.monotonic() - t0 < 4.0:
                    s = snapshot()
                    trace.append((round(time.monotonic() - t0, 2), s['robot_state'], round(s['posx'][2], 2)))
                    if abs(s['posx'][2] - goal[2]) < 0.3 and s['robot_state'] == 1 and time.monotonic() - t0 > 0.3:
                        break
                    time.sleep(0.1)
                log('move', leg=name, accepted=accepted, goal_z=round(goal[2], 2), trace=trace)
            log('after', **snapshot(), regs=loop_regs())
        elif args.step == 'loop-stop':
            ok = loop.stop_resident_loop()
            t0 = time.monotonic()
            states = []
            while time.monotonic() - t0 < 5.0:
                d = arm.drl_state()
                states.append((round(time.monotonic() - t0, 2), d))
                if d in (1, 3):
                    break
                time.sleep(0.2)
            log('loop_stop', wrote=ok, drl_states=states, **snapshot())
        return 0
    except Exception as exc:          # noqa: BLE001 - 현장 도구: 무엇이 실패했는지 남기고 끝낸다
        log('abort', reason=type(exc).__name__, detail=str(exc))
        return 2
    finally:
        caller.close()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
