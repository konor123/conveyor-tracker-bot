"""Doosan E0509(dsr_bringup2 드라이버) 서비스와 그리퍼 DRL을 부르는 얇은 클라이언트.

단위 변환은 robot_control.motion_adapter에서만 한다. 그리퍼 DRL은 drl/gripper_move.drl을 채워 drl_start로 보낸다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from robot_control.motion_adapter import (joints_doosan_to_si, pose_doosan_to_si, pose_si_to_doosan,
                                          speed_si_to_doosan)

ROBOT_STATE_STANDBY = 1
ROBOT_MODE_AUTONOMOUS = 1
DRL_STATE_PLAY, DRL_STATE_STOP, DRL_STATE_LAST = 0, 1, 3
DRL_DONE = 250
DRL_START_GRACE_S = 2.0          # drl_start 뒤 PLAY가 보여야 하는 시간
DRL_FAILURES = {241: 'GRIPPER_NO_RESPONSE', 242: 'GRIPPER_WRITE_FAILED', 244: 'GRIPPER_HARDWARE_ERROR',
                245: 'GRIPPER_NOT_SETTLED', 246: 'GRIPPER_MODEL_MISMATCH', 247: 'GRIPPER_MODE_NOT_5',
                248: 'GRASP_EMPTY', 249: 'MOVE_FAILED'}
_DRL = Path(__file__).with_name('drl') / 'gripper_move.drl'
_CYCLE_DRL = Path(__file__).with_name('drl') / 'cycle.drl'


class RobotError(RuntimeError):
    def __init__(self, reason: str, detail: str = ''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason


@dataclass(frozen=True)
class GripperOutcome:
    target_pulse: int
    position_pulse: int
    current_ma: int
    status: int                      # 1 목표 도달, 2 막혀 멈춤(파지)


def render_gripper_drl(pulse: int, current_ma: int, grasp: bool, read_timeout_s: float = 0.5) -> str:
    if type(pulse) is not int or not 0 <= pulse <= 750:
        raise ValueError('pulse must be an integer in 0..750 (현장 닫힘 끝점 750)')
    if type(current_ma) is not int or not 1 <= current_ma <= 150:
        raise ValueError('current must be an integer in 1..150 mA (2026-11-09 사용자 승인 상한)')
    if not 0.01 <= read_timeout_s <= 0.5:
        raise ValueError('read_timeout_s must be in 0.01..0.5 s')
    return (_DRL.read_text().replace('__PULSE__', str(pulse)).replace('__CURRENT__', str(current_ma))
            .replace('__GRASP__', 'True' if grasp else 'False').replace('__READ_TIMEOUT__', repr(float(read_timeout_s))))


@dataclass(frozen=True)
class CycleOutcome:
    grasp_pulse: int
    grasp_current_ma: int
    grasp_status: int              # 0 = 이번 실행에서 파지 없음, 1 = 끝점 도달(빈 손), 2 = 막힘(쥠)
    open_pulse: int
    open_status: int               # 0 = 이번 실행에서 열기 없음, 1 = 도달, 3 = 이미 그 위치라 생략
    step_times_s: tuple            # (단계 이름, 그 단계가 시작된 뒤 다음 단계 시작까지 PC에서 본 시간)


def render_cycle_drl(steps, quat_xyzw, read_timeout_s: float = 0.5, ang_v_radps: float = 0.35,
                     ang_a_radps2: float = 0.7) -> str:
    """계획 단계(`plan.Step`) 전체를 DRL 프로그램 하나로 만든다. 값 검사는 일회성 그리퍼 DRL과 같다."""
    if not 0.01 <= read_timeout_s <= 0.5:
        raise ValueError('read_timeout_s must be in 0.01..0.5 s')
    if not steps:
        raise ValueError('steps is empty')
    rows = []
    for s in steps:
        if s.kind == 'move':
            pose = pose_si_to_doosan(s.pos_m, quat_xyzw)
            if not 0.0 < s.v_mps <= 0.25 or not 0.0 < s.a_mps2 <= 1.0:
                raise ValueError(f'{s.name}: v must be in (0, 0.25] m/s, a in (0, 1.0] m/s²')
            v = speed_si_to_doosan([s.v_mps, 0.0, 0.0], [0.0, 0.0, 0.0])[0]
            a = speed_si_to_doosan([s.a_mps2, 0.0, 0.0], [0.0, 0.0, 0.0])[0]
            rows.append('[0, ' + ', '.join(f'{x:.3f}' for x in [*pose, v, a]) + ']')
        elif s.kind == 'gripper':
            if type(s.pulse) is not int or not 0 <= s.pulse <= 750:
                raise ValueError('pulse must be an integer in 0..750 (현장 닫힘 끝점 750)')
            if type(s.current_ma) is not int or not 1 <= s.current_ma <= 150:
                raise ValueError('current must be an integer in 1..150 mA (2026-11-09 사용자 승인 상한)')
            rows.append(f'[1, {s.pulse}, {s.current_ma}, {bool(s.grasp)}]')
        else:
            raise ValueError(f'unknown step kind: {s.kind}')
    ang = speed_si_to_doosan([0.0, 0.0, 0.0], [ang_v_radps, ang_a_radps2, 0.0])
    return (_CYCLE_DRL.read_text(encoding='utf-8').replace('__STEPS__', '[' + ', '.join(rows) + ']')
            .replace('__READ_TIMEOUT__', repr(float(read_timeout_s)))
            .replace('__ANG_V__', f'{ang[3]:.3f}').replace('__ANG_A__', f'{ang[4]:.3f}'))


def decode_register_byte(value: int) -> int:
    """제어기 출력 레지스터 값 → DRL이 쓴 바이트(tools/bench/drl/gripper_timing_run.py와 같은 해석)."""
    return (int(value) >> 24) & 255


class DoosanClient:
    def __init__(self, node, namespace: str = 'dsr01', call_timeout_s: float = 5.0):
        self.node = node
        self.ns = namespace
        self.call_timeout_s = call_timeout_s
        self._clients = {}

    # ------------------------------------------------------------ 서비스 호출
    def _call(self, srv_type, name: str, timeout_s: float | None = None, **fields):
        import rclpy
        key = (srv_type, name)
        if key not in self._clients:
            client = self.node.create_client(srv_type, f'/{self.ns}/{name}')
            if not client.wait_for_service(3.0):
                raise RobotError('SERVICE_UNAVAILABLE', name)
            self._clients[key] = client
        future = self._clients[key].call_async(srv_type.Request(**fields))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s or self.call_timeout_s)
        if not future.done() or future.result() is None:
            raise RobotError('SERVICE_TIMEOUT', name)
        res = future.result()
        if hasattr(res, 'success') and not res.success:
            raise RobotError('SERVICE_FAILED', name)
        return res

    # ------------------------------------------------------------ 상태
    def status(self) -> dict:
        from dsr_msgs2.srv import GetDrlState, GetLastAlarm, GetRobotMode, GetRobotState
        alarm = self._call(GetLastAlarm, 'system/get_last_alarm').log_alarm
        return {'robot_state': self._call(GetRobotState, 'system/get_robot_state').robot_state,
                'robot_mode': self._call(GetRobotMode, 'system/get_robot_mode').robot_mode,
                'drl_state': self._call(GetDrlState, 'drl/get_drl_state').drl_state,
                'alarm': (alarm.level, alarm.group, alarm.index, list(alarm.param))}

    def check_ready(self) -> dict:
        """STANDBY·AUTONOMOUS(DRL 실행 조건)·다른 DRL 없음. 지난 알람은 기록만 하고 막지 않는다(11-09 1206 사례)."""
        st = self.status()
        if st['robot_state'] != ROBOT_STATE_STANDBY:
            raise RobotError('NOT_STANDBY', str(st))
        if st['robot_mode'] != ROBOT_MODE_AUTONOMOUS:
            raise RobotError('NOT_AUTONOMOUS', '수동 모드에서는 DRL(그리퍼)이 거부된다. 펜던트/set_robot_mode 1')
        if st['drl_state'] not in (DRL_STATE_STOP, DRL_STATE_LAST):
            raise RobotError('DRL_BUSY', str(st))
        return st

    def tcp_pose(self) -> tuple[np.ndarray, np.ndarray]:
        from dsr_msgs2.srv import GetCurrentPosx
        res = self._call(GetCurrentPosx, 'aux_control/get_current_posx', ref=0)
        return pose_doosan_to_si(list(res.task_pos_info[0].data)[:6])

    def ik_joints(self, pos_m, quat_xyzw, sol_space: int = 2) -> np.ndarray:
        """역기구학 [rad×6]. 도달 경계에서는 J3가 0으로 붙는다."""
        from dsr_msgs2.srv import Ikin
        res = self._call(Ikin, 'motion/ikin', pos=pose_si_to_doosan(pos_m, quat_xyzw), sol_space=sol_space, ref=0)
        return joints_doosan_to_si(list(res.conv_posj))

    # ------------------------------------------------------------ 동작
    def move_line(self, pos_m, quat_xyzw, v_mps: float, a_mps2: float, ang_v_radps: float = 0.35,
                  ang_a_radps2: float = 0.7, tol_m: float = 0.001) -> np.ndarray:
        """절대 직선 이동(base). 끝난 뒤 실제 위치가 tol_m 안인지 확인하고 실제 위치를 돌려준다."""
        from dsr_msgs2.srv import MoveLine
        start, _ = self.tcp_pose()
        target = np.asarray(pos_m, float)
        vel = speed_si_to_doosan([v_mps, 0.0, 0.0], [ang_v_radps, 0.0, 0.0])
        acc = speed_si_to_doosan([a_mps2, 0.0, 0.0], [ang_a_radps2, 0.0, 0.0])
        duration = float(np.linalg.norm(target - start)) / max(v_mps, 1e-6)
        self._call(MoveLine, 'motion/move_line', timeout_s=duration + 15.0,
                   pos=pose_si_to_doosan(target, quat_xyzw), vel=[vel[0], vel[3]], acc=[acc[0], acc[3]],
                   time=0.0, radius=0.0, ref=0, mode=0, blend_type=0, sync_type=0)
        reached, _ = self.tcp_pose()
        if float(np.linalg.norm(reached - target)) > tol_m:
            raise RobotError('MOVE_NOT_REACHED', f'목표 {np.round(target, 4)} 실제 {np.round(reached, 4)}')
        return reached

    def gripper(self, pulse: int, current_ma: int, grasp: bool = False, timeout_s: float = 15.0,
                read_timeout_s: float = 0.5) -> GripperOutcome:
        from dsr_msgs2.srv import DrlStart, GetDrlState, GetOutputRegisterInt
        code = render_gripper_drl(pulse, current_ma, grasp, read_timeout_s)
        state = self._call(GetDrlState, 'drl/get_drl_state').drl_state
        if state not in (DRL_STATE_STOP, DRL_STATE_LAST):
            raise RobotError('DRL_BUSY', f'drl_state={state}')

        def byte(addr):
            return decode_register_byte(self._call(GetOutputRegisterInt, 'plc/get_output_register_int',
                                                   address=addr, timeout_ms=1000).value)

        # 지난 실행의 결과(250 등)를 이번 결과로 읽지 않도록 DRL이 15를 0으로 지운 것을 본 뒤에만 받는다
        reset_seen = byte(15) == 0
        self._call(DrlStart, 'drl/drl_start', robot_system=0, code=code)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            marker = byte(15)
            if marker == 0:
                reset_seen = True
            elif not reset_seen:
                pass
            elif marker == DRL_DONE:
                b = [byte(a) for a in range(16, 24)]
                # 결과를 쓴 직후 DRL은 아직 종료 중(PLAY)이고, 그때 보낸 move_line은 움직이지 않고 끝난다(2026-11-09)
                while self._call(GetDrlState, 'drl/get_drl_state').drl_state not in (DRL_STATE_STOP, DRL_STATE_LAST):
                    if time.monotonic() > deadline:
                        raise RobotError('DRL_NOT_STOPPED', 'gripper DRL finished but program still running')
                    time.sleep(0.05)
                return GripperOutcome(b[0] * 256 + b[1], b[2] * 256 + b[3], b[4] * 256 + b[5], b[6])
            elif marker in DRL_FAILURES:
                raise RobotError(DRL_FAILURES[marker], f'marker={marker}')
            time.sleep(0.05)
        raise RobotError('GRIPPER_DRL_TIMEOUT', f'{timeout_s} s')

    def run_cycle(self, steps, quat_xyzw, timeout_s: float, read_timeout_s: float = 0.5,
                  on_step=None) -> CycleOutcome:
        """사이클 전체를 DRL 하나로 실행한다. 진행 레지스터(14)가 바뀔 때마다 `on_step(이름, 걸린 시간)`을 부른다.

        프로그램이 끝나 STOP이 된 뒤에만 돌아온다. 실패 코드는 `RobotError`로 올리고 팔은 그 단계에서 멈춰 있다.
        """
        from dsr_msgs2.srv import DrlStart, GetDrlState, GetOutputRegisterInt
        code = render_cycle_drl(steps, quat_xyzw, read_timeout_s)
        state = self._call(GetDrlState, 'drl/get_drl_state').drl_state
        if state not in (DRL_STATE_STOP, DRL_STATE_LAST):
            raise RobotError('DRL_BUSY', f'drl_state={state}')

        def byte(addr):
            return decode_register_byte(self._call(GetOutputRegisterInt, 'plc/get_output_register_int',
                                                   address=addr, timeout_ms=1000).value)

        reset_seen = byte(15) == 0
        self._call(DrlStart, 'drl/drl_start', robot_system=0, code=code)
        deadline = time.monotonic() + timeout_s
        times: list[tuple[str, float]] = []
        current, t_step = 0, time.monotonic()

        def advance(to: int) -> None:
            nonlocal current, t_step
            now = time.monotonic()
            while current < to:
                if current >= 1:
                    name = steps[current - 1].name
                    times.append((name, now - t_step))
                    if on_step is not None:
                        on_step(name, now - t_step)
                current += 1
                t_step = now

        started_at = time.monotonic()
        seen_play = stop_seen = False
        while time.monotonic() < deadline:
            marker = byte(15)
            if marker == 0:
                reset_seen = True
            if reset_seen:
                progress = byte(14)
                if 1 <= progress <= len(steps):
                    advance(progress)
                if marker == DRL_DONE:
                    advance(len(steps) + 1)
                    b = [byte(a) for a in range(16, 24)]
                    while self._call(GetDrlState, 'drl/get_drl_state').drl_state not in (DRL_STATE_STOP, DRL_STATE_LAST):
                        if time.monotonic() > deadline:
                            raise RobotError('DRL_NOT_STOPPED', 'cycle DRL finished but program still running')
                        time.sleep(0.05)
                    return CycleOutcome(b[0] * 256 + b[1], b[2] * 256 + b[3], b[4], b[5] * 256 + b[6], b[7],
                                        tuple(times))
                if marker in DRL_FAILURES:
                    step = steps[current - 1].name if 1 <= current <= len(steps) else 'init'
                    raise RobotError(DRL_FAILURES[marker], f'marker={marker} step={step}')
            # 결과 표지 없이 프로그램이 멈추면(예외 정지 등) 제한시간까지 기다리지 않는다.
            # 시작 직후에는 지난 실행의 STOP이 보일 수 있으므로 PLAY를 한 번 본 뒤에만 STOP을 판정한다.
            drl_state = self._call(GetDrlState, 'drl/get_drl_state').drl_state
            if drl_state == DRL_STATE_PLAY:
                seen_play = True
            elif drl_state in (DRL_STATE_STOP, DRL_STATE_LAST):
                if seen_play:
                    if stop_seen:            # 마지막 문장인 결과 쓰기를 한 번 더 읽은 뒤에도 없다
                        step = steps[current - 1].name if 1 <= current <= len(steps) else 'init'
                        raise RobotError('DRL_STOPPED_NO_RESULT', f'marker={marker} step={step}')
                    stop_seen = True
                    continue
                if time.monotonic() - started_at > DRL_START_GRACE_S:
                    raise RobotError('DRL_NOT_STARTED', f'no PLAY within {DRL_START_GRACE_S} s')
            time.sleep(0.05)
        raise RobotError('CYCLE_DRL_TIMEOUT', f'{timeout_s} s')
