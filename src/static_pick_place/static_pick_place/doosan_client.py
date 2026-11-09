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
DRL_FAILURES = {241: 'GRIPPER_NO_RESPONSE', 242: 'GRIPPER_WRITE_FAILED', 244: 'GRIPPER_HARDWARE_ERROR',
                245: 'GRIPPER_NOT_SETTLED', 246: 'GRIPPER_MODEL_MISMATCH', 247: 'GRIPPER_MODE_NOT_5'}
_DRL = Path(__file__).with_name('drl') / 'gripper_move.drl'


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
