"""상주 루프 DRL(`drl/gripper_resident.drl`)과 제어기 레지스터로 대화하는 그리퍼 transport(REQ-GRP-016).

PC는 그리퍼와 직접 통신하지 않는다. 명령 레지스터에 쓰고, 루프가 올려 둔 상태 레지스터를 읽는다.
`GripperAdapter`가 요구하는 transport 계약(읽기·쓰기·트리거·루프 기동·생존 카운터)을 만족한다.
실기 미검증: 레지스터 방향·값 해석과 루프 안 왕복 시간은 OFF-2에서 확인한다.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

from robot_control.drl_common import expand
from robot_control.gripper_adapter import MODBUS, MODEL_NUMBER_RH_P12_RN_A, GripperFault, encode7, next_seq

_DRL = Path(__file__).with_name('drl') / 'gripper_resident.drl'
INIT_OK = 1
CMD_OK = 0


@dataclass(frozen=True)
class ResidentRegs:
    # PC → DRL (0~127)
    seq: int = 10
    pos_hi: int = 11
    cur_hi: int = 13
    arm: int = 24
    stop: int = 25
    # DRL → PC (0~255)
    alive: int = 30
    st_pos_hi: int = 31
    st_cur_hi: int = 33
    error: int = 35
    init: int = 36
    mode: int = 37
    firmware: int = 38
    done_seq: int = 39
    cmd_result: int = 40             # 실행한 seq의 결과: 0 정상, 242 쓰기 실패(39와 함께 갱신)


class RegisterIO(Protocol):
    """제어기 출력 레지스터와 DRL 기동. 값은 이미 바이트로 해석된 정수다."""

    def get(self, address: int) -> int: ...
    def set(self, address: int, value: int) -> bool: ...
    def drl_start(self, code: str) -> bool: ...


def render_resident_drl(read_timeout_s: float = 0.5, spin_s: float = 0.005, arm_timeout_s: float = 10.0) -> str:
    if not 0.01 <= read_timeout_s <= 0.5:
        raise ValueError('read_timeout_s must be in 0.01..0.5 s')
    if not 0.001 <= spin_s <= 0.05:
        raise ValueError('spin_s must be in 0.001..0.05 s')
    if not 1.0 <= arm_timeout_s <= 60.0:
        raise ValueError('arm_timeout_s must be in 1..60 s')
    return (expand(_DRL.read_text(encoding='utf-8')).replace('__READ_TIMEOUT__', repr(float(read_timeout_s)))
            .replace('__SPIN__', repr(float(spin_s))).replace('__ARM_TIMEOUT__', repr(float(arm_timeout_s))))


class ResidentLoopTransport:
    def __init__(self, io: RegisterIO, clock, sleep: Callable[[float], None],
                 regs: ResidentRegs = ResidentRegs(), ready_timeout_s: float = 10.0,
                 drl_code: str | None = None, sample_period_s: float = 0.58):
        self.io = io
        self.clock = clock
        self._sleep = sleep
        self.regs = regs
        self.ready_timeout_s = float(ready_timeout_s)
        self._drl_code = drl_code
        # 루프가 상태를 새로 올리는 주기(Modbus 왕복 1회). 제안값 0.58 s는 일회성 DRL 왕복 측정값(2026-11-09)이다.
        # GripperAdapter는 이 값으로 응답·생존 제한시간이 판정 가능한지 생성 때 검사한다.
        if not sample_period_s > 0.0:
            raise ValueError('sample_period_s must be positive')
        self.sample_period_s = float(sample_period_s)
        self._goal_current: int | None = None
        self._last_seq: int | None = None   # 마지막으로 트리거한 seq

    # ------------------------------------------------------------ 기동·종료
    def start_resident_loop(self) -> bool:
        """DRL을 띄우고 시작 상태가 올라올 때까지 기다린다. 정상(1)이 아니면 예외."""
        code = self._drl_code if self._drl_code is not None else render_resident_drl()
        self._last_seq = None             # 새 루프는 기동 전 seq를 실행한 것으로 표시하므로 지난 지령을 잊는다
        if not self.io.drl_start(code):
            return False
        deadline = self.clock.steady_now() + self.ready_timeout_s
        while self.clock.steady_now() <= deadline:
            status = int(self.io.get(self.regs.init))
            if status == INIT_OK:
                return True
            if status != 0:
                raise GripperFault('RESIDENT_INIT', detail=f'status={status}')
            self._sleep(0.05)
        raise GripperFault('RESIDENT_INIT_TIMEOUT', detail=f'{self.ready_timeout_s} s')

    def stop_resident_loop(self) -> bool:
        return bool(self.io.set(self.regs.stop, 1))

    # ------------------------------------------------------------ 읽기
    def read_register(self, address: int, count: int = 1):
        r = self.regs
        if address == MODBUS.model_number:
            # 모델 번호는 루프가 직접 대조한다. 시작 상태가 정상일 때만 기대값을 돌려준다
            return MODEL_NUMBER_RH_P12_RN_A if int(self.io.get(r.init)) == INIT_OK else 0
        if address == MODBUS.firmware_version:
            return int(self.io.get(r.firmware))
        if address == MODBUS.operating_mode:
            return int(self.io.get(r.mode))
        if address == MODBUS.hardware_error_status:
            return int(self.io.get(r.error))
        if address == MODBUS.present_position:
            return self._word(r.st_pos_hi)
        if address == MODBUS.present_current:
            return self._word(r.st_cur_hi)
        if address == MODBUS.goal_current:
            return self._goal_current if self._goal_current is not None else 0
        raise ValueError(f'상주 루프가 올리지 않는 레지스터: {address}')

    def read_loop_counter(self) -> int:
        return int(self.io.get(self.regs.alive))

    def last_command(self) -> tuple[bool, int] | None:
        """마지막으로 트리거한 지령의 (실행됨, 결과). 트리거한 적이 없으면 None.

        결과는 루프가 실행한 seq(39)가 이 지령의 seq일 때만 의미가 있다(40은 39보다 먼저 쓰인다).
        """
        if self._last_seq is None:
            return None
        if int(self.io.get(self.regs.done_seq)) != self._last_seq:
            return False, CMD_OK
        return True, int(self.io.get(self.regs.cmd_result))

    # ------------------------------------------------------------ 쓰기
    def write_register(self, address: int, value: int) -> bool:
        if address != MODBUS.goal_current:
            return False                      # EEPROM·토크 등은 이 경로로 쓰지 않는다
        ok = self._set7(self.regs.cur_hi, int(value))
        if ok:
            self._goal_current = int(value)
        return ok

    def write_goal_position(self, pulse: int) -> bool:
        """목표 위치를 쓰고 예약 표시를 올린다. 루프는 Modbus 읽기를 멈추고 트리거만 기다린다."""
        return self._set7(self.regs.pos_hi, int(pulse)) and bool(self.io.set(self.regs.arm, 1))

    def trigger(self) -> bool:
        seq = next_seq(int(self.io.get(self.regs.seq)))
        ok = bool(self.io.set(self.regs.seq, seq))
        if ok:
            self._last_seq = seq
        return ok

    # ------------------------------------------------------------ 내부
    def _word(self, hi_address: int) -> int:
        return (int(self.io.get(hi_address)) & 255) * 256 + (int(self.io.get(hi_address + 1)) & 255)

    def _set7(self, hi_address: int, value: int) -> bool:
        hi, lo = encode7(value)
        return bool(self.io.set(hi_address, hi)) and bool(self.io.set(hi_address + 1, lo))
