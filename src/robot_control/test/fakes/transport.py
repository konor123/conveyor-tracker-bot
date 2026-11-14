"""FakeTransport: RH-P12-RN(A) 제어기·상주 루프 대역.

실제 하드웨어로 가지 않는다. 지연·무응답·오류 비트·생존 카운터·pulse 응답을
주입해 adapter의 실패 경로를 시험한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from robot_control.gripper_adapter import (
    DYNAMIXEL2,
    MODBUS,
    MODEL_NUMBER_RH_P12_RN_A,
    HARDWARE_PULSE_MAX,
    HARDWARE_PULSE_MIN,
)


@dataclass
class FakeTransport:
    clock: object
    model_number: int = MODEL_NUMBER_RH_P12_RN_A
    firmware_version: int = 12
    operating_mode: int = 5
    error_bits: int = 0
    present_pulse: int = HARDWARE_PULSE_MIN
    present_current_ma: int = 5
    loop_counter: int = 0
    frozen_loop: bool = False
    move_delay_s: float = 0.02
    no_response: bool = False
    fail_trigger: bool = False
    fail_write: set[int] = field(default_factory=set)
    protocol: str = 'modbus'
    writes: list[tuple[int, int]] = field(default_factory=list)
    triggers: int = 0
    goal_position: int | None = None
    goal_current_ma: int | None = None
    start_loop_calls: int = 0
    _arrive_s: float | None = None
    _target_pulse: int | None = None
    _start_pulse: int | None = None

    @property
    def table(self):
        return MODBUS if self.protocol == 'modbus' else DYNAMIXEL2

    def read_register(self, address: int, count: int = 1):
        if self.no_response:
            raise TimeoutError('fake transport no response')
        table = self.table
        if address == table.model_number:
            return self.model_number
        if address == table.firmware_version:
            return self.firmware_version
        if address == table.operating_mode:
            return self.operating_mode
        if address == table.hardware_error_status:
            return self.error_bits
        if address == table.goal_current:
            return self.goal_current_ma if self.goal_current_ma is not None else 0
        if address == table.present_current:
            return self.present_current_ma
        if address == table.present_position:
            return self._current_pulse()
        if address == table.goal_position:
            return self.goal_position if self.goal_position is not None else 0
        return 0

    def write_register(self, address: int, value: int) -> bool:
        self.writes.append((address, int(value)))
        if self.no_response or address in self.fail_write:
            return False
        if address == self.table.goal_current:
            self.goal_current_ma = int(value)
        elif address == self.table.goal_position:
            self.goal_position = int(value)
        elif address == self.table.operating_mode:
            self.operating_mode = int(value)
        return True

    def write_goal_position(self, pulse: int) -> bool:
        if not (HARDWARE_PULSE_MIN <= int(pulse) <= HARDWARE_PULSE_MAX):
            raise ValueError(f'pulse {pulse}')
        self.goal_position = int(pulse)
        self.writes.append((self.table.goal_position, int(pulse)))
        return not self.no_response and self.table.goal_position not in self.fail_write

    def trigger(self) -> bool:
        if self.fail_trigger or self.no_response:
            return False
        self.triggers += 1
        self._start_pulse = self._current_pulse()
        if self.goal_position is not None and not self.no_response:
            self._target_pulse = self.goal_position
            self._arrive_s = self.clock.steady_now() + self.move_delay_s
        return True

    def start_resident_loop(self) -> bool:
        self.start_loop_calls += 1
        return True

    def read_loop_counter(self) -> int:
        if not self.frozen_loop:
            self.loop_counter += 1
        return self.loop_counter

    def _current_pulse(self) -> int:
        if self._target_pulse is None or self._arrive_s is None:
            return int(self.present_pulse)
        if self.clock.steady_now() >= self._arrive_s:
            self.present_pulse = int(self._target_pulse)
        return int(self.present_pulse)
