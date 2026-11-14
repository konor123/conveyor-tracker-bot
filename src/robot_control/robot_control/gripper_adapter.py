"""RH-P12-RN(A) 그리퍼 어댑터.

이 파일만 그리퍼 transport를 호출한다. 하드웨어 Goal Position
0~1150 pulse를 그대로 쓰며, 참고 구현의 0~750 제한·`750 − 값` 반전 규약은 쓰지
않는다. 실측 pulse↔간격 표가 없으면 폭 지령을 만들지 않는다(추정 보정 금지).

경계:
- hardware capability: 0~1150 pulse (`HARDWARE_PULSE_MIN/MAX`)
- 현장 일회성 CLI envelope: 0~750 pulse (도구 쪽 규약, 이 adapter와 별개)
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from typing import Callable, Protocol, Sequence

HARDWARE_PULSE_MIN = 0
HARDWARE_PULSE_MAX = 1150
CURRENT_MIN_MA = 1
CURRENT_MAX_MA = 150           # 2026-11-09 실기 파지(659 pulse·149 mA)에 쓴 값. 사용자 승인 상한
CURRENT_DEFAULT_MA = 100
POSITION_TOLERANCE_PULSE = 3
EPS = 1e-12

MODEL_NUMBER_RH_P12_RN_A = 35074
MIN_FIRMWARE_VERSION = 12

FC03 = 0x03
FC06 = 0x06
FC16 = 0x10


class GripperError(Exception):
    """그리퍼 계약 위반. `reason`은 결과 코드와 별도로 세부 원인을 담는다."""

    def __init__(self, reason: str, detail: str = ''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason
        self.detail = detail


class OutOfRange(GripperError):
    """표 범위 밖 요청. 클램프·외삽하지 않는다."""


class TableValidationError(GripperError):
    """비단조·중복·비유한 실측표."""


class PreconditionError(GripperError):
    """설정·시작 확인·모드 등 선행조건 미충족."""


class GripperFault(GripperError):
    """하드웨어·통신 이상."""

    def __init__(self, reason: str, bits: int = 0, detail: str = ''):
        super().__init__(reason, detail)
        self.bits = int(bits)


HARDWARE_ERROR_BITS = {
    0x0001: 'INPUT_VOLTAGE',
    0x0004: 'OVERHEAT',
    0x0008: 'MOTOR_ENCODER',
    0x0010: 'ELECTRICAL_SHOCK',
    0x0020: 'OVERLOAD',
    0x0080: 'PROCESS_ERROR',
}


def fault_bits_names(bits: int) -> list[str]:
    names = [name for bit, name in HARDWARE_ERROR_BITS.items() if bits & bit]
    unknown = bits & ~sum(HARDWARE_ERROR_BITS)
    if unknown:
        names.append(f'UNKNOWN_0x{unknown:X}')
    return names


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise GripperError('NONFINITE', name)
    return value


def _as_int(value, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise OutOfRange(name, repr(value))
    return int(value)


@dataclass(frozen=True)
class PulseTable:
    """실측 (pulse, width_m) 표. pulse·width 모두 강단조여야 하고 외삽하지 않는다."""

    pulses: tuple[int, ...]
    widths_m: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.pulses) != len(self.widths_m):
            raise TableValidationError('TABLE_LENGTH', 'pulse·width 길이 불일치')
        if len(self.pulses) < 2:
            raise TableValidationError('TABLE_LENGTH', '표는 두 점 이상 필요')
        for p in self.pulses:
            _as_int(p, 'pulse')
            if not (HARDWARE_PULSE_MIN <= p <= HARDWARE_PULSE_MAX):
                raise TableValidationError('TABLE_RANGE', f'pulse {p}')
        for w in self.widths_m:
            _finite(w, 'width_m')
            if w < 0.0:
                raise TableValidationError('TABLE_RANGE', f'width {w}')
        if any(b <= a for a, b in zip(self.pulses, self.pulses[1:])):
            raise TableValidationError('TABLE_NOT_MONOTONIC', 'pulse가 증가하지 않음')
        diffs = [b - a for a, b in zip(self.widths_m, self.widths_m[1:])]
        if not (all(d > 0.0 for d in diffs) or all(d < 0.0 for d in diffs)):
            raise TableValidationError('TABLE_NOT_MONOTONIC', 'width가 단조가 아님')

    @classmethod
    def from_samples(cls, samples: Sequence[tuple[int, float]]) -> 'PulseTable':
        ordered = sorted((int(p), float(w)) for p, w in samples)
        return cls(tuple(p for p, _ in ordered), tuple(w for _, w in ordered))

    @classmethod
    def from_csv(cls, path: str) -> 'PulseTable':
        """`pulse,width_m` 열을 가진 CSV(또는 `cmd_pulse,gap_mm` 측정 CSV)."""
        samples: list[tuple[int, float]] = []
        with open(path, newline='', encoding='utf-8') as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                pulse_key = 'pulse' if 'pulse' in row else 'cmd_pulse'
                width_key = 'width_m' if 'width_m' in row else 'gap_mm'
                if row.get(pulse_key) in (None, '') or row.get(width_key) in (None, ''):
                    continue
                pulse = int(float(row[pulse_key]))
                width = float(row[width_key])
                if width_key == 'gap_mm':
                    width = width / 1_000.0
                samples.append((pulse, width))
        if not samples:
            raise TableValidationError('TABLE_EMPTY', path)
        return cls.from_samples(samples)

    @property
    def min_width_m(self) -> float:
        return min(self.widths_m)

    @property
    def max_width_m(self) -> float:
        return max(self.widths_m)

    def width_to_pulse(self, width_m: float) -> int:
        width = _finite(width_m, 'width_m')
        if width < self.min_width_m - EPS or width > self.max_width_m + EPS:
            raise OutOfRange('WIDTH', f'{width} not in [{self.min_width_m}, {self.max_width_m}]')
        for (p0, w0), (p1, w1) in zip(zip(self.pulses, self.widths_m),
                                       zip(self.pulses[1:], self.widths_m[1:])):
            if min(w0, w1) - EPS <= width <= max(w0, w1) + EPS:
                if abs(w1 - w0) <= EPS:
                    return int(p0)
                ratio = (width - w0) / (w1 - w0)
                return int(round(p0 + ratio * (p1 - p0)))
        raise OutOfRange('WIDTH', f'{width}')

    def pulse_to_width(self, pulse: int) -> float:
        p = _as_int(pulse, 'pulse')
        if p < self.pulses[0] or p > self.pulses[-1]:
            raise OutOfRange('PULSE', f'{p} not in [{self.pulses[0]}, {self.pulses[-1]}]')
        for (p0, w0), (p1, w1) in zip(zip(self.pulses, self.widths_m),
                                       zip(self.pulses[1:], self.widths_m[1:])):
            if p0 <= p <= p1:
                if p1 == p0:
                    return float(w0)
                ratio = (p - p0) / (p1 - p0)
                return float(w0 + ratio * (w1 - w0))
        raise OutOfRange('PULSE', f'{p}')


@dataclass(frozen=True)
class AddressTable:
    """프로토콜별 주소표. 두 표를 섞어 쓰지 않는다."""

    protocol: str
    model_number: int
    firmware_version: int
    protocol_type: int
    operating_mode: int
    torque_enable: int
    hardware_error_status: int
    goal_current: int
    profile_velocity: int
    goal_position: int
    present_current: int
    present_position: int


DYNAMIXEL2 = AddressTable(
    protocol='dynamixel2',
    model_number=0, firmware_version=6, protocol_type=13, operating_mode=11,
    torque_enable=512, hardware_error_status=518, goal_current=550,
    profile_velocity=560, goal_position=564, present_current=574, present_position=580,
)
MODBUS = AddressTable(
    protocol='modbus',
    model_number=0, firmware_version=3, protocol_type=6, operating_mode=5,
    torque_enable=256, hardware_error_status=259, goal_current=275,
    profile_velocity=280, goal_position=282, present_current=287, present_position=290,
)
ADDRESS_TABLES = {'dynamixel2': DYNAMIXEL2, 'modbus': MODBUS}


def address_table(protocol: str) -> AddressTable:
    try:
        return ADDRESS_TABLES[str(protocol)]
    except KeyError as exc:
        raise PreconditionError('ADDRESS_TABLE_UNKNOWN', str(protocol)) from exc


# ---------------------------------------------------------------- Modbus-RTU
def crc16_modbus(data: bytes | bytearray | Sequence[int]) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= int(byte) & 0xFF
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def _with_crc(body: Sequence[int]) -> bytes:
    payload = bytes(int(b) & 0xFF for b in body)
    return payload + crc16_modbus(payload)


def modbus_read_frame(slave: int, address: int, count: int = 1) -> bytes:
    if not (1 <= slave <= 247):
        raise OutOfRange('SLAVE_ID', str(slave))
    if not (1 <= count <= 125):
        raise OutOfRange('READ_COUNT', str(count))
    return _with_crc((slave, FC03, (address >> 8) & 0xFF, address & 0xFF,
                      (count >> 8) & 0xFF, count & 0xFF))


def modbus_write_single_frame(slave: int, address: int, value: int) -> bytes:
    if not (1 <= slave <= 247):
        raise OutOfRange('SLAVE_ID', str(slave))
    if not (0 <= value <= 0xFFFF):
        raise OutOfRange('VALUE', str(value))
    return _with_crc((slave, FC06, (address >> 8) & 0xFF, address & 0xFF,
                      (value >> 8) & 0xFF, value & 0xFF))


def modbus_write_multiple_frame(slave: int, address: int, values: Sequence[int]) -> bytes:
    if not (1 <= slave <= 247):
        raise OutOfRange('SLAVE_ID', str(slave))
    if not values or len(values) > 123:
        raise OutOfRange('WRITE_COUNT', str(len(values)))
    words = [int(v) for v in values]
    if any(not (0 <= v <= 0xFFFF) for v in words):
        raise OutOfRange('VALUE', str(words))
    count = len(words)
    body = [slave, FC16, (address >> 8) & 0xFF, address & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF, count * 2]
    for value in words:
        body += [(value >> 8) & 0xFF, value & 0xFF]
    return _with_crc(body)


def check_modbus_response(frame: bytes, slave: int, function: int,
                          address: int | None = None, count: int | None = None) -> bytes:
    """응답 길이·ID·기능·주소·바이트 수·CRC를 검사하고 데이터 부분을 돌려준다."""
    data = bytes(frame)
    if len(data) < 5:
        raise GripperFault('MODBUS_SHORT', detail=f'{len(data)} bytes')
    if crc16_modbus(data[:-2]) != data[-2:]:
        raise GripperFault('MODBUS_CRC')
    if data[0] != (slave & 0xFF):
        raise GripperFault('MODBUS_SLAVE', detail=f'{data[0]} != {slave}')
    if data[1] == (function | 0x80):
        raise GripperFault('MODBUS_EXCEPTION', detail=f'code {data[2]}')
    if data[1] != function:
        raise GripperFault('MODBUS_FUNCTION', detail=f'{data[1]} != {function}')
    if function == FC03:
        byte_count = data[2]
        if count is not None and byte_count != 2 * count:
            raise GripperFault('MODBUS_BYTE_COUNT', detail=f'{byte_count} != {2 * count}')
        if len(data) != 3 + byte_count + 2:
            raise GripperFault('MODBUS_LENGTH', detail=f'{len(data)}')
        return data[3:3 + byte_count]
    if function in (FC06, FC16):
        if len(data) != 8:
            raise GripperFault('MODBUS_LENGTH', detail=f'{len(data)}')
        resp_address = (data[2] << 8) | data[3]
        if address is not None and resp_address != address:
            raise GripperFault('MODBUS_ADDRESS', detail=f'{resp_address} != {address}')
        return data[2:6]
    return data[2:-2]


# ---------------------------------------------------------------- 레지스터 코덱
def encode7(value: int) -> tuple[int, int]:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0 or value > 0x3FFF:
        raise OutOfRange('REGISTER_VALUE', repr(value))
    return (value >> 7) & 0x7F, value & 0x7F


def decode7(hi: int, lo: int) -> int:
    return (int(hi) << 7) | int(lo)


def next_seq(prev: int) -> int:
    return (int(prev) % 127) + 1


@dataclass(frozen=True)
class RegisterMap:
    """참고 구현 상주 루프의 제어기 출력 레지스터. 프로토콜 주소표와 다르다."""

    seq: int = 10
    pos_hi: int = 11
    pos_lo: int = 12
    cur_hi: int = 13
    cur_lo: int = 14


class GripperTransport(Protocol):
    """adapter가 요구하는 최소 transport 계약. 구현은 상주 루프/Modbus를 감춘다."""

    def read_register(self, address: int, count: int = 1): ...
    def write_register(self, address: int, value: int) -> bool: ...
    def write_goal_position(self, pulse: int) -> bool: ...
    def trigger(self) -> bool: ...
    def start_resident_loop(self) -> bool: ...
    def read_loop_counter(self) -> int: ...


@dataclass(frozen=True)
class GripperCfg:
    protocol: str = 'modbus'
    default_current_ma: int = CURRENT_DEFAULT_MA
    current_min_ma: int = CURRENT_MIN_MA
    current_max_ma: int = CURRENT_MAX_MA
    response_timeout_s: float = 0.5
    loop_timeout_s: float = 0.1
    position_tolerance_pulse: int = POSITION_TOLERANCE_PULSE
    read_period_s: float = 0.01
    close_pulse: int | None = None
    required_model_number: int = MODEL_NUMBER_RH_P12_RN_A
    min_firmware_version: int = MIN_FIRMWARE_VERSION
    operating_mode: int = 5
    check_firmware_version: bool = True
    grip_min_width_m: float | None = None
    grip_max_width_m: float | None = None
    grip_min_current_ma: float | None = None
    # 막힘 파지 판정. None이면 닫힘 목표 도달만 본다
    grasp_stall_pulse: int | None = None       # 연속 두 표본의 위치 차가 이 값 이하면 멈춘 것
    grasp_min_travel_pulse: int = 10           # 명령 전 위치에서 이만큼 넘게 움직였어야 한다
    grasp_current_ratio: float = 0.8           # 또는 전류가 한도의 이 비율 이상(이미 쥔 상태)
    regs: RegisterMap = field(default_factory=RegisterMap)


@dataclass(frozen=True)
class GripperState:
    width_m: float
    current_ma: float
    error_bits: int
    t_read: float
    present_pulse: int
    loop_counter: int


@dataclass(frozen=True)
class GripResult:
    result: int                    # drop_interfaces 공통 결과 코드와 같은 정수
    reason_code: str
    width_m: float | None = None
    current_ma: float | None = None
    holding: bool | None = None
    t_cmd: float | None = None
    t_ack: float | None = None
    t_motion_start: float | None = None
    t_done: float | None = None


@dataclass(frozen=True)
class OpenResult:
    result: int
    reason_code: str
    width_m: float | None = None
    target_pulse: int | None = None
    present_pulse: int | None = None
    t_cmd: float | None = None
    t_ack: float | None = None
    t_motion_start: float | None = None
    t_done: float | None = None


RESULT_OK = 0
RESULT_FAILED = 1
RESULT_TIMEOUT = 2
RESULT_PRECONDITION = 3
RESULT_CANCELED = 4
RESULT_FAULT = 5


class GripperAdapter:
    """그리퍼 지령의 유일한 경계. 실패는 예외 또는 FAULT 결과로 닫힌다."""

    def __init__(self, transport: GripperTransport, table: PulseTable, cfg: GripperCfg,
                 clock, sleep: Callable[[float], None] | None = None):
        self.transport = transport
        self.table = table
        self.cfg = cfg
        self.clock = clock
        self._sleep = sleep if sleep is not None else (lambda _dt: None)
        self.addresses = address_table(cfg.protocol)
        self._check_timeouts_against_loop_period()
        self._started = False
        self._last_loop_counter: int | None = None
        self._last_loop_change_s: float | None = None
        self._prepared: tuple[int, int] | None = None

    # ------------------------------------------------------------ 시작 확인
    def startup_check(self, robot_standby: bool | None = None) -> None:
        if robot_standby is False:
            raise PreconditionError('ROBOT_NOT_STANDBY')
        model = self._read_word(self.addresses.model_number)
        if model != self.cfg.required_model_number:
            raise PreconditionError('MODEL_NUMBER', f'{model} != {self.cfg.required_model_number}')
        if self.cfg.check_firmware_version:
            firmware = self._read_word(self.addresses.firmware_version)
            if firmware < self.cfg.min_firmware_version:
                raise PreconditionError('FIRMWARE_OLD', f'{firmware} < {self.cfg.min_firmware_version}')
        bits = self._read_word(self.addresses.hardware_error_status)
        if bits:
            raise GripperFault('GRIPPER_ERROR', bits=bits, detail=','.join(fault_bits_names(bits)))
        mode = self._read_word(self.addresses.operating_mode)
        if mode != self.cfg.operating_mode:
            raise PreconditionError('GRIPPER_MODE', f'{mode} != {self.cfg.operating_mode}')
        self._started = True

    def start_resident_loop(self, robot_standby: bool) -> bool:
        """STANDBY가 아니면 drl_start를 호출하지 않는다."""
        if not robot_standby:
            raise PreconditionError('ROBOT_NOT_STANDBY')
        ok = bool(self.transport.start_resident_loop())
        if not ok:
            raise GripperFault('DRL_START_FAILED')
        self._last_loop_counter = None
        self._last_loop_change_s = None
        return True

    def set_operating_mode(self, mode: int, torque_enabled: bool) -> None:
        """EEPROM 쓰기는 토크가 꺼진 상태에서만 허용한다."""
        if torque_enabled:
            raise PreconditionError('TORQUE_ON')
        if not self.transport.write_register(self.addresses.operating_mode, int(mode)):
            raise GripperFault('WRITE_OPERATING_MODE_FAILED')

    # ------------------------------------------------------------ 지령
    def open(self, width_m: float, current_limit_ma: int | None = None,
             prewritten: bool = False) -> OpenResult:
        self._require_started()
        pulse = self.table.width_to_pulse(width_m)
        current = self._validate_current(
            self.cfg.default_current_ma if current_limit_ma is None else current_limit_ma)
        t_cmd = self.clock.steady_now()
        initial = self._read_present_pulse()
        if prewritten and self._prepared == (pulse, current):
            self._prepared = None
            if not self.transport.trigger():
                raise GripperFault('TRIGGER_FAILED')
        else:
            self._send_goal(pulse, current)
        t_ack = self.clock.steady_now()
        t_motion, present, t_done = self._wait_position(pulse, initial, t_cmd)
        return OpenResult(RESULT_OK, 'OK', self.table.pulse_to_width(present), pulse, present,
                          t_cmd, t_ack, t_motion, t_done)

    def prewrite_open(self, width_m: float, current_limit_ma: int | None = None) -> int:
        """해제 직전 지연을 줄이려 목표값만 미리 쓴다. 트리거는 `open(..., prewritten=True)`."""
        self._require_started()
        pulse = self.table.width_to_pulse(width_m)
        current = self._validate_current(
            self.cfg.default_current_ma if current_limit_ma is None else current_limit_ma)
        self._write_payload(pulse, current)
        self._prepared = (pulse, current)
        return pulse

    def close(self, current_limit_ma: int) -> GripResult:
        self._require_started()
        current = self._validate_current(current_limit_ma)
        if self.cfg.close_pulse is None:
            raise PreconditionError('CLOSE_PULSE_UNSET', '실측표로 닫힘 위치를 정하기 전에는 닫지 않는다')
        pulse = _as_int(self.cfg.close_pulse, 'close_pulse')
        if not (HARDWARE_PULSE_MIN <= pulse <= HARDWARE_PULSE_MAX):
            raise OutOfRange('PULSE', str(pulse))
        t_cmd = self.clock.steady_now()
        initial = self._read_present_pulse()
        self._send_goal(pulse, current)
        t_ack = self.clock.steady_now()
        if self.cfg.grasp_stall_pulse is not None:
            t_motion, present, t_done, state_current, stalled = self._wait_grasp(pulse, initial, t_cmd, current)
            # 대상에 막혀 멈췄으면 쥔 것, 닫힘 끝점까지 갔으면 손가락 사이가 비었다
            return GripResult(RESULT_OK, 'STALLED' if stalled else 'REACHED_END',
                              self.table.pulse_to_width(present), float(state_current), stalled,
                              t_cmd, t_ack, t_motion, t_done)
        t_motion, present, t_done = self._wait_position(pulse, initial, t_cmd)
        width = self.table.pulse_to_width(present)
        state_current = self._read_signed_word(self.addresses.present_current)
        holding = self._holding(width, state_current)
        return GripResult(RESULT_OK, 'OK', width, float(state_current), holding,
                          t_cmd, t_ack, t_motion, t_done)

    def read_state(self) -> GripperState:
        present = self._read_present_pulse()
        current = self._read_signed_word(self.addresses.present_current)
        bits = self._read_word(self.addresses.hardware_error_status)
        counter = self._observe_loop()
        if bits:
            raise GripperFault('GRIPPER_ERROR', bits=bits, detail=','.join(fault_bits_names(bits)))
        return GripperState(self.table.pulse_to_width(present), float(current), bits,
                            self.clock.steady_now(), present, counter)

    def check_loop_alive(self, counter: int | None = None) -> bool:
        """생존 카운터가 `loop_timeout_s` 넘게 변하지 않으면 FAULT."""
        if counter is None:
            counter = self.transport.read_loop_counter()
        now = self.clock.steady_now()
        counter = int(counter)
        if self._last_loop_counter is None or counter != self._last_loop_counter:
            self._last_loop_counter = counter
            self._last_loop_change_s = now
            return True
        changed = self._last_loop_change_s if self._last_loop_change_s is not None else now
        if now - changed > self.cfg.loop_timeout_s + EPS:
            raise GripperFault('GRIPPER_LOOP_DEAD', detail=f'{now - changed:.3f} s')
        return True

    # ------------------------------------------------------------ 내부
    def _require_started(self) -> None:
        if not self._started:
            raise PreconditionError('STARTUP_CHECK_REQUIRED')

    def _validate_current(self, current) -> int:
        cur = _as_int(current, 'current_limit_ma')
        lo = max(CURRENT_MIN_MA, int(self.cfg.current_min_ma))
        hi = min(CURRENT_MAX_MA, int(self.cfg.current_max_ma))
        if not (lo <= cur <= hi):
            raise OutOfRange('CURRENT', f'{cur} not in [{lo}, {hi}] mA')
        return cur

    def _read_word(self, address: int) -> int:
        try:
            value = self.transport.read_register(address, 1)
        except GripperError:
            raise
        except Exception as exc:      # transport 단절·timeout
            raise GripperFault('GRIPPER_TIMEOUT', detail=str(exc)) from exc
        if isinstance(value, (list, tuple)):
            if not value:
                raise GripperFault('READ_EMPTY', detail=str(address))
            value = value[0]
        if value is None:
            raise GripperFault('READ_NONE', detail=str(address))
        return int(value)

    def _read_signed_word(self, address: int) -> int:
        value = self._read_word(address) & 0xFFFF
        return value - 0x10000 if value & 0x8000 else value

    def _read_present_pulse(self) -> int:
        present = self._read_word(self.addresses.present_position)
        if not (HARDWARE_PULSE_MIN <= present <= HARDWARE_PULSE_MAX):
            raise GripperFault('PRESENT_PULSE_RANGE', detail=str(present))
        return present

    def _send_goal(self, pulse: int, current: int) -> None:
        self._check_error()
        self._prepared = None          # 목표 레지스터를 덮어쓰므로 앞선 예약(prewrite)은 무효다
        self._write_payload(pulse, current)
        if not self.transport.trigger():
            raise GripperFault('TRIGGER_FAILED')

    def _write_payload(self, pulse: int, current: int) -> None:
        pulse = _as_int(pulse, 'pulse')
        if not (HARDWARE_PULSE_MIN <= pulse <= HARDWARE_PULSE_MAX):
            raise OutOfRange('PULSE', str(pulse))
        current = self._validate_current(current)
        try:
            if not self.transport.write_register(self.addresses.goal_current, current):
                raise GripperFault('WRITE_GOAL_CURRENT_FAILED')
            if not self.transport.write_goal_position(pulse):
                raise GripperFault('WRITE_GOAL_POSITION_FAILED')
        except GripperError:
            raise
        except Exception as exc:
            raise GripperFault('TRANSPORT_ERROR', detail=str(exc)) from exc

    def _check_error(self) -> None:
        bits = self._read_word(self.addresses.hardware_error_status)
        if bits:
            raise GripperFault('GRIPPER_ERROR', bits=bits, detail=','.join(fault_bits_names(bits)))

    def _observe_loop(self) -> int:
        counter = int(self.transport.read_loop_counter())
        self.check_loop_alive(counter)
        return counter

    def _command_sample_ready(self, counter: int, seen: list) -> bool:
        """지령 실행을 알리는 transport(상주 루프)에서는 루프가 이 지령을 실행했고(결과 0)
        그 뒤 새 표본이 올라왔을 때만 판정한다. 실패 결과면 이 지령의 FAULT다. 알리지 않는 transport는 항상 참.
        `seen`은 [실행을 처음 본 표본의 카운터]를 담는 호출자 상태다.
        """
        status = getattr(self.transport, 'last_command', None)
        if status is None:
            return True
        st = status()
        if st is None:
            return True
        executed, result = st
        if not executed:
            return False
        if result != 0:
            raise GripperFault('RESIDENT_WRITE_FAILED', detail=f'result={result}')
        if not seen:
            seen.append(counter)
            return False
        return counter != seen[0]

    def _wait_position(self, target: int, initial: int, t_cmd: float):
        motion_start: float | None = None
        seen: list = []
        max_iters = int(self.cfg.response_timeout_s / max(self.cfg.read_period_s, EPS)) + 3
        for _ in range(max_iters):
            self._sleep(self.cfg.read_period_s)
            now = self.clock.steady_now()
            if now - t_cmd > self.cfg.response_timeout_s + EPS:
                raise GripperFault('GRIPPER_TIMEOUT', detail=f'{now - t_cmd:.3f} s')
            present = self._read_present_pulse()
            self._check_error()
            counter = self._observe_loop()
            if motion_start is None and present != initial:
                motion_start = now
            if not self._command_sample_ready(counter, seen):
                continue
            if abs(present - target) <= self.cfg.position_tolerance_pulse:
                return motion_start, present, now
        raise GripperFault('GRIPPER_TIMEOUT', detail='position not reached')

    def _wait_grasp(self, target: int, initial: int, t_cmd: float, limit_ma: int):
        """닫힘 목표 도달 또는 막힘까지 기다린다. 막힘은 상주 루프가 새로 올린 표본끼리만 비교한다.

        생존 카운터가 같은 동안에는 같은 표본이므로 멈춘 것으로 세지 않는다.
        """
        cfg = self.cfg
        motion_start: float | None = None
        last: int | None = None
        last_counter: int | None = None
        seen: list = []
        max_iters = int(cfg.response_timeout_s / max(cfg.read_period_s, EPS)) + 3
        for _ in range(max_iters):
            self._sleep(cfg.read_period_s)
            now = self.clock.steady_now()
            if now - t_cmd > cfg.response_timeout_s + EPS:
                raise GripperFault('GRIPPER_TIMEOUT', detail=f'{now - t_cmd:.3f} s')
            present = self._read_present_pulse()
            current = self._read_signed_word(self.addresses.present_current)
            self._check_error()
            counter = self._observe_loop()
            if motion_start is None and present != initial:
                motion_start = now
            if not self._command_sample_ready(counter, seen):
                continue
            if abs(present - target) <= cfg.position_tolerance_pulse:
                return motion_start, present, now, current, False
            if counter != last_counter:
                blocked = (abs(present - initial) > cfg.grasp_min_travel_pulse
                           or abs(current) >= cfg.grasp_current_ratio * limit_ma)
                if last is not None and abs(present - last) <= cfg.grasp_stall_pulse and blocked:
                    return motion_start, present, now, current, True
                last, last_counter = present, counter
        raise GripperFault('GRIPPER_TIMEOUT', detail='grasp not settled')

    def _check_timeouts_against_loop_period(self) -> None:
        """상태를 표본 주기로 올리는 transport(상주 루프)에서 판정이 끝날 수 없는 제한시간을 거부한다.

        상주 루프 모델의 최악 경우(전류를 바꾸는 지령): 트리거가 읽기 직후에 닿아 1주기 대기, 전류·위치 쓰기 2주기,
        실행 뒤 새 표본 1주기, 막힘 판정의 두 번째 표본 1주기 = 응답 5주기. 쓰기 2회와 다음 읽기 동안 생존 카운터가
        멈추므로 생존 감시 3주기. 가짜 제어기에서 위상 30가지 모두 이 값으로 끝나고, 응답 4.9주기는 일부 위상(7/30),
        생존 2.9주기는 모든 위상에서 끝나지 않는다. 실제 루프에서도 충분한지는 실측이 판정한다. 주기를 알리지 않는 transport는 검사하지 않는다.
        """
        period = getattr(self.transport, 'sample_period_s', None)
        if period is None:
            return
        cfg = self.cfg
        if cfg.response_timeout_s + EPS < 5.0 * period or cfg.loop_timeout_s + EPS < 3.0 * period:
            raise PreconditionError(
                'TIMEOUT_BELOW_LOOP_PERIOD',
                f'response {cfg.response_timeout_s} s < 5×{period} s or loop {cfg.loop_timeout_s} s < 3×{period} s')

    def _holding(self, width_m: float, current_ma: float) -> bool | None:
        cfg = self.cfg
        if (cfg.grip_min_width_m is None or cfg.grip_max_width_m is None
                or cfg.grip_min_current_ma is None):
            return None
        return (cfg.grip_min_width_m - EPS <= width_m <= cfg.grip_max_width_m + EPS
                and current_ma >= cfg.grip_min_current_ma - EPS)
