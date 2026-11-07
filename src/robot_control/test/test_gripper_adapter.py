"""TC-GRP-001~013 중 이번 범위: 표·주소·시작 확인·순서·부분 개방·시간·오류·프레임·코덱·생존."""
from __future__ import annotations

import math

import pytest

from fakes.clock import FakeClock
from fakes.transport import FakeTransport
from robot_control.gripper_adapter import (
    CURRENT_MAX_MA,
    DYNAMIXEL2,
    MODBUS,
    GripperAdapter,
    GripperCfg,
    GripperFault,
    OutOfRange,
    PreconditionError,
    PulseTable,
    TableValidationError,
    address_table,
    check_modbus_response,
    crc16_modbus,
    decode7,
    encode7,
    modbus_read_frame,
    modbus_write_multiple_frame,
    modbus_write_single_frame,
    RESULT_OK,
)

PULSE_MAX = 1150


def linear_table() -> PulseTable:
    samples = [(p, 0.106 * (1.0 - p / PULSE_MAX)) for p in range(0, PULSE_MAX + 1, 50)]
    return PulseTable.from_samples(samples)


def build(move_delay_s: float = 0.02, **cfg_kw):
    clock = FakeClock(steady=100.0)
    transport = FakeTransport(clock=clock, move_delay_s=move_delay_s)
    cfg = GripperCfg(close_pulse=1150, response_timeout_s=0.5, read_period_s=0.01, **cfg_kw)
    adapter = GripperAdapter(transport, linear_table(), cfg, clock,
                             sleep=clock.advance)
    return adapter, transport, clock


@pytest.mark.req('REQ-GRP-001')
@pytest.mark.tc('TC-GRP-001-01')
@pytest.mark.level('U')
def test_pulse_width_roundtrip_within_tolerance():
    table = linear_table()
    for pulse in range(0, PULSE_MAX + 1):
        width = table.pulse_to_width(pulse)
        back = table.width_to_pulse(width)
        assert abs(back - pulse) <= 1
        assert abs(table.pulse_to_width(back) - width) <= 0.05 / 1000.0


@pytest.mark.req('REQ-GRP-001')
@pytest.mark.tc('TC-GRP-001-02')
@pytest.mark.level('U')
def test_table_rejects_nonmonotonic_and_out_of_range():
    with pytest.raises(TableValidationError):
        PulseTable.from_samples([(0, 0.106), (500, 0.050), (1000, 0.080), (1150, 0.0)])
    with pytest.raises(TableValidationError):
        PulseTable.from_samples([(0, 0.106), (0, 0.050)])
    table = linear_table()
    with pytest.raises(OutOfRange):
        table.width_to_pulse(0.2)
    with pytest.raises(OutOfRange):
        table.pulse_to_width(1200)


@pytest.mark.req('REQ-GRP-002')
@pytest.mark.tc('TC-GRP-002-01')
@pytest.mark.level('U')
def test_startup_checks_model_firmware_and_error():
    adapter, transport, _ = build()
    adapter.startup_check(robot_standby=True)
    assert adapter._started

    adapter, transport, _ = build()
    transport.model_number = 12345
    with pytest.raises(PreconditionError) as exc:
        adapter.startup_check()
    assert exc.value.reason == 'MODEL_NUMBER'

    adapter, transport, _ = build()
    transport.firmware_version = 11
    with pytest.raises(PreconditionError) as exc:
        adapter.startup_check()
    assert exc.value.reason == 'FIRMWARE_OLD'

    adapter, transport, _ = build()
    transport.error_bits = 0x0004
    with pytest.raises(GripperFault) as exc:
        adapter.startup_check()
    assert exc.value.bits == 0x0004 and 'OVERHEAT' in exc.value.detail


@pytest.mark.req('REQ-GRP-003')
@pytest.mark.tc('TC-GRP-003-01')
@pytest.mark.level('U')
def test_protocol_separated_address_tables():
    assert (DYNAMIXEL2.torque_enable, DYNAMIXEL2.hardware_error_status,
            DYNAMIXEL2.goal_current, DYNAMIXEL2.profile_velocity,
            DYNAMIXEL2.goal_position, DYNAMIXEL2.present_current,
            DYNAMIXEL2.present_position) == (512, 518, 550, 560, 564, 574, 580)
    assert (MODBUS.torque_enable, MODBUS.hardware_error_status,
            MODBUS.goal_current, MODBUS.goal_position,
            MODBUS.present_current, MODBUS.present_position) == (256, 259, 275, 282, 287, 290)
    assert address_table('modbus') is MODBUS
    assert address_table('dynamixel2') is DYNAMIXEL2
    with pytest.raises(PreconditionError):
        address_table('mixed')


@pytest.mark.req('REQ-GRP-003')
@pytest.mark.tc('TC-GRP-003-02')
@pytest.mark.level('U')
def test_eeprom_write_requires_torque_off():
    adapter, transport, _ = build()
    adapter.startup_check()
    before = len(transport.writes)
    with pytest.raises(PreconditionError) as exc:
        adapter.set_operating_mode(5, torque_enabled=True)
    assert exc.value.reason == 'TORQUE_ON'
    assert len(transport.writes) == before
    adapter.set_operating_mode(5, torque_enabled=False)
    assert transport.writes[-1][0] == MODBUS.operating_mode


@pytest.mark.req('REQ-GRP-004')
@pytest.mark.tc('TC-GRP-004-01')
@pytest.mark.level('U')
def test_close_order_and_current_limit():
    adapter, transport, _ = build()
    adapter.startup_check()
    adapter.close(100)
    kinds = [(a, v) for a, v in transport.writes
             if a in (MODBUS.goal_current, MODBUS.goal_position)]
    assert kinds[0][0] == MODBUS.goal_current
    assert kinds[0][1] == 100
    assert kinds[1][0] == MODBUS.goal_position
    assert kinds[1][1] == 1150
    assert transport.triggers == 1

    adapter, transport, _ = build()
    adapter.startup_check()
    before = len(transport.writes)
    with pytest.raises(OutOfRange):
        adapter.close(101)
    assert len(transport.writes) == before

    adapter, transport, _ = build()
    adapter.startup_check()
    with pytest.raises(OutOfRange):
        adapter.close(0)


@pytest.mark.req('REQ-GRP-005')
@pytest.mark.tc('TC-GRP-005-01')
@pytest.mark.level('U')
def test_partial_open_uses_measured_table():
    adapter, transport, _ = build()
    adapter.startup_check()
    expected = linear_table().width_to_pulse(0.019)
    result = adapter.open(0.019)
    assert result.result == RESULT_OK
    assert result.target_pulse == expected
    assert result.present_pulse == expected
    assert transport.goal_position == expected


@pytest.mark.req('REQ-GRP-004')
@pytest.mark.tc('TC-GRP-004-01')
@pytest.mark.level('U')
def test_no_inverse_convention_and_hardware_range_beyond_cli_envelope():
    # 하드웨어 1100 pulse(좁은 간격)를 750으로 제한하거나 반전하지 않는다.
    adapter, transport, _ = build()
    adapter.startup_check()
    result = adapter.open(linear_table().pulse_to_width(1100))
    assert result.target_pulse == 1100
    assert transport.goal_position == 1100


@pytest.mark.req('REQ-GRP-007')
@pytest.mark.tc('TC-GRP-007-01')
@pytest.mark.level('U')
def test_response_timeout_boundary_and_no_response():
    adapter, _, _ = build(move_delay_s=0.500)
    adapter.startup_check()
    assert adapter.open(0.022).result == RESULT_OK

    adapter, _, _ = build(move_delay_s=0.501)
    adapter.startup_check()
    with pytest.raises(GripperFault) as exc:
        adapter.open(0.022)
    assert exc.value.reason == 'GRIPPER_TIMEOUT'

    adapter, transport, _ = build()
    adapter.startup_check()
    transport.no_response = True
    with pytest.raises(GripperFault):
        adapter.open(0.022)


@pytest.mark.req('REQ-GRP-008')
@pytest.mark.tc('TC-GRP-008-01')
@pytest.mark.level('U')
def test_error_bits_reason_contains_name():
    adapter, transport, _ = build()
    adapter.startup_check()
    transport.error_bits = 0x0020
    with pytest.raises(GripperFault) as exc:
        adapter.read_state()
    assert 'OVERLOAD' in exc.value.detail
    assert exc.value.reason == 'GRIPPER_ERROR'


@pytest.mark.req('REQ-GRP-009')
@pytest.mark.tc('TC-GRP-009-01')
@pytest.mark.level('U')
def test_timing_chain_is_monotonic():
    adapter, _, _ = build(move_delay_s=0.05)
    adapter.startup_check()
    result = adapter.open(0.022)
    times = [result.t_cmd, result.t_ack, result.t_motion_start, result.t_done]
    assert all(t is not None for t in times)
    assert times == sorted(times)


@pytest.mark.req('REQ-GRP-011')
@pytest.mark.tc('TC-GRP-011-01')
@pytest.mark.level('U')
def test_modbus_frames_match_standard_vectors():
    assert modbus_read_frame(1, 0, 1).hex(' ').upper() == '01 03 00 00 00 01 84 0A'
    assert modbus_write_single_frame(1, 256, 1).hex(' ').upper() == '01 06 01 00 00 01 49 F6'
    assert modbus_write_multiple_frame(1, 282, [300, 0]).hex(' ').upper() == (
        '01 10 01 1A 00 02 04 01 2C 00 00 BF 79')
    assert modbus_read_frame(1, 287, 1).hex(' ').upper() == '01 03 01 1F 00 01 B4 30'
    response = bytes.fromhex('01 03 02 01 2C')
    response += crc16_modbus(response)
    assert check_modbus_response(response, 1, 3, count=1) == b'\x01\x2c'


@pytest.mark.req('REQ-GRP-011')
@pytest.mark.tc('TC-GRP-011-01')
@pytest.mark.level('U')
def test_modbus_response_rejects_bad_crc_and_length():
    frame = bytearray(bytes.fromhex('01 03 02 01 2C'))
    frame += crc16_modbus(bytes(frame))
    frame[-1] ^= 0xFF
    with pytest.raises(GripperFault) as exc:
        check_modbus_response(bytes(frame), 1, 3, count=1)
    assert exc.value.reason == 'MODBUS_CRC'
    with pytest.raises(GripperFault) as exc:
        check_modbus_response(b'\x01\x03\x00', 1, 3)
    assert exc.value.reason == 'MODBUS_SHORT'


@pytest.mark.req('REQ-GRP-012')
@pytest.mark.tc('TC-GRP-012-01')
@pytest.mark.level('U')
def test_register_codec_roundtrip_and_range():
    for value in range(0, 0x4000):
        hi, lo = encode7(value)
        assert decode7(hi, lo) == value
    for value in (-1, 0x4000, 1.5, True):
        with pytest.raises(OutOfRange):
            encode7(value)


@pytest.mark.req('REQ-GRP-012')
@pytest.mark.tc('TC-GRP-012-02')
@pytest.mark.level('U')
def test_prewrite_then_release_triggers_once():
    adapter, transport, _ = build()
    adapter.startup_check()
    pulse = adapter.prewrite_open(0.019)
    assert transport.triggers == 0
    result = adapter.open(0.019, prewritten=True)
    assert transport.triggers == 1
    assert result.target_pulse == pulse
    assert sum(1 for a, _ in transport.writes if a == MODBUS.goal_current) == 1


@pytest.mark.req('REQ-GRP-013')
@pytest.mark.tc('TC-GRP-013-01')
@pytest.mark.level('U')
def test_loop_counter_stall_boundary():
    adapter, transport, clock = build()
    transport.frozen_loop = True
    transport.loop_counter = 7
    assert adapter.check_loop_alive() is True
    clock.advance(0.100)
    assert adapter.check_loop_alive(7) is True
    clock.advance(0.001)
    with pytest.raises(GripperFault) as exc:
        adapter.check_loop_alive(7)
    assert exc.value.reason == 'GRIPPER_LOOP_DEAD'


@pytest.mark.req('REQ-GRP-013')
@pytest.mark.tc('TC-GRP-013-02')
@pytest.mark.level('U')
def test_loop_start_requires_standby():
    adapter, transport, _ = build()
    with pytest.raises(PreconditionError) as exc:
        adapter.start_resident_loop(robot_standby=False)
    assert exc.value.reason == 'ROBOT_NOT_STANDBY'
    assert transport.start_loop_calls == 0
    adapter.start_resident_loop(robot_standby=True)
    assert transport.start_loop_calls == 1


@pytest.mark.req('REQ-GRP-004')
@pytest.mark.tc('TC-GRP-004-01')
@pytest.mark.level('U')
def test_transport_write_failure_closes_as_fault():
    adapter, transport, _ = build()
    adapter.startup_check()
    transport.fail_write = {MODBUS.goal_position}
    with pytest.raises(GripperFault) as exc:
        adapter.open(0.022)
    assert exc.value.reason == 'WRITE_GOAL_POSITION_FAILED'


@pytest.mark.req('REQ-GRP-006')
@pytest.mark.tc('TC-GRP-006-01')
@pytest.mark.level('U')
def test_holding_boundaries_three_points():
    adapter, _, _ = build(grip_min_width_m=0.016, grip_max_width_m=0.019,
                          grip_min_current_ma=20.0)
    assert adapter._holding(0.0161, 20.0) is True
    assert adapter._holding(0.0159, 20.0) is False
    assert adapter._holding(0.0191, 20.0) is False
    assert adapter._holding(0.0175, 19.9) is False
    adapter_none, _, _ = build()
    assert adapter_none._holding(0.0175, 40.0) is None


@pytest.mark.req('REQ-GRP-010')
@pytest.mark.tc('TC-GRP-010-01')
@pytest.mark.level('U')
def test_transport_import_only_in_adapter_and_tests():
    import re
    from pathlib import Path
    root = Path(__file__).resolve().parents[3]
    offenders = []
    for path in (root / 'src').rglob('*.py'):
        parts = path.parts
        if 'test' in parts or 'build' in parts or 'install' in parts:
            continue
        if path.name == 'gripper_adapter.py':
            continue
        text = path.read_text(encoding='utf-8')
        if re.search(r'GripperTransport', text):
            offenders.append(str(path.relative_to(root)))
    assert not offenders, offenders
