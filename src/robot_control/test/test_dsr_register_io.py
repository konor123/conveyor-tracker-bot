"""Doosan 서비스 레지스터 경로. 가짜 서비스만 쓴다(실기 미검증).

`dsr_msgs2`는 CI·시험 PC에 없다. 서비스 형식은 이름표 형식(`fakes.dsr.fake_types`)을 주입한다.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from fakes.clock import FakeClock
from fakes.dsr import FakeDsrServices, encode_register_value, fake_types
from fakes.resident import FakeResidentController
from robot_control.dsr_register_io import DsrError, DsrRegisterIO, decode_register_byte
from robot_control.gripper_adapter import GripperAdapter, GripperCfg, GripperError, PulseTable
from robot_control.gripper_session import GripperSession
from robot_control.resident_transport import ResidentLoopTransport

PULSE_MAX = 1150
TABLE = PulseTable.from_samples([(p, 0.106 * (1.0 - p / PULSE_MAX)) for p in range(0, PULSE_MAX + 1, 50)])


def io_for(services, **kw):
    return DsrRegisterIO(services, srv_types=fake_types(), **kw)


def _doosan_client_decode():
    path = Path(__file__).resolve().parents[2] / 'static_pick_place' / 'static_pick_place' / 'doosan_client.py'
    spec = importlib.util.spec_from_file_location('_doosan_client_for_decode', path)
    module = sys.modules.get(spec.name)
    if module is None:
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module                              # dataclass가 모듈을 찾는다
        spec.loader.exec_module(module)
    return module.decode_register_byte


@pytest.mark.parametrize('byte', [0, 1, 39, 127, 128, 242, 250, 255])
def test_read_decodes_the_top_byte_like_doosan_client(byte):
    raw = encode_register_value(byte)
    assert decode_register_byte(raw) == byte == _doosan_client_decode()(raw)
    services = FakeDsrServices()
    services.raw_override[30] = raw | 0x00ABCDEF if raw >= 0 else raw     # 하위 3바이트는 무시한다
    assert io_for(services).get(30) == byte
    assert services.calls == [('GetOutputRegisterInt', '/dsr01/plc/get_output_register_int',
                               {'address': 30, 'timeout_ms': 1000})]


def test_write_sends_only_pc_values_and_reports_failed_writes():
    services = FakeDsrServices()
    io = io_for(services, namespace='/dsr01/')
    assert io.set(10, 127) is True and services.store[10] == 127
    assert services.calls[-1] == ('SetOutputRegisterInt', '/dsr01/plc/set_output_register_int',
                                  {'address': 10, 'value': 127 << 24})      # 최상위 바이트에 담아 보낸다
    for bad in (128, -1, 1.0, True):
        with pytest.raises(ValueError):
            io.set(11, bad)
    assert len(services.calls) == 1                                  # 범위 밖 값은 보내지 않는다
    services.fail.add('set_output_register_int')
    assert io.set(11, 5) is False and 11 not in services.store


def test_failures_and_timeouts_are_errors_not_values():
    services = FakeDsrServices()
    io = io_for(services)
    services.fail.add('get_output_register_int')
    with pytest.raises(DsrError) as err:
        io.get(30)
    assert err.value.reason == 'SERVICE_FAILED'
    services.fail.clear()
    services.timeout.update({'get_output_register_int', 'set_output_register_int'})
    with pytest.raises(DsrError) as err:
        io.get(30)
    assert err.value.reason == 'SERVICE_TIMEOUT'
    with pytest.raises(DsrError):
        io.set(10, 1)                                                # 썼는지 모르면 False가 아니라 예외


@pytest.mark.parametrize('robot_state,drl_state,expect', [
    (1, 1, True), (1, 3, True),                                      # STANDBY + STOP·LAST
    (1, 0, 'DRL_BUSY'), (1, 2, 'DRL_BUSY'),                          # PLAY·HOLD: 다른 DRL이 있다
    (2, 1, 'ROBOT_NOT_STANDBY'), (5, 3, 'ROBOT_NOT_STANDBY'),
])
def test_drl_start_checks_robot_and_drl_state_first(robot_state, drl_state, expect):
    services = FakeDsrServices(robot_state=robot_state, drl_state=drl_state)
    io = io_for(services)
    if expect is True:
        assert io.drl_start('# code') is True
        assert services.calls[-1] == ('DrlStart', '/dsr01/drl/drl_start', {'robot_system': 0, 'code': '# code'})
    else:
        with pytest.raises(DsrError) as err:
            io.drl_start('# code')
        assert err.value.reason == expect
        assert '/dsr01/drl/drl_start' not in services.names()
    assert services.names()[:1] == ['/dsr01/system/get_robot_state']


def test_resident_session_runs_over_service_register_io():
    """가짜 서비스(바이트 표현) 뒤의 가짜 상주 루프: 세션 시작·열기·파지가 RegisterIO 경로로 끝난다."""
    clock = FakeClock(steady=100.0)
    ctrl = FakeResidentController(clock, rtt_s=0.58, stone_pulse=659, present_pulse=600)
    services = FakeDsrServices(regs=ctrl)
    io = io_for(services)
    transport = ResidentLoopTransport(io, clock, sleep=clock.advance, drl_code='# fake')
    cfg = GripperCfg(close_pulse=750, response_timeout_s=8.0, read_period_s=0.05, loop_timeout_s=3.0,
                     grasp_stall_pulse=2)
    session = GripperSession(GripperAdapter(transport, TABLE, cfg, clock, sleep=clock.advance), 0.0005)
    session.start(robot_standby=io.robot_standby())
    assert ctrl.drl_starts == 1
    opened = session.ensure_open(TABLE.pulse_to_width(300))
    assert abs(opened.present_pulse - 300) <= 3
    grip = session.grasp(150)
    assert grip.holding is True and abs(TABLE.width_to_pulse(grip.width_m) - 659) <= 3
    assert all(c[2]['value'] & 0xFFFFFF == 0 and 0 <= c[2]['value'] >> 24 <= 127      # 최상위 바이트에 0~127
               for c in services.calls if c[1].endswith('set_output_register_int'))
    # 지령 중 레지스터 읽기 시간 초과: 성공으로 보고하지 않는다
    services.timeout.add('get_output_register_int')
    with pytest.raises(GripperError):
        session.ensure_open(TABLE.pulse_to_width(600))


@pytest.mark.parametrize('byte', [0, 1, 5, 88, 127])
def test_written_byte_reads_back_like_the_controller(byte):
    """2026-11-14 현장: 제어기는 쓰기 값의 최상위 바이트만 저장한다. 그대로 보낸 5는 0으로 읽혔다."""
    services = FakeDsrServices()
    io = io_for(services)
    assert io.set(22, byte) is True
    assert io.get(22) == byte
