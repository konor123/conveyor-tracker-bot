"""TC-GRP-015-01, TC-GRP-016-01~03: 막힘 파지, 상주 루프 transport, 트리거 지연, 가짜 제어기 위 정적 사이클 시간.

시간은 `fakes/resident.py`의 모델 값이다. 실기 측정이 아니다.
"""
from __future__ import annotations

import pytest

from fakes.clock import FakeClock
from fakes.drl_env import ResidentDrlEnv
from fakes.resident import FakeResidentController
from robot_control.gripper_adapter import (MODBUS, GripperAdapter, GripperCfg, GripperFault, PreconditionError,
                                          PulseTable)
from robot_control.gripper_session import GripperSession
from robot_control.resident_transport import ResidentLoopTransport, ResidentRegs, render_resident_drl

PULSE_MAX = 1150
TABLE = PulseTable.from_samples([(p, 0.106 * (1.0 - p / PULSE_MAX)) for p in range(0, PULSE_MAX + 1, 50)])
OPEN_PULSE, CLOSE_PULSE, STONE_PULSE = 600, 750, 659
OPEN_M = TABLE.pulse_to_width(OPEN_PULSE)
# 2026-11-09 3회차 실기 로그의 이동 시간 [s](records/trials/2026-11-09-static-pick-place/pp-run3.jsonl)
MOVES = {'over_stone': 2.88, 'descend_grasp': 3.26, 'lift_stone': 3.25, 'over_cap': 1.76,
         'descend_release': 1.17, 'retreat': 1.17, 'park': 3.10}
MEASURED_GRIPPER_S = 19.76


def build(rtt_s=0.58, write_effect_s=None, stone=STONE_PULSE, init_status=1, start=True):
    clock = FakeClock(steady=100.0)
    ctrl = FakeResidentController(clock, rtt_s=rtt_s, write_effect_s=write_effect_s, stone_pulse=stone,
                                  present_pulse=OPEN_PULSE, init_status=init_status)
    transport = ResidentLoopTransport(ctrl, clock, sleep=clock.advance, drl_code='# fake')
    cfg = GripperCfg(close_pulse=CLOSE_PULSE, response_timeout_s=8.0, read_period_s=0.05, loop_timeout_s=3.0,
                     grasp_stall_pulse=2)
    session = GripperSession(GripperAdapter(transport, TABLE, cfg, clock, sleep=clock.advance), 0.0005)
    if start:
        session.start(robot_standby=True)
    return session, transport, ctrl, clock


def fake_cycle(rtt_s=0.58, write_effect_s=None) -> dict:
    """정적 사이클을 공통 순서로 돌린 시간. 이동은 실기 측정값만큼 시계를 보낸다."""
    session, _, ctrl, clock = build(rtt_s, write_effect_s, start=False)
    t0 = clock.steady_now()
    session.start(robot_standby=True)
    t_start = clock.steady_now()
    steps: dict[str, float] = {}

    def timed(name, fn):
        t = clock.steady_now()
        out = fn()
        steps[name] = clock.steady_now() - t
        return out

    timed('open_before_pick', lambda: session.ensure_open(OPEN_M))
    for name in ('over_stone', 'descend_grasp'):
        timed(name, lambda n=name: clock.advance(MOVES[n]))
    grip = timed('close_grasp', lambda: session.grasp(150))      # 실기 파지 전류(2026-11-09)
    session.arm_open(OPEN_M, 150)
    for name in ('lift_stone', 'over_cap', 'descend_release'):
        timed(name, lambda n=name: clock.advance(MOVES[n]))
    t_fire = clock.steady_now()
    timed('open_release', session.fire_open)
    for name in ('retreat', 'park'):
        timed(name, lambda n=name: clock.advance(MOVES[n]))
    gripper = sum(v for k, v in steps.items() if k not in MOVES)
    return {'steps': steps, 'session_start_s': t_start - t0, 'gripper_s': gripper,
            'cycle_s': clock.steady_now() - t_start, 'holding': grip.holding,
            'release_delay_s': ctrl.motion_starts[-1] - t_fire, 'final_pulse': ctrl.pulse_at(clock.steady_now())}


@pytest.mark.req('REQ-GRP-016')
@pytest.mark.tc('TC-GRP-016-01')
@pytest.mark.level('U')
def test_transport_maps_registers_and_starts_loop_once():
    session, transport, ctrl, _ = build()
    r = ResidentRegs()
    assert ctrl.drl_starts == 1 and session.started
    assert transport.read_register(MODBUS.present_position) == OPEN_PULSE
    assert transport.read_register(MODBUS.operating_mode) == 5
    with pytest.raises(ValueError):
        transport.read_register(MODBUS.profile_velocity)          # 루프가 올리지 않는 값은 지어내지 않는다
    assert transport.write_register(MODBUS.operating_mode, 5) is False        # EEPROM 쓰기 경로 없음

    session.arm_open(TABLE.pulse_to_width(300), 100)
    assert (ctrl.reg[r.pos_hi] * 128 + ctrl.reg[r.pos_hi + 1], ctrl.reg[r.cur_hi] * 128 + ctrl.reg[r.cur_hi + 1]) \
        == (300, 100)
    assert ctrl.reg[r.arm] == 1 and ctrl.reg[r.seq] == 0          # 예약만으로 seq는 바뀌지 않는다
    assert all(0 <= ctrl.reg[a] <= 127 for a in (r.pos_hi, r.pos_hi + 1, r.cur_hi, r.cur_hi + 1, r.seq))


@pytest.mark.req('REQ-GRP-016')
@pytest.mark.tc('TC-GRP-016-01')
@pytest.mark.level('U')
def test_loop_init_failure_is_a_fault_and_drl_text_matches_registers():
    with pytest.raises(GripperFault) as err:
        build(init_status=246)
    assert err.value.reason == 'RESIDENT_INIT'

    code = render_resident_drl()
    assert '__' not in code
    r = ResidentRegs()
    for needle in (f'get_output_register_int({r.seq})', f'reg7({r.pos_hi})', f'reg7({r.cur_hi})',
                   f'get_output_register_int({r.arm})', f'get_output_register_int({r.stop})',
                   f'set_output_register_int({r.alive}, alive)', f'set_output_register_int({r.st_pos_hi},',
                   f'set_output_register_int({r.st_cur_hi},', f'set_output_register_int({r.error},',
                   f'set_output_register_int({r.init}, status)', f'set_output_register_int({r.done_seq}, done_seq)',
                   f'set_output_register_int({r.cmd_result}, result)'):
        assert needle in code, needle
    with pytest.raises(ValueError):
        render_resident_drl(read_timeout_s=1.0)


@pytest.mark.req('REQ-GRP-016')
@pytest.mark.tc('TC-GRP-016-01')
@pytest.mark.level('U')
def test_loop_write_failure_is_a_fault_for_that_command_only():
    session, _, ctrl, _ = build()
    ctrl.fail_writes = True                            # 루프의 Modbus 쓰기가 실패한다(대기 끝까지 유지)
    with pytest.raises(GripperFault) as err:
        session.grasp(100)
    assert err.value.reason == 'RESIDENT_WRITE_FAILED'
    assert ctrl.motion_starts == []                    # 그리퍼는 목표를 받지 않았다

    ctrl.fail_writes = False                           # 앞 지령의 실패가 다음 지령에 남지 않는다
    held = session.grasp(100)
    assert (held.reason_code, held.holding) == ('STALLED', True)


@pytest.mark.req('REQ-GRP-016')
@pytest.mark.tc('TC-GRP-016-01')
@pytest.mark.level('U')
def test_success_waits_for_the_loop_to_execute_the_command():
    session, transport, ctrl, clock = build()
    session.grasp(100)                                 # 공깃돌을 쥔 상태(이미 목표·막힘 상태)
    ctrl.fail_writes = True
    with pytest.raises(GripperFault) as err:           # 실행 전 표본으로 성공을 내지 않는다
        session.grasp(120)                             # 전류를 바꾸는 재파지
    assert err.value.reason == 'RESIDENT_WRITE_FAILED'
    ctrl.fail_writes = False
    clock.advance(3.0)
    assert session.grasp(100).reason_code == 'STALLED'  # 다음 지령은 앞 실패를 받지 않는다

    session, transport, ctrl, clock = build(stone=None)
    seq_before = ctrl.reg[ctrl.regs.done_seq]
    session.ensure_open(TABLE.pulse_to_width(OPEN_PULSE + 100))
    assert ctrl.reg[ctrl.regs.done_seq] != seq_before  # 루프가 실행한 뒤에 돌아왔다
    assert ctrl.motion_starts and ctrl.motion_starts[-1] <= clock.steady_now()

    # 이미 목표 위치여도 실행을 본 순간의 표본(실행 전에 읽은 값)으로 끝내지 않고, 그 뒤 새 표본까지 기다린다
    session, transport, ctrl, clock = build(stone=None)
    result = session.adapter.open(OPEN_M, 100)
    assert result.t_done >= ctrl.motion_starts[-1] + ctrl.rtt_s - 1e-9


@pytest.mark.req('REQ-GRP-016')
@pytest.mark.tc('TC-GRP-016-04')
@pytest.mark.level('U')
def test_timeouts_shorter_than_the_loop_sample_period_are_rejected():
    clock = FakeClock(steady=100.0)
    transport = ResidentLoopTransport(FakeResidentController(clock), clock, sleep=clock.advance,
                                      drl_code='# fake', sample_period_s=0.58)
    with pytest.raises(PreconditionError) as err:              # 기본값(응답 0.5 s, 생존 0.1 s)
        GripperAdapter(transport, TABLE, GripperCfg(), clock)
    assert err.value.reason == 'TIMEOUT_BELOW_LOOP_PERIOD'
    ok = dict(close_pulse=CLOSE_PULSE, response_timeout_s=3 * 0.58, loop_timeout_s=2 * 0.58)
    GripperAdapter(transport, TABLE, GripperCfg(**ok), clock)  # 경계: 응답 3주기, 생존 2주기
    for bad in (dict(ok, response_timeout_s=3 * 0.58 - 0.001), dict(ok, loop_timeout_s=2 * 0.58 - 0.001)):
        with pytest.raises(PreconditionError):
            GripperAdapter(transport, TABLE, GripperCfg(**bad), clock)


@pytest.mark.req('REQ-GRP-016')
@pytest.mark.tc('TC-GRP-016-01')
@pytest.mark.level('U')
def test_resident_drl_body_reports_each_command_result_before_its_seq():
    """DRL 본문을 그대로 실행한다: 정상 지령, 같은 전류, 전류 쓰기 실패, 위치 쓰기 실패."""
    def run(**kw):
        env = ResidentDrlEnv(**kw)
        env.command(at_loop=3, seq=1, pulse=750, current=120)
        env.command(at_loop=10, seq=2, pulse=600, current=120)      # 같은 전류: 전류 쓰기 생략
        env.run(render_resident_drl())
        return env

    def command_sets(env, seq):
        k = env.sets.index((39, seq))
        return env.sets[k - 1], env.sets[k]

    env = run()
    assert env.reg[36] == 1
    assert command_sets(env, 1) == ((40, 0), (39, 1)) and command_sets(env, 2) == ((40, 0), (39, 2))
    assert env.modbus_writes == [(6, 275, 120), (16, 282, 750), (16, 282, 600)]

    env = run(fail_current_write=True)                               # 전류를 못 바꾸면 위치도 쓰지 않는다
    assert command_sets(env, 1) == ((40, 242), (39, 1))
    assert [w for w in env.modbus_writes if w[0] == 16] == []

    env = run(fail_position_write=True)
    assert command_sets(env, 1) == ((40, 242), (39, 1)) and command_sets(env, 2) == ((40, 242), (39, 2))


@pytest.mark.req('REQ-GRP-016')
@pytest.mark.tc('TC-GRP-016-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('write_effect_s', [None, 0.01])
def test_armed_trigger_delay_is_spin_plus_one_write(write_effect_s):
    session, _, ctrl, clock = build(write_effect_s=write_effect_s)
    session.grasp(100)                                 # 전류 100 mA가 이미 쓰인 상태를 만든다
    session.arm_open(OPEN_M, 100)
    clock.advance(3.0)                                 # 팔이 이동하는 동안 루프는 Modbus 읽기를 멈추고 기다린다
    trips = ctrl.modbus_round_trips
    t_fire = clock.steady_now()
    session.fire_open()
    delay = ctrl.motion_starts[-1] - t_fire
    assert delay <= ctrl.spin_s + ctrl.write_effect_s + 1e-9
    assert ctrl.pulse_at(clock.steady_now()) == OPEN_PULSE
    assert ctrl.modbus_round_trips - trips <= 3        # 위치 쓰기 1 + 확인 읽기


@pytest.mark.req('REQ-GRP-015')
@pytest.mark.tc('TC-GRP-015-01')
@pytest.mark.level('U')
def test_grasp_is_judged_by_stall_not_by_reaching_the_end():
    session, _, ctrl, clock = build()
    held = session.grasp(100)
    assert (held.reason_code, held.holding) == ('STALLED', True)
    assert ctrl.pulse_at(clock.steady_now()) == STONE_PULSE

    session, _, ctrl, clock = build(stone=None)        # 손가락 사이가 비었다
    empty = session.grasp(100)
    assert (empty.reason_code, empty.holding) == ('REACHED_END', False)
    assert ctrl.pulse_at(clock.steady_now()) == CLOSE_PULSE


@pytest.mark.req('REQ-GRP-016')
@pytest.mark.tc('TC-GRP-016-03')
@pytest.mark.level('U')
def test_fake_static_cycle_spends_far_less_time_on_the_gripper():
    out = fake_cycle()
    assert out['holding'] is True and out['final_pulse'] == OPEN_PULSE
    assert out['steps']['open_before_pick'] == pytest.approx(0.0)       # 이미 열려 있어 생략
    assert out['gripper_s'] < 0.3 * MEASURED_GRIPPER_S
    assert out['cycle_s'] == pytest.approx(sum(MOVES.values()) + out['gripper_s'])
