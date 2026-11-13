"""Doosan 정적 backend 어댑터(SPEC-500 REQ-ROB-017, TEST_SPEC.md TC-ROB-017-01~03). **실기 미검증** 코드의 U 시험.

팔은 `fakes/doosan.py`(Doosan 단위 대역), 그리퍼는 가짜 상주 루프(`fakes/resident.py`) 위 `GripperSession`이다.
시간은 FakeClock이고 대기는 시계를 보낸다(실제 시간 없음).
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from fakes.clock import FakeClock
from fakes.doosan import MOVING, STANDBY, FakeDoosanArm
from fakes.resident import FakeResidentController
from robot_control.doosan_backend import DoosanBackend, DoosanBackendCfg
from robot_control.gripper_adapter import GripperAdapter, GripperCfg, PulseTable
from robot_control.gripper_session import GripperSession
from robot_control.motion_adapter import quat_to_matrix, rotation_error
from robot_control.resident_transport import ResidentLoopTransport
from robot_control.skill_logic import Result, StaticSkillCfg, StaticSkillExecutor

DOWN = (0.0, 1.0, 0.0, 0.0)
CFG = DoosanBackendCfg()
PULSE_MAX = 1150
TABLE = PulseTable.from_samples([(p, 0.106 * (1.0 - p / PULSE_MAX)) for p in range(0, PULSE_MAX + 1, 50)])


def rig(gripper=None, **arm_kw):
    clock = FakeClock(steady=100.0)
    arm = FakeDoosanArm(clock, **arm_kw)
    flags = {'cancel_at': math.inf}
    backend = DoosanBackend(arm, gripper, clock, clock.advance, CFG,
                            cancel_requested=lambda: clock.steady_now() >= flags['cancel_at'])
    return backend, arm, clock, flags


def rot_about_z(deg):
    """DOWN을 robot_base z축으로 deg만큼 돌린 자세."""
    h = math.radians(deg) / 2.0
    qz = np.array([0.0, 0.0, math.sin(h), math.cos(h)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = DOWN
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# ------------------------------------------------------------------ TC-ROB-017-01 지령 전 거부·변환·완료 판정
@pytest.mark.req('REQ-ROB-017')
@pytest.mark.tc('TC-ROB-017-01')
@pytest.mark.level('U')
def test_move_line_converts_units_and_completes_on_measured_pose():
    backend, arm, clock, _ = rig()
    t0 = clock.steady_now()
    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert out.completed and out.reason_code == ''
    assert out.pose_error_m <= CFG.reach_tol_m
    posx, vel, acc = arm.moves[0]
    assert posx[:3] == pytest.approx([500.0, 0.0, 250.0]) and posx[4] == pytest.approx(180.0)
    assert vel == pytest.approx([30.0, math.degrees(0.35)])           # [mm/s, deg/s]
    assert acc == pytest.approx([50.0, math.degrees(0.7)])            # [mm/s², deg/s²]
    assert arm.robot_state() == STANDBY
    assert clock.steady_now() - t0 >= 0.1 + 50.0 / 30.0                 # 도착 전에는 끝내지 않는다


@pytest.mark.req('REQ-ROB-017')
@pytest.mark.tc('TC-ROB-017-01')
@pytest.mark.level('U')
def test_pure_rotation_waits_for_orientation_and_standby():
    backend, arm, clock, _ = rig()
    t0 = clock.steady_now()
    target_q = rot_about_z(30.0)
    out = backend.move_line([0.5, 0.0, 0.2], target_q)                  # 위치 오차는 처음부터 0
    assert out.completed
    assert clock.steady_now() - t0 >= 0.1 + 30.0 / math.degrees(0.35) - 1e-6
    pos, q = backend.read_tcp_pose()
    assert rotation_error(quat_to_matrix(q), quat_to_matrix(target_q)) <= CFG.reach_tol_rad


@pytest.mark.req('REQ-ROB-017')
@pytest.mark.tc('TC-ROB-017-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('drl_state', [0, 2])                          # PLAY(상주 루프 포함)·HOLD
def test_refuses_motion_while_a_drl_program_runs(drl_state):
    backend, arm, _, _ = rig(drl_state=drl_state)
    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert (out.completed, out.reason_code) == (False, 'DRL_BUSY')
    assert arm.moves == []


@pytest.mark.req('REQ-ROB-017')
@pytest.mark.tc('TC-ROB-017-01')
@pytest.mark.level('U')
def test_refusals_and_unreached_moves_report_measured_error():
    backend, arm, _, _ = rig()
    arm.state_override = 5                                              # SAFE_STOP 등 STANDBY 아님
    assert backend.move_line([0.5, 0.0, 0.25], DOWN).reason_code == 'NOT_STANDBY'
    for target, quat in (([0.5, math.nan, 0.25], DOWN), ([0.5, 0.0, 0.25], (0.0, 0.0, 0.0, 0.0))):
        assert backend.move_line(target, quat).reason_code == 'RANGE'
    assert arm.moves == []
    arm.state_override = None
    arm.accept = False
    assert backend.move_line([0.5, 0.0, 0.25], DOWN).reason_code == 'MOVE_REJECTED'

    backend, arm, clock, _ = rig()
    arm.ignore_motion = True                                            # 받고도 움직이지 않음(2026-11-09 관찰)
    t0 = clock.steady_now()
    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert (out.completed, out.reason_code) == (False, 'MOVE_NOT_STARTED')
    assert out.pose_error_m == pytest.approx(0.05)
    assert CFG.start_grace_s <= clock.steady_now() - t0 <= CFG.start_grace_s + 2 * CFG.poll_s

    backend, arm, _, _ = rig()
    arm.stop_short_mm = 3.0
    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert (out.completed, out.reason_code) == (False, 'MOVE_NOT_REACHED')
    assert out.pose_error_m == pytest.approx(0.003, abs=1e-6)


# ------------------------------------------------------------------ TC-ROB-017-02 cancel·deadline·Stop
@pytest.mark.req('REQ-ROB-017', 'REQ-ROB-007')
@pytest.mark.tc('TC-ROB-017-02')
@pytest.mark.level('U')
def test_cancel_issues_move_stop_and_waits_for_stopped_state():
    backend, arm, clock, flags = rig()
    flags['cancel_at'] = clock.steady_now() + 0.6
    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert (out.completed, out.reason_code) == (False, 'CANCELED')
    assert len(arm.stops) == 1 and arm.robot_state() == STANDBY
    assert clock.steady_now() - flags['cancel_at'] <= 0.5                # REQ-ROB-007 cancel 0.5 s
    assert 0.0 < out.pose_error_m < 0.05                                # 멈춘 자리의 측정 오차


@pytest.mark.req('REQ-ROB-017', 'REQ-ROB-007')
@pytest.mark.tc('TC-ROB-017-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('why', ['cancel', 'deadline'])
def test_executor_reports_cancel_and_deadline_through_backend(why):
    clock = FakeClock(steady=100.0)
    arm = FakeDoosanArm(clock)
    t_req = clock.ros_now() + 0.6
    backend = DoosanBackend(arm, None, clock, clock.advance, CFG,
                            cancel_requested=lambda: clock.ros_now() >= t_req)   # 노드 `_backend_abort`와 같은 역할
    ex = StaticSkillExecutor(backend, StaticSkillCfg(pose_tolerance_m=0.002), clock)
    if why == 'cancel':
        res = ex.move_to_pose([0.5, 0.0, 0.25], DOWN, cancel_requested=lambda: clock.ros_now() >= t_req)
        assert (res.result, res.reason_code) == (Result.CANCELED, 'CANCELED')
    else:
        res = ex.move_to_pose([0.5, 0.0, 0.25], DOWN, deadline_s=t_req)
        assert (res.result, res.reason_code) == (Result.TIMEOUT, 'DEADLINE')
    assert clock.ros_now() - t_req <= 0.5 and len(arm.stops) == 1


@pytest.mark.req('REQ-ROB-017', 'REQ-ROB-007')
@pytest.mark.tc('TC-ROB-017-02')
@pytest.mark.level('U')
def test_unconfirmed_stop_is_a_fault_not_a_clean_cancel():
    clock = FakeClock(steady=100.0)
    arm = FakeDoosanArm(clock)
    arm.stop_ignored = True
    t_req = clock.ros_now() + 0.6
    backend = DoosanBackend(arm, None, clock, clock.advance, CFG, cancel_requested=lambda: clock.ros_now() >= t_req)
    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert (out.completed, out.reason_code) == (False, 'STOP_NOT_CONFIRMED')
    assert clock.ros_now() - t_req <= CFG.poll_s + CFG.cancel_stop_timeout_s + 1e-9
    assert arm.robot_state() == MOVING                                  # 실제로 아직 움직인다

    clock = FakeClock(steady=100.0)
    arm = FakeDoosanArm(clock)
    arm.stop_ignored = True
    t_req = clock.ros_now() + 0.6
    backend = DoosanBackend(arm, None, clock, clock.advance, CFG, cancel_requested=lambda: clock.ros_now() >= t_req)
    ex = StaticSkillExecutor(backend, StaticSkillCfg(pose_tolerance_m=0.002), clock)
    res = ex.move_to_pose([0.5, 0.0, 0.25], DOWN, cancel_requested=lambda: clock.ros_now() >= t_req)
    assert (res.result, res.reason_code) == (Result.FAULT, 'STOP_NOT_CONFIRMED')


@pytest.mark.req('REQ-ROB-017')
@pytest.mark.tc('TC-ROB-017-02')
@pytest.mark.level('U')
def test_move_timeout_stops_the_arm():
    backend, arm, clock, _ = rig()
    arm.speed_scale = 0.1                                               # 지령보다 10배 느림
    t0 = clock.steady_now()
    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert (out.completed, out.reason_code) == (False, 'MOVE_TIMEOUT')
    assert len(arm.stops) == 1 and arm.robot_state() == STANDBY
    assert clock.steady_now() - t0 <= 50.0 / 30.0 + CFG.move_timeout_margin_s + CFG.poll_s \
        + CFG.cancel_stop_timeout_s + 1e-9


@pytest.mark.req('REQ-ROB-017', 'REQ-ROB-007')
@pytest.mark.tc('TC-ROB-017-02')
@pytest.mark.level('U')
def test_stop_during_motion_latches_until_reset():
    backend, arm, clock, _ = rig()
    ex = StaticSkillExecutor(backend, StaticSkillCfg(pose_tolerance_m=0.002), clock)
    stop_res = []
    # Stop service 스레드가 이동 중에 들어온 것과 같은 자리: 다음 TCP 읽기 때 executor.stop() 실행
    arm.hooks.append((clock.steady_now() + 0.5, lambda: stop_res.append(ex.stop('TEST'))))
    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert (out.completed, out.reason_code) == (False, 'STOPPED')
    assert stop_res and stop_res[0].stopped is True and stop_res[0].result == Result.OK
    assert len(arm.stops) == 1 and backend.is_stopped()
    n = len(arm.moves)
    assert backend.move_line([0.5, 0.0, 0.25], DOWN).reason_code == 'STOPPED'
    assert backend.gripper_open(0.02).reason_code == 'STOPPED'
    assert len(arm.moves) == n                                          # 정지 유지 중 지령 없음
    backend.reset_stop()
    assert backend.move_line([0.5, 0.0, 0.25], DOWN).completed


@pytest.mark.req('REQ-ROB-017', 'REQ-ROB-007')
@pytest.mark.tc('TC-ROB-017-02')
@pytest.mark.level('U')
def test_is_stopped_reads_the_robot_state_each_time():
    backend, arm, clock, _ = rig()
    arm.stop_ignored = True
    arm.move_line_async([600.0, 0.0, 200.0, 0.0, 180.0, 0.0], [30.0, 20.0], [50.0, 40.0])
    clock.advance(0.2)
    assert backend.is_stopped() is False                                # MOVING
    ex = StaticSkillExecutor(backend, StaticSkillCfg(), clock)
    res = ex.stop('TEST')
    assert (res.result, res.reason_code, res.stopped) == (Result.FAULT, 'STOP_NOT_CONFIRMED', False)
    arm.fail_reads = True
    assert backend.is_stopped() is False                                # 읽지 못하면 정지로 보지 않는다


# ------------------------------------------------------------------ TC-ROB-017-03 측정값만 보고
def gripper_session(clock):
    ctrl = FakeResidentController(clock, rtt_s=0.58, stone_pulse=659, present_pulse=600)
    transport = ResidentLoopTransport(ctrl, clock, sleep=clock.advance, drl_code='# fake')
    cfg = GripperCfg(close_pulse=750, response_timeout_s=8.0, read_period_s=0.05, loop_timeout_s=3.0,
                     grasp_stall_pulse=2)
    return GripperSession(GripperAdapter(transport, TABLE, cfg, clock, sleep=clock.advance), 0.0005), ctrl


@pytest.mark.req('REQ-ROB-017')
@pytest.mark.tc('TC-ROB-017-03')
@pytest.mark.level('U')
def test_tcp_pose_is_measured_or_nan():
    backend, arm, _, _ = rig(posx=(400.0, -100.0, 300.0, 0.0, 180.0, 0.0))
    pos, q = backend.read_tcp_pose()
    assert pos == pytest.approx([0.4, -0.1, 0.3])
    assert rotation_error(quat_to_matrix(q), quat_to_matrix(DOWN)) < 1e-9
    arm.fail_reads = True
    pos, q = backend.read_tcp_pose()
    assert np.all(np.isnan(pos)) and np.all(np.isnan(q))                # 값을 지어내지 않는다


@pytest.mark.req('REQ-ROB-017')
@pytest.mark.tc('TC-ROB-017-01')
@pytest.mark.level('U')
def test_unusable_pose_before_motion_is_a_read_failure_without_command():
    """지령 전 측정이 비유한 값이면(client가 형식 검사를 거치지 않은 값을 돌려준 경우) 예외를 밖으로 내지 않고
    지령 없이 `TCP_READ_FAILED`다. 지령 뒤 측정 실패와 같은 이유 코드다."""
    backend, arm, _, _ = rig()
    arm.current_posx = lambda: [math.nan, 0.0, 200.0, 0.0, 180.0, 0.0]
    out = backend.move_line(np.array([0.5, 0.0, 0.25]), DOWN)
    assert (out.completed, out.reason_code) == (False, "TCP_READ_FAILED") and math.isinf(out.pose_error_m), out
    assert arm.moves == []


@pytest.mark.req('REQ-ROB-017', 'REQ-GRP-014')
@pytest.mark.tc('TC-ROB-017-03')
@pytest.mark.level('U')
def test_gripper_goes_through_the_session_and_reports_measured_values():
    clock = FakeClock(steady=100.0)
    session, ctrl = gripper_session(clock)
    arm = FakeDoosanArm(clock)
    backend = DoosanBackend(arm, session, clock, clock.advance, CFG)
    assert backend.gripper_open(0.03).reason_code == 'SESSION_NOT_STARTED'
    backend.start_gripper()
    assert session.started and ctrl.drl_starts == 1
    width = TABLE.pulse_to_width(300)
    out = backend.gripper_open(width)
    assert out.opened and abs(TABLE.width_to_pulse(out.width_m) - 300) <= 3
    triggers = ctrl.reg[10]
    out = backend.gripper_open(width)                                   # 이미 목표 폭: 지령 없이 방금 읽은 폭
    assert out.opened and ctrl.reg[10] == triggers and abs(out.width_m - width) <= 0.0005
    grip = backend.gripper_close(150)
    assert grip.holding is True and abs(TABLE.width_to_pulse(grip.width_m) - 659) <= 3
    assert grip.current_ma is not None and grip.current_ma > 0
    assert [e.name for e in session.events] == ['ensure_open', 'ensure_open', 'grasp']


@pytest.mark.req('REQ-ROB-017')
@pytest.mark.tc('TC-ROB-017-03')
@pytest.mark.level('U')
def test_gripper_failures_are_reported_without_values():
    backend, arm, clock, _ = rig()
    assert backend.gripper_open(0.03).reason_code == 'GRIPPER_UNAVAILABLE'
    g = backend.gripper_close(100)
    assert (g.width_m, g.current_ma, g.holding, g.reason_code) == (None, None, None, 'GRIPPER_UNAVAILABLE')
    session, ctrl = gripper_session(clock)
    backend = DoosanBackend(arm, session, clock, clock.advance, CFG)
    arm.state_override = 2                                              # STANDBY 아님: 루프를 띄우지 않는다
    with pytest.raises(Exception) as err:
        backend.start_gripper()
    assert getattr(err.value, 'reason', '') == 'ROBOT_NOT_STANDBY' and ctrl.drl_starts == 0
    arm.state_override = None
    backend.start_gripper()
    out = backend.gripper_open(0.2)                                     # 실측표 밖
    assert (out.opened, out.width_m, out.reason_code) == (False, None, 'WIDTH')
    g = backend.gripper_close(151)                                      # 전류 상한 밖
    assert (g.width_m, g.holding, g.reason_code) == (None, None, 'CURRENT')
