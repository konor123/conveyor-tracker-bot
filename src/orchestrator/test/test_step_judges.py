"""단계 판정(SPEC-100 4.3·4.4절). 경계값은 TEST_SPEC.md 3.3절 공통 파라미터를 fixture로 고정한다."""
import dataclasses

import pytest
from spec_metrics import EXPECTED_METRIC_KEYS

from orchestrator import step_judges as sj
from orchestrator.abort_policy import AbortCtx, Event, decide
from orchestrator.common import AbortLevel, Result, State, metrics_to_record

MM = 1e-3

# TEST_SPEC.md 3.3절 공통 파라미터
D_STONE_MIN = 17 * MM
D_STONE_MAX = 18 * MM
DELTA_OPEN = 4 * MM
E_XY_MAX = 3.25 * MM
V_REL_MAX = 5 * MM
H_CLEAR = 2 * MM

P_PRE = sj.PrecheckParams(heartbeat_age_max_s=0.3, gripper_response_max_s=0.5, max_belt_speed_mps=0.06)
P_DET = sj.DetectParams(confidence_min_ratio=0.9, window_std_max_m=0.5 * MM, clearance_min_m=20 * MM)
P_PICK = sj.PickParams(d_stone_min_m=D_STONE_MIN, d_stone_max_m=D_STONE_MAX, width_tol_m=1 * MM,
                       grip_current_min_ma=100,
                       detection_age_max_s=1.0)
P_STB = sj.StandbyParams(pose_error_max_m=2 * MM)
P_WAIT = sj.WaitCapParams(observation_count_min_count=30, speed_std_max_mps=1 * MM, margin_s=1.0,
                          plan_lead_s=3.0)
P_PLAN = sj.PlanParams(time_margin_min_s=1.0, position_std_max_m=3.25 * MM, t_approach_s=1.0)
P_APP = sj.ApproachParams(pose_error_max_m=2 * MM)
P_SYNC = sj.SyncParams(e_xy_max_m=E_XY_MAX, v_rel_max_mps=V_REL_MAX, hold_s=0.3, timeout_s=2.0)
P_ENTRY = sj.EntryParams(entry_time_tol_s=0.05, v_rel_max_mps=V_REL_MAX)
P_DESC = sj.DescendParams(height_tol_m=0.5 * MM, h_clear_m=H_CLEAR)
P_STATIC = sj.StaticPlaceParams(target_error_max_m=1 * MM, height_tol_m=0.5 * MM, h_clear_m=H_CLEAR)
P_REL = sj.ReleaseParams(d_stone_max_m=D_STONE_MAX, delta_open_m=DELTA_OPEN, open_tol_m=1 * MM)
P_RET = sj.RetreatParams(safe_z_m=0.10)
P_VER = sj.VerifyParams(timeout_s=3.0)

from fakes.measures import (OK_APP, OK_DESC, OK_DET, OK_ENTRY, OK_PICK, OK_PLAN,  # noqa: E402
                            OK_PRE, OK_REL, OK_RET, OK_STATIC, OK_STB, OK_VER, OK_WAIT)

def rep(m, **kw):
    return dataclasses.replace(m, **kw)


def check(j, result, reason=None):
    assert j.result == result, j
    if reason is not None:
        assert j.reason_code == reason, j


# ---------------------------------------------------------------- PRECHECK
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("age, result", [(0.299, Result.OK), (0.300, Result.OK), (0.301, Result.FAILED)])
def test_precheck_heartbeat_age(age, result):
    j = sj.judge_precheck(rep(OK_PRE, heartbeat_age_max_s=age), P_PRE)
    check(j, result, None if result == Result.OK else 'HEARTBEAT_STALE')


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-02")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason", [
    ({'gripper_response_s': 0.5}, Result.OK, ''),
    ({'gripper_response_s': 0.501}, Result.FAILED, 'GRIPPER_SLOW'),
    ({'gripper_error_bits': 0x04}, Result.FAILED, 'GRIPPER_ERROR'),
    ({'gripper_open': False}, Result.FAILED, 'GRIPPER_NOT_OPEN'),
])
def test_precheck_gripper(change, result, reason):
    check(sj.judge_precheck(rep(OK_PRE, **change), P_PRE), result, reason)


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, reason", [
    ({'nodes_active': False}, 'NODE_INACTIVE'),
    ({'calibration_valid': False}, 'CALIBRATION_INVALID'),
    ({'calibration_valid': False, 'calibration_mismatch': True}, 'CALIBRATION_MISMATCH'),
    ({'protective_stop': True}, 'PROTECTIVE_STOP'),
    ({'robot_ready': False}, 'ROBOT_NOT_READY'),
    ({'gripper_alive': False}, 'GRIPPER_DEAD'),
    ({'belt_steady': False}, 'BELT_UNSTEADY'),
    ({'belt_speed_mps': 0.0601}, 'BELT_TOO_FAST'),
])
def test_precheck_other_reasons(change, reason):
    check(sj.judge_precheck(rep(OK_PRE, **change), P_PRE), Result.FAILED, reason)


# ---------------------------------------------------------------- DETECT_STONE
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-03")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason", [
    ({'confidence': 0.89}, Result.FAILED, 'LOW_CONFIDENCE'),
    ({'confidence': 0.90}, Result.OK, ''),
    ({'window_std_m': 0.50 * MM}, Result.OK, ''),
    ({'window_std_m': 0.51 * MM}, Result.FAILED, 'POSITION_UNSTABLE'),
    ({'stone_count': 0}, Result.FAILED, 'NO_STONE'),
    ({'position_valid': False}, Result.FAILED, 'POSITION_INVALID'),
    ({'clearance_m': 19 * MM}, Result.FAILED, 'CLEARANCE'),
])
def test_detect_stone(change, result, reason):
    check(sj.judge_detect_stone(rep(OK_DET, **change), P_DET), result, reason)


# ---------------------------------------------------------------- PICK_STONE
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-04")
@pytest.mark.level("U")
@pytest.mark.parametrize("width_mm, result", [
    (15.9, Result.FAILED), (16.0, Result.OK), (19.0, Result.OK), (19.1, Result.FAILED)])
def test_pick_width(width_mm, result):
    j = sj.judge_pick_stone(rep(OK_PICK, grip_width_m=width_mm * MM), P_PICK)
    check(j, result, None if result == Result.OK else 'GRIP_WIDTH')


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-05")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, reason", [
    ({'grip_current_ma': 99}, 'GRIP_CURRENT_LOW'),
    ({'stone_on_mat': True}, 'STONE_ON_MAT'),
    ({'stone_on_mat': True, 'grip_width_m': 0.0}, 'STONE_ON_MAT'),   # 매트에 남음이 간격보다 먼저
    ({'robot_error': True}, 'ROBOT_ERROR'),
    ({'gripper_error': True}, 'GRIPPER_ERROR'),
])
def test_pick_current_and_mat(change, reason):
    check(sj.judge_pick_stone(rep(OK_PICK, **change), P_PICK), Result.FAILED, reason)


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-05")
@pytest.mark.level("U")
def test_pick_current_at_minimum_is_ok():
    check(sj.judge_pick_stone(rep(OK_PICK, grip_current_ma=100), P_PICK), Result.OK)


# ---------------------------------------------------------------- MOVE_STANDBY
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-06")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason", [
    ({'pose_error_m': 2.0 * MM}, Result.OK, ''),
    ({'pose_error_m': 2.1 * MM}, Result.FAILED, 'POSE_ERROR'),
    ({'holding': False}, Result.FAILED, 'DROPPED'),
])
def test_move_standby(change, result, reason):
    check(sj.judge_move_standby(rep(OK_STB, **change), P_STB), result, reason)


# ---------------------------------------------------------------- WAIT_CAP
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-07")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason, detail", [
    ({'observation_count': 29}, Result.FAILED, 'WAIT', 'OBSERVATION_COUNT'),
    ({'observation_count': 30}, Result.OK, '', ''),
    ({'speed_std_mps': 1.00 * MM}, Result.OK, '', ''),
    ({'speed_std_mps': 1.01 * MM}, Result.FAILED, 'WAIT', 'SPEED_STD'),
    ({'belt_steady': False}, Result.FAILED, 'WAIT', 'BELT_UNSTEADY'),
    ({'track_observed': False}, Result.FAILED, 'WAIT', 'TRACK_NOT_OBSERVED'),
])
def test_wait_cap_track_ready(change, result, reason, detail):
    j = sj.judge_wait_cap(rep(OK_WAIT, **change), P_WAIT)
    check(j, result, reason)
    assert j.detail == detail


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-08")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason, detail", [
    ({'stone_inside': True}, Result.FAILED, 'CAP_NOT_EMPTY', ''),
    ({'stone_inside_valid': False}, Result.FAILED, 'PASS_SKIP', 'CAP_STATE_UNKNOWN'),
    ({'t_available_s': 4.99, 't_required_s': 4.0}, Result.FAILED, 'PASS_SKIP', 'TIME_MARGIN'),
    ({'t_available_s': 5.00, 't_required_s': 4.0}, Result.OK, '', ''),
    ({'t_available_s': 5.01, 't_required_s': 4.0}, Result.OK, '', ''),
])
def test_wait_cap_empty_and_budget(change, result, reason, detail):
    j = sj.judge_wait_cap(rep(OK_WAIT, **change), P_WAIT)
    check(j, result, reason)
    assert j.detail == detail


@pytest.mark.req("REQ-ORC-013")
@pytest.mark.tc("TC-ORC-013-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("t_to_window, result, reason, detail", [
    (3.001, Result.FAILED, 'WAIT', 'TOO_EARLY'),
    (3.000, Result.OK, '', ''),
    (2.999, Result.OK, '', ''),
])
def test_wait_cap_plan_lead(t_to_window, result, reason, detail):
    j = sj.judge_wait_cap(rep(OK_WAIT, t_to_window_s=t_to_window), P_WAIT)
    check(j, result, reason)
    assert j.detail == detail
    assert j.metrics['t_to_window_s'] == pytest.approx(t_to_window)


# ---------------------------------------------------------------- PLAN_INTERCEPT
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-09")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason", [
    ({'time_margin_s': 0.99}, Result.FAILED, 'TIME_MARGIN'),
    ({'time_margin_s': 1.00}, Result.OK, ''),
    ({'position_std_m': 3.25 * MM}, Result.OK, ''),
    ({'position_std_m': 3.26 * MM}, Result.FAILED, 'POSITION_STD'),
    ({'valid_until': 101.0}, Result.FAILED, 'PLAN_EXPIRES'),       # = now + T_approach, 경계는 '>'
    ({'valid_until': 101.001}, Result.OK, ''),
    ({'plan_ok': False}, Result.FAILED, 'PLAN_STATUS'),
    ({'epoch_match': False}, Result.FAILED, 'EPOCH_MISMATCH'),
])
def test_plan_intercept(change, result, reason):
    check(sj.judge_plan_intercept(rep(OK_PLAN, **change), P_PLAN), result, reason)


# ---------------------------------------------------------------- APPROACH
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-06")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason", [
    ({'pose_error_m': 2.0 * MM}, Result.OK, ''),
    ({'pose_error_m': 2.1 * MM}, Result.FAILED, 'POSE_ERROR'),
    ({'plan_valid': False}, Result.FAILED, 'PLAN_INVALID'),
])
def test_approach(change, result, reason):
    check(sj.judge_approach(rep(OK_APP, **change), P_APP), result, reason)


# ---------------------------------------------------------------- SYNCHRONIZE
def _samples(t_end, bad_at=(), rate_hz=100):
    n = int(round(t_end * rate_hz))
    out = []
    for i in range(n + 1):
        t = 200.0 + i / rate_hz
        bad = any(abs(i / rate_hz - b) < 1e-6 for b in bad_at)
        out.append(sj.SyncSample(t=t, e_xy_m=E_XY_MAX * (1.5 if bad else 0.5), v_rel_mps=1 * MM))
    return out


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-10")
@pytest.mark.level("U")
def test_synchronize_restarts_count_after_violation():
    assert sj.judge_synchronize(_samples(0.55, bad_at=[0.25]), P_SYNC).reason_code == 'WAIT'
    j = sj.judge_synchronize(_samples(0.56, bad_at=[0.25]), P_SYNC)
    check(j, Result.OK)
    assert j.metrics['elapsed_s'] == pytest.approx(0.56, abs=1e-6)
    assert j.metrics['hold_s'] == pytest.approx(0.30, abs=1e-6)


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-10")
@pytest.mark.level("U")
def test_synchronize_ok_without_violation():
    assert sj.judge_synchronize(_samples(0.29), P_SYNC).reason_code == 'WAIT'
    check(sj.judge_synchronize(_samples(0.30), P_SYNC), Result.OK)


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-10")
@pytest.mark.level("U")
def test_synchronize_timeout():
    every_third = [i / 100 for i in range(0, 201, 3)]       # 0.3 s 연속을 끝내 못 채움
    assert sj.judge_synchronize(_samples(1.99, bad_at=every_third), P_SYNC).reason_code == 'WAIT'
    j = sj.judge_synchronize(_samples(2.00, bad_at=every_third), P_SYNC)
    check(j, Result.TIMEOUT, 'TIMEOUT')


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-10")
@pytest.mark.level("U")
def test_synchronize_velocity_gate():
    s = [sj.SyncSample(t=200.0 + i / 100, e_xy_m=0.0, v_rel_mps=V_REL_MAX * 1.01) for i in range(31)]
    assert sj.judge_synchronize(s, P_SYNC).reason_code == 'WAIT'


@pytest.mark.req("REQ-ORC-001")
@pytest.mark.tc("TC-ORC-001-03")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason", [
    ({'entry_time_error_s': 0.05}, Result.OK, ''),
    ({'entry_time_error_s': -0.051}, Result.FAILED, 'ENTRY_TIME'),
    ({'v_rel_mps': 5.1 * MM}, Result.FAILED, 'V_REL'),
])
def test_predicted_entry(change, result, reason):
    check(sj.judge_predicted_entry(rep(OK_ENTRY, **change), P_ENTRY), result, reason)


# ---------------------------------------------------------------- DESCEND
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-11")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason", [
    ({'rim_clearance_m': 1.9 * MM}, Result.FAILED, 'RIM_CLEARANCE'),
    ({'rim_clearance_m': 2.0 * MM}, Result.OK, ''),
    ({'height_error_m': 0.51 * MM}, Result.FAILED, 'HEIGHT_ERROR'),
    ({'sync_ok': False}, Result.FAILED, 'SYNC_LOST'),
])
def test_descend(change, result, reason):
    check(sj.judge_descend(rep(OK_DESC, **change), P_DESC), result, reason)


@pytest.mark.req("REQ-ORC-001")
@pytest.mark.tc("TC-ORC-001-02")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, result, reason", [
    ({'target_error_m': 1.0 * MM}, Result.OK, ''),
    ({'target_error_m': 1.1 * MM}, Result.FAILED, 'TARGET_ERROR'),
    ({'rim_clearance_m': 1.9 * MM}, Result.FAILED, 'RIM_CLEARANCE'),
])
def test_static_place(change, result, reason):
    check(sj.judge_static_place(rep(OK_STATIC, **change), P_STATIC), result, reason)


# ---------------------------------------------------------------- RELEASE
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-12")
@pytest.mark.level("U")
@pytest.mark.parametrize("width_mm, result", [(20.9, Result.FAILED), (21.0, Result.OK)])
def test_release_open_width(width_mm, result):
    j = sj.judge_release(rep(OK_REL, open_width_m=width_mm * MM), P_REL)
    check(j, result, None if result == Result.OK else 'OPEN_WIDTH')


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-12")
@pytest.mark.level("U")
def test_release_sync_lost():
    check(sj.judge_release(rep(OK_REL, sync_held=False), P_REL), Result.FAILED, 'SYNC_LOST')


# ---------------------------------------------------------------- RETREAT
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-13")
@pytest.mark.level("U")
@pytest.mark.parametrize("change, reason", [
    ({'tcp_z_m': 0.0999}, 'Z_LOW'),
    ({'stone_in_gripper': True}, 'STONE_IN_GRIPPER'),
    ({'outside_workspace': False}, 'IN_WORKSPACE'),
])
def test_retreat_failure_is_l4(change, reason):
    check(sj.judge_retreat(rep(OK_RET, **change), P_RET), Result.FAILED, reason)
    c = AbortCtx(state=State.RETREAT, holding=False, released=True)
    assert decide(Event.RETREAT_FAILED, c) == AbortLevel.L4


# ---------------------------------------------------------------- VERIFY
@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-14")
@pytest.mark.level("U")
def test_verify_no_response_is_unknown():
    j = sj.judge_verify(sj.VerifyMeas(responded=False, verdict=0), P_VER)
    check(j, Result.TIMEOUT, 'VERIFY_TIMEOUT')
    assert j.detail == 'UNKNOWN'
    assert j.metrics['verdict'] == sj.VERDICT_UNKNOWN


# ---------------------------------------------------------------- 기록 키
CASES = [
    ('PRECHECK', lambda: sj.judge_precheck(OK_PRE, P_PRE)),
    ('DETECT_STONE', lambda: sj.judge_detect_stone(OK_DET, P_DET)),
    ('PICK_STONE', lambda: sj.judge_pick_stone(OK_PICK, P_PICK)),
    ('MOVE_STANDBY', lambda: sj.judge_move_standby(OK_STB, P_STB)),
    ('WAIT_CAP', lambda: sj.judge_wait_cap(OK_WAIT, P_WAIT)),
    ('PLAN_INTERCEPT', lambda: sj.judge_plan_intercept(OK_PLAN, P_PLAN)),
    ('APPROACH', lambda: sj.judge_approach(OK_APP, P_APP)),
    ('SYNCHRONIZE', lambda: sj.judge_synchronize(_samples(0.3), P_SYNC)),
    ('SYNCHRONIZE_PREDICTED', lambda: sj.judge_predicted_entry(OK_ENTRY, P_ENTRY)),
    ('DESCEND', lambda: sj.judge_descend(OK_DESC, P_DESC)),
    ('DESCEND_STATIC', lambda: sj.judge_static_place(OK_STATIC, P_STATIC)),
    ('RELEASE', lambda: sj.judge_release(OK_REL, P_REL)),
    ('RETREAT', lambda: sj.judge_retreat(OK_RET, P_RET)),
    ('VERIFY', lambda: sj.judge_verify(OK_VER, P_VER)),
]


@pytest.mark.req("REQ-ORC-004")
@pytest.mark.tc("TC-ORC-004-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("step, call", CASES, ids=[c[0] for c in CASES])
def test_metric_keys_match_spec(step, call):
    j = call()
    keys = set(j.metrics)
    if step == 'WAIT_CAP':
        budget = {k for k in keys if k.startswith('budget_')}
        assert budget == {f'budget_{n}_s' for n in OK_WAIT.budget_components}
        keys -= budget
    assert keys == EXPECTED_METRIC_KEYS[step]
    assert all(isinstance(v, float) for v in j.metrics.values())
    rk, rv = metrics_to_record(j)
    assert rk == list(j.metrics) and rv == list(j.metrics.values())


@pytest.mark.req("REQ-ORC-004")
@pytest.mark.tc("TC-ORC-004-01")
@pytest.mark.level("U")
def test_metric_keys_present_on_failure_too():
    j = sj.judge_pick_stone(rep(OK_PICK, grip_width_m=0.0), P_PICK)
    assert set(j.metrics) == EXPECTED_METRIC_KEYS['PICK_STONE']


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-01")
@pytest.mark.level("U")
def test_params_load_from_config(orch_cfg):
    j = orch_cfg['judge']
    assert sj.PrecheckParams.from_dict(j['precheck'], max_belt_speed_mps=0.06).heartbeat_age_max_s == 0.3
    assert sj.SyncParams.from_dict(j['sync']).hold_s == 0.3
    assert sj.ReleaseParams.from_dict(j['release']).delta_open_m == 0.004
