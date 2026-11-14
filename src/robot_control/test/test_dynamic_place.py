"""DynamicPlace 순서기. 시계는 FakeClock, transport는 `SimulatedPlaceTransport`(1차 지연 29 ms 가정값)다.
"""
import math
from dataclasses import replace

import numpy as np
import pytest

from robot_control.dynamic_place import (MODE_PREDICTED, MODE_SYNCHRONIZED, PHASES, TRACK_COASTING, TRACK_LOST,
                                         KeepSyncRetreat)
from robot_control.skill_logic import Result

from fakes.place_world import CFG, PERIOD_S, Rig

# phase 끝 feedback 키
KEYS = {
    MODE_SYNCHRONIZED: {'APPROACH': {'plan_valid', 'pose_error_m'}, 'SYNCHRONIZE': {'e_xy_m', 'v_rel_mps'},
                        'DESCEND': {'sync_ok', 'rim_clearance_m', 'height_error_m'},
                        'RELEASE': {'open_width_m', 'sync_held'},
                        'RETREAT': {'tcp_z_m', 'outside_workspace', 'stone_in_gripper'}},
}
KEYS[MODE_PREDICTED] = dict(KEYS[MODE_SYNCHRONIZED], SYNCHRONIZE={'entry_time_error_s', 'v_rel_mps'})
UNCERTAIN = 3


def at_phase(phase, then):
    """phase에 처음 들어간 뒤 tick마다 then(rig)을 부른다."""
    def hook(r):
        if r.seq.phase == phase or getattr(hook, 'on', False):
            hook.on = True
            return then(r)
        return None
    return hook


def commit_due(r):
    return r.world.release_s - r.transport.capabilities.commit_lead_s


# ------------------------------------------------------------------ 정상 순서
@pytest.mark.parametrize('mode', [MODE_SYNCHRONIZED, MODE_PREDICTED])
def test_nominal_sequence_feedback_and_single_commit(mode):
    r = Rig(mode)
    res = r.run()
    assert (res.result, res.reason_code) == (Result.OK, ''), res
    assert res.released and r.transport.commit_count == 1
    assert r.done_phases() == list(PHASES)
    for phase in PHASES:
        entered = [t for t, f in r.feedback if f.phase == phase and not f.phase_done]
        done = [(t, f) for t, f in r.feedback if f.phase == phase and f.phase_done]
        assert entered and len(done) == 1 and entered[0] <= done[0][0], phase
        assert KEYS[mode][phase] <= set(done[0][1].metrics), phase
    sync_fb = [t for t, f in r.feedback if f.phase == 'SYNCHRONIZE' and not f.phase_done]
    assert max(np.diff(sync_fb)) <= .05 + 1e-9                       # 20 Hz 이상
    assert all(math.isfinite(res.gate[k]) for k in
               ('gate_e_xy_m', 'gate_v_rel_mps', 'gate_t_since_obs_s', 'gate_position_std_m'))
    t_drop, xy = r.transport.stone_drop
    assert np.linalg.norm(np.array(xy) - r.world.cap_xy(t_drop)) <= CFG.e_xy_max_m
    assert r.feedback[-1][1].plan_revision >= 1


# ------------------------------------------------------------------ revision·epoch·만료
def test_lower_revision_ignored():
    r = Rig()

    def old(r):
        if r.seq.phase == 'SYNCHRONIZE' and not getattr(old, 'sent', False):
            old.sent = True
            rev = r.seq.plan_revision
            r.tick(plans=[r.world.plan(revision=rev - 1, release_xy_m=(.51, 0.))])
            assert r.seq.plan_revision == rev
            assert r.seq.release_xy == pytest.approx((.50, 0.))
        return None
    assert r.run(old).result == Result.OK and old.sent


@pytest.mark.parametrize('change', ['epoch', 'expiry'])
def test_epoch_change_or_expiry_fails_plan_invalid(change):
    if change == 'epoch':
        r = Rig()
        hook = at_phase('SYNCHRONIZE', lambda r: r.world.override.update(epoch=3))
    else:
        r = Rig(world_kw=dict(valid_extra_s=-1.4))           # valid_until = 출발 + 0.6 s
        hook = None
    res = r.run(hook)
    assert (res.result, res.reason_code) == (Result.FAILED, 'PLAN_INVALID')
    assert not res.released and r.transport.commit_count == 0


# ------------------------------------------------------------------ 시작 조건
def test_preconditions_and_missing_plan():
    r = Rig(mode=0)
    assert (r.run().result, r.result.reason_code) == (Result.PRECONDITION, 'MODE_UNSUPPORTED')
    for field, reason in (('fingertip_below_tcp_m', 'FINGERTIP_OFFSET_UNSET'), ('safe_z_m', 'SAFE_Z_UNSET'),
                          ('open_width_m', 'OPEN_WIDTH_UNSET')):
        r = Rig(cfg=replace(CFG, **{field: -1.}))
        assert (r.run().result, r.result.reason_code) == (Result.PRECONDITION, reason)
        assert r.transport.updates == 0
    # 계획상 핑거 끝 여유(0.1656 − 0.16 = 5.6 mm)가 h_clear보다 작으면 움직이지 않는다
    r = Rig(limits=replace(Rig().limits, rim_clearance_m=.006))
    assert (r.run().result, r.result.reason_code) == (Result.PRECONDITION, 'RIM_CLEARANCE_UNREACHABLE')
    r = Rig(goal_kw=dict(plan_id=5), world_kw=dict(plan_id=6))   # goal과 다른 plan_id만 들어온다
    res = r.run()
    assert (res.result, res.reason_code) == (Result.FAILED, 'NO_PLAN')
    assert r.result_t - r.world.t0 == pytest.approx(CFG.plan_wait_s, abs=2 * PERIOD_S)


def test_uncertain_longer_than_hold_fails():
    r = Rig()
    res = r.run(at_phase('SYNCHRONIZE', lambda r: setattr(r.world, 'status', UNCERTAIN)))
    assert (res.result, res.reason_code) == (Result.FAILED, 'UNCERTAIN_HOLD')
    assert r.transport.commit_count == 0


# ------------------------------------------------------------------ cancel·Stop·deadline
PHASE_CASES = ['APPROACH', 'SYNCHRONIZE', 'DESCEND', 'RELEASE', 'RETREAT']


@pytest.mark.parametrize('phase', PHASE_CASES)
@pytest.mark.parametrize('action', ['cancel', 'stop', 'deadline'])
def test_cancel_stop_deadline_each_phase(phase, action):
    r = Rig(world_kw=dict(steady=True))
    if phase == 'RELEASE':                 # commit 전 RELEASE에 머물게 벨트 경보를 건다
        def close(r):
            if r.seq.phase == 'DESCEND':
                r.world.steady = False
    else:
        close = None
    state = {}

    def hook(r):
        if close is not None:
            close(r)
        if r.seq.phase == phase and 'sent' not in state:
            state['sent'] = r.clock.ros_now() + PERIOD_S
            state['committed'] = r.transport.commit_count
            if action == 'deadline':
                r.seq.goal = replace(r.seq.goal, deadline_s=r.clock.ros_now() + PERIOD_S / 2)
                return None
            return action
        return None
    res = r.run(hook)
    expected = {'cancel': (Result.CANCELED, 'CANCELED'), 'stop': (Result.CANCELED, 'STOPPED'),
                'deadline': (Result.TIMEOUT, 'DEADLINE')}[action]
    assert (res.result, res.reason_code) == expected
    assert r.result_t - state['sent'] <= .5                  # 응답 0.5 s 안
    committed = phase == 'RETREAT'
    assert r.transport.commit_count == (1 if committed else 0)
    assert res.released is committed
    if action != 'stop':
        keep = phase != 'APPROACH'
        assert r.transport.aborts[-1][1] is keep


# ------------------------------------------------------------------ 게이트 항목별 위반
def _at_commit(change):
    def hook(r):
        if r.clock.ros_now() + PERIOD_S >= commit_due(r) - 1e-9 and not getattr(hook, 'done', False):
            hook.done = True
            change(r)
        return None
    return hook


VIOLATIONS = {
    'GATE_E_XY': lambda r: setattr(r.world, 'release_xy', (.506, 0.)),
    'GATE_V_REL': lambda r: setattr(r.world, 'v', (.03, 0.)),
    'GATE_TRACK_LOST': lambda r: setattr(r.world, 'track_status', TRACK_LOST),
    'GATE_COASTING': lambda r: (setattr(r.world, 'track_status', TRACK_COASTING),
                                setattr(r.world, 'obs_age_s', .3)),
    'GATE_TRACK_STALE': lambda r: setattr(r.world, 'track_age_s', .4),
    'GATE_POSITION_STD': lambda r: setattr(r.world, 'std', .0006),
    'GATE_EXPIRED': lambda r: setattr(r.world, 'valid_extra_s', -.001),
    'GATE_EPOCH': lambda r: setattr(r.world, 'track_epoch', 3),
    'GATE_RIM_CLEARANCE': lambda r: setattr(r.world, 'z_rel', r.world.z_rel + .005),
    'GATE_BELT_UNSTEADY': lambda r: setattr(r.world, 'steady', False),
    'GATE_BELT_STALE': lambda r: setattr(r.world, 'steady', None),
    'GATE_PLAN_UNCERTAIN': lambda r: setattr(r.world, 'status', UNCERTAIN),
}


@pytest.mark.parametrize('reason', sorted(VIOLATIONS))
def test_each_gate_violation_opens_zero_and_requests_l3(reason):
    r = Rig(MODE_PREDICTED, world_kw=dict(plan_every=1))
    res = r.run(_at_commit(VIOLATIONS[reason]))
    assert (res.result, res.reason_code) == (Result.FAILED, reason)
    assert r.transport.commit_count == 0 and r.transport.gripper_open_count == 0
    assert not res.released and res.abort_request == 3
    assert r.transport.aborts[-1][1] is True                 # 동기 유지한 채 멈춤(뒤이은 SafeRetreat keep_sync)
    assert res.gate['gate_ok'] == 0.


# ------------------------------------------------------------------ SYNCHRONIZED는 기다린다
@pytest.mark.parametrize('pulse', ['belt', 'uncertain'])
def test_synchronized_waits_through_short_pulse(pulse):
    r = Rig(world_kw=dict(plan_every=1))
    state = {}

    def hook(r):
        if r.seq.phase == 'RELEASE' and 't' not in state:
            state['t'] = r.clock.ros_now()
        if 't' in state:
            on = r.clock.ros_now() - state['t'] < .5
            if pulse == 'belt':
                r.world.steady = not on
            else:
                r.world.status = UNCERTAIN if on else 0
        return None
    res = r.run(hook)
    assert (res.result, res.reason_code) == (Result.OK, '')
    assert r.transport.commit_count == 1
    assert r.transport.commit_times[0] - state['t'] >= .5 - 1e-9


def test_synchronized_gives_up_when_window_ends():
    r = Rig()
    res = r.run(at_phase('RELEASE', lambda r: setattr(r.world, 'steady', False)))
    assert (res.result, res.reason_code) == (Result.FAILED, 'GATE_BELT_UNSTEADY')
    assert r.transport.commit_count == 0 and res.abort_request == 3


# ------------------------------------------------------------------ PREDICTED는 기다리지 않는다
def test_predicted_commits_at_planned_time_or_fails_without_waiting():
    r = Rig(MODE_PREDICTED)
    assert r.run().result == Result.OK
    assert r.transport.commit_times[0] == pytest.approx(commit_due(r), abs=PERIOD_S + 1e-9)
    assert r.transport.commit_times[0] >= commit_due(r) - 1e-9

    r = Rig(MODE_PREDICTED)
    res = r.run(at_phase('DESCEND', lambda r: setattr(r.world, 'steady', False)))
    assert (res.result, res.reason_code) == (Result.FAILED, 'GATE_BELT_UNSTEADY')
    assert r.result_t == pytest.approx(commit_due(r), abs=PERIOD_S + 1e-9)
    assert r.transport.commit_count == 0


# ------------------------------------------------------------------ 긴 horizon transport
def test_program_transport_commits_once_ahead_and_ignores_updates():
    kw = dict(mid_course_update=False, commit_lead_s=1.0)
    r = Rig(MODE_PREDICTED, transport_kw=kw)
    res = r.run()
    assert (res.result, res.reason_code) == (Result.OK, ''), res
    assert r.transport.commit_count == 1
    assert r.transport.commit_times[0] == pytest.approx(r.world.release_s - 1.0, abs=PERIOD_S + 1e-9)
    assert r.transport.updates_after_commit == 0
    assert r.done_phases() == list(PHASES)
    t_drop, xy = r.transport.stone_drop
    assert np.linalg.norm(np.array(xy) - r.world.cap_xy(t_drop)) <= CFG.e_xy_max_m

    r = Rig(MODE_SYNCHRONIZED, transport_kw=kw)
    assert (r.run().result, r.result.reason_code) == (Result.PRECONDITION, 'MODE_NEEDS_MID_COURSE_UPDATE')

    # 1 s 앞 예측 표준편차가 0.5 mm를 넘으면 commit하지 않는다
    r = Rig(MODE_PREDICTED, transport_kw=kw, world_kw=dict(std=.0008))
    res = r.run()
    assert (res.result, res.reason_code) == (Result.FAILED, 'GATE_POSITION_STD')
    assert r.transport.commit_count == 0


# ------------------------------------------------------------------ 동기 유지 후퇴
def _retreat_speed_errors(transport, t_from, safe_z, v):
    errs = []
    for s in transport.log:
        if s.t_s < t_from:
            continue
        errs.append(float(np.linalg.norm(np.array(s.vel_mps[:2]) - np.array(v))))
        if s.pos_m[2] >= safe_z:
            break
    return errs


def test_post_release_retreat_keeps_plan_speed():
    r = Rig()
    assert r.run().result == Result.OK
    t_retreat = next(t for t, f in r.feedback if f.phase == 'RETREAT' and not f.phase_done)
    errs = _retreat_speed_errors(r.transport, t_retreat, CFG.safe_z_m, r.world.v)
    assert len(errs) > 10 and max(errs) <= CFG.sync_speed_tol_mps
    done = next(f for _, f in r.feedback if f.phase == 'RETREAT' and f.phase_done)
    assert done.metrics['tcp_z_m'] >= CFG.safe_z_m
    assert done.metrics['outside_workspace'] == 1. and done.metrics['stone_in_gripper'] == 0.


def _abort_then_safe_retreat(xy_speed_scale=1.):
    r = Rig(MODE_PREDICTED)
    res = r.run(at_phase('DESCEND', lambda r: setattr(r.world, 'steady', False)))
    assert res.reason_code == 'GATE_BELT_UNSTEADY' and r.transport.aborts[-1][1] is True
    for _ in range(10):                                   # goal 사이 0.1 s
        r.clock.advance(PERIOD_S)
    r.transport.xy_speed_scale = xy_speed_scale
    retreat = KeepSyncRetreat(r.transport, r.seq.plan_velocity_xy, CFG)
    t0 = r.clock.ros_now()
    for _ in range(500):
        r.clock.advance(PERIOD_S)
        out = retreat.tick(r.clock.ros_now(), r.clock.steady_now())
        if out.result is not None:
            return r, out.result, t0
    raise AssertionError('후퇴 결과 없음')


def test_safe_retreat_after_pre_release_abort_keeps_sync():
    r, res, t0 = _abort_then_safe_retreat()
    assert (res.result, res.reason_code) == (Result.OK, ''), res
    assert res.metrics['tcp_z_m'] >= CFG.safe_z_m
    errs = _retreat_speed_errors(r.transport, t0, CFG.safe_z_m, r.world.v)
    assert len(errs) > 10 and max(errs) <= CFG.sync_speed_tol_mps


# ------------------------------------------------------------------ 상승 전 XY 속도 정착
def _cancel_then_keep_sync(phase, gap_s):
    """phase에서 DynamicPlace를 cancel하고 gap_s 뒤 SafeRetreat(keep_sync)의 동기 유지 후퇴를 끝까지 돌린다."""
    r = Rig(MODE_SYNCHRONIZED, start_xyz=(.40, 0., .20))     # 멀리서 다가와 SYNCHRONIZE 초입에 추종 보정이 큼
    ticks = []

    def hook(rr):
        if rr.seq.phase == phase:
            ticks.append(1)
            if len(ticks) >= (20 if phase == 'APPROACH' else 1):   # APPROACH는 계획을 받아 움직이는 중에
                return rr.tick(cancel=True)
        return None
    res = r.run(hook)
    assert (res.result, res.reason_code) == (Result.CANCELED, 'CANCELED')
    for _ in range(round(gap_s / PERIOD_S)):
        r.clock.advance(PERIOD_S)
    retreat = KeepSyncRetreat(r.transport, r.seq.plan_velocity_xy, CFG)
    t0, z0 = r.clock.ros_now(), r.transport.pos[2]
    for _ in range(600):
        out = retreat.tick(r.clock.ros_now(), r.clock.steady_now())
        if out.result is not None:
            return r, out.result, t0, z0
        r.clock.advance(PERIOD_S)
    raise AssertionError('후퇴 결과 없음')


@pytest.mark.parametrize('phase', ['APPROACH', 'SYNCHRONIZE'])
def test_keep_sync_retreat_settles_xy_speed_before_rising(phase):
    """해제 전 cancel 직후(0.03 s) SafeRetreat(keep_sync): 남은 추종 보정 속도(SYNCHRONIZE)나 정지(APPROACH의
    abort(keep_sync=false))에서 XY 속도가 계획 속도 ±1 mm/s에 들 때까지(최대 `keep_sync_settle_s`) z를 올리지 않고,
    올리는 동안의 표본은 모두 그 안이다(2026-11-13 I0: SYNCHRONIZE cancel 뒤 첫 표본부터 판정해 SYNC_SPEED → L4)."""
    r, res, t0, z0 = _cancel_then_keep_sync(phase, .03)
    assert (res.result, res.reason_code) == (Result.OK, ''), res
    assert res.metrics['tcp_z_m'] >= CFG.safe_z_m
    rising = [s for s in r.transport.log if s.t_s >= t0 and s.pos_m[2] > z0 + 1e-4 and s.pos_m[2] < CFG.safe_z_m]
    assert rising and max(math.hypot(s.vel_mps[0] - r.world.v[0], s.vel_mps[1] - r.world.v[1])
                          for s in rising) <= CFG.sync_speed_tol_mps
    assert rising[0].t_s - t0 <= CFG.keep_sync_settle_s + 2 * PERIOD_S


def test_keep_sync_never_succeeds_without_capability_or_speed():
    r, res, _ = _abort_then_safe_retreat(xy_speed_scale=.9)
    assert (res.result, res.reason_code) == (Result.FAILED, 'SYNC_SPEED')
    assert res.metrics['tcp_z_m'] >= CFG.safe_z_m        # 실패여도 안전 높이까지는 올린다

    r = Rig(transport_kw=dict(keep_sync_retreat=False))
    out = KeepSyncRetreat(r.transport, (.02, 0.), CFG).tick(r.clock.ros_now(), r.clock.steady_now())
    assert (out.result.result, out.result.reason_code) == (Result.PRECONDITION, 'SYNC_NOT_AVAILABLE')
    out = KeepSyncRetreat(Rig().transport, None, CFG).tick(r.clock.ros_now(), r.clock.steady_now())
    assert (out.result.result, out.result.reason_code) == (Result.PRECONDITION, 'SYNC_NOT_AVAILABLE')


# ------------------------------------------------------------------ 표본 시각이 tick 시각보다 늦음
@pytest.mark.parametrize('offset_s,ok', [(1e-6, True), (PERIOD_S, True), (.05, False)])
def test_sample_read_after_tick_time_is_not_stale(offset_s, ok):
    # 노드는 tick 시각을 먼저 읽고 poll()이 그 뒤에 TCP를 읽는다: 한 주기 안의 늦은 표본 시각은 미래 시각이 아니다
    r = Rig()
    r.transport.stamp_offset_s = offset_s
    res = r.run()
    if ok:
        assert res.result == Result.OK, res
    else:
        assert (res.result, res.reason_code) == (Result.FAILED, 'TCP_STALE')


# ------------------------------------------------------------------ 수신 쪽이 다시 재는 동기 유지 시간
def _receiver_hold(samples, e_max, v_max):
    """orchestrator `judge_synchronize`처럼 SYNCHRONIZE feedback 열(수신 시각, e_xy, v_rel)의 마지막 연속 통과 시간."""
    run = None
    for t, e, v in samples:
        good = e <= e_max + 1e-9 and v <= v_max + 1e-9
        run = (run if run is not None else t) if good else None
    return 0.0 if run is None else samples[-1][0] - run


@pytest.mark.parametrize('start_xyz', [None, (.455, .003, .21)], ids=['nominal', 'run_starts_between_samples'])
def test_synchronize_feedback_covers_hold_as_received(start_xyz):
    """MODE_SYNCHRONIZED: 연속 통과가 시작한 주기에 feedback을 내고, `sync_hold_s + sync_hold_margin_s`를 채운 뒤에
    끝낸다. 수신 쪽이 feedback 수신 시각으로 다시 잰 연속 시간은 수신 지연 차가 여유(0.03 s) 안이면 `sync_hold_s` 이상이다
    (2026-11-13 I0: 같은 0.3 s를 controller 주기 시각과 orchestrator 수신 시각으로 따로 재서 0.2998 s → TIMEOUT)."""
    r = Rig(MODE_SYNCHRONIZED, **({} if start_xyz is None else {'start_xyz': start_xyz}))
    assert r.run().result == Result.OK
    sync = [(t, f) for t, f in r.feedback if f.phase == 'SYNCHRONIZE']
    done_t, done = next((t, f) for t, f in sync if f.phase_done)
    samples = [(t, f.e_xy_m, f.v_rel_mps) for t, f in sync]
    lim = r.seq.goal.limits
    hold = done.metrics['hold_s']
    assert hold >= CFG.sync_hold_s + CFG.sync_hold_margin_s - 1e-9
    # controller의 연속 통과 시작 주기에 낸 표본이 있다(주기 표본 사이에서 시작해도 수신 쪽 시작이 늦어지지 않는다)
    assert any(abs(t - (done_t - hold)) <= 1e-9 for t, _, _ in samples)
    sent = _receiver_hold(samples, lim.e_xy_max_m, lim.v_rel_max_mps)
    assert sent >= hold - 1e-9
    # 수신 지연: 연속 통과 첫 표본은 늦게(여유 − 1 ms), 끝 표본은 바로 받은 경우가 가장 짧다
    first = done_t - sent
    worst = [(t + (CFG.sync_hold_margin_s - .001 if abs(t - first) <= 1e-9 else 0.), e, v) for t, e, v in samples]
    assert _receiver_hold(worst, lim.e_xy_max_m, lim.v_rel_max_mps) >= CFG.sync_hold_s


def test_keep_sync_already_at_safe_height_needs_no_motion():
    r = Rig(start_xyz=(.5, 0., CFG.safe_z_m + .01))
    out = KeepSyncRetreat(r.transport, (.02, 0.), CFG).tick(r.clock.ros_now(), r.clock.steady_now())
    assert (out.result.result, out.result.reason_code) == (Result.OK, '')
    assert r.transport.updates == 0 and out.result.metrics['tcp_z_m'] >= CFG.safe_z_m


@pytest.mark.parametrize('kw', [{}, dict(mid_course_update=False, commit_lead_s=1.0)])
def test_fake_transport_idle_after_goal_is_cheap_and_still(kw):
    r = Rig(MODE_PREDICTED, transport_kw=kw)
    assert r.run().result == Result.OK
    r.transport.poll()
    pos, steps = np.array(r.transport.pos), r.transport.steps
    r.clock.advance(300.)
    r.transport.poll()
    assert r.transport.steps - steps <= 1                     # 긴 정지 구간을 2 ms씩 적분하지 않는다
    assert np.array(r.transport.pos) == pytest.approx(pos, abs=1e-9)


def test_keep_sync_abort_hold_then_long_silence_is_bounded():
    r = Rig(MODE_PREDICTED)
    r.run(at_phase('DESCEND', lambda r: setattr(r.world, 'steady', False)))
    steps = r.transport.steps
    r.clock.advance(300.)
    s = r.transport.poll()
    hold_steps = int(r.transport.abort_sync_hold_s / r.transport.step_s) + 2
    assert r.transport.steps - steps <= hold_steps + 1        # 유지 시간만 단계 적분, 나머지는 한 번
    assert np.linalg.norm(s.vel_mps) < 1e-9
