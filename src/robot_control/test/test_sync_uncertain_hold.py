"""UNCERTAIN 계획 revision 보류.

벨트 `steady=false` 경보는 속도 공분산을 되돌려 planner revision을 잠깐 `UNCERTAIN`으로 만든다. 추종 코어는 그
revision으로 멈추지 않고 버리며, 마지막 OK 계획을 `uncertain_hold_s` 동안 등속으로 따라간다.
"""
from dataclasses import replace

import pytest

from robot_control.belt_sync import BeltSynchronizer, PlanSample, SyncConfig

UNCERTAIN = 3


def plan(**kw):
    return replace(PlanSample(1, 7, 2, 1, 1., 1., 2., 3., (.52, 0.), (.02, 0.)), **kw)


def uncertain(revision, t, **kw):
    return plan(revision=revision, generated_s=t, source_stamp_s=t, status=UNCERTAIN, position_std_m=.006, **kw)


def armed(first=None):
    c = BeltSynchronizer(SyncConfig())
    c.arm(1, 7, 2)
    if first is not False:
        assert c.accept(first or plan(), 1., 10.)
    return c


def step(c, ros, steady, pos=(.5, 0.)):
    return c.step(ros, steady, pos, ros, 'robot_base')


def test_uncertain_revision_is_discarded_without_latch():
    c = armed()
    assert not c.accept(uncertain(2, 1.), 1., 10.)     # 버림
    assert c.holding and c.fault is None
    out = step(c, 1., 10.)
    assert out.active and out.reason == 'OK'
    assert c.plan.revision == 1                         # 마지막 OK 계획을 계속 쓴다


def test_last_ok_plan_followed_at_constant_velocity_beyond_plan_age():
    c = armed()
    c.accept(uncertain(2, 1.), 1., 10.)
    for k in range(51):                                 # 0.5 s: max_plan_age_s 0.25 s를 넘는다
        out = step(c, 1. + k * .01, 10. + k * .01)
    assert out.active, out.reason
    assert out.target_xy_m == pytest.approx((.52 + .02 * (1.5 - 2.), 0.))


@pytest.mark.parametrize('dt,active', [(.599, True), (.600, True), (.601, False)])
def test_hold_limit_boundary(dt, active):
    c = armed()
    c.accept(uncertain(2, 1.), 1., 10.)
    for k in range(1, 60):
        assert step(c, 1. + k * .01, 10. + k * .01).active
    out = step(c, 1. + dt, 10. + dt)
    assert out.active is active
    if not active:
        assert out.reason == 'UNCERTAIN_HOLD'
        assert out.velocity_xy_mps == (0., 0.)


def test_later_uncertain_revisions_do_not_extend_hold():
    c = armed()
    c.accept(uncertain(2, 1.), 1., 10.)
    t = 0.
    for k in range(1, 62):
        t = k * .01
        if k in (30, 50):
            assert not c.accept(uncertain(k, 1. + t), 1. + t, 10. + t)
        out = step(c, 1. + t, 10. + t)
    assert not out.active and out.reason == 'UNCERTAIN_HOLD'


def test_ok_revision_clears_hold_and_restores_plan_age():
    c = armed()
    c.accept(uncertain(2, 1.), 1., 10.)
    for k in range(1, 51):
        step(c, 1. + k * .01, 10. + k * .01)
    assert c.accept(plan(revision=3, generated_s=1.5, source_stamp_s=1.5), 1.5, 10.5)
    assert not c.holding
    reasons = [step(c, 1.5 + k * .01, 10.5 + k * .01).reason for k in range(1, 27)]
    assert reasons[24] == 'OK'                           # OK 수신 뒤 0.25 s
    assert reasons[25] == 'PLAN_STALE'                   # 보류가 풀렸으므로 수신 감시가 다시 적용된다


def test_release_uncertainty_closed_during_hold():
    c = armed(plan(position_std_m=.0004))
    assert c.release_uncertainty_ok(1., 10.)
    c.accept(uncertain(2, 1.), 1., 10.)
    assert c.holding and c.fault is None
    assert not c.release_uncertainty_ok(1., 10.)


def test_invalid_uncertain_revision_still_latches():
    c = armed()
    assert not c.accept(uncertain(2, 1., frame_id='conveyor_frame'), 1., 10.)
    assert c.fault == 'PLAN_INVALID'


def test_uncertain_before_any_ok_plan():
    c = armed(first=False)
    assert not c.accept(uncertain(1, 1.), 1., 10.)
    assert step(c, 1.3, 10.3).reason == 'WAIT_PLAN'
    assert c.accept(plan(revision=2, generated_s=1.4, source_stamp_s=1.4), 1.4, 10.4)
    assert step(c, 1.4, 10.4).active

    c = armed(first=False)
    c.accept(uncertain(1, 1.), 1., 10.)
    assert step(c, 1.601, 10.601).reason == 'UNCERTAIN_HOLD'


def test_hold_config_validated():
    assert SyncConfig().uncertain_hold_s == pytest.approx(.6)
    for bad in (0., -1., float('nan')):
        with pytest.raises(ValueError):
            SyncConfig(uncertain_hold_s=bad)
