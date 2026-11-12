"""해제 게이트 순수 함수(SPEC-500 REQ-ROB-009, TEST_SPEC.md TC-ROB-009-04).

게이트는 commit 직전 한 번, `horizon_s`(transport `commit_lead_s`) 앞으로 외삽한 값으로 판정한다.
"""
from dataclasses import replace

import pytest

from robot_control.dynamic_place import (TRACK_COASTING, TRACK_LOST, TRACK_OBSERVED, BeltInfo, GateInputs,
                                         GateLimits, TrackInfo, release_gate)

pytestmark = [pytest.mark.level('U'), pytest.mark.req('REQ-ROB-009'), pytest.mark.tc('TC-ROB-009-04')]
LIM = GateLimits(e_xy_max_m=.00325, v_rel_max_mps=.005, tau_max_s=.3, position_std_max_m=.0005,
                 rim_clearance_m=.002)
NOW = 100.


def inputs(**kw):
    base = GateInputs(t_now_s=NOW, stone_xy_m=(.5, 0.), stone_v_xy_mps=(.02, 0.), cap_xy_m=(.5, 0.),
                      cap_v_xy_mps=(.02, 0.), fingertip_z_m=.10, fingertip_vz_mps=0., rim_z_m=.095,
                      position_std_m=.0003, valid_until_s=NOW + 1., goal_epoch=2, plan_epoch=2,
                      track=TrackInfo(7, 2, TRACK_OBSERVED, NOW - .03, NOW - .03),
                      belt=BeltInfo(NOW - .03, True), plan_hold=False)
    return replace(base, **kw)


def test_nominal_opens_and_reports_metrics():
    d = release_gate(inputs(), LIM, .03)
    assert d.ok and d.reason == '' and not d.terminal
    assert d.metrics['gate_e_xy_m'] == pytest.approx(0.)
    assert d.metrics['gate_v_rel_mps'] == pytest.approx(0.)
    assert d.metrics['gate_t_since_obs_s'] == pytest.approx(.06)
    assert d.metrics['gate_position_std_m'] == pytest.approx(.0003)
    assert d.metrics['gate_rim_clearance_m'] == pytest.approx(.005)
    assert d.metrics['gate_horizon_s'] == pytest.approx(.03)


@pytest.mark.parametrize('dx,ok', [(.00325 - 1e-6, True), (.00325, True), (.00325 + 1e-6, False)])
def test_e_xy_boundary(dx, ok):
    d = release_gate(inputs(stone_xy_m=(.5 + dx, 0.)), LIM, 0.)
    assert d.ok is ok
    assert ok or d.reason == 'GATE_E_XY'


def test_e_xy_extrapolated_to_horizon():
    # 지금 오차 0이지만 상대 속도 4 mm/s가 1 s 동안 쌓이면 4 mm
    d = release_gate(inputs(stone_v_xy_mps=(.024, 0.)), LIM, 1.)
    assert d.metrics['gate_e_xy_m'] == pytest.approx(.004)
    assert d.reason == 'GATE_E_XY'
    assert release_gate(inputs(stone_v_xy_mps=(.024, 0.)), LIM, .5).ok


@pytest.mark.parametrize('dv,ok', [(.005 - 1e-6, True), (.005, True), (.005 + 1e-6, False)])
def test_v_rel_boundary(dv, ok):
    d = release_gate(inputs(stone_v_xy_mps=(.02 + dv, 0.)), LIM, 0.)
    assert d.ok is ok
    assert ok or d.reason == 'GATE_V_REL'


@pytest.mark.parametrize('age,ok', [(.299, True), (.3, True), (.301, False)])
def test_coasting_age_at_horizon(age, ok):
    # 관측 나이는 해제 순간(t_now + horizon) 기준이다
    track = TrackInfo(7, 2, TRACK_COASTING, NOW, NOW + .05 - age)
    d = release_gate(inputs(track=track), LIM, .05)
    assert d.ok is ok
    assert ok or d.reason == 'GATE_COASTING'


def test_track_lost_stale_or_missing():
    assert release_gate(inputs(track=TrackInfo(7, 2, TRACK_LOST, NOW, NOW - 1.)), LIM, 0.).reason == 'GATE_TRACK_LOST'
    stale = TrackInfo(7, 2, TRACK_OBSERVED, NOW - .31, NOW - .31)
    assert release_gate(inputs(track=stale), LIM, 0.).reason == 'GATE_TRACK_STALE'
    assert release_gate(inputs(track=None), LIM, 0.).reason == 'GATE_TRACK_STALE'


@pytest.mark.parametrize('std,ok', [(.0005 - 1e-9, True), (.0005, True), (.0005 + 1e-9, False)])
def test_position_std_boundary(std, ok):
    d = release_gate(inputs(position_std_m=std), LIM, 0.)
    assert d.ok is ok
    assert ok or d.reason == 'GATE_POSITION_STD'


@pytest.mark.parametrize('clear,ok', [(.002 + 1e-6, True), (.002, True), (.002 - 1e-6, False)])
def test_rim_clearance_boundary_is_terminal(clear, ok):
    d = release_gate(inputs(fingertip_z_m=.095 + clear), LIM, 0.)
    assert d.ok is ok
    if not ok:
        assert d.reason == 'GATE_RIM_CLEARANCE' and d.terminal


def test_rim_clearance_extrapolated_with_descent_speed():
    d = release_gate(inputs(fingertip_z_m=.098, fingertip_vz_mps=-.02), LIM, .1)
    assert d.metrics['gate_rim_clearance_m'] == pytest.approx(.001)
    assert d.reason == 'GATE_RIM_CLEARANCE'


def test_validity_epoch_belt_and_hold():
    d = release_gate(inputs(valid_until_s=NOW + .049), LIM, .05)
    assert d.reason == 'GATE_EXPIRED' and d.terminal
    assert release_gate(inputs(valid_until_s=NOW + .05), LIM, .05).ok
    d = release_gate(inputs(track=TrackInfo(7, 3, TRACK_OBSERVED, NOW, NOW)), LIM, 0.)
    assert d.reason == 'GATE_EPOCH' and d.terminal
    assert release_gate(inputs(plan_epoch=3), LIM, 0.).reason == 'GATE_EPOCH'
    d = release_gate(inputs(belt=BeltInfo(NOW, False)), LIM, 0.)
    assert d.reason == 'GATE_BELT_UNSTEADY' and not d.terminal
    assert release_gate(inputs(belt=None), LIM, 0.).reason == 'GATE_BELT_STALE'
    assert release_gate(inputs(belt=BeltInfo(NOW - .31, True)), LIM, 0.).reason == 'GATE_BELT_STALE'
    d = release_gate(inputs(plan_hold=True), LIM, 0.)
    assert d.reason == 'GATE_PLAN_UNCERTAIN' and not d.terminal


def test_nonfinite_is_terminal():
    d = release_gate(inputs(stone_xy_m=(float('nan'), 0.)), LIM, 0.)
    assert not d.ok and d.reason == 'GATE_NONFINITE' and d.terminal


def test_limits_from_goal_fall_back_to_defaults():
    goal = {'e_xy_max': .002, 'v_rel_max': 0., 'tau_max': -1., 'position_std_max': .0004,
            'rim_clearance': float('nan')}
    lim = GateLimits.from_goal(goal, LIM)
    assert lim == GateLimits(.002, .005, .3, .0004, .002)
