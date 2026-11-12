"""SPEC-400 pure-unit acceptance tests; all clocks are explicit float seconds."""
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from intercept_planner.time_budget import evaluate_budget
from intercept_planner.prediction import TrackState, predict
from intercept_planner.intercept import PlanRequest, PlannerParams, compute_plan
from intercept_planner.plan_session import PlanSession
from intercept_planner.tau import tau_max


@pytest.fixture
def params():
    cfg = json.loads((Path(__file__).parents[1] / 'config/planner.yaml').read_text())
    return PlannerParams.from_mapping(cfg)


@pytest.fixture
def track():
    return TrackState(7, 0, 0, 10., np.array([.18, 0.]),
                      np.array([.02, 0.]), np.eye(4) * 1e-10, 30., 10.)


@pytest.fixture
def plan_request():
    return PlanRequest(target_track_id=7, target_epoch=0, mode=2,
                       deadline=30., earliest_release=0.)


@pytest.mark.req('REQ-PLN-001')
@pytest.mark.tc('TC-PLN-001-01')
@pytest.mark.level('U')
def test_budget_formula(params):
    b = evaluate_budget(0., .02, params.x_exit_m, params.components,
                        params.margin_s, v_min=params.v_min_mps)
    assert b.t_available == pytest.approx(15., abs=1e-9)
    assert b.t_required == pytest.approx(4., abs=1e-9)
    assert b.ok and b.reason == ''
    assert b.components == params.components


@pytest.mark.req('REQ-PLN-001')
@pytest.mark.tc('TC-PLN-001-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('delta,ok', [(-1e-6, False), (0., True), (1e-6, True)])
def test_budget_boundary(delta, ok):
    b = evaluate_budget(0., 1., 5. + delta, {'travel': 4.}, 1.)
    assert b.ok is ok
    assert b.reason == ('' if ok else 'TIME_MARGIN')


@pytest.mark.req('REQ-PLN-001')
@pytest.mark.tc('TC-PLN-001-03')
@pytest.mark.level('U')
@pytest.mark.parametrize('speed,reason', [(-.002001, 'BELT_REVERSED'),
    (-.002, 'BELT_REVERSED'), (-.001999, 'BELT_STOPPED'),
    (0., 'BELT_STOPPED'), (.001999, 'BELT_STOPPED'), (.002, 'BELT_STOPPED'), (.002001, '')])
def test_belt_boundary(speed, reason):
    b = evaluate_budget(0., speed, .30, {'travel': 4.}, 1., v_min=.002)
    assert b.reason == reason
    assert b.ok is (not reason)
    if reason:
        assert b.t_available == 0.


@pytest.mark.req('REQ-PLN-002')
@pytest.mark.tc('TC-PLN-002-01')
@pytest.mark.level('U')
def test_prediction_state_time(track):
    pos, _ = predict(replace(track, t_state=9.9), 12.)
    assert pos == pytest.approx(track.pos + track.vel * 2.1, abs=1e-9)


@pytest.mark.req('REQ-PLN-003')
@pytest.mark.tc('TC-PLN-003-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('seed', range(20))
def test_covariance(track, seed):
    rng = np.random.default_rng(seed)
    a = rng.normal(size=(4, 4))
    cov = a @ a.T
    dt = .7
    _, actual = predict(replace(track, cov=cov), track.t_state + dt)
    h = np.array([[1., 0., dt, 0.], [0., 1., 0., dt]])
    assert actual == pytest.approx(h @ cov @ h.T, abs=1e-12)


@pytest.mark.req('REQ-PLN-004')
@pytest.mark.tc('TC-PLN-004-01')
@pytest.mark.level('U')
def test_target_offset_transform(track, plan_request, params):
    transform = np.array([[0., -1., 0., .4], [1., 0., 0., -.2],
                          [0., 0., 1., .03], [0., 0., 0., 1.]])
    p = replace(params, tcp_offset_m=(.001, -.0005), T_base_conveyor=transform)
    track = replace(track, z=.022)                 # 뚜껑 테두리 윗면(conveyor_frame)
    reply = compute_plan(track, 10., plan_request, p)
    assert reply.result == 0
    plan = reply.plan
    pos, _ = predict(track, plan.release_time)
    expected = transform @ np.array([*pos, .022, 1.])
    # z = 테두리 z(robot_base) + release_above_rim_m: 0.03 + 0.022 + 0.1656
    assert plan.target_tcp_pose.position == pytest.approx(
        [expected[0] - .001, expected[1] + .0005, .2176], abs=1e-9)
    assert p.release_above_rim_m == pytest.approx(.1656)
    assert plan.target_tcp_pose.orientation == pytest.approx([1., 0., 0., 0.])
    assert plan.target_velocity == pytest.approx([0., .02, 0.], abs=1e-9)


@pytest.mark.req('REQ-PLN-005')
@pytest.mark.tc('TC-PLN-005-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('side', ['min', 'max'])
@pytest.mark.parametrize('delta', [-1e-6, 0., 1e-6])
def test_workspace_boundary(track, plan_request, params, axis, side, delta):
    target = compute_plan(track, 10., plan_request, params).plan.target_tcp_pose.position
    bounds = list(getattr(params, f'workspace_{side}_m'))
    bounds[axis] = target[axis] + delta
    reply = compute_plan(track, 10., plan_request,
                         replace(params, **{f'workspace_{side}_m': bounds}))
    outside = delta > 0 if side == 'min' else delta < 0
    assert reply.plan.status == (2 if outside else 0)
    assert reply.reason_code == ('UNREACHABLE' if outside else '')


@pytest.mark.req('REQ-PLN-008')
@pytest.mark.tc('TC-PLN-008-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('std,status', [(.00324, 0), (.00325, 0), (.00326, 3)])
def test_uncertainty(track, plan_request, params, std, status):
    cov = np.diag([std ** 2, std ** 2, 0., 0.])
    reply = compute_plan(replace(track, cov=cov), 10., plan_request, params)
    assert reply.plan.position_std == pytest.approx(std, abs=1e-12)
    assert reply.plan.status == status


@pytest.mark.req('REQ-PLN-009')
@pytest.mark.tc('TC-PLN-009-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('earliest', [12.799999, 12.8, 12.800001, 15.])
def test_earliest_release(track, plan_request, params, earliest):
    reply = compute_plan(track, 10., replace(plan_request, earliest_release=earliest), params)
    plan = reply.plan
    assert plan.release_time == pytest.approx(max(12.8, earliest))
    assert plan.valid_until == pytest.approx(plan.release_time + 1.2)
    assert plan.time_margin == pytest.approx(6. - max(4., plan.release_time - 10. + 1.2))
    assert plan.status == (1 if earliest == 15. else 0)


@pytest.mark.req('REQ-PLN-010')
@pytest.mark.tc('TC-PLN-010-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('a,dv', [(0., .0005), (.002, .0005), (.002, 0.), (0., 0.)])
def test_tau(a, dv):
    actual = tau_max(.00275, .0003, dv, a, .02, .005)
    remaining = .00275 - .0003 - .02 * .005
    if a:
        expected = (-dv + np.sqrt(dv ** 2 + 2 * a * remaining)) / a
    elif dv:
        expected = 4.7
    else:
        expected = float('inf')
    assert actual == pytest.approx(expected, abs=1e-6)
    if np.isfinite(actual):
        assert .0003 + abs(dv) * actual + .5 * a * actual ** 2 + .02 * .005 == pytest.approx(.00275)


@pytest.mark.req('REQ-PLN-010')
@pytest.mark.level('U')
@pytest.mark.parametrize('e_pos', [.002649999, .00265, .002650001])
def test_tau_exhausted(e_pos):
    assert tau_max(.00275, e_pos, -.0005, 0., -.02, .005) == pytest.approx(
        max(0., (.00275 - e_pos - .0001) / .0005), abs=1e-12)


@pytest.mark.req('REQ-PLN-007')
@pytest.mark.level('U')
@pytest.mark.parametrize('stage,code,reason,status', [
    (1, 2, '', None), (2, 3, 'MODE_UNSUPPORTED', None),
    (3, 1, 'EPOCH_MISMATCH', None), (4, 1, 'TRACK_NOT_FOUND', None),
    (5, 1, 'TRACK_LOST', 4), (6, 1, 'BELT_STOPPED', 1),
    (7, 1, 'UNREACHABLE', 2), (8, 1, 'UNCERTAIN', 3), (9, 0, '', 0)])
def test_priority(track, plan_request, params, stage, code, reason, status):
    # Every later failure is present; the earliest failing row must win.
    req = replace(plan_request, deadline=9. if stage <= 1 else 30.,
                  mode=0 if stage <= 2 else 2, target_epoch=1 if stage <= 3 else 0)
    trk = replace(track, status=2 if stage <= 5 else 0,
                  vel=np.array([0. if stage <= 6 else .02, 0.]),
                  cov=np.eye(4) * (1. if stage <= 8 else 1e-10))
    if stage <= 4:
        trk = None
    p = replace(params, workspace_max_m=(.21 if stage <= 7 else .70, .4, .4))
    reply = compute_plan(trk, 10., req, p)
    assert (reply.result, reply.reason_code) == (code, reason)
    assert (reply.plan.status if reply.plan else None) == status


@pytest.mark.req('REQ-PLN-006')
@pytest.mark.level('U')
def test_session_revisions_fixed_release(track, plan_request, params):
    session = PlanSession(params)
    first = session.compute(track, 10., plan_request).plan
    second = session.update(replace(track, t_state=10.1, pos=track.pos + .002), 10.1).plan
    third = session.update(replace(track, t_state=10.2), 10.2).plan
    assert [first.revision, second.revision, third.revision] == [1, 2, 3]
    assert first.plan_id == second.plan_id == third.plan_id
    assert first.release_time == second.release_time == third.release_time
    assert third.target_track_id == first.target_track_id
    assert third.target_epoch == first.target_epoch
    assert second.target_tcp_pose.position != pytest.approx(first.target_tcp_pose.position)
    assert session.compute(track, 10.3, plan_request).plan.plan_id > first.plan_id


@pytest.mark.req('REQ-PLN-006')
@pytest.mark.level('U')
@pytest.mark.parametrize('cause', ['expiry', 'epoch', 'lost'])
def test_session_closes(track, plan_request, params, cause):
    session = PlanSession(params)
    first = session.compute(track, 10., plan_request).plan
    now = first.valid_until + 1e-6 if cause == 'expiry' else 10.1
    trk = replace(track, epoch=1 if cause == 'epoch' else 0, status=2 if cause == 'lost' else 0)
    expired = session.update(trk, now).plan
    assert expired.status == 4 and expired.revision == 2
    assert expired.plan_id == first.plan_id
    assert expired.release_time == first.release_time
    assert session.update(track, now + .1) is None


@pytest.mark.req('REQ-PLN-009')
@pytest.mark.level('U')
def test_reach_entry_delays_release(track, plan_request, params):
    trk = replace(track, pos=np.array([-.20, 0.]))
    p = replace(params, workspace_min_m=(-.3, -.4, 0.))
    assert compute_plan(trk, 10., plan_request, p).plan.release_time == pytest.approx(17.5)


@pytest.mark.req('REQ-PLN-007')
@pytest.mark.level('U')
@pytest.mark.parametrize('delta,expected', [(-1e-6, 0), (0., 0), (1e-6, 2)])
def test_request_deadline_boundary(track, plan_request, params, delta, expected):
    reply = compute_plan(track, 10. + delta, replace(plan_request, deadline=10.), params)
    assert reply.result == expected


@pytest.mark.req('REQ-PLN-006')
@pytest.mark.level('U')
@pytest.mark.parametrize('delta,status', [(-1e-6, 0), (0., 0), (1e-6, 4)])
def test_track_validity_boundary(track, plan_request, params, delta, status):
    reply = compute_plan(replace(track, valid_until=10.), 10. + delta, plan_request, params)
    assert reply.plan.status == status


@pytest.mark.req('REQ-PLN-006')
@pytest.mark.level('U')
@pytest.mark.parametrize('delta,closed', [(-1e-6, False), (0., False), (1e-6, True)])
def test_session_validity_boundary(track, plan_request, params, delta, closed):
    session = PlanSession(params)
    first = session.compute(track, 10., plan_request).plan
    reply = session.update(None, first.valid_until + delta)
    assert (session.active is None) is closed
    assert (reply is not None) is closed
    if closed:
        assert reply.plan.status == 4


@pytest.mark.req('REQ-PLN-006')
@pytest.mark.level('U')
def test_session_failed_update_and_service_deadline(track, plan_request, params):
    session = PlanSession(params)
    first = session.compute(track, 10., replace(plan_request, deadline=10.)).plan
    uncertain = replace(track, cov=np.eye(4))
    second = session.update(uncertain, 10.1).plan
    assert second.status == 3 and second.revision == 2
    assert second.release_time == first.release_time
    third = session.update(track, 10.2).plan
    assert third.status == 0 and third.revision == 3
    stopped = session.update(replace(track, vel=np.zeros(2)), 10.3).plan
    assert stopped.status == 1 and stopped.reason == 'BELT_STOPPED'
    assert stopped.release_time == first.release_time
    assert stopped.valid_until == first.valid_until


@pytest.mark.req('REQ-PLN-007')
@pytest.mark.level('U')
def test_track_identity_and_current_epoch(track, plan_request, params):
    assert compute_plan(replace(track, track_id=99), 10., plan_request, params).reason_code == 'TRACK_NOT_FOUND'
    assert compute_plan(None, 10., plan_request, replace(params, current_epoch=1)).reason_code == 'EPOCH_MISMATCH'
    assert compute_plan(track, 10., replace(plan_request, mode=99), params).reason_code == 'MODE_UNSUPPORTED'
    reply = compute_plan(replace(track, epoch=2), 10., replace(plan_request, target_epoch=2),
                         replace(params, current_epoch=2))
    assert reply.result == 0


@pytest.mark.req('REQ-PLN-006')
@pytest.mark.level('U')
def test_session_replacement_and_unrelated_updates(track, plan_request, params):
    session = PlanSession(params)
    first = session.compute(track, 10., plan_request).plan
    assert session.update(replace(track, track_id=99), 10.1) is None
    assert session.active.plan_id == first.plan_id
    rejected = session.compute(None, 10.2, plan_request)
    assert rejected.plan is None and session.active is None
    assert session.update(track, 10.3) is None
    assert session.compute(track, 10.4, plan_request).plan.plan_id == first.plan_id + 2


@pytest.mark.req('REQ-PLN-005')
@pytest.mark.level('U')
@pytest.mark.parametrize('delta,status', [(-1e-6, 0), (0., 0), (1e-6, 2)])
def test_downstream_exit_boundary(track, plan_request, params, delta, status):
    # A negative synthetic margin isolates reachability from budget priority.
    p = replace(params, margin_s=-10., workspace_min_m=(-1., -1., -1.),
                workspace_max_m=(1., 1., 1.))
    req = replace(plan_request, earliest_release=16. + delta)
    reply = compute_plan(track, 10., req, p)
    assert reply.plan.status == status


@pytest.mark.req('REQ-PLN-001')
@pytest.mark.level('U')
def test_zero_default_threshold():
    assert evaluate_budget(0., 0., .3, {}, 0.).reason == 'BELT_STOPPED'
    assert evaluate_budget(0., -.001, .3, {}, 0.).reason == 'BELT_REVERSED'


@pytest.mark.req('REQ-PLN-006')
@pytest.mark.level('U')
def test_session_closes_when_track_epoch_differs_from_requested_epoch(track, plan_request, params):
    session = PlanSession(params)
    first = session.compute(track, 10., plan_request).plan
    reply = session.update(replace(track, epoch=first.target_epoch + 1), 10.1,
                           current_epoch=first.target_epoch)
    assert reply.result == 1 and reply.plan.status == 4
    assert reply.plan.plan_id == first.plan_id and reply.plan.revision == 2
    assert session.active is None
    assert session.update(track, 10.2) is None
