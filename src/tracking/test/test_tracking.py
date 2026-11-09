"""SPEC-300 unit acceptance tests; all quantities are SI."""
from dataclasses import replace
import numpy as np
import pytest
from fixtures import TrajectoryFixture
from tracking.kalman import KalmanCV
from tracking.association import Det, associate
from tracking.speed_change import SpeedChangeDetector
from tracking.track_manager import TrackManager, OBSERVED, COASTING, LOST
from tracking.belt_motion import estimate_belt_motion
from tracking.select_target import select_target


def manager():
    # A 60 s accuracy run travels 1.2 m: bypass only the physical FOV exit.
    return TrackManager(x_obs_max_m=10.)


def run_trajectory(seed, **kwargs):
    m = manager()
    errors, valid, alarms = [], [], []
    for f in TrajectoryFixture(seed=seed, **kwargs):
        if f.dropped:
            valid.append(any(t.status != LOST and f.t <= t.valid_until for t in m.tracks))
            continue  # transport loss, not an empty detection frame
        tracks = m.process_frame(f.t, f.dets)
        valid.append(any(t.status != LOST for t in tracks))
        alarms.append(m.belt_motion.last_change_stamp if m.belt_motion else None)
        if tracks and tracks[0].observation_count >= 30:
            errors.append(np.r_[tracks[0].position[:2], tracks[0].velocity[:2]] - f.truth)
    return m, np.asarray(errors), valid, alarms


@pytest.mark.req('REQ-TRK-001')
@pytest.mark.tc('TC-TRK-001-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('seed', range(20))
def test_constant_accuracy(seed):
    _, e, _, _ = run_trajectory(seed)
    assert np.percentile(np.linalg.norm(e[:, :2], axis=1), 95) <= .0003
    assert np.percentile(np.linalg.norm(e[:, 2:], axis=1), 95) <= .0005


@pytest.fixture(scope='module')
def nees_samples():
    # One terminal error per independent 60 s run avoids temporal correlation.
    values = []
    for seed in range(20):
        frames = TrajectoryFixture(seed=seed, duration=60., sigma_accel=manager().sigma_accel_mps2)
        first = frames.frames[0]
        kf = KalmanCV(manager().sigma_accel_mps2, np.r_[first.dets[0].position, .02, 0.],
                      np.diag([.0003**2]*2 + [0., 0.]))
        kf.predict(0.)
        for f in frames.frames[1:]:
            kf.update(f.t, f.dets[0].position, np.eye(2)*.0003**2)
        e = kf.x - f.truth
        values.append(e @ np.linalg.solve(kf.P, e))
    return values


@pytest.mark.req('REQ-TRK-001')
@pytest.mark.tc('TC-TRK-001-02')
@pytest.mark.level('U')
def test_nees(nees_samples):
    # Independent terminal errors: sum ~ chi2(4N), N=20, dof=80.
    # Wilson-Hilferty approximation: chi2 quantiles ~ nu*(1-2/9nu+z*sqrt(2/9nu))³.
    n = len(nees_samples)
    nu = 4*n
    lo, hi = [nu*(1-2/(9*nu)+z*np.sqrt(2/(9*nu)))**3/n for z in (-1.96, 1.96)]
    assert lo <= np.mean(nees_samples) <= hi


@pytest.mark.req('REQ-TRK-002')
@pytest.mark.tc('TC-TRK-002-01')
@pytest.mark.level('U')
def test_reverse_stamp():
    m = manager()
    m.process_frame(2., [Det([0., 0.])])
    before = m.tracks[0]
    assert m.process_frame(1., [Det([1., 1.])]) == []
    assert m.dropped_frames == 1
    assert m.tracks[0].position == pytest.approx(before.position)
    assert m.tracks[0].state_cov == pytest.approx(before.state_cov)
    assert m.tracks[0].t_state == pytest.approx(2.)


@pytest.mark.req('REQ-TRK-003')
@pytest.mark.tc('TC-TRK-003-01')
@pytest.mark.level('U')
def test_coasting():
    m, _, _, _ = run_trajectory(0, duration=2.)
    initial = m.tracks[0]
    traces = [np.trace(initial.state_cov)]
    for t in np.linspace(2.01, 2.5, 20):
        track, = m.process_frame(t, [])
        assert track.status == COASTING
        assert track.last_observed == pytest.approx(2.)
        assert track.valid_until == pytest.approx(3.)
        traces.append(np.trace(track.state_cov))
    assert np.all(np.diff(traces) > 0)


@pytest.mark.req('REQ-TRK-003')
@pytest.mark.tc('TC-TRK-003-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('offset, same', [(0., True), (.5, False)])
def test_reacquire(offset, same):
    m, _, _, _ = run_trajectory(0, duration=2.)
    old = m.tracks[0].track_id
    m.process_frame(2.5, [])
    tracks = m.process_frame(2.6, [Det([-.2+.02*2.6+offset, 0.])])
    observed = [t for t in tracks if t.status == OBSERVED]
    assert len(observed) == 1
    assert (observed[0].track_id == old) == same


@pytest.mark.req('REQ-TRK-004')
@pytest.mark.tc('TC-TRK-004-01')
@pytest.mark.level('U')
def test_lost_once():
    m = manager()
    m.process_frame(0., [Det([0., 0.])])
    assert m.process_frame(1., [])[0].status == COASTING
    assert m.process_frame(1.001, [])[0].status == LOST
    assert m.process_frame(1.1, []) == []


@pytest.mark.req('REQ-TRK-005')
@pytest.mark.tc('TC-TRK-005-01')
@pytest.mark.level('U')
def test_pass_new_id():
    m = TrackManager(x_obs_max_m=-.17, exit_margin_m=0.)
    old = m.process_frame(0., [Det([-.2, 0.])])[0]
    statuses = [m.process_frame(f.t, f.dets) for f in TrajectoryFixture(duration=2.)]
    assert any(t.status == LOST for ts in statuses for t in ts)
    new = m.process_frame(20., [Det([-.2, 0.])])[-1]
    assert new.track_id > old.track_id
    assert new.epoch == old.epoch


@pytest.mark.req('REQ-TRK-006')
@pytest.mark.tc('TC-TRK-006-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('seed', range(20))
def test_speed_step(seed):
    m = manager()
    detected = []
    for f in TrajectoryFixture(seed, duration=13., step=(10., .03)):
        m.process_frame(f.t, f.dets)
        if not m.belt_motion.steady:
            detected.append(f.t)
    assert detected and 10. <= min(detected) <= 11.


@pytest.mark.req('REQ-TRK-006')
@pytest.mark.tc('TC-TRK-006-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('seed', range(20))
def test_no_false_alarm(seed):
    _, _, _, alarms = run_trajectory(seed)
    assert all(a is None for a in alarms)


@pytest.mark.req('REQ-TRK-007')
@pytest.mark.tc('TC-TRK-007-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('seed', range(20))
def test_dropped_frames(seed):
    _, e, valid, _ = run_trajectory(seed, drop_fraction=.1)
    assert np.mean(valid) >= .99
    assert np.percentile(np.linalg.norm(e[:, :2], axis=1), 95) <= .00033
    assert np.percentile(np.linalg.norm(e[:, 2:], axis=1), 95) <= .00055


@pytest.mark.req('REQ-TRK-008')
@pytest.mark.tc('TC-TRK-008-01')
@pytest.mark.level('U')
def test_clock_reset():
    m = manager()
    m.on_clock(10.)
    old = m.process_frame(10., [Det([0., 0.])])[0]
    assert m.on_clock(9.)
    assert m.tracks == [] and m.epoch == 1 and m.belt_motion is None
    new = m.process_frame(9., [Det([0., 0.])])[0]
    assert new.track_id > old.track_id and new.epoch == 1


@pytest.mark.req('REQ-TRK-009')
@pytest.mark.level('U')
@pytest.mark.parametrize('region', [0, 1, 2, 99])
def test_reset(region):
    m = manager()
    m.process_frame(0., [Det([0., 0.])])
    reply = m.reset(region)
    assert reply.result == 0 and reply.new_epoch == 1
    assert m.tracks == []


@pytest.mark.req('REQ-TRK-010')
@pytest.mark.level('U')
def test_selection():
    m, _, _, _ = run_trajectory(0, duration=2.)
    a = replace(m.tracks[0], position=np.array([-.1, 0.]), velocity=np.array([.02, 0.]))
    b = replace(a, track_id=100, position=np.array([-.08, 0.]))
    result = select_target([a, b], m.belt_motion, epoch=7, min_observation_count=30)
    assert result.result == 0 and result.track_id == 100 and result.epoch == 7
    assert result.t_enter_window == pytest.approx(b.t_state+1.5, abs=.05)
    for tracks in ([], [replace(a, position=np.array([0., 0.]))],
                   [replace(a, status=COASTING)], [replace(a, observation_count=29)]):
        r = select_target(tracks, m.belt_motion, epoch=7, min_observation_count=30)
        assert r.result == 3 and r.reason == 'NO_TARGET'
    r = select_target([a], replace(m.belt_motion, speed=.002), epoch=7)
    assert r.result == 3 and r.reason == 'BELT_STOPPED'


@pytest.mark.req('REQ-TRK-011')
@pytest.mark.level('U')
def test_belt_estimate():
    m, _, _, _ = run_trajectory(0, duration=2.)
    b = m.belt_motion
    assert b.source == 0 and np.linalg.norm(b.direction) == pytest.approx(1., abs=1e-9)
    assert b.speed == pytest.approx(.02, abs=.0005)
    assert b.window_sec == pytest.approx(2.)
    assert estimate_belt_motion([], previous=b) is b
    track = m.tracks[0]
    newer = replace(track, observation_count=1, velocity=np.array([.03, 0.]))
    assert estimate_belt_motion([track, newer]).speed == pytest.approx(b.speed)
