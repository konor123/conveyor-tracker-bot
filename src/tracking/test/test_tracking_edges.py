"""Boundary checks supplementary to the statistical TC acceptance tests."""
from dataclasses import replace
import inspect
from pathlib import Path
import numpy as np
import pytest
from fixtures import TrajectoryFixture
from tracking.association import Det, associate
from tracking.kalman import KalmanCV, measurement_covariance
from tracking.track_manager import TrackManager, OBSERVED, COASTING, LOST
from tracking.speed_change import SpeedChangeDetector
from tracking.belt_motion import estimate_belt_motion
from tracking.select_target import select_target


@pytest.mark.req('REQ-TRK-001')
@pytest.mark.level('U')
def test_filter_algebra_and_backwards_prediction():
    x = np.array([1., 2., 3., 4.])
    p = np.diag([1., 2., 3., 4.])
    k = KalmanCV(.02, x, p)
    k.predict(5.)
    k.predict(5.5)
    f = np.eye(4)
    f[:2, 2:] = .5*np.eye(2)
    g = np.vstack((.125*np.eye(2), .5*np.eye(2)))
    assert k.x == pytest.approx(f @ x)
    expected = f @ p @ f.T + .02**2*g @ g.T
    assert k.P == pytest.approx(expected)
    k.predict(4.)
    assert k.P == pytest.approx(expected)
    z = k.x[:2]+[.001, -.002]
    r = np.eye(2)*.0003**2
    innovation = z-k.x[:2]
    nis = innovation @ np.linalg.solve(expected[:2, :2]+r, innovation)
    assert k.update(5.5, z, r) == pytest.approx(nis)
    assert k.P == pytest.approx(k.P.T)
    assert np.all(np.linalg.eigvalsh(k.P) > 0)


@pytest.mark.req('REQ-TRK-001')
@pytest.mark.level('U')
@pytest.mark.parametrize('cov', [None, [], [[1., 2.], [2., 1.]],
                                      [[1., 0.], [0., 0.]], [[np.nan, 0.], [0., 1.]]])
def test_bad_covariance_falls_back(cov):
    assert measurement_covariance(cov) == pytest.approx(np.eye(2)*.0003**2)


@pytest.mark.req('REQ-TRK-003')
@pytest.mark.level('U')
def test_association_greedy_one_to_one_and_gate_boundary():
    m = TrackManager()
    track = m.process_frame(0., [Det([0., 0.])])[0]
    track = replace(track, state_cov=np.eye(4))
    # S = 2 I and dx = 2 -> d²=2 exactly (inclusive boundary).
    result = associate([track], [Det([2., 0.], np.eye(2))], 2.)
    assert result.matches == [(0, 0)]
    assert associate([track], [Det([2.001, 0.], np.eye(2))], 2.).matches == []
    other = replace(track, position=np.array([.1, 0., 0.]))
    result = associate([track, other], [Det([.09, 0.]), Det([0., 0.])], 2.)
    assert set(result.matches) == {(0, 1), (1, 0)}
    assert result.unmatched_tracks == result.unmatched_dets == []


@pytest.mark.req('REQ-TRK-003')
@pytest.mark.level('U')
def test_ignore_invalid_and_average_height():
    m = TrackManager()
    assert m.process_frame(0., [Det([0., 0.], object_class=1),
                                Det([0., 0.], position_valid=False)]) == []
    a = m.process_frame(.1, [Det([0., 0., .01])])[0]
    b = m.process_frame(.2, [Det([0., 0., .03])])[0]
    assert a.position[2] == pytest.approx(.01)
    assert b.position[2] == pytest.approx(.02)
    assert b.velocity[2] == pytest.approx(0.)


@pytest.mark.req('REQ-TRK-006')
@pytest.mark.level('U')
def test_cusum_two_sided_threshold_and_recovery():
    for sign in (-1, 1):
        d = SpeedChangeDetector(cusum_k=.5, cusum_h=8., recover_s=2.)
        assert not d.update(0., sign*8.5)  # equals h, no alarm
        assert d.update(.1, sign*1.)
        assert d.g_plus == d.g_minus == 0 and not d.steady
        assert d.last_change_stamp == pytest.approx(.1)
        d.advance(2.099)
        assert not d.steady
        d.advance(2.1)
        assert d.steady


@pytest.mark.req('REQ-TRK-008')
@pytest.mark.level('U')
def test_clock_tolerance_and_reset_detector():
    m = TrackManager(backstep_tol_s=.5)
    m.on_clock(5.)
    assert not m.on_clock(4.5)
    m.detector.update(4.5, 100.)
    assert m.on_clock(3.9)
    assert m.detector.steady and m.detector.last_change_stamp is None


@pytest.mark.req('REQ-TRK-011')
@pytest.mark.level('U')
def test_belt_covariance_and_stale_estimate():
    m = TrackManager()
    t = m.process_frame(0., [Det([0., 0.])])[0]
    t = replace(t, velocity=np.array([.03, .04, 0.]), state_cov=np.diag([1., 1., .01, .04]))
    b = estimate_belt_motion([t])
    assert b.direction == pytest.approx([.6, .8])
    assert b.speed == pytest.approx(.05)
    assert b.speed_std == pytest.approx(np.sqrt(.6**2*.01+.8**2*.04))
    assert estimate_belt_motion([replace(t, status=LOST)], b) is b
    stopped = estimate_belt_motion([replace(t, velocity=np.array([0., -.002, 0.]))])
    assert stopped.direction == pytest.approx([1., 0.])


@pytest.mark.req('REQ-TRK-002')
@pytest.mark.level('U')
def test_fixture_reverse_and_occlusion():
    m = TrackManager()
    for f in TrajectoryFixture(duration=1., reverse=True, occlusion=(.5, .7)):
        m.process_frame(f.t, f.dets)
    assert m.dropped_frames == 1


@pytest.mark.req('REQ-TRK-001')
@pytest.mark.level('U')
def test_config_matches_runtime_defaults():
    # Minimal scalar reader keeps this pure U suite independent of PyYAML.
    values = {}
    section = None
    for line in (Path(__file__).parents[1]/'config/tracking.yaml').read_text().splitlines():
        if not line.strip() or line.startswith(('#', 'unitless:')):
            continue
        key, value = line.strip().split(':', 1)
        if not value.strip():
            section = key
        else:
            values[f'{section}.{key}'] = float(value)
    mapped = {'coast.max_s': 'coast_max_s'}
    signature = inspect.signature(TrackManager)
    for key, value in values.items():
        name = mapped.get(key, key.split('.')[-1])
        if name in signature.parameters:
            assert signature.parameters[name].default == pytest.approx(value)


@pytest.mark.req('REQ-TRK-005')
@pytest.mark.level('U')
def test_predicted_exit_cannot_be_reacquired_in_same_frame():
    m = TrackManager(x_obs_max_m=.01, exit_margin_m=0.)
    first = m.process_frame(0., [Det([0., 0.])])[0]
    m.process_frame(1., [Det([.005, 0.])])
    tracks = m.process_frame(3., [Det([.009, 0.])])
    assert next(t for t in tracks if t.track_id == first.track_id).status == LOST
    assert next(t for t in tracks if t.status == OBSERVED).track_id > first.track_id
