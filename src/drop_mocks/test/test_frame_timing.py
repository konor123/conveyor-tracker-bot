"""Deterministic schedule/observation regression tests; no ROS or wall sleeps."""
import math

import pytest

from drop_mocks.scenario_engine import COASTING, ScenarioEngine
from frame_timing import assert_event_phase


@pytest.mark.parametrize('event,wake', [('occlusion', 3.1), ('occlusion', 3.2),
                                       ('stone_in_cap', 188 / 30)])
def test_latest_frame_after_delayed_wake_preserves_event_phase(load_scenario, event, wake):
    engine = ScenarioEngine(load_scenario('all_events'))
    ev = next(e for e in engine.events if e['type'] == event)
    start, end = ev['t_s'], ev['t_s'] + ev['duration_s']
    # Fake ROS clock: the callback next wakes at the same frame as the CI failure.
    clock = iter([start - 1 / 30, wake, end + 2 / 30])
    frames = [engine.frame(engine.frame_index(next(clock))) for _ in range(3)]
    def effect(fr):
        if event == 'occlusion':
            return any(tr.status == COASTING for tr in fr.tracks)
        return any(d.stone_inside for d in fr.detections)
    assert frames[1].t - start > 1 / 30 + 1e-6  # old first-received oracle fails
    assert_event_phase([(fr.t, effect(fr)) for fr in frames], start, end)


@pytest.mark.parametrize('samples', [[(2.9, True), (3.1, True)],
                                    [(2.9, False), (3.1, False)],
                                    [(2.9, False), (3.1, False), (3.2, True)],
                                    [(3.1, True), (3.5, True)]])
def test_phase_oracle_rejects_early_late_and_stuck_effects(samples):
    with pytest.raises(AssertionError):
        assert_event_phase(samples, 3.0, 3.4)


@pytest.mark.parametrize('start', [3.0, 3.007, 3.033, 3.034, 6.2])
def test_first_scheduled_frame_straddles_event_exactly(load_scenario, start):
    scenario = load_scenario('all_events')
    scenario['events'] = [{'type': 'occlusion', 't_s': start, 'duration_s': 0.4}]
    engine = ScenarioEngine(scenario)
    k = math.ceil(start * engine.rate)
    fake_clock = iter([(k - 1) / engine.rate, k / engine.rate])
    before, after = [engine.frame(engine.frame_index(next(fake_clock))) for _ in range(2)]
    assert before.t < start <= after.t
    assert 0 <= after.t - start <= 1 / engine.rate + 1e-6
    assert all(tr.status != COASTING for tr in before.tracks)
    assert any(tr.status == COASTING for tr in after.tracks)
