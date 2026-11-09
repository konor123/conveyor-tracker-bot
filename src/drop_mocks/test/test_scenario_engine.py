"""ScenarioEngine 단위 시험(SPEC-700 4절, TEST_SPEC.md 3.11절)."""
import copy
import glob
import hashlib
import math
import os
import re

import pytest

from conftest import SCENARIOS
from drop_mocks.scenario_engine import (
    CLASS_CAP, CLASS_STONE, COASTING, LOST, OBSERVED, REGION_BELT, REGION_MAT, RESULT, VERDICT,
    ScenarioEngine, classify_params)

DT = 1.0 / 30.0
UNIT_SUFFIX = re.compile(r'_(m|s|mps|mps2|rad|hz|ma|pulse|count|ratio)$')


def _digest(engine, n=600):
    h = hashlib.sha256()
    for k in range(n):
        h.update(repr(engine.frame(k)).encode())
    return h.hexdigest()


@pytest.mark.req('REQ-MCK-001')
@pytest.mark.tc('TC-MCK-001-01')
@pytest.mark.level('U')
def test_same_scenario_and_seed_give_same_output(load_scenario):
    s = load_scenario('nominal_20mms')
    s['world']['noise']['position_std_m'] = 0.0003
    s['world']['camera']['pixel_std_ratio'] = 0.2
    a = _digest(ScenarioEngine(copy.deepcopy(s), seed=3))
    b = _digest(ScenarioEngine(copy.deepcopy(s), seed=3))
    c = _digest(ScenarioEngine(copy.deepcopy(s), seed=4))
    assert a == b
    assert a != c                  # seed가 실제로 출력에 들어간다


def _cap_track(fr):
    return next(tr for tr in fr.tracks if tr.track_id % 100 == 1)


def _effects(eng, k):
    """이벤트 종류별로 프레임 k에서 효과가 나타났는지."""
    fr = eng.frame(k)
    t = eng.frame_time(k)
    caps = [d for d in fr.detections if d.object_class == CLASS_CAP]
    return {
        'occlusion': not caps and _cap_track(fr).status == COASTING,
        'track_lost': _cap_track(fr).status == LOST,
        'epoch_increment': _cap_track(fr).epoch >= 1,
        'speed_change': fr.belt.speed == pytest.approx(0.030) and not fr.belt.steady,
        'frame_drop': fr.dropped,
        'heartbeat_stop': not eng.heartbeat_alive('vision', t),
        'stone_in_cap': bool(caps) and caps[0].stone_inside,
        'gripper_delay': eng.gripper_delay(t) == pytest.approx(0.6),
        'protective_stop': eng.protective_stop(t),
        'clock_jump': fr.stamp_offset == pytest.approx(-5.0),
    }


@pytest.mark.req('REQ-MCK-005')
@pytest.mark.tc('TC-MCK-005-02')
@pytest.mark.level('U')
def test_events_take_effect_at_scheduled_frame(load_scenario):
    s = load_scenario('all_events')
    eng = ScenarioEngine(s)
    assert sorted(e['type'] for e in s['events']) == sorted(_effects(eng, 75))  # 10종 모두
    for e in s['events']:
        k = math.ceil(e['t_s'] * eng.rate)
        before = _effects(eng, k - 1)[e['type']]
        after = _effects(eng, k)[e['type']]
        assert not before, e['type']
        assert after, e['type']
        assert 0 <= eng.frame_time(k) - e['t_s'] <= DT + 1e-9


@pytest.mark.req('REQ-MCK-006')
@pytest.mark.tc('TC-MCK-006-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('path', sorted(glob.glob(os.path.join(SCENARIOS, '*.yaml'))),
                         ids=os.path.basename)
def test_every_value_is_assumed_or_measured(path):
    eng = ScenarioEngine.from_file(path)
    cls, errors = classify_params(eng.scenario)
    assert errors == []
    assert cls                                           # 빈 분류가 아님
    assert eng.assumed_params() == sorted(p for p, kind in cls.items() if kind == 'assumed')
    assert 'world.objects.cap_outer_diameter_m' not in eng.assumed_params()


@pytest.mark.req('REQ-MCK-006')
@pytest.mark.tc('TC-MCK-006-01')
@pytest.mark.level('U')
def test_unclassified_or_double_classified_value_is_rejected(load_scenario):
    s = load_scenario('nominal_20mms')
    s['world']['belt']['extra_s'] = 1.0
    s['assumed'] = [p for p in s['assumed'] if p != 'world.belt'] + ['world.belt.speed_mps']
    s['measured']['world.belt.speed_mps'] = 'test'
    _, errors = classify_params(s)
    assert any('world.belt.extra_s' in e for e in errors)
    assert any('world.belt.speed_mps' in e and 'measured' in e for e in errors)
    with pytest.raises(ValueError):
        ScenarioEngine(s)


@pytest.mark.level('U')
@pytest.mark.parametrize('path', sorted(glob.glob(os.path.join(SCENARIOS, '*.yaml'))),
                         ids=os.path.basename)
def test_scenario_numeric_keys_have_unit_suffix(path):
    """TC-SYS-002-01 규칙을 시나리오 파일에 미리 적용한다."""
    import yaml
    s = yaml.safe_load(open(path, encoding='utf-8'))
    unitless = set(s.get('unitless') or [])

    def walk(node, key=None):
        if isinstance(node, dict):
            for k, v in node.items():
                if k not in ('measured',):
                    yield from walk(v, k)
        elif isinstance(node, list):
            for x in node:
                yield from walk(x, key)
        elif isinstance(node, (int, float)) and not isinstance(node, bool):
            yield key
    bad = {k for k in walk(s) if not UNIT_SUFFIX.search(k) and k not in unitless}
    assert bad == set()


# ------------------------------------------------------------------ 세계 모델(SPEC-700 4.2절)
@pytest.mark.level('U')
def test_belt_travel_integrates_speed_changes(load_scenario):
    s = load_scenario('nominal_20mms')
    s['events'] = [{'t_s': 10.0, 'type': 'speed_change', 'speed_mps': 0.040}]
    eng = ScenarioEngine(s)
    assert eng.belt_s(10.0) == pytest.approx(0.20)
    assert eng.belt_s(15.0) == pytest.approx(0.40)
    assert eng.time_at_s(0.40) == pytest.approx(15.0)
    assert eng.time_at_s(0.10) == pytest.approx(5.0)


@pytest.mark.level('U')
def test_caps_wrap_and_get_new_track_each_pass(load_scenario):
    eng = ScenarioEngine(load_scenario('nominal_20mms'))
    first = {tr.track_id for tr in eng.frame(eng.frame_index(10.0)).tracks}
    second = {tr.track_id for tr in eng.frame(eng.frame_index(70.0)).tracks}   # 한 바퀴 60 s 뒤
    assert len(first) == 1 and len(second) == 1 and first != second
    assert eng.frame(eng.frame_index(1.0)).tracks == ()                         # 아직 시야 밖


@pytest.mark.level('U')
def test_detection_and_track_describe_same_cap(load_scenario):
    s = load_scenario('nominal_20mms')
    s['world']['noise']['position_std_m'] = 0.0005
    eng = ScenarioEngine(s)
    fr = eng.frame(300)
    cap = next(d for d in fr.detections if d.object_class == CLASS_CAP)
    tr = fr.tracks[0]
    assert eng.base_to_conveyor(cap.position) == pytest.approx(tr.position, abs=1e-12)
    assert tr.status == OBSERVED and tr.velocity[0] == pytest.approx(0.020)
    stone = next(d for d in fr.detections if d.object_class == CLASS_STONE)
    assert stone.region == REGION_MAT and stone.position[2] == pytest.approx(0.008)
    assert cap.diameter_px == pytest.approx(640 * 0.032 / 0.34, rel=1e-3)      # f = 640 px


@pytest.mark.level('U')
def test_select_target_picks_cap_before_reach_window(load_scenario):
    eng = ScenarioEngine(load_scenario('nominal_20mms'))
    r = eng.select_target(5.0, min_observation_count=30)
    assert r.result == RESULT['OK']
    assert r.t_enter_window == pytest.approx(12.5, abs=1e-6)    # x0 −0.05 → 0.20 at 20 mm/s
    assert eng.select_target(2.6, 30).reason_code == 'NO_TARGET'   # 관측 수 부족
    assert eng.select_target(13.0, 1).reason_code == 'NO_TARGET'   # 이미 도달 구간 안
    assert eng.select_target(5.0, 30, resets=2).epoch == 2


@pytest.mark.level('U')
def test_detect_and_verify_replies(load_scenario):
    eng = ScenarioEngine(load_scenario('nominal_20mms'))
    assert eng.detect(5.0, REGION_MAT, CLASS_STONE, 1).result == RESULT['OK']
    assert eng.detect(1.0, REGION_BELT, CLASS_CAP, 1).reason_code == 'NOT_ENOUGH_OBJECTS'
    v = eng.verify(5)
    assert v.verdict == VERDICT['SUCCESS'] and v.stone_offset_valid


@pytest.mark.level('U')
def test_skill_script_repeats_last_entry(load_scenario):
    s = load_scenario('robot_hang')
    s['robot']['duration_scale_ratio'] = 2.0
    eng = ScenarioEngine(s)
    first = eng.skill('PickStone', 0)
    assert first.hang and not first.cancel_ack and first.duration_s == pytest.approx(8.0)
    assert eng.skill('PickStone', 7).hang
    mv = eng.skill('MoveToPose', 0)
    assert mv.result == RESULT['OK'] and mv.values['pose_error_m'] == pytest.approx(0.0005)
    with pytest.raises(ValueError):
        eng.skill('Teleport', 0)


@pytest.mark.level('U')
def test_speed_zero_never_reaches(load_scenario):
    s = load_scenario('nominal_20mms')
    s['events'] = [{'t_s': 1.0, 'type': 'speed_change', 'speed_mps': 0.0}]
    eng = ScenarioEngine(s)
    assert math.isinf(eng.time_at_s(0.5))
    assert eng.time_at_s(0.01) == pytest.approx(0.5)


@pytest.mark.level('U')
def test_phase_metrics_defaults_overrides_and_assumed(load_scenario):
    s = load_scenario('nominal_20mms')
    s['robot']['skills']['DynamicPlace'] = [{
        'duration_s': 0.5,
        'phase_metrics': {'DESCEND': {'rim_clearance_m': 0.001, 'sync_ok': 0.0}},
    }]
    eng = ScenarioEngine(s)
    rep = eng.skill('DynamicPlace', 0)
    k_sync, v_sync = rep.metrics_for('SYNCHRONIZE', predicted=False)
    assert k_sync == ['e_xy_m', 'v_rel_mps'] and v_sync == pytest.approx([0.001, 0.002])
    k_pred, v_pred = rep.metrics_for('SYNCHRONIZE', predicted=True)
    assert k_pred == ['entry_time_error_s', 'v_rel_mps'] and v_pred == pytest.approx([0.01, 0.002])
    k_desc, v_desc = rep.metrics_for('DESCEND', predicted=False)
    assert dict(zip(k_desc, v_desc)) == pytest.approx(
        {'sync_ok': 0.0, 'rim_clearance_m': 0.001, 'height_error_m': 0.0002})
    assert 'robot.skills.DynamicPlace.phase_metrics.DESCEND.rim_clearance_m' in eng.assumed_params()
    assert eng.robot_status_pose() == pytest.approx((0.40, 0.0, 0.20, 0.0, 0.0, 0.0, 1.0))


@pytest.mark.level('U')
def test_compute_intercept_and_time_insufficient(load_scenario):
    eng = ScenarioEngine(load_scenario('nominal_20mms'))
    sel = eng.select_target(5.0, min_observation_count=30)
    assert sel.result == RESULT['OK']
    plan = eng.compute_intercept(5.0, sel.track_id, sel.epoch)
    assert plan.result == RESULT['OK'] and plan.status == 0
    assert plan.time_margin_s > 1.0
    assert eng.compute_intercept(5.0, sel.track_id, sel.epoch + 1).reason_code == 'EPOCH_MISMATCH'
    assert eng.compute_intercept(5.0, 999, sel.epoch).reason_code == 'NO_TRACK'

    eng_ti = ScenarioEngine(load_scenario('time_insufficient'))
    s1 = eng_ti.select_target(6.0, min_observation_count=30)
    p1 = eng_ti.compute_intercept(6.0, s1.track_id, s1.epoch)
    assert p1.time_margin_s < 1.0
    s4 = eng_ti.select_target(10.5, min_observation_count=30)
    p4 = eng_ti.compute_intercept(10.5, s4.track_id, s4.epoch)
    assert p4.result == RESULT['OK'] and p4.time_margin_s > 5.0
