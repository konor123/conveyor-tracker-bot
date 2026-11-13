"""DynamicPlace 끝 진단 한 줄(SPEC-500 4.7절 REQ-ROB-018, TEST_SPEC.md TC-ROB-018-01).

단계 진입 시각, commit 시각, 해제 게이트가 닫혀 기다린 이유를 남긴다. 시계는 FakeClock, transport는
`SimulatedPlaceTransport`(commit → 개방 0.1 s, 개방 확인 유지 0.3 s)다.
"""
import pytest

from robot_control.skill_logic import Result

from fakes.place_world import Rig


def _hold_track_stale(hold_s, state):
    """RELEASE에 들어간 뒤 `hold_s` 동안 track을 오래된 것으로 만든다(게이트 GATE_TRACK_STALE, 기다리는 이유)."""
    def hook(r):
        if r.seq.phase == 'RELEASE' and 't' not in state:
            state['t'] = r.clock.ros_now()
        if 't' in state:
            r.world.track_age_s = .5 if r.clock.ros_now() - state['t'] < hold_s else .03
        return None
    return hook


def _entered(seq, phase):
    return next(t for p, t in seq.phase_log if p == phase)


@pytest.mark.req('REQ-ROB-018')
@pytest.mark.tc('TC-ROB-018-01')
@pytest.mark.level('U')
def test_summary_reports_release_wait_and_gate_hold_reason():
    r = Rig(world_kw=dict(plan_every=1))
    state = {}
    res = r.run(_hold_track_stale(.7, state))
    assert (res.result, res.reason_code) == (Result.OK, '')
    seq = r.seq
    assert [p for p, _ in seq.phase_log] == ['APPROACH', 'SYNCHRONIZE', 'DESCEND', 'RELEASE', 'RETREAT']
    release = _entered(seq, 'RELEASE')
    wait = seq.commit_t - release
    assert .7 - 1e-9 <= wait < .7 + .02
    assert set(seq.gate_holds) == {'GATE_TRACK_STALE'} and seq.gate_holds['GATE_TRACK_STALE'] >= 69
    # RELEASE 길이 = 게이트 대기 + commit→개방(0.1 s) + 개방 확인 유지(0.3 s)
    assert _entered(seq, 'RETREAT') - release == pytest.approx(wait + .1 + .3, abs=.02)
    line = seq.summary()
    assert line.startswith('OK -; 단계 APPROACH@0.000 SYNCHRONIZE@')
    assert f'RELEASE 진입→commit {wait:.3f} s' in line
    assert line.endswith(f"게이트 대기 GATE_TRACK_STALE×{seq.gate_holds['GATE_TRACK_STALE']}")


@pytest.mark.req('REQ-ROB-018')
@pytest.mark.tc('TC-ROB-018-01')
@pytest.mark.level('U')
def test_summary_without_gate_wait():
    r = Rig(world_kw=dict(plan_every=1))
    res = r.run()
    assert res.result == Result.OK
    seq = r.seq
    assert seq.gate_holds == {}
    assert seq.commit_t - _entered(seq, 'RELEASE') < .02
    assert seq.summary().endswith('게이트 대기 -')


@pytest.mark.req('REQ-ROB-018')
@pytest.mark.tc('TC-ROB-018-01')
@pytest.mark.level('U')
def test_summary_of_failed_goal_names_reason_and_last_phase():
    r = Rig(world_kw=dict(plan_every=1))
    state = {}
    res = r.run(_hold_track_stale(10., state))       # 게이트가 끝까지 닫혀 대기 창(release_wait_max_s)에서 실패
    assert (res.result, res.reason_code) == (Result.FAILED, 'GATE_TRACK_STALE')
    line = r.seq.summary()
    assert line.startswith('FAILED GATE_TRACK_STALE; 단계 APPROACH@0.000')
    assert 'RELEASE@' in line and 'RETREAT@' not in line and 'RELEASE 진입→commit -;' in line
