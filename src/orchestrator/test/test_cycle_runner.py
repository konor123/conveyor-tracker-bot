"""사이클 실행기(SPEC-100 4.1·4.2·4.5·4.6절)."""
from dataclasses import replace

import pytest
from spec_metrics import EXPECTED_METRIC_KEYS

from fakes import measures as M
from fakes.skills import FakeSkillPort, Reply
from orchestrator import step_judges as sj
from orchestrator.abort_policy import Event, HARD_FAULTS
from orchestrator.common import AbortLevel, Mode, Result, State
from orchestrator.cycle_runner import (CapObservation, CycleRunner, DetectResult, DropGoal,
                                       OrchestratorCfg)

S = State
SEQ_SYNC = [S.IDLE, S.PRECHECK, S.DETECT_STONE, S.PICK_STONE, S.MOVE_STANDBY, S.WAIT_CAP,
            S.PLAN_INTERCEPT, S.APPROACH, S.SYNCHRONIZE, S.DESCEND, S.RELEASE, S.RETREAT, S.VERIFY,
            S.DONE]


@pytest.fixture
def cfg(orch_cfg):
    return OrchestratorCfg.from_dict(orch_cfg)


def goal(mode=Mode.SYNCHRONIZED, max_passes=2):
    return DropGoal(mode=mode, max_belt_speed_mps=0.06, timeout_s=90.0, max_passes=max_passes)


def run(fake_clock, cfg, script=None, g=None, **kw):
    port = FakeSkillPort(fake_clock, script, **kw)
    runner = CycleRunner(port, fake_clock, cfg)
    return runner, port, runner.run(g or goal())


def wait_cap_obs(meas):
    return lambda port: CapObservation(meas, port.clock.ros_now() + 8.0)


BAD_MARGIN = replace(M.OK_WAIT, t_available_s=4.5, t_required_s=4.0)     # margin 0.5 < 1.0


# ---------------------------------------------------------------- 상태 열
@pytest.mark.req("REQ-ORC-001")
@pytest.mark.tc("TC-ORC-001-01")
@pytest.mark.level("U")
def test_synchronized_nominal_sequence(fake_clock, cfg):
    _, port, out = run(fake_clock, cfg)
    assert out.states == SEQ_SYNC
    assert out.result == 'SUCCESS'
    assert [r.step_name for r in out.records] == [s.name for s in SEQ_SYNC[1:-1]]
    assert all(r.result == Result.OK for r in out.records)


@pytest.mark.req("REQ-ORC-001")
@pytest.mark.tc("TC-ORC-001-02")
@pytest.mark.level("U")
def test_static_skips_synchronize_and_detects_cap(fake_clock, cfg):
    _, port, out = run(fake_clock, cfg, g=goal(Mode.STATIC))
    assert out.states == [s for s in SEQ_SYNC if s not in (S.PLAN_INTERCEPT, S.SYNCHRONIZE)]
    names = port.names()
    assert 'DetectObjects.BELT' in names and 'SelectTarget' not in names
    assert ('DetectObjects.BELT', {'region': 'BELT', 'object_class': 'CAP'}) in port.calls
    assert 'PlaceStone' in names and 'DynamicPlace' not in names
    descend = next(r for r in out.records if r.step_name == 'DESCEND')
    assert set(descend.metric_keys) == EXPECTED_METRIC_KEYS['DESCEND_STATIC']


@pytest.mark.req("REQ-ORC-001")
@pytest.mark.tc("TC-ORC-001-03")
@pytest.mark.level("U")
def test_predicted_uses_entry_judgement(fake_clock, cfg):
    _, port, out = run(fake_clock, cfg, g=goal(Mode.PREDICTED), mode_entry=M.OK_ENTRY)
    assert out.states == SEQ_SYNC
    sync = next(r for r in out.records if r.step_name == 'SYNCHRONIZE')
    assert set(sync.metric_keys) == EXPECTED_METRIC_KEYS['SYNCHRONIZE_PREDICTED']


# ---------------------------------------------------------------- 실패 뒤 다음 단계 금지
FAIL_CASES = {
    # 단계: (대본, 다음 단계 skill 키)
    'DETECT_STONE': ({'DetectObjects.MAT': [Reply(lambda p: DetectResult(
        replace(M.OK_DET, stone_count=0), p.clock.ros_now(), None), 0.3)]}, 'PickStone'),
    'PICK_STONE': ({'PickStone': [Reply(replace(M.OK_PICK, grip_width_m=0.0), 4.0)]},
                   'MoveToPose.standby'),
    'MOVE_STANDBY': ({'MoveToPose.standby': [Reply(replace(M.OK_STB, holding=False), 2.0)]},
                     'SelectTarget'),
    'WAIT_CAP': ({'SelectTarget': [Reply(wait_cap_obs(replace(M.OK_WAIT, stone_inside=True)))]},
                 'ComputeIntercept'),
    'PLAN_INTERCEPT': ({'ComputeIntercept': [Reply(replace(M.OK_PLAN, plan_ok=False), 0.05)]},
                       'DynamicPlace'),
    'APPROACH': ({'DynamicPlace': [Reply(replace(M.OK_APP, pose_error_m=0.01), 1.0)]},
                 'feedback.SYNCHRONIZE'),
    'SYNCHRONIZE': ({'feedback.SYNCHRONIZE': [Reply(lambda p: M.sync_samples(
        t0=p.clock.ros_now(), t_end=2.0, bad_at=[i / 100 for i in range(0, 201, 3)]), 2.0)]},
        'feedback.DESCEND'),
    'DESCEND': ({'feedback.DESCEND': [Reply(replace(M.OK_DESC, rim_clearance_m=0.001), 0.8)]},
                'feedback.RELEASE'),
    'RETREAT': ({'result.RETREAT': [Reply(replace(M.OK_RET, tcp_z_m=0.05), 1.0)]}, 'VerifyDrop'),
}


@pytest.mark.req("REQ-ORC-002")
@pytest.mark.tc("TC-ORC-002-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("step", list(FAIL_CASES))
def test_no_next_skill_after_failure(fake_clock, cfg, step):
    script, next_key = FAIL_CASES[step]
    _, port, out = run(fake_clock, cfg, script)
    assert next_key not in port.names()
    assert out.states[-1] is S.WAIT_OPERATOR
    assert any(r.step_name == step and r.result != Result.OK for r in out.records)


@pytest.mark.req("REQ-ORC-002")
@pytest.mark.tc("TC-ORC-002-02")
@pytest.mark.level("U")
def test_release_failure_continues_to_retreat_and_verify(fake_clock, cfg):
    script = {'feedback.RELEASE': [Reply(replace(M.OK_REL, open_width_m=0.010), 0.3)]}
    _, port, out = run(fake_clock, cfg, script)
    assert out.states[-4:] == [S.RELEASE, S.RETREAT, S.VERIFY, S.DONE]
    assert 'VerifyDrop' in port.names()
    assert out.result == 'FAILURE'


@pytest.mark.req("REQ-ORC-002")
@pytest.mark.tc("TC-ORC-002-01")
@pytest.mark.level("U")
def test_retreat_failure_is_protective_stop(fake_clock, cfg):
    _, port, out = run(fake_clock, cfg, FAIL_CASES['RETREAT'][0])
    assert out.abort_level == AbortLevel.L4
    assert out.result == 'PROTECTIVE_STOP'
    assert 'Stop' in port.names()


# ---------------------------------------------------------------- 선행조건
@pytest.mark.req("REQ-ORC-005")
@pytest.mark.tc("TC-ORC-005-01")
@pytest.mark.level("U")
def test_stale_detection_blocks_pick(fake_clock, cfg):
    stale = Reply(lambda p: DetectResult(M.OK_DET, p.clock.ros_now() - 1.01, (0.3, 0.0, 0.0)), 0.3)
    _, port, out = run(fake_clock, cfg, {'DetectObjects.MAT': [stale]})
    assert 'PickStone' not in port.names()
    picks = [r for r in out.records if r.step_name == 'PICK_STONE']
    assert len(picks) == 1 and picks[0].result == Result.PRECONDITION
    assert port.names().count('DetectObjects.MAT') == 1       # 재검출하지 않는다
    assert out.abort_level == AbortLevel.L2


@pytest.mark.req("REQ-ORC-005")
@pytest.mark.tc("TC-ORC-005-01")
@pytest.mark.level("U")
def test_detection_at_age_limit_allows_pick(fake_clock, cfg):
    edge = Reply(lambda p: DetectResult(M.OK_DET, p.clock.ros_now() - 1.0, (0.3, 0.0, 0.0)), 0.3)
    _, port, out = run(fake_clock, cfg, {'DetectObjects.MAT': [edge]})
    assert port.names().count('PickStone') == 1
    assert out.result == 'SUCCESS'


# ---------------------------------------------------------------- 재시도 없음
def _detect(count):
    return Reply(lambda p: DetectResult(replace(M.OK_DET, stone_count=count), p.clock.ros_now(),
                                        (0.3, 0.0, 0.0)), 0.3)


@pytest.mark.req("REQ-ORC-006")
@pytest.mark.tc("TC-ORC-006-01")
@pytest.mark.level("U")
def test_detect_failure_is_l2_without_retry(fake_clock, cfg):
    _, port, out = run(fake_clock, cfg, {'DetectObjects.MAT': [_detect(0), _detect(1)]})
    assert port.names().count('DetectObjects.MAT') == 1
    detects = [r for r in out.records if r.step_name == 'DETECT_STONE']
    assert [(r.attempt, r.abort_level) for r in detects] == [(1, 2)]
    assert out.abort_level == AbortLevel.L2
    assert out.result == 'FAILURE'
    assert 'MoveToPose.safe' in port.names() and 'PickStone' not in port.names()


@pytest.mark.req("REQ-ORC-006")
@pytest.mark.tc("TC-ORC-006-02")
@pytest.mark.level("U")
@pytest.mark.parametrize('change', [{'stone_on_mat': True}, {'grip_current_ma': 10}])
def test_pick_failure_is_l2_without_regrasp_or_redetect(fake_clock, cfg, change):
    script = {'PickStone': [Reply(replace(M.OK_PICK, **change), 4.0), Reply(M.OK_PICK, 4.0)]}
    _, port, out = run(fake_clock, cfg, script)
    seq = [n for n in port.names() if n in ('DetectObjects.MAT', 'PickStone')]
    assert seq == ['DetectObjects.MAT', 'PickStone']
    assert out.abort_level == AbortLevel.L2
    assert out.result == 'FAILURE'


# ---------------------------------------------------------------- 통과 보내기
@pytest.mark.req("REQ-ORC-007")
@pytest.mark.tc("TC-ORC-007-01")
@pytest.mark.level("U")
def test_pass_skip_then_proceed(fake_clock, cfg):
    script = {'SelectTarget': [Reply(wait_cap_obs(BAD_MARGIN)), Reply(wait_cap_obs(M.OK_WAIT))]}
    _, port, out = run(fake_clock, cfg, script, g=goal(max_passes=2))
    assert out.passes_skipped == 1
    assert S.PLAN_INTERCEPT in out.states
    assert out.result == 'SUCCESS'


@pytest.mark.req("REQ-ORC-007")
@pytest.mark.tc("TC-ORC-007-02")
@pytest.mark.level("U")
def test_passes_exhausted_is_not_attempted(fake_clock, cfg):
    script = {'SelectTarget': [Reply(wait_cap_obs(BAD_MARGIN))]}
    _, port, out = run(fake_clock, cfg, script, g=goal(max_passes=2))
    assert port.names().count('SelectTarget') == 3
    assert out.abort_level == AbortLevel.L3
    assert out.result == 'NOT_ATTEMPTED'
    assert out.passes_skipped == 3
    assert ('SafeRetreat', {'keep_sync': False, 'place_back': True, 'place_back_pose': 'mat'}) in port.calls
    assert 'ComputeIntercept' not in port.names()


@pytest.mark.req("REQ-ORC-007")
@pytest.mark.tc("TC-ORC-007-01")
@pytest.mark.level("U")
def test_wait_then_ok_within_pass(fake_clock, cfg):
    waiting = replace(M.OK_WAIT, observation_count=10)
    script = {'SelectTarget': [Reply(wait_cap_obs(waiting))],
              'observe_cap': [Reply(wait_cap_obs(waiting)), Reply(wait_cap_obs(M.OK_WAIT))]}
    _, port, out = run(fake_clock, cfg, script)
    assert out.passes_skipped == 0
    assert port.names().count('observe_cap') == 2
    assert [r.step_name for r in out.records].count('WAIT_CAP') == 1     # WAIT은 기록하지 않는다


# ---------------------------------------------------------------- 계획 실패 → 다음 통과
@pytest.mark.req("REQ-ORC-008")
@pytest.mark.tc("TC-ORC-008-01")
@pytest.mark.level("U")
def test_plan_failure_goes_to_next_pass(fake_clock, cfg):
    def plan(port):
        if port.names().count('SelectTarget') < 2:
            return replace(M.OK_PLAN, plan_ok=False)
        now = port.clock.ros_now()
        return replace(M.OK_PLAN, now=now, valid_until=now + 5.0)
    _, port, out = run(fake_clock, cfg, {'ComputeIntercept': [Reply(plan, 0.05)]})
    names = port.names()
    second_select = [i for i, n in enumerate(names) if n == 'SelectTarget'][1]
    assert 'DynamicPlace' not in names[:second_select]
    i = out.states.index(S.PLAN_INTERCEPT)
    assert out.states[i + 1] is S.WAIT_CAP
    assert out.passes_skipped == 1
    assert out.result == 'SUCCESS'


# ---------------------------------------------------------------- WAIT_OPERATOR 재개
@pytest.mark.req("REQ-ORC-009")
@pytest.mark.tc("TC-ORC-009-01")
@pytest.mark.level("U")
def test_resume_requires_reason(fake_clock, cfg):
    runner, port, out = run(fake_clock, cfg, {'DetectObjects.MAT': [_detect(0), _detect(1)]})
    assert out.states[-1] is S.WAIT_OPERATOR
    assert out.wait_operator is not None and out.wait_operator.reason
    t_enter = out.wait_operator.t_enter
    n_calls = len(port.calls)
    refused = runner.run(goal())                       # 재개 전에는 실행하지 않는다
    assert refused.result == 'NOT_ATTEMPTED' and refused.reason_code == 'WAIT_OPERATOR'
    assert len(port.calls) == n_calls
    assert runner.resume('') is False
    assert runner.resume('   ') is False
    assert runner.resume('공깃돌 다시 놓음') is True
    assert runner.resumes[-1].reason == '공깃돌 다시 놓음'
    assert runner.resumes[-1].t_enter == t_enter
    again = runner.run(goal())
    assert again.states[:2] == [S.IDLE, S.PRECHECK]
    assert again.result == 'SUCCESS'


@pytest.mark.req("REQ-ORC-009")
@pytest.mark.tc("TC-ORC-009-01")
@pytest.mark.level("U")
def test_precheck_failure_returns_to_idle_without_wait_operator(fake_clock, cfg):
    script = {'status': [Reply(replace(M.OK_PRE, robot_ready=False), 0.0)]}
    runner, port, out = run(fake_clock, cfg, script)
    assert out.states == [S.IDLE, S.PRECHECK, S.IDLE]
    assert out.result == 'NOT_ATTEMPTED' and out.reason_code == 'ROBOT_NOT_READY'
    assert port.names() == ['status']
    assert out.wait_operator is None


# ---------------------------------------------------------------- action 무응답
@pytest.mark.req("REQ-ABT-004")
@pytest.mark.tc("TC-ABT-004-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("cancel_hang, level, result", [
    (False, AbortLevel.L2, 'FAILURE'),        # cancel 응답 → L2 그대로
    (True, AbortLevel.L3, 'SAFE_ABORT'),      # cancel 0.5 s 무응답 → 한 단계 승격
])
def test_pick_hang_cancel_escalation(fake_clock, cfg, cancel_hang, level, result):
    script = {'PickStone': [Reply(None, hang=True)], 'cancel': [Reply(True, 0.05, hang=cancel_hang)]}
    _, port, out = run(fake_clock, cfg, script)
    assert 'cancel' in port.names()
    assert out.abort_level == level and out.result == result
    assert 'MoveToPose.standby' not in port.names()


@pytest.mark.req("REQ-ABT-003")
@pytest.mark.tc("TC-ABT-003-02")
@pytest.mark.level("U")
def test_abort_path_hang_escalates_to_stop(fake_clock, cfg):
    script = {'SelectTarget': [Reply(wait_cap_obs(BAD_MARGIN))], 'SafeRetreat': [Reply(None, hang=True)]}
    _, port, out = run(fake_clock, cfg, script, g=goal(max_passes=0))
    assert out.abort_level == AbortLevel.L4
    assert out.result == 'PROTECTIVE_STOP'
    assert port.names()[-1] == 'Stop'


# ---------------------------------------------------------------- 비동기 장애·진행 알림(SPEC-100 v0.2 4.7절)
class FaultAfter(FakeSkillPort):
    """`trigger` skill을 부른 뒤부터 `event`를 장애로 돌려준다."""

    def __init__(self, clock, event, trigger):
        super().__init__(clock)
        self.event, self.trigger, self.fault = event, trigger, None

    def _call(self, key, deadline, **kw):
        r = super()._call(key, deadline, **kw)
        if key == self.trigger:
            self.fault = self.event
        return r

    def pending_fault(self):
        return self.fault


@pytest.mark.level("U")
def test_protective_stop_during_descend_ends_in_protective_stop(fake_clock, cfg):
    from orchestrator.abort_policy import Event
    port = FaultAfter(fake_clock, Event.PROTECTIVE_STOP, 'feedback.DESCEND')
    out = CycleRunner(port, fake_clock, cfg).run(goal())
    assert out.states[-3:] == [S.DESCEND, S.PROTECTIVE_STOP, S.WAIT_OPERATOR]
    assert out.result == 'PROTECTIVE_STOP' and out.abort_level is AbortLevel.L4
    assert out.records[-1].step_name == 'DESCEND' and out.records[-1].result == Result.FAULT
    assert out.records[-1].reason_code == 'PROTECTIVE_STOP'
    assert 'Stop' in port.names() and 'feedback.RELEASE' not in port.names()


@pytest.mark.level("U")
def test_vision_heartbeat_loss_before_pick_is_l2(fake_clock, cfg):
    from orchestrator.abort_policy import Event
    port = FaultAfter(fake_clock, Event.HEARTBEAT_LOST, 'DetectObjects.MAT')
    out = CycleRunner(port, fake_clock, cfg).run(goal())
    assert S.ABORT_CYCLE in out.states and out.result == 'FAILURE'
    assert 'PickStone' not in port.names()


@pytest.mark.level("U")
def test_progress_callbacks_match_outcome(fake_clock, cfg):
    seen_states, seen_records = [], []
    port = FakeSkillPort(fake_clock)
    out = CycleRunner(port, fake_clock, cfg, on_state=seen_states.append,
                      on_record=seen_records.append).run(goal())
    assert seen_states == out.states[1:]           # IDLE은 시작 상태라 알리지 않는다
    assert seen_records == out.records


@pytest.mark.req("REQ-ABT-001")
@pytest.mark.tc("TC-ABT-001-03")
@pytest.mark.level("U")
@pytest.mark.parametrize('event', list(Event))
@pytest.mark.parametrize('point', ['after_release', 'before_retreat', 'after_retreat',
                                  'before_verify', 'after_verify'])
def test_released_fault_executes_cleanup(fake_clock, cfg, event, point):
    trigger = {'after_release': 'feedback.RELEASE', 'after_retreat': 'result.RETREAT',
               'after_verify': 'VerifyDrop'}.get(point)
    cycle_end = fake_clock.ros_now() + goal().timeout_s
    class ExpiringFaultPort(FaultAfter):
        def _call(self, key, deadline, **kw):
            result = super()._call(key, deadline, **kw)
            if event is Event.CYCLE_TIMEOUT and key == trigger:
                fake_clock.advance_ros(max(0., cycle_end - fake_clock.ros_now() + .1))
            return result
    port = ExpiringFaultPort(fake_clock, event, trigger)

    def on_state(state):
        if (point == 'before_retreat' and state is S.RETREAT
                or point == 'before_verify' and state is S.VERIFY):
            port.fault = event
            if event is Event.CYCLE_TIMEOUT:
                fake_clock.advance_ros(max(0., cycle_end - fake_clock.ros_now() + .1))

    out = CycleRunner(port, fake_clock, cfg, on_state=on_state).run(goal())
    if event in HARD_FAULTS:
        assert out.abort_level is AbortLevel.L4
        assert out.result == 'PROTECTIVE_STOP'
        assert out.states[-2:] == [S.PROTECTIVE_STOP, S.WAIT_OPERATOR]
        assert 'Stop' in port.names()
    else:
        assert out.abort_level is AbortLevel.CONTINUE_TO_RETREAT_VERIFY
        assert out.states[-3:] == [S.RETREAT, S.VERIFY, S.DONE]
        assert port.names().count('result.RETREAT') == 1
        assert port.names().count('VerifyDrop') == 1
        assert out.result == ('CANCELED' if event is Event.USER_CANCEL else 'SUCCESS')
        assert 'Stop' not in port.names()


@pytest.mark.req("REQ-ABT-001")
@pytest.mark.tc("TC-ABT-001-03")
@pytest.mark.level("U")
@pytest.mark.parametrize('retreat_reply', [Reply(None, hang=True),
                                         Reply(replace(M.OK_RET, tcp_z_m=0.05), 1.0)])
def test_retreat_failure_overrides_latched_soft_fault(fake_clock, cfg, retreat_reply):
    port = FaultAfter(fake_clock, Event.HEARTBEAT_LOST, 'feedback.RELEASE')
    port.script['result.RETREAT'] = [retreat_reply]
    out = CycleRunner(port, fake_clock, cfg).run(goal())
    assert out.abort_level is AbortLevel.L4
    assert out.result == 'PROTECTIVE_STOP'
    assert port.names().count('result.RETREAT') == 1
    assert 'VerifyDrop' not in port.names()
    assert 'Stop' in port.names()
    assert out.records[-1].reason_code in ('STEP_TIMEOUT', 'Z_LOW')


@pytest.mark.req("REQ-ABT-002")
@pytest.mark.tc("TC-ABT-002-01")
@pytest.mark.level("U")
def test_released_soft_fault_escalates_to_hard_fault(fake_clock, cfg):
    port = FaultAfter(fake_clock, Event.HEARTBEAT_LOST, 'feedback.RELEASE')

    def on_state(state):
        if state is S.VERIFY:
            port.fault = Event.CONTROLLER_HEARTBEAT_LOST

    out = CycleRunner(port, fake_clock, cfg, on_state=on_state).run(goal())
    assert out.abort_level is AbortLevel.L4
    assert 'result.RETREAT' in port.names()
    assert 'VerifyDrop' not in port.names()
    assert 'Stop' in port.names()


@pytest.mark.req("REQ-ORC-003")
@pytest.mark.tc("TC-ORC-003-14")
@pytest.mark.level("U")
@pytest.mark.parametrize("duration, hang, expected", [
    (2.999, False, 'SUCCESS'), (3.000, False, 'SUCCESS'),
    (3.001, False, 'UNKNOWN'), (0.0, True, 'UNKNOWN'),
])
def test_verify_deadline_and_final_outcome(fake_clock, cfg, duration, hang, expected):
    class TimedPort(FakeSkillPort):
        def verify(self, deadline):
            self.verify_start = self.clock.ros_now()
            self.verify_deadline = deadline
            return super().verify(deadline)
    port = TimedPort(fake_clock, {'VerifyDrop': [Reply(M.OK_VER, duration, hang=hang)]})
    out = CycleRunner(port, fake_clock, cfg).run(goal())
    assert port.verify_deadline - port.verify_start == pytest.approx(3.0)
    assert fake_clock.ros_now() - port.verify_start == pytest.approx(min(duration, 3.0) if not hang else 3.0)
    assert out.result == expected
    assert port.names()[port.names().index('VerifyDrop') + 1:] == []
    assert out.states[-1] is S.DONE
    if expected == 'UNKNOWN':
        assert out.reason_code == 'VERIFY_TIMEOUT'
        assert out.records[-1].reason_code == 'VERIFY_TIMEOUT'


@pytest.mark.req('REQ-ABT-004')
@pytest.mark.tc('TC-ABT-004-01')
@pytest.mark.level('U')
def test_cancel_escalation_timeline(fake_clock, cfg, monkeypatch):
    import orchestrator.cycle_runner as cr
    observed = []
    class TracedTracker(cr.AbortTracker):
        def on_cancel_sent(self):
            observed.append(self.level)
            super().on_cancel_sent()
        def _set(self, level):
            super()._set(level)
            if observed and observed[-1] is not self.level:
                observed.append(self.level)
    monkeypatch.setattr(cr, "AbortTracker", TracedTracker)
    class CancelTimeline(FakeSkillPort):
        def cancel_active(self, wait_s):
            self.sent_at = self.clock.steady_now()
            assert wait_s == 0.5
            return super().cancel_active(wait_s)

        def stop(self):
            self.stop_at = self.clock.steady_now()
            return super().stop()
    port = CancelTimeline(fake_clock, {'PickStone': [Reply(None, hang=True)],
                                      'cancel': [Reply(None, hang=True)]})
    runner = CycleRunner(port, fake_clock, cfg)
    out = runner.run(replace(goal(), timeout_s=1.2))
    assert observed[:2] == [AbortLevel.L2, AbortLevel.L3]
    assert observed[-1] is AbortLevel.L4
    assert port.names().count('cancel') == 1
    assert port.stop_at - port.sent_at == pytest.approx(0.5)
    assert out.abort_level is AbortLevel.L4


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('point', ['before_retreat', 'during_retreat', 'before_verify'])
@pytest.mark.parametrize('failed', [False, True])
def test_cleanup_after_real_cycle_deadline(fake_clock, cfg, point, failed):
    cycle_end = fake_clock.ros_now() + goal().timeout_s
    class DeadlinePort(FakeSkillPort):
        def place_feedback(self, phase, deadline):
            if phase == 'RETREAT':
                assert deadline > self.clock.ros_now()
                if point == 'during_retreat':
                    self.clock.advance_ros(cycle_end - self.clock.ros_now() + .1)
                    assert deadline > self.clock.ros_now()
            return super().place_feedback(phase, deadline)
        def verify(self, deadline):
            assert deadline - self.clock.ros_now() == pytest.approx(cfg.verify.timeout_s)
            return super().verify(deadline)
    port = DeadlinePort(fake_clock)
    if failed:
        port.script['result.RETREAT'] = [Reply(replace(M.OK_RET, tcp_z_m=.05), .1)]
    def on_state(state):
        if ((point == 'before_retreat' and state is S.RETREAT)
                or (point == 'before_verify' and state is S.VERIFY)):
            fake_clock.advance_ros(cycle_end - fake_clock.ros_now() + .1)
        elif point == 'during_retreat' and state is S.RETREAT:
            fake_clock.advance_ros(cycle_end - fake_clock.ros_now() - .1)
    out = CycleRunner(port, fake_clock, cfg, on_state=on_state).run(goal())
    if failed:
        assert out.abort_level is AbortLevel.L4
        assert 'Stop' in port.names() and 'VerifyDrop' not in port.names()
    else:
        assert out.result == 'SUCCESS'
        assert out.states[-3:] == [S.RETREAT, S.VERIFY, S.DONE]
        assert port.names().count('VerifyDrop') == 1


# ---------------------------------------------------------------- 여러 단계에 걸친 goal의 deadline
class _PlaceSpy(FakeSkillPort):
    def place_start(self, mode, deadline, goal_deadline=None):
        self.place = (self.clock.ros_now(), deadline, goal_deadline)
        return super().place_start(mode, deadline, goal_deadline=goal_deadline)


@pytest.mark.req("REQ-ORC-015")
@pytest.mark.tc("TC-ORC-015-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("mode, steps", [
    (Mode.SYNCHRONIZED, ('approach_s', 'synchronize_s', 'descend_s', 'release_s', 'retreat_s')),
    (Mode.PREDICTED, ('approach_s', 'synchronize_s', 'descend_s', 'release_s', 'retreat_s')),
    (Mode.STATIC, ('approach_s', 'descend_s', 'release_s', 'retreat_s')),
])
def test_place_goal_deadline_covers_all_phases(fake_clock, cfg, orch_cfg, mode, steps):
    port = _PlaceSpy(fake_clock, mode_entry=M.OK_ENTRY if mode is Mode.PREDICTED else None)
    out = CycleRunner(port, fake_clock, cfg).run(goal(mode=mode))
    assert out.result == 'SUCCESS'
    t_send, step_deadline, goal_deadline = port.place
    total = sum(float(orch_cfg['step_timeout'][k]) for k in steps)
    assert goal_deadline == pytest.approx(t_send + total)
    assert step_deadline <= t_send + float(orch_cfg['step_timeout']['approach_s']) + 1e-9
    assert goal_deadline > step_deadline            # goal 전체 상한은 APPROACH 단계 deadline과 다르다


# ---------------------------------------------------------------- 대상 track 소실 → 같은 통과에서 다시 선택
@pytest.mark.req("REQ-ORC-016")
@pytest.mark.tc("TC-ORC-016-01")
@pytest.mark.level("U")
def test_lost_target_is_reselected_within_pass(fake_clock, cfg):
    counting = replace(M.OK_WAIT, observation_count=10)
    lost = replace(M.OK_WAIT, track_observed=False, observation_count=0)
    script = {'SelectTarget': [Reply(wait_cap_obs(counting)), Reply(wait_cap_obs(M.OK_WAIT))],
              'observe_cap': [Reply(lambda port: CapObservation(lost, port.clock.ros_now() + 8.0, target_lost=True))]}
    _, port, out = run(fake_clock, cfg, script)
    selects = [kw['pass_index'] for name, kw in port.calls if name == 'SelectTarget']
    assert selects == [1, 1]                         # 같은 통과 안에서 다시 고른다
    assert port.names().count('observe_cap') == 1
    assert out.passes_skipped == 0
    assert out.result == 'SUCCESS'


@pytest.mark.req("REQ-ORC-016")
@pytest.mark.tc("TC-ORC-016-01")
@pytest.mark.level("U")
def test_coasting_target_is_not_reselected(fake_clock, cfg):
    coasting = replace(M.OK_WAIT, track_observed=False)
    script = {'SelectTarget': [Reply(wait_cap_obs(coasting))],
              'observe_cap': [Reply(wait_cap_obs(coasting)), Reply(wait_cap_obs(M.OK_WAIT))]}
    _, port, out = run(fake_clock, cfg, script)
    assert port.names().count('SelectTarget') == 1   # 가림(COASTING)은 기다린다
    assert out.result == 'SUCCESS'


# ---------------------------------------------------------------- 대기 중 끝난 WAIT_CAP의 기록
@pytest.mark.req("REQ-ORC-017")
@pytest.mark.tc("TC-ORC-017-01")
@pytest.mark.level("U")
def test_wait_cap_ended_while_waiting_keeps_last_wait_reason(fake_clock, cfg):
    """대기 중 사이클 제한으로 끝난 WAIT_CAP 기록에 마지막 대기 사유와 지표가 남는다(원인 추적용)."""
    counting = replace(M.OK_WAIT, observation_count=10)
    script = {'SelectTarget': [Reply(wait_cap_obs(counting))], 'observe_cap': [Reply(hang=True)]}
    port = FakeSkillPort(fake_clock, script)
    out = CycleRunner(port, fake_clock, cfg).run(replace(goal(), timeout_s=20.0))
    rec = next(r for r in out.records if r.step_name == 'WAIT_CAP')
    assert rec.reason_code == 'CYCLE_TIMEOUT'
    assert rec.detail == 'OBSERVATION_COUNT'
    assert dict(zip(rec.metric_keys, rec.metric_values))['observation_count'] == 10


@pytest.mark.req("REQ-ORC-017")
@pytest.mark.tc("TC-ORC-017-01")
@pytest.mark.level("U")
def test_wait_cap_without_any_target_is_no_target(fake_clock, cfg):
    script = {'SelectTarget': [Reply(hang=True)]}
    port = FakeSkillPort(fake_clock, script)
    out = CycleRunner(port, fake_clock, cfg).run(replace(goal(), timeout_s=20.0))
    rec = next(r for r in out.records if r.step_name == 'WAIT_CAP')
    assert (rec.detail, rec.metric_keys) == ('NO_TARGET', [])
