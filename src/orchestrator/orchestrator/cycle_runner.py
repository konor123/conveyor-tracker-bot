"""한 사이클 실행기(SPEC-100 4.2·4.5·4.6·4.7절). ROS에 의존하지 않는다.

skill은 SkillPort로 부른다. 포트 메서드는 응답(측정값)을 돌려주고, deadline 안에 응답이 없으면 None이다.
orchestrator_node는 ROS service·action 클라이언트로 SkillPort를 구현해 이 실행기를 작업 스레드에서 돌린다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Protocol

from . import step_judges as sj
from .abort_policy import AbortCtx, AbortParams, AbortTracker, Event
from .common import (DYNAMIC_STATES, AbortLevel, Clock, Judgement, Mode, Result, State, le,
                     metrics_to_record)
from .deadlines import effective_deadline
from .state_machine import CycleCtx, next_state
from .watchdogs import CycleTimer

S = State
L = AbortLevel


# ---------------------------------------------------------------- 입출력 타입
@dataclass(frozen=True)
class DropGoal:
    mode: Mode
    max_belt_speed_mps: float
    timeout_s: float
    max_passes: int


@dataclass(frozen=True)
class DetectResult:
    meas: sj.DetectMeas
    stamp: float                      # 검출에 쓴 영상 노출 시각(ROS)
    target: Any                       # 공깃돌 위치(robot_base, m). PickStone goal에 그대로 넘긴다


@dataclass(frozen=True)
class CapObservation:
    meas: sj.WaitCapMeas
    dynamic_deadline: float | None    # 이 통과의 동적 deadline(ROS). STATIC은 None


@dataclass(frozen=True)
class StepRec:
    step_seq: int
    step_name: str
    t_start: float
    t_end: float
    result: Result
    reason_code: str
    attempt: int
    abort_level: int                  # 0 = 없음, 1~4
    metric_keys: list[str]
    metric_values: list[float]
    detail: str = ''


@dataclass(frozen=True)
class WaitOperatorEntry:
    t_enter: float                    # ROS 시각
    reason: str
    level: AbortLevel


@dataclass(frozen=True)
class ResumeRecord:
    t_enter: float
    t_resume: float
    reason: str


@dataclass
class CycleOutcome:
    states: list[State]
    records: list[StepRec]
    result: str                       # DropTask 결과 이름(SUCCESS … NOT_ATTEMPTED)
    reason_code: str
    abort_level: AbortLevel
    passes_skipped: int
    wait_operator: WaitOperatorEntry | None = None
    cycle_time_s: float = 0.0


class SkillPort(Protocol):
    def status(self) -> sj.PrecheckMeas: ...
    def detect_stones(self, deadline: float) -> DetectResult | None: ...
    def pick_stone(self, target: Any, deadline: float) -> sj.PickMeas | None: ...
    def move_to_pose(self, name: str, deadline: float) -> Any: ...
    def select_target(self, pass_index: int, deadline: float) -> CapObservation | None: ...
    def detect_cap(self, deadline: float) -> CapObservation | None: ...
    def observe_cap(self, deadline: float) -> CapObservation | None: ...
    def compute_intercept(self, deadline: float) -> sj.PlanMeas | None: ...
    def place_start(self, mode: Mode, deadline: float) -> sj.ApproachMeas | None: ...
    def place_feedback(self, phase: str, deadline: float) -> Any: ...
    def verify(self, deadline: float) -> sj.VerifyMeas | None: ...
    def safe_retreat(self, keep_sync: bool, place_back: bool, deadline: float) -> Any: ...
    def stop(self) -> Any: ...
    def cancel_active(self, wait_s: float) -> bool: ...
    def cancel_requested(self) -> bool: ...
    # 선택: 비동기 장애(SPEC-100 4.7절). 없으면 장애 없음으로 본다
    # def pending_fault(self) -> Event | None: ...


# ---------------------------------------------------------------- 설정
@dataclass(frozen=True)
class OrchestratorCfg:
    precheck: dict
    detect: sj.DetectParams
    pick: sj.PickParams
    standby: sj.StandbyParams
    wait_cap: sj.WaitCapParams
    plan: sj.PlanParams
    approach: sj.ApproachParams
    sync: sj.SyncParams
    entry: sj.EntryParams
    descend: sj.DescendParams
    static_place: sj.StaticPlaceParams
    release: sj.ReleaseParams
    retreat: sj.RetreatParams
    verify: sj.VerifyParams
    step_timeout: dict
    retry: dict
    abort: AbortParams
    watchdog: dict

    @classmethod
    def from_dict(cls, d: dict) -> 'OrchestratorCfg':
        j = d['judge']
        return cls(
            precheck=dict(j['precheck']),
            detect=sj.DetectParams.from_dict(j['detect_stone']),
            pick=sj.PickParams.from_dict(j['pick_stone']),
            standby=sj.StandbyParams.from_dict(j['standby']),
            wait_cap=sj.WaitCapParams.from_dict(j['wait_cap']),
            plan=sj.PlanParams.from_dict(j['plan']),
            approach=sj.ApproachParams.from_dict(j['approach']),
            sync=sj.SyncParams.from_dict(j['sync']),
            entry=sj.EntryParams.from_dict(j['predicted']),
            descend=sj.DescendParams.from_dict(j['descend']),
            static_place=sj.StaticPlaceParams.from_dict(j['static_place']),
            release=sj.ReleaseParams.from_dict(j['release']),
            retreat=sj.RetreatParams.from_dict(j['retreat']),
            verify=sj.VerifyParams.from_dict(j['verify']),
            step_timeout=dict(d['step_timeout']),
            retry=dict(d['retry']),
            abort=AbortParams.from_dict(d['abort']),
            watchdog=dict(d['watchdog']),
        )

    def step_limit_s(self, state: State, max_passes: int = 0) -> float:
        if state is State.WAIT_CAP:          # T_lap × (max_passes + 1)
            return float(self.step_timeout['lap_s']) * (max_passes + 1)
        return float(self.step_timeout[f'{state.name.lower()}_s'])


def _timeout(reason: str = 'STEP_TIMEOUT') -> Judgement:
    return Judgement(Result.TIMEOUT, reason, {})


# ---------------------------------------------------------------- 실행기
@dataclass
class _Cycle:
    goal: DropGoal
    t0: float
    deadline: float
    timer: CycleTimer
    tracker: AbortTracker
    holding: bool = False
    released: bool = False
    release_failed: bool = False
    detection: DetectResult | None = None
    cap: CapObservation | None = None
    pass_index: int = 0
    passes_skipped: int = 0
    precondition_count: int = 0
    attempts: dict = field(default_factory=dict)
    retries_used: dict = field(default_factory=dict)
    verdict: str = 'UNKNOWN'
    active_goal: bool = False


class CycleRunner:
    def __init__(self, port: SkillPort, clock: Clock, cfg: OrchestratorCfg,
                 on_state: Callable[[State], None] | None = None,
                 on_record: Callable[[StepRec], None] | None = None):
        self._port = port
        self._clock = clock
        self._cfg = cfg
        self._on_state = on_state
        self._on_record = on_record
        self._waiting: WaitOperatorEntry | None = None
        self.resumes: list[ResumeRecord] = []

    # ------------------------------------------------------------ 공개
    def resume(self, reason: str) -> bool:
        """WAIT_OPERATOR 재개. 빈 사유는 거부한다. 다음 run()은 PRECHECK부터 시작한다."""
        if self._waiting is None or not reason or not reason.strip():
            return False
        self.resumes.append(ResumeRecord(self._waiting.t_enter, self._clock.ros_now(), reason.strip()))
        self._waiting = None
        return True

    @property
    def waiting_operator(self) -> WaitOperatorEntry | None:
        return self._waiting

    def run(self, goal: DropGoal) -> CycleOutcome:
        if self._waiting is not None:
            return CycleOutcome([], [], 'NOT_ATTEMPTED', 'WAIT_OPERATOR', self._waiting.level, 0,
                                self._waiting)
        clock, cfg = self._clock, self._cfg
        t0 = clock.ros_now()
        c = _Cycle(goal=goal, t0=t0, deadline=t0 + goal.timeout_s,
                   timer=CycleTimer(clock, t0 + goal.timeout_s,
                                    goal.timeout_s * cfg.watchdog['cycle_steady_factor_ratio']),
                   tracker=AbortTracker(cfg.abort, clock))
        states: list[State] = [S.IDLE]
        records: list[StepRec] = []
        result, reason = '', ''
        state = S.PRECHECK
        while True:
            states.append(state)
            if self._on_state is not None:
                self._on_state(state)
            if state in (S.DONE, S.IDLE):
                break
            if state in (S.ABORT_CYCLE, S.SAFE_ABORT, S.PROTECTIVE_STOP):
                self._abort_path(c, state, states[-2])
                if c.tracker.level is L.L4:           # 중단 경로 자체가 L4로 승격된 경우 포함
                    result = 'PROTECTIVE_STOP'
                states.append(S.WAIT_OPERATOR)
                if self._on_state is not None:
                    self._on_state(S.WAIT_OPERATOR)
                self._waiting = WaitOperatorEntry(clock.ros_now(), reason or state.name, c.tracker.level)
                break
            event = self._pending_event(c, state)
            if event is not None:
                j = Judgement(self._event_result(event), event.name, {})
                level = self._level(c, state, event, j)
                if event is Event.USER_CANCEL:
                    result = 'CANCELED'
            else:
                t_start = clock.ros_now()
                j, event = self._execute(c, state)
                fault = self._fault()
                if fault is not None:                 # 단계 중 생긴 비동기 장애가 판정보다 우선
                    j, event = Judgement(Result.FAULT, fault.name, j.metrics), fault
                level = self._level(c, state, event, j) if event is not None else L.NONE
                if j.reason_code != sj.WAIT:
                    self._record(records, c, state, j, t_start, level)
            if state is S.PRECHECK and j.result is not Result.OK:
                result, reason = 'NOT_ATTEMPTED', j.reason_code
            elif level in (L.L2, L.L3, L.L4) and not result:
                result = self._abort_result(level, event)
                reason = j.reason_code
            ctx = CycleCtx(detection_fresh=self._detection_fresh(c))
            state = next_state(state, goal.mode, j, level, ctx)
            if state is S.DONE:
                result = result or ('FAILURE' if c.release_failed and c.verdict == 'SUCCESS'
                                    else c.verdict)
        return CycleOutcome(states, records, result, reason, c.tracker.level, c.passes_skipped,
                            self._waiting, clock.ros_now() - t0)

    # ------------------------------------------------------------ 단계 실행
    def _deadline(self, c: _Cycle, state: State, limit: float | None = None) -> float:
        dynamic = None
        if c.cap is not None and state in (S.PLAN_INTERCEPT, S.APPROACH, S.SYNCHRONIZE, S.DESCEND,
                                           S.RELEASE):
            dynamic = c.cap.dynamic_deadline
        step = self._cfg.step_limit_s(state) if limit is None else limit
        return effective_deadline(self._clock.ros_now(), step, dynamic, c.deadline)

    def _execute(self, c: _Cycle, state: State) -> tuple[Judgement, Event | None]:
        """판정과 (실패 시) 중단 판정용 이벤트를 돌려준다. 이벤트 None = 중단 판정 없음."""
        c.attempts[state] = c.attempts.get(state, 0) + 1
        fn = getattr(self, f'_do_{state.name.lower()}')
        return fn(c)

    def _fail_event(self, j: Judgement) -> Event | None:
        if j.result is Result.OK:
            return None
        return Event.STEP_TIMEOUT if j.result is Result.TIMEOUT else Event.STEP_FAILED

    def _do_precheck(self, c):
        m = self._port.status()
        p = sj.PrecheckParams.from_dict(self._cfg.precheck, max_belt_speed_mps=c.goal.max_belt_speed_mps)
        return sj.judge_precheck(m, p), None          # 실패 시 시도하지 않고 IDLE

    def _do_detect_stone(self, c):
        r = self._port.detect_stones(self._deadline(c, S.DETECT_STONE))
        if r is None:
            return _timeout(), Event.STEP_TIMEOUT
        c.detection = r
        j = sj.judge_detect_stone(r.meas, self._cfg.detect)
        return j, self._fail_event(j)

    def _detection_fresh(self, c) -> bool:
        return (c.detection is not None
                and le(self._clock.ros_now() - c.detection.stamp, self._cfg.pick.detection_age_max_s))

    def _do_pick_stone(self, c):
        if not self._detection_fresh(c):
            c.precondition_count += 1
            j = Judgement(Result.PRECONDITION, 'DETECTION_STALE', {})
            # 재검출은 retry.pick_stone_count번까지. 넘으면 재시도 불가 실패(L2)
            over = c.precondition_count > self._cfg.retry['pick_stone_count']
            return j, (Event.STEP_FAILED if over else None)
        m = self._run_action(c, lambda d: self._port.pick_stone(c.detection.target, d), S.PICK_STONE)
        if m is None:
            return _timeout(), Event.STEP_TIMEOUT
        j = sj.judge_pick_stone(m, self._cfg.pick)
        c.holding = j.result is Result.OK
        return j, self._fail_event(j)

    def _do_move_standby(self, c):
        m = self._run_action(c, lambda d: self._port.move_to_pose('standby', d), S.MOVE_STANDBY)
        if m is None:
            return _timeout(), Event.STEP_TIMEOUT
        j = sj.judge_move_standby(m, self._cfg.standby)
        return j, self._fail_event(j)

    def _do_wait_cap(self, c):
        clock, g = self._clock, c.goal
        lap = float(self._cfg.step_timeout['lap_s'])
        total_end = c.t0 + c.goal.timeout_s
        pass_end = min(clock.ros_now() + lap, total_end)
        c.pass_index += 1
        if g.mode is Mode.STATIC:
            obs = self._port.detect_cap(pass_end)
        else:
            obs = self._port.select_target(c.pass_index, pass_end)
        while True:
            if obs is None:
                j = Judgement(Result.FAILED, sj.PASS_SKIP, {}, 'NO_TARGET')
                break
            j = sj.judge_wait_cap(obs.meas, self._cfg.wait_cap)
            if j.reason_code != sj.WAIT:
                break
            if clock.ros_now() >= pass_end:
                j = Judgement(Result.FAILED, sj.PASS_SKIP, j.metrics, 'PASS_TIMEOUT')
                break
            # 포트는 새 관측이 올 때까지(또는 deadline까지) 기다렸다가 돌려준다
            obs = (self._port.detect_cap(pass_end) if g.mode is Mode.STATIC
                   else self._port.observe_cap(pass_end))
        if j.result is Result.OK:
            c.cap = obs
            return j, None
        if j.reason_code == sj.PASS_SKIP:
            return j, self._skip_pass(c)
        return j, Event.STEP_FAILED

    def _skip_pass(self, c) -> Event | None:
        c.passes_skipped += 1
        c.cap = None
        return Event.PASSES_EXHAUSTED if c.passes_skipped > c.goal.max_passes else None

    def _do_plan_intercept(self, c):
        deadline = self._deadline(c, S.PLAN_INTERCEPT)
        j = _timeout('PLAN_DEADLINE')
        while self._clock.ros_now() < deadline:
            m = self._port.compute_intercept(deadline)
            if m is None:
                break
            j = sj.judge_plan_intercept(m, self._cfg.plan)
            if j.result is Result.OK:
                return j, None
        # 동적 deadline까지 OK가 없으면 이번 통과를 보낸다(대기 자세, 동적 구간 아님)
        return j, self._skip_pass(c)

    def _do_approach(self, c):
        m = self._run_action(c, lambda d: self._port.place_start(c.goal.mode, d), S.APPROACH,
                             keeps_running=True)
        if m is None:
            return _timeout(), Event.STEP_TIMEOUT
        j = sj.judge_approach(m, self._cfg.approach)
        return j, self._fail_event(j)

    def _do_synchronize(self, c):
        fb = self._port.place_feedback('SYNCHRONIZE', self._deadline(c, S.SYNCHRONIZE))
        if fb is None:
            return _timeout(), Event.STEP_TIMEOUT
        if c.goal.mode is Mode.PREDICTED:
            j = sj.judge_predicted_entry(fb, self._cfg.entry)
        else:
            j = sj.judge_synchronize(fb, self._cfg.sync)
            if j.reason_code == sj.WAIT:          # feedback이 끝났는데 판정이 없음
                j = Judgement(Result.TIMEOUT, 'TIMEOUT', j.metrics)
        return j, self._fail_event(j)

    def _do_descend(self, c):
        fb = self._port.place_feedback('DESCEND', self._deadline(c, S.DESCEND))
        if fb is None:
            return _timeout(), Event.STEP_TIMEOUT
        if c.goal.mode is Mode.STATIC:
            j = sj.judge_static_place(fb, self._cfg.static_place)
        else:
            j = sj.judge_descend(fb, self._cfg.descend)
        return j, self._fail_event(j)

    def _do_release(self, c):
        fb = self._port.place_feedback('RELEASE', self._deadline(c, S.RELEASE))
        if fb is None:
            return _timeout(), Event.STEP_TIMEOUT
        c.released, c.holding = True, False
        j = sj.judge_release(fb, self._cfg.release)
        c.release_failed = j.result is not Result.OK
        return j, self._fail_event(j)

    def _do_retreat(self, c):
        fb = self._port.place_feedback('RETREAT', self._deadline(c, S.RETREAT))
        c.active_goal = False
        if fb is None:
            return _timeout(), Event.RETREAT_FAILED
        j = sj.judge_retreat(fb, self._cfg.retreat)
        return j, (Event.RETREAT_FAILED if j.result is not Result.OK else None)

    def _do_verify(self, c):
        m = self._port.verify(self._deadline(c, S.VERIFY, self._cfg.verify.timeout_s))
        j = sj.judge_verify(m if m is not None else sj.VerifyMeas(False, sj.VERDICT_UNKNOWN),
                            self._cfg.verify)
        c.verdict = j.detail or 'UNKNOWN'
        return j, None

    def _run_action(self, c, call, state, keeps_running=False):
        """action goal을 보낸다. 응답이 없거나(취소 필요) 이후 단계까지 이어지는 goal이면 진행 중으로 둔다."""
        c.active_goal = True
        m = call(self._deadline(c, state))
        c.active_goal = keeps_running or m is None
        return m

    # ------------------------------------------------------------ 중단
    def _fault(self) -> Event | None:
        pending = getattr(self._port, 'pending_fault', None)
        return pending() if pending is not None else None

    @staticmethod
    def _event_result(event: Event) -> Result:
        if event is Event.USER_CANCEL:
            return Result.CANCELED
        if event is Event.CYCLE_TIMEOUT:
            return Result.TIMEOUT
        return Result.FAULT

    def _pending_event(self, c, state) -> Event | None:
        fault = self._fault()
        if fault is not None:
            return fault
        if self._port.cancel_requested():
            return Event.USER_CANCEL
        if not c.released and c.timer.expired():
            return Event.CYCLE_TIMEOUT
        return None

    def _level(self, c, state, event, j) -> AbortLevel:
        retry_key = {S.DETECT_STONE: 'detect_stone_count', S.PICK_STONE: 'pick_stone_count'}.get(state)
        retry_left = 0
        if retry_key is not None:
            retry_left = self._cfg.retry[retry_key] - c.retries_used.get(state, 0)
        retryable = state is S.DETECT_STONE or (state is S.PICK_STONE and j.reason_code == 'STONE_ON_MAT')
        ctx = AbortCtx(state=state, holding=c.holding, released=c.released, retry_left=retry_left,
                       retryable=retryable, now=self._clock.ros_now(),
                       est_duration_s=self._cfg.step_limit_s(state, c.goal.max_passes),
                       deadline=c.deadline)
        level = c.tracker.on_event(event, ctx)
        if level is L.L1:
            c.retries_used[state] = c.retries_used.get(state, 0) + 1
        if event is Event.STEP_TIMEOUT and c.active_goal:
            # action 무응답: cancel 후 cancel_response_s 안에 응답이 없으면 한 단계 승격
            c.tracker.on_cancel_sent()
            if self._port.cancel_active(self._cfg.abort.cancel_response_s):
                c.tracker.on_cancel_ack()
            c.active_goal = False
            level = c.tracker.tick()
        return level

    @staticmethod
    def _abort_result(level: AbortLevel, event: Event | None) -> str:
        if event is Event.USER_CANCEL:
            return 'CANCELED'
        if event is Event.PASSES_EXHAUSTED:
            return 'NOT_ATTEMPTED'
        return {L.L2: 'FAILURE', L.L3: 'SAFE_ABORT', L.L4: 'PROTECTIVE_STOP'}[level]

    def _abort_path(self, c, abort_state: State, failed_state: State) -> None:
        """L2: 안전 자세 / L3: (동기 유지) 상승 → 매트에 내려놓음 / L4: Stop. 경로 제한시간 초과 시 L4."""
        port, clock, p = self._port, self._clock, self._cfg.abort
        if c.active_goal:
            c.tracker.on_cancel_sent()
            if port.cancel_active(p.cancel_response_s):
                c.tracker.on_cancel_ack()
            c.active_goal = False
            c.tracker.tick()
        ok = True
        if c.tracker.level is L.L2:
            ok = bool(port.move_to_pose('safe', clock.ros_now() + p.abort_path_l2_s + 0.001))
        elif c.tracker.level is L.L3:
            ok = bool(port.safe_retreat(keep_sync=failed_state in DYNAMIC_STATES, place_back=True,
                                        deadline=clock.ros_now() + p.abort_path_l3_s + 0.001))
        if not ok:
            c.tracker._set(L.L4)
        if c.tracker.tick() is L.L4:
            port.stop()
        c.tracker.on_path_done()

    def _record(self, records, c, state, j, t_start, level) -> None:
        keys, values = metrics_to_record(j)
        lv = int(level) if level in (L.L1, L.L2, L.L3, L.L4) else 0
        rec = StepRec(len(records) + 1, state.name, t_start, self._clock.ros_now(), j.result,
                      j.reason_code, c.attempts.get(state, 1), lv, keys, values, j.detail)
        records.append(rec)
        if self._on_record is not None:
            self._on_record(rec)
