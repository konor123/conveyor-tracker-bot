"""중단 등급 판정·승격·중단 경로 제한시간(SPEC-110 4.1·4.2절)."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from .common import DYNAMIC_STATES, AbortLevel, FromDict, State, SteadyClock, ge

L = AbortLevel


class Event(Enum):
    # 하드 이상(단계와 무관하게 L4)
    PROTECTIVE_STOP = auto()
    ESTOP = auto()
    ROBOT_COMM_LOST = auto()
    CONTROLLER_HEARTBEAT_LOST = auto()
    ABORT_PATH_TIMEOUT = auto()
    RETREAT_FAILED = auto()
    # 단계 결과
    STEP_FAILED = auto()
    STEP_TIMEOUT = auto()
    USER_CANCEL = auto()
    PASSES_EXHAUSTED = auto()
    CYCLE_TIMEOUT = auto()
    PLAN_INVALID = auto()
    # 감시(watchdogs.py)
    HEARTBEAT_LOST = auto()       # vision·tracker·planner
    OBS_STALE = auto()
    CLOCK_JUMP = auto()
    BELT_UNSTEADY = auto()
    GRIPPER_FAULT = auto()


HARD_FAULTS = frozenset({Event.PROTECTIVE_STOP, Event.ESTOP, Event.ROBOT_COMM_LOST,
                         Event.CONTROLLER_HEARTBEAT_LOST, Event.ABORT_PATH_TIMEOUT,
                         Event.RETREAT_FAILED})
RETRYABLE_EVENTS = frozenset({Event.STEP_FAILED, Event.STEP_TIMEOUT})

# 심각도 순서(SPEC-110 4.2절): NONE < L1 < L2 < L3 < CONTINUE_TO_RETREAT_VERIFY < L4
_RANK = {L.NONE: 0, L.L1: 1, L.L2: 2, L.L3: 3, L.CONTINUE_TO_RETREAT_VERIFY: 4, L.L4: 5}
_ONE_UP = {L.NONE: L.L2, L.L1: L.L2, L.L2: L.L3, L.L3: L.L4,
           L.CONTINUE_TO_RETREAT_VERIFY: L.L4, L.L4: L.L4}


@dataclass(frozen=True)
class AbortCtx:
    state: State
    holding: bool
    released: bool
    retry_left: int          # 이 단계의 남은 재시도 횟수
    retryable: bool          # 이 실패가 재시도 대상인지(예: PICK_STONE은 STONE_ON_MAT만)
    now: float               # ROS 시각 [s]
    est_duration_s: float    # 이 단계를 다시 하는 데 걸리는 예상 시간 [s]
    deadline: float          # 실제 deadline(ROS 시각) [s]


@dataclass(frozen=True)
class AbortParams(FromDict):
    abort_path_l2_s: float = 10.0
    abort_path_l3_s: float = 15.0
    cancel_response_s: float = 0.5
    call_timeout_s: float = 1.0


def decide(event: Event, ctx: AbortCtx) -> AbortLevel:
    """SPEC-110 4.1절 순서."""
    if event in HARD_FAULTS:
        return L.L4
    if ctx.released:
        return L.CONTINUE_TO_RETREAT_VERIFY
    if ctx.holding or ctx.state in DYNAMIC_STATES or event is Event.PASSES_EXHAUSTED:
        return L.L3
    if (event in RETRYABLE_EVENTS and ctx.retryable and ctx.retry_left > 0
            and ctx.now + ctx.est_duration_s < ctx.deadline):
        return L.L1
    return L.L2


def escalate(current: AbortLevel, new: AbortLevel) -> AbortLevel:
    return new if _RANK[new] > _RANK[current] else current


class AbortTracker:
    """진행 중 중단 등급과 중단 경로·cancel 응답 제한시간(steady 시계)."""

    def __init__(self, p: AbortParams, clock: SteadyClock):
        self._p = p
        self._clock = clock
        self.level = L.NONE
        self.waiting_operator = False
        self._path_start: float | None = None
        self._cancel_sent: float | None = None

    def on_event(self, event: Event, ctx: AbortCtx) -> AbortLevel:
        """판정한 등급을 돌려준다. L1은 재시도로 처리하므로 진행 중 등급으로 남기지 않는다."""
        new = decide(event, ctx)
        if new is L.L1:
            return new if self.level is L.NONE else self.level
        self._set(escalate(self.level, new))
        return self.level

    def on_cancel_sent(self) -> None:
        self._cancel_sent = self._clock.steady_now()

    def on_cancel_ack(self) -> None:
        self._cancel_sent = None

    def on_path_done(self) -> None:
        """중단 경로가 끝나 WAIT_OPERATOR에 들어간다. 이후 제한시간이 없다."""
        self._path_start = None
        self._cancel_sent = None
        self.waiting_operator = True

    def tick(self) -> AbortLevel:
        now = self._clock.steady_now()
        if self._cancel_sent is not None and ge(now - self._cancel_sent, self._p.cancel_response_s):
            self._cancel_sent = None
            self._set(_ONE_UP[self.level])
        if self._path_start is not None:
            limit = {L.L2: self._p.abort_path_l2_s, L.L3: self._p.abort_path_l3_s}.get(self.level)
            if limit is not None and now - self._path_start > limit + 1e-9:
                self._set(L.L4)
        return self.level

    def _set(self, level: AbortLevel) -> None:
        if level is self.level:
            return
        self.level = level
        # L2·L3 경로는 그 등급에 들어간 시점부터 잰다. L4·CONTINUE는 경로 제한시간이 없다.
        self._path_start = self._clock.steady_now() if level in (L.L2, L.L3) else None
        if level in (L.L2, L.L3, L.L4):
            self.waiting_operator = False
