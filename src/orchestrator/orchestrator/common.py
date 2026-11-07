"""오케스트레이터 공통 타입(SPEC-100 2절, SPEC-110 2절). ROS에 의존하지 않는다."""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Protocol

# 경계 비교 허용오차(SPEC-110 3절). 시각·길이 모두 같은 값을 쓴다.
EPS = 1e-9


def le(a: float, b: float) -> bool:
    """a ≤ b (허용오차 포함)."""
    return a <= b + EPS


def ge(a: float, b: float) -> bool:
    """a ≥ b (허용오차 포함)."""
    return a >= b - EPS


class Result(IntEnum):
    OK = 0
    FAILED = 1
    TIMEOUT = 2
    PRECONDITION = 3
    CANCELED = 4
    FAULT = 5


class AbortLevel(IntEnum):
    NONE = 0
    L1 = 1
    L2 = 2
    L3 = 3
    L4 = 4
    CONTINUE_TO_RETREAT_VERIFY = 10


class Mode(IntEnum):
    STATIC = 0          # DropTask.MODE_STATIC
    PREDICTED = 1       # DropTask.MODE_PREDICTED
    SYNCHRONIZED = 2    # DropTask.MODE_SYNCHRONIZED


class State(IntEnum):
    IDLE = 0
    PRECHECK = 1
    DETECT_STONE = 2
    PICK_STONE = 3
    MOVE_STANDBY = 4
    WAIT_CAP = 5
    PLAN_INTERCEPT = 6
    APPROACH = 7
    SYNCHRONIZE = 8
    DESCEND = 9
    RELEASE = 10
    RETREAT = 11
    VERIFY = 12
    DONE = 13
    ABORT_CYCLE = 20      # L2
    SAFE_ABORT = 21       # L3
    PROTECTIVE_STOP = 22  # L4
    WAIT_OPERATOR = 23


DYNAMIC_STATES = frozenset({State.APPROACH, State.SYNCHRONIZE, State.DESCEND, State.RELEASE})


@dataclass(frozen=True)
class Judgement:
    result: Result
    reason_code: str
    metrics: dict[str, float] = field(default_factory=dict)
    detail: str = ""


def metrics_to_record(j: Judgement) -> tuple[list[str], list[float]]:
    """StepRecord.metric_keys·metric_values로 그대로 복사한다(REQ-ORC-004)."""
    return list(j.metrics.keys()), [float(v) for v in j.metrics.values()]


class Clock(Protocol):
    def ros_now(self) -> float: ...
    def steady_now(self) -> float: ...


class SteadyClock(Protocol):
    def steady_now(self) -> float: ...


def from_dict(cls, d: dict, **extra):
    """config 절(dict)에서 dataclass를 만든다. 필드 이름 = config 키. 없는 키는 extra로 채운다."""
    names = {f.name for f in dataclasses.fields(cls)}
    kw = {k: v for k, v in d.items() if k in names}
    kw.update(extra)
    return cls(**kw)


class FromDict:
    """파라미터 dataclass 공통: `XParams.from_dict(cfg['judge']['x'])`."""

    @classmethod
    def from_dict(cls, d: dict, **extra):
        return from_dict(cls, d, **extra)
