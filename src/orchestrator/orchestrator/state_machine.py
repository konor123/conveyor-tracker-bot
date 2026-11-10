"""상태·전이 표(SPEC-100 4.1절). next_state()는 판정과 중단 등급만으로 다음 상태를 정한다."""
from __future__ import annotations

from .common import AbortLevel, Judgement, Mode, Result, State

S = State

_DYNAMIC_SEQ = (S.PRECHECK, S.DETECT_STONE, S.PICK_STONE, S.MOVE_STANDBY, S.WAIT_CAP,
                S.PLAN_INTERCEPT, S.APPROACH, S.SYNCHRONIZE, S.DESCEND, S.RELEASE, S.RETREAT,
                S.VERIFY, S.DONE)
SEQUENCES: dict[Mode, tuple[State, ...]] = {
    Mode.SYNCHRONIZED: _DYNAMIC_SEQ,
    Mode.PREDICTED: _DYNAMIC_SEQ,
    Mode.STATIC: tuple(s for s in _DYNAMIC_SEQ if s not in (S.PLAN_INTERCEPT, S.SYNCHRONIZE)),
}
ABORT_STATES = {AbortLevel.L2: S.ABORT_CYCLE, AbortLevel.L3: S.SAFE_ABORT,
                AbortLevel.L4: S.PROTECTIVE_STOP}
PASS_SKIP = 'PASS_SKIP'


def next_state(state: State, mode: Mode, j: Judgement | None, abort: AbortLevel) -> State:
    if abort in ABORT_STATES:
        return ABORT_STATES[abort]
    if state in ABORT_STATES.values():
        return S.WAIT_OPERATOR
    if state is S.WAIT_OPERATOR:
        return S.WAIT_OPERATOR
    if abort is AbortLevel.CONTINUE_TO_RETREAT_VERIFY:
        return {S.RETREAT: S.VERIFY, S.VERIFY: S.DONE}.get(state, S.RETREAT)
    if state is S.VERIFY:
        return S.DONE
    if j is None:
        raise ValueError(f'{state.name}: 판정 없이 진행할 수 없다')
    if state is S.PRECHECK and j.result is not Result.OK:
        return S.IDLE
    if state is S.WAIT_CAP and j.reason_code == PASS_SKIP:
        return S.WAIT_CAP
    if state is S.PLAN_INTERCEPT and j.result is not Result.OK:
        return S.WAIT_CAP
    if j.result is Result.OK:
        seq = SEQUENCES[mode]
        return seq[seq.index(state) + 1]
    raise ValueError(f'{state.name}: 판정 {j.reason_code}에 중단 등급이 없다')
