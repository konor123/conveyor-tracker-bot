"""중단 등급과상태 전이의 조합 계약."""
from itertools import product

import pytest

from orchestrator.abort_policy import AbortCtx, Event, decide
from orchestrator.common import AbortLevel as L, Judgement, Mode, Result, State as S
from orchestrator.state_machine import next_state

# 명세의 하드 이상 목록을 독립적으로 둔다(구현의 집합을 기대값으로 쓰지 않는다).
HARD = {Event.PROTECTIVE_STOP, Event.ESTOP, Event.ROBOT_COMM_LOST,
        Event.CONTROLLER_HEARTBEAT_LOST, Event.ABORT_PATH_TIMEOUT, Event.RETREAT_FAILED,
        Event.ROBOT_STOPPED}
DYNAMIC = {S.APPROACH, S.SYNCHRONIZE, S.DESCEND, S.RELEASE}


@pytest.mark.parametrize('state,event', list(product(S, Event)))
def test_all_state_event_transitions(state, event):
    for holding, released in product((False, True), repeat=2):
        c = AbortCtx(state, holding, released)
        if event in HARD:
            expected = L.L4
        elif released:
            expected = L.CONTINUE_TO_RETREAT_VERIFY
        elif holding or state in DYNAMIC or event is Event.PASSES_EXHAUSTED:
            expected = L.L3
        else:
            expected = L.L2
        level = decide(event, c)
        assert level is expected
        if level in (L.L2, L.L3, L.L4):
            target = {L.L2: S.ABORT_CYCLE, L.L3: S.SAFE_ABORT,
                      L.L4: S.PROTECTIVE_STOP}[level]
        elif state in (S.ABORT_CYCLE, S.SAFE_ABORT, S.PROTECTIVE_STOP, S.WAIT_OPERATOR):
            target = S.WAIT_OPERATOR
        elif level is L.CONTINUE_TO_RETREAT_VERIFY:
            target = {S.RETREAT: S.VERIFY, S.VERIFY: S.DONE}.get(state, S.RETREAT)
        elif state is S.VERIFY:
            target = S.DONE
        elif state is S.PRECHECK:
            target = S.IDLE
        else:
            target = state
        for mode in Mode:
            assert next_state(state, mode, Judgement(Result.FAULT, event.name),
                              level) is target
