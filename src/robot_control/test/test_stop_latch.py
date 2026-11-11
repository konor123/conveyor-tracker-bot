"""Stop 정지 유지와 작업자 해제 판정(SPEC-010 4.5절, SPEC-500 REQ-ROB-014, TEST_SPEC TC-ROB-014-01)."""
import threading

import pytest
from robot_control.stop_latch import ERROR_NONE, ERROR_PROTECTIVE_STOP, ERROR_STOP_LATCHED, StopLatch

pytestmark = [pytest.mark.level('U'), pytest.mark.req('REQ-ROB-014'), pytest.mark.tc('TC-ROB-014-01')]


def test_codes_match_interface_table():
    assert (ERROR_NONE, ERROR_PROTECTIVE_STOP, ERROR_STOP_LATCHED) == (0, 100, 101)


def test_fresh_latch_is_clear():
    s = StopLatch()
    assert not s.latched and s.reason == ''
    assert s.ready(active=True, protective_stop=False)
    assert not s.ready(active=False, protective_stop=False)
    assert s.heartbeat_code(protective_stop=False) == ERROR_NONE


def test_stop_latches_with_reason_and_blocks_ready():
    s = StopLatch()
    s.latch('SUPERVISOR:PROTECTIVE_STOP')
    assert s.latched and s.reason == 'SUPERVISOR:PROTECTIVE_STOP'
    assert not s.ready(active=True, protective_stop=False)
    assert s.heartbeat_code(protective_stop=False) == ERROR_STOP_LATCHED


def test_protective_stop_code_takes_precedence():
    s = StopLatch()
    assert s.heartbeat_code(protective_stop=True) == ERROR_PROTECTIVE_STOP
    s.latch('X')
    assert s.heartbeat_code(protective_stop=True) == ERROR_PROTECTIVE_STOP


def test_second_stop_keeps_first_reason():
    s = StopLatch()
    s.latch('FIRST')
    s.latch('SECOND')
    assert s.reason == 'FIRST' and s.count == 2


@pytest.mark.parametrize('reason,goal_active,pstop,code', [
    ('', False, False, 'REASON_REQUIRED'),
    ('   ', False, False, 'REASON_REQUIRED'),
    ('checked', True, False, 'GOAL_ACTIVE'),
    ('checked', False, True, 'PROTECTIVE_STOP'),
])
def test_release_refusals_keep_latch(reason, goal_active, pstop, code):
    s = StopLatch()
    s.latch('X')
    assert s.release(reason, goal_active=goal_active, protective_stop=pstop) == code
    assert s.latched and s.heartbeat_code(protective_stop=False) == ERROR_STOP_LATCHED


def test_release_with_reason_clears_and_records():
    s = StopLatch()
    s.latch('X')
    assert s.release(' operator checked ', goal_active=False, protective_stop=False) == ''
    assert not s.latched and s.reason == ''
    assert s.ready(active=True, protective_stop=False)
    assert s.heartbeat_code(protective_stop=False) == ERROR_NONE
    assert s.releases == ['operator checked']


def test_release_when_not_latched_is_a_noop():
    s = StopLatch()
    assert s.release('', goal_active=False, protective_stop=False) == ''
    assert s.release('noop', goal_active=True, protective_stop=True) == ''
    assert not s.latched and s.releases == []


def test_latch_is_thread_safe():
    s = StopLatch()
    threads = [threading.Thread(target=s.latch, args=(f'R{i}',)) for i in range(50)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert s.latched and s.count == 50
