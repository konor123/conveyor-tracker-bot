"""RosSkillPort 비동기 장애 계약(SPEC-100 4.7절 REQ-ORC-010, TEST_SPEC.md TC-ORC-010-01)."""
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from orchestrator.abort_policy import Event  # noqa: E402
from orchestrator.orchestrator_node import RosSkillPort  # noqa: E402


class _Log:
    def warning(self, msg):
        pass


class _Clock:
    def __init__(self):
        self.t = 0.0

    def ros_now(self):
        return self.t


def _port():
    node = SimpleNamespace(cfg_dict={'watchdog': {'cycle_steady_factor_ratio': 2.0}},
                           clock=_Clock(), get_logger=lambda: _Log(), _watch=lambda: None)
    port = RosSkillPort(node)
    port.reset(goal_handle=SimpleNamespace(is_cancel_requested=False))
    return port


def _released(port):
    """RELEASE 응답을 받은 상태로 만든다."""
    port._feedback = [(0.0, SimpleNamespace(phase='RELEASE', phase_done=True, metric_keys=[], metric_values=[]))]
    assert port.place_feedback('RELEASE', deadline=1.0) is not None


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-01')
@pytest.mark.level('U')
def test_soft_fault_ends_wait_before_release():
    port = _port()
    port.raise_fault(Event.HEARTBEAT_LOST)
    assert port._wait(lambda: False, deadline=10.0) is False


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-01')
@pytest.mark.level('U')
def test_soft_fault_does_not_end_wait_after_release():
    port = _port()
    _released(port)
    port.raise_fault(Event.HEARTBEAT_LOST)
    calls = []

    def ready():
        calls.append(1)
        return len(calls) >= 3            # 후퇴 결과가 조금 뒤에 온다

    assert port._wait(ready, deadline=10.0) is True
    assert port.pending_fault() is Event.HEARTBEAT_LOST     # latch는 유지되어 실행기가 CONTINUE로 처리한다


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('hard', [Event.CONTROLLER_HEARTBEAT_LOST, Event.PROTECTIVE_STOP, Event.ROBOT_COMM_LOST])
def test_hard_fault_ends_wait_after_release_and_overrides_soft(hard):
    port = _port()
    _released(port)
    port.raise_fault(Event.HEARTBEAT_LOST)
    port.raise_fault(hard)                # latch된 비하드 장애를 하드 이상이 덮는다
    assert port._wait(lambda: False, deadline=10.0) is False
    assert port.pending_fault() is hard


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-01')
@pytest.mark.level('U')
def test_soft_fault_does_not_override_hard():
    port = _port()
    port.raise_fault(Event.PROTECTIVE_STOP)
    port.raise_fault(Event.HEARTBEAT_LOST)
    assert port.pending_fault() is Event.PROTECTIVE_STOP


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-01')
@pytest.mark.level('U')
def test_reset_clears_release():
    port = _port()
    _released(port)
    port.reset(goal_handle=SimpleNamespace(is_cancel_requested=False))
    port.raise_fault(Event.HEARTBEAT_LOST)
    assert port._wait(lambda: False, deadline=10.0) is False


class _GatedLock:
    """진짜 Lock을 감싼 계측 lock. 지정한 스레드가 임계구역에 들어가면 알리고, 풀어 줄 때까지 붙잡는다."""

    def __init__(self, hold_thread: str):
        self._lock = threading.Lock()
        self._hold = hold_thread
        self.entered = threading.Event()
        self.release = threading.Event()

    def __enter__(self):
        self._lock.acquire()
        if threading.current_thread().name == self._hold and not self.entered.is_set():
            self.entered.set()
            assert self.release.wait(2.0)
        return self

    def __exit__(self, *exc):
        self._lock.release()
        return False


def _race(port, first, second, hold='first'):
    """`first`가 lock 안에 머무는 동안 `second`를 시작해 교차를 강제한다. 예외는 주 스레드로 올린다."""
    errors = []

    def run(fn):
        try:
            fn()
        except BaseException as e:  # noqa: BLE001
            errors.append(e)
    gate = _GatedLock(hold)
    port._fault_lock = gate
    t1 = threading.Thread(target=run, args=(first,), name='first')
    t2 = threading.Thread(target=run, args=(second,), name='second')
    t1.start()
    assert gate.entered.wait(2.0), '첫 스레드가 lock으로 보호된 임계구역에 들어가지 않았다'
    t2.start()
    t2.join(0.2)                       # lock이 있으면 두 번째 스레드는 여기서 끝나지 못한다
    gate.release.set()
    t1.join(2.0)
    t2.join(2.0)
    assert not t1.is_alive() and not t2.is_alive()
    if errors:
        raise errors[0]


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('soft_first', [True, False])
def test_concurrent_soft_and_hard_raise_keeps_hard(soft_first):
    port = _port()
    soft = lambda: port.raise_fault(Event.HEARTBEAT_LOST)              # noqa: E731
    hard = lambda: port.raise_fault(Event.CONTROLLER_HEARTBEAT_LOST)   # noqa: E731
    _race(port, *((soft, hard) if soft_first else (hard, soft)))
    assert port.pending_fault() is Event.CONTROLLER_HEARTBEAT_LOST


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-01')
@pytest.mark.level('U')
def test_concurrent_consume_and_hard_raise_loses_nothing():
    port = _port()
    port.raise_fault(Event.HEARTBEAT_LOST)
    got = []
    _race(port, lambda: got.append(port.pending_fault()),
          lambda: port.raise_fault(Event.CONTROLLER_HEARTBEAT_LOST))
    seen = [got[0], port.pending_fault()]
    assert seen.count(Event.CONTROLLER_HEARTBEAT_LOST) == 1         # 소비 결과나 남은 latch 중 한 곳에서만 보인다


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-02')
@pytest.mark.level('U')
def test_ready_response_after_original_cycle_deadline():
    port = _port()
    _released(port)
    port.clock.t = 91.0
    port.raise_fault(Event.CYCLE_TIMEOUT)
    assert port._wait(lambda: True, deadline=90.0)
    assert not port._wait(lambda: False, deadline=90.0)
