"""phase 끝 feedback을 못 받은 경로 기록(SPEC-100 4.10절 REQ-ORC-020, TEST_SPEC.md TC-ORC-020-01).

판정(포트가 None → `STEP_TIMEOUT`)은 바꾸지 않고, 어느 경로였는지만 경고 한 줄로 남기는지 본다.
"""
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from orchestrator.orchestrator_node import RosSkillPort  # noqa: E402


class _Log:
    def __init__(self):
        self.lines = []

    def warning(self, msg):
        self.lines.append(msg)


class _Clock:
    def __init__(self):
        self.t = 10.0

    def ros_now(self):
        return self.t


def _port():
    log = _Log()
    node = SimpleNamespace(cfg_dict={'watchdog': {'cycle_steady_factor_ratio': 2.0}},
                           clock=_Clock(), get_logger=lambda: log, _watch=lambda: None)
    port = RosSkillPort(node)
    port.reset(goal_handle=SimpleNamespace(is_cancel_requested=False))
    port._feedback = [(9.5, SimpleNamespace(phase='DESCEND', phase_done=True, metric_keys=[], metric_values=[]))]
    return port, log


@pytest.mark.req('REQ-ORC-020')
@pytest.mark.tc('TC-ORC-020-01')
@pytest.mark.level('U')
def test_result_before_release_feedback_is_logged_with_controller_reason():
    port, log = _port()
    result = SimpleNamespace(result=1, reason_code='GATE_TRACK_STALE')
    port._result_future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(result=result))
    assert port.place_feedback('RELEASE', deadline=11.0) is None
    assert len(log.lines) == 1
    line = log.lines[0]
    assert line.startswith('RELEASE 끝 feedback 없음: phase 끝 feedback 전에 결과(result=1 reason=GATE_TRACK_STALE); '
                           '대기 ROS 0.000 s')
    assert line.endswith('마지막 feedback DESCEND 끝 0.500 s 전; 장애 -')


@pytest.mark.req('REQ-ORC-020')
@pytest.mark.tc('TC-ORC-020-01')
@pytest.mark.level('U')
def test_wait_end_without_feedback_is_logged_with_elapsed_times():
    port, log = _port()                    # ROS 시계가 멈춰 있어 steady 상한(남은 0.05 s × 2)에서 끝난다
    assert port.place_feedback('RELEASE', deadline=10.05) is None
    assert len(log.lines) == 1
    line = log.lines[0]
    assert line.startswith('RELEASE 끝 feedback 없음: 대기 끝(제한시간·steady 상한·장애); 대기 ROS 0.000 s·steady 0.1')
    assert line.endswith('마지막 feedback DESCEND 끝 0.500 s 전; 장애 -')
