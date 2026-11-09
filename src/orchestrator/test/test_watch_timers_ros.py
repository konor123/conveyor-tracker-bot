"""감시 타이머 시계 시험(SPEC-110 REQ-ABT-009, TEST_SPEC.md TC-ABT-009-02).

use_sim_time인데 /clock이 없으면 ROS 시각이 0에서 멈춘다. 감시 타이머가 steady 시계면 이 상태에서도 돌고,
ROS 시각 타이머면 한 번도 돌지 않는다. 시스템 시계가 NTP 보정으로 뒤로 점프할 때와 같은 결함을 잡는다.
"""
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from drop_interfaces.msg import Heartbeat  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_ros import DOMAIN, Harness, own_qos_event_handlers  # noqa: E402
from rclpy.clock import ClockType  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from orchestrator.orchestrator_node import OrchestratorNode  # noqa: E402
from orchestrator.safety_supervisor_node import SafetySupervisorNode  # noqa: E402

SIM = [Parameter('use_sim_time', value=True)]


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.try_shutdown()


def _timer_clock_type(node, callback):
    timers = [t for t in node.timers if getattr(t, 'callback', None) == callback]
    assert len(timers) == 1, f'{callback.__name__} 타이머 {len(timers)}개'
    return timers[0].clock.clock_type


@pytest.mark.req('REQ-ABT-009')
@pytest.mark.tc('TC-ABT-009-02')
@pytest.mark.level('C')
def test_orchestrator_watch_timer_uses_steady_clock(ros):
    node = OrchestratorNode(parameter_overrides=SIM)
    try:
        assert _timer_clock_type(node, node._watch) == ClockType.STEADY_TIME
    finally:
        node.destroy_node()


@pytest.mark.req('REQ-ABT-009')
@pytest.mark.tc('TC-ABT-009-02')
@pytest.mark.level('C')
def test_supervisor_detects_heartbeat_loss_when_ros_time_frozen(ros):
    sup = SafetySupervisorNode(parameter_overrides=SIM)
    h = Harness(sup)
    stops = []

    def on_stop(req, res):
        stops.append((time.monotonic(), req.reason_code))
        res.result = 0
        res.stopped = True
        return res

    h.client.create_service(Stop, 'robot/stop', on_stop)
    hb_pub = h.client.create_publisher(Heartbeat, 'system/heartbeat', 10)
    try:
        assert _timer_clock_type(sup, sup._tick) == ClockType.STEADY_TIME
        # discovery(수 s 걸릴 수 있다)를 끝낸 뒤에 측정한다
        h.wait_for(lambda: hb_pub.get_subscription_count() > 0, timeout=10.0, what='supervisor heartbeat 구독 매칭')
        for seq in range(1, 4):
            hb_pub.publish(Heartbeat(node_name='robot_controller_node', lifecycle_state=3, seq=seq, error_code=0))
            t_last = time.monotonic()
            time.sleep(0.1)
        assert not stops                                       # 10 Hz로 받는 동안에는 손실이 아니다
        h.wait_for(lambda: stops, timeout=2.0, what='ROS 시각 정지 중 heartbeat 손실 Stop 호출')
        assert sup.get_clock().now().nanoseconds == 0
        assert stops[0][1] == 'CONTROLLER_HEARTBEAT_LOST'
        assert 0.3 - 0.05 <= stops[0][0] - t_last <= 0.3 + 0.2
    finally:
        h.close()


# ------------------------------------------------------------------ TC-IF-006-01
@pytest.mark.req('REQ-IF-006')
@pytest.mark.tc('TC-IF-006-01')
@pytest.mark.level('C')
def test_orchestrator_nodes_have_no_qos_event_handlers(ros):
    from orchestrator.record_logger_node import RecordLoggerNode
    nodes = [OrchestratorNode(), SafetySupervisorNode(), RecordLoggerNode()]
    try:
        for n in nodes:
            assert list(n.subscriptions)
            assert own_qos_event_handlers(n) == [], n.get_name()
    finally:
        for n in nodes:
            n.destroy_node()
