"""robot_controller_node heartbeat·상태 발행 시계 시험(SPEC-110 REQ-ABT-009, TEST_SPEC.md TC-ABT-009-02)."""
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from drop_interfaces.msg import Heartbeat, RobotStatus  # noqa: E402
from drop_ros import DOMAIN, Harness, own_qos_event_handlers  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from robot_control.robot_controller_node import RobotControllerNode  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.try_shutdown()


@pytest.mark.req('REQ-ABT-009')
@pytest.mark.tc('TC-ABT-009-02')
@pytest.mark.level('C')
def test_heartbeat_and_status_keep_publishing_when_ros_time_frozen(ros):
    """use_sim_time인데 /clock이 없으면 ROS 시각이 멈춘다. heartbeat·robot/status는 steady 10 Hz를 유지한다."""
    node = RobotControllerNode(parameter_overrides=[Parameter('use_sim_time', value=True)])
    h = Harness(node)
    try:
        hbs = h.record(Heartbeat, 'system/heartbeat')
        statuses = h.record(RobotStatus, 'robot/status')
        node.trigger_configure()
        node.trigger_activate()
        h.wait_for(lambda: hbs and statuses, timeout=10.0, what='첫 heartbeat·robot/status 수신')  # discovery
        n0, s0 = len(hbs), len(statuses)
        time.sleep(1.0)
        assert node.get_clock().now().nanoseconds == 0
        assert sum(1 for _, m in hbs[n0:] if m.node_name == 'robot_controller_node') >= 7
        assert len(statuses) - s0 >= 7
    finally:
        h.close()


@pytest.mark.req('REQ-IF-006')
@pytest.mark.tc('TC-IF-006-01')
@pytest.mark.level('C')
def test_robot_controller_has_no_qos_event_handlers(ros):
    node = RobotControllerNode()
    try:
        node.trigger_configure()
        node.trigger_activate()
        assert node.status_pub is not None and node.hb_pub is not None
        assert own_qos_event_handlers(node) == []
    finally:
        node.destroy_node()
