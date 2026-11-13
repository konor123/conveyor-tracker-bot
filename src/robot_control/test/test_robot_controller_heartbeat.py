"""robot_controller_node heartbeat·상태 발행 시험(SPEC-500 REQ-ROB-012, SPEC-110 REQ-ABT-009, TEST_SPEC.md TC-ROB-012-01·TC-ABT-009-02)."""
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


def _rate_hz(times):
    return (len(times) - 1) / (times[-1] - times[0])


@pytest.mark.req('REQ-ROB-012')
@pytest.mark.tc('TC-ROB-012-01')
@pytest.mark.level('C')
@pytest.mark.realtime_pc                # 10 s 동안 10 Hz ±10%: CI는 기록만, 판정은 PC(TEST_SPEC.md 0.3절)
def test_heartbeat_and_status_10hz_for_10s(ros):
    """backend=fake로 10 s 관찰: heartbeat 10 Hz ±10%·seq 연속·lifecycle ACTIVE, robot/status 10 Hz ±10%·연결·준비·TCP·파지.

    주기는 시험 프로세스의 steady 수신 시각으로 잰다(시스템 시계 보정과 무관). 10 s 평균이라 수신 지터는 양 끝만 영향을 준다.
    """
    tcp = (0.31, -0.02, 0.27)
    node = RobotControllerNode(parameter_overrides=[
        Parameter('backend', value='fake'),
        *[Parameter(f'fake_tcp_{a}_m', value=v) for a, v in zip('xyz', tcp)]])
    h = Harness(node)
    try:
        hbs = h.record(Heartbeat, 'system/heartbeat', steady=True)
        statuses = h.record(RobotStatus, 'robot/status', steady=True)
        assert node.trigger_configure().name == 'SUCCESS'
        assert node.trigger_activate().name == 'SUCCESS'
        h.wait_for(lambda: hbs and statuses, timeout=10.0, what='첫 heartbeat·robot/status 수신')  # discovery
        n0, s0 = len(hbs), len(statuses)
        time.sleep(10.0)
        beats = [(t, m) for t, m in hbs[n0:] if m.node_name == 'robot_controller_node']
        status = statuses[s0:]
        assert 90 <= len(beats) <= 110, len(beats)
        assert 9.0 <= _rate_hz([t for t, _ in beats]) <= 11.0
        seqs = [m.seq for _, m in beats]
        assert all(b - a == 1 for a, b in zip(seqs, seqs[1:])), seqs   # 수신 누락·중복 없이 1씩 증가
        assert all(m.lifecycle_state == 3 and m.error_code == 0 for _, m in beats)   # ACTIVE
        assert 90 <= len(status) <= 110, len(status)
        assert 9.0 <= _rate_hz([t for t, _ in status]) <= 11.0
        for _, m in status:
            assert m.connected and m.ready and not m.protective_stop and not m.holding and m.gripper_alive
            p = m.tcp_pose.position
            assert (p.x, p.y, p.z) == pytest.approx(tcp, abs=1e-12)
            assert m.header.frame_id == 'robot_base'
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
