"""safety_supervisor_node 시험."""
import time

import pytest

from orchestrator.safety_supervisor_node import (ERROR_PROTECTIVE_STOP, SafetySupervisorNode,
                                                 SupervisorMonitor)


def test_supervisor_monitor_detects_protective_stop_and_heartbeat_loss(fake_clock):
    mon = SupervisorMonitor(heartbeat_timeout_s=0.3, clock=fake_clock)
    assert mon.check() is None

    # 정상 heartbeat 수신 중에는 이상 없음
    mon.on_heartbeat('robot_controller_node', error_code=0)
    fake_clock.advance(0.299)
    assert mon.check() is None

    # 0.300 s 경과 시 CONTROLLER_HEARTBEAT_LOST 감지 (1회 발화 후 latch)
    fake_clock.advance(0.001)
    assert mon.check() == 'CONTROLLER_HEARTBEAT_LOST'
    assert mon.check() is None

    # heartbeat 복구 후 보호정지 감지
    mon.on_heartbeat('robot_controller_node', error_code=0)
    assert mon.check() is None
    mon.on_robot_status(connected=True, protective_stop=True)
    assert mon.check() == 'PROTECTIVE_STOP'
    assert mon.check() is None

    # 복구 후 통신 단절 감지
    mon.on_robot_status(connected=True, protective_stop=False)
    assert mon.check() is None
    mon.on_robot_status(connected=False, protective_stop=False)
    assert mon.check() == 'ROBOT_COMM_LOST'


def test_supervisor_monitor_detects_heartbeat_error_code(fake_clock):
    mon = SupervisorMonitor(heartbeat_timeout_s=0.3, clock=fake_clock)
    mon.on_heartbeat('robot_controller_node', error_code=ERROR_PROTECTIVE_STOP)
    assert mon.check() == 'PROTECTIVE_STOP'


def test_safety_supervisor_node_calls_stop_only():
    rclpy = pytest.importorskip('rclpy')
    from drop_interfaces.msg import Heartbeat, RobotStatus
    from drop_interfaces.srv import Stop
    from drop_ros import DOMAIN, Harness

    rclpy.init(domain_id=DOMAIN)
    sup = SafetySupervisorNode()
    h = Harness(sup)
    stop_reqs = []

    def on_stop(req, res):
        stop_reqs.append(( time.monotonic(), req.requester, req.reason_code))
        res.result = 0
        res.stopped = True
        return res

    h.client.create_service(Stop, 'robot/stop', on_stop)
    status_pub = h.client.create_publisher(RobotStatus, 'robot/status', 10)
    hb_pub = h.client.create_publisher(Heartbeat, 'system/heartbeat', 10)
    try:
        # 로봇 직접 제어용 액션 클라이언트나 스트림 발행자가 없어야 한다 (단일 지령 권한)
        assert {c.srv_name.lstrip('/') for c in sup.clients} == {'robot/stop'}
        pubs = {p.topic_name.lstrip('/') for p in sup.publishers}
        assert 'robot/stream_target' not in pubs

        # 보호정지 상태 발행 -> 50 ms 이내 Stop(SUPERVISOR) 호출
        t0 = time.monotonic()
        status_pub.publish(RobotStatus(connected=True, ready=False, protective_stop=True))
        h.wait_for(lambda: len(stop_reqs) >= 1, timeout=1.0, what='Stop(SUPERVISOR) 호출')
        t_call, requester, reason = stop_reqs[0]
        assert requester == Stop.Request.SUPERVISOR
        assert reason == 'PROTECTIVE_STOP'
        assert t_call - t0 <= 0.1

        # 정상 복구 후 robot_controller_node heartbeat 손실 -> Stop(SUPERVISOR) 호출
        status_pub.publish(RobotStatus(connected=True, ready=True, protective_stop=False))
        hb_pub.publish(Heartbeat(node_name='robot_controller_node', lifecycle_state=3, seq=1, error_code=0))
        time.sleep(0.05)
        h.wait_for(lambda: len(stop_reqs) >= 2, timeout=1.0, what='heartbeat 손실 Stop 호출')
        assert stop_reqs[-1][1] == Stop.Request.SUPERVISOR
        assert stop_reqs[-1][2] == 'CONTROLLER_HEARTBEAT_LOST'
    finally:
        h.close()
        rclpy.try_shutdown()
