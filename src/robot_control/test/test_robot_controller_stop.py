"""robot_controller_node: 실행 중 Stop과 cancel의 결과 구분(SPEC-010 3.2절, REQ-ROB-007, TEST_SPEC.md TC-ROB-007-02).

backend:=fake(시뮬레이션)로 띄운다. 실제 로봇·그리퍼에 연결하지 않는다.
"""
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from action_msgs.msg import GoalStatus  # noqa: E402
from drop_interfaces.action import PickStone  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_ros import DOMAIN, Harness, wait_future  # noqa: E402
from geometry_msgs.msg import Point  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from robot_control.robot_controller_node import RobotControllerNode  # noqa: E402

CANCELED = 4


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.try_shutdown()


def _node():
    params = {'backend': 'fake', 'fake_move_time_s': 1.0, 'pose_tolerance_m': 0.002,
              'grip_min_width_m': 0.010, 'grip_max_width_m': 0.020, 'grip_min_current_ma': 10.0}
    node = RobotControllerNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    node.trigger_configure()
    node.trigger_activate()
    return node


def _pick(h):
    ac = h.action(PickStone, 'robot/pick_stone')
    goal = PickStone.Goal(cycle_id='t', step_id=1, target_position=Point(x=0.3, y=0.0, z=0.02),
                          approach_height=0.05, grip_current_limit=50)
    gh = wait_future(ac.send_goal_async(goal), 5.0)
    assert gh.accepted
    time.sleep(0.2)                     # APPROACH 이동 중
    return gh


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-02')
@pytest.mark.level('C')
def test_stop_preempts_goal_with_stopped_reason(ros):
    h = Harness(_node())
    try:
        gh = _pick(h)
        r = h.call(Stop, 'robot/stop', requester=Stop.Request.SUPERVISOR, reason_code='TEST')
        assert r.stopped
        res = wait_future(gh.get_result_async(), 5.0)
        assert (res.result.result, res.result.reason_code) == (CANCELED, 'STOPPED')
        assert res.status == GoalStatus.STATUS_ABORTED       # cancel 요청이 없는 goal
    finally:
        h.close()


@pytest.mark.req('REQ-ROB-007')
@pytest.mark.tc('TC-ROB-007-02')
@pytest.mark.level('C')
def test_client_cancel_keeps_canceled_reason(ros):
    h = Harness(_node())
    try:
        gh = _pick(h)
        wait_future(gh.cancel_goal_async(), 2.0)
        res = wait_future(gh.get_result_async(), 5.0)
        assert (res.result.result, res.result.reason_code) == (CANCELED, 'CANCELED')
        assert res.status == GoalStatus.STATUS_CANCELED
    finally:
        h.close()
