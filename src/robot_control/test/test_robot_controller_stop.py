"""robot_controller_node: 실행 중 Stop과 cancel의 결과 구분.

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


def test_stop_latches_until_operator_release(ros):
    """Stop 뒤 정지 유지: goal은 이동 지령 없이 STOPPED, lifecycle 재활성에도 유지, 사유 있는 해제로만 풀린다."""
    from drop_interfaces.msg import Heartbeat, RobotStatus

    node = _node()
    h = Harness(node)
    moves = []
    move_line = node.backend.move_line

    def counting_move(*args):
        moves.append(args)
        return move_line(*args)
    node.backend.move_line = counting_move
    try:
        hbs = h.record(Heartbeat, 'system/heartbeat')
        sts = h.record(RobotStatus, 'robot/status')
        h.wait_for(lambda: hbs and sts, timeout=10.0, what='첫 heartbeat·status')
        r = h.call(Stop, 'robot/stop', requester=Stop.Request.SUPERVISOR, reason_code='IDLE_TEST')
        assert r.stopped

        def latched_since(n_hb, n_st):
            return (len(hbs) > n_hb and len(sts) > n_st and hbs[-1][1].error_code == 101
                    and not sts[-1][1].ready)
        n_hb, n_st = len(hbs), len(sts)
        h.wait_for(lambda: latched_since(n_hb, n_st), timeout=2.0, what='정지 유지 표시')

        ac = h.action(PickStone, 'robot/pick_stone')
        goal = PickStone.Goal(cycle_id='t', step_id=2, target_position=Point(x=0.3, y=0.0, z=0.02),
                              approach_height=0.05, grip_current_limit=50)
        gh = wait_future(ac.send_goal_async(goal), 5.0)
        assert gh.accepted
        res = wait_future(gh.get_result_async(), 5.0)
        assert (res.result.result, res.result.reason_code) == (CANCELED, 'STOPPED')
        assert moves == []

        assert node.trigger_deactivate() is not None and node.trigger_activate() is not None
        n_hb, n_st = len(hbs), len(sts)
        h.wait_for(lambda: latched_since(n_hb, n_st), timeout=2.0, what='재활성 뒤에도 정지 유지')

        bad = node.set_parameters([Parameter('stop_release_reason', value='')])[0]
        assert not bad.successful and 'REASON_REQUIRED' in bad.reason
        ok = node.set_parameters([Parameter('stop_release_reason', value='operator checked')])[0]
        assert ok.successful, ok.reason
        n_hb, n_st = len(hbs), len(sts)
        h.wait_for(lambda: len(hbs) > n_hb and len(sts) > n_st and hbs[-1][1].error_code == 0
                   and sts[-1][1].ready, timeout=2.0, what='해제 뒤 정상')

        gh = wait_future(ac.send_goal_async(goal), 5.0)
        assert gh.accepted
        res = wait_future(gh.get_result_async(), 10.0)
        assert res.result.reason_code != 'STOPPED'
        assert moves
    finally:
        h.close()


def test_release_refused_while_goal_active(ros):
    node = _node()
    h = Harness(node)
    try:
        r = h.call(Stop, 'robot/stop', requester=Stop.Request.ORCHESTRATOR, reason_code='T')
        assert r.stopped
        node._active_goal = True             # 실행 중 goal 모사(해제 조건만 본다)
        res = node.set_parameters([Parameter('stop_release_reason', value='operator checked')])[0]
        assert not res.successful and 'GOAL_ACTIVE' in res.reason
        node._active_goal = False
        assert node.set_parameters([Parameter('stop_release_reason', value='operator checked')])[0].successful
    finally:
        h.close()


def test_accept_reserves_goal_and_noop_release_keeps_backend_stop(ros):
    """수락한 goal은 실행 전에도 해제를 막고(GOAL_ACTIVE), 정지 유지가 없을 때의 해제는 backend를 건드리지 않는다."""
    from rclpy.action import GoalResponse
    node = _node()
    h = Harness(node)
    try:
        assert h.call(Stop, 'robot/stop', requester=Stop.Request.ORCHESTRATOR, reason_code='T').stopped
        assert node._on_goal(None) == GoalResponse.ACCEPT        # 실행 callback 전
        res = node.set_parameters([Parameter('stop_release_reason', value='operator checked')])[0]
        assert not res.successful and 'GOAL_ACTIVE' in res.reason
        node._active_goal = False                                  # 실행이 끝난 상태
        assert node.set_parameters([Parameter('stop_release_reason', value='operator checked')])[0].successful
        node.backend.stop()                                        # 정지 유지 없이 backend만 정지
        assert node.set_parameters([Parameter('stop_release_reason', value='')])[0].successful
        assert node.backend.is_stopped()
    finally:
        h.close()
