"""robot_controller_node SafeRetreat place-back 연결.

backend:=fake(시뮬레이션 backend와 해제 transport)로 띄운다. 실제 로봇·그리퍼에 연결하지 않는다.
"""
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from drop_interfaces.action import SafeRetreat  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_ros import DOMAIN, Harness, wait_future  # noqa: E402
from geometry_msgs.msg import Point, Pose  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from robot_control.robot_controller_node import RobotControllerNode  # noqa: E402

OK, PRECONDITION, CANCELED = 0, 3, 4
SAFE_Z = 0.12
MAT = [0.40, -0.15, 0.06]


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.try_shutdown()


def _node(**extra):
    params = {'backend': 'fake', 'fake_move_time_s': 0.2, 'pose_tolerance_m': 0.002, 'safe_z_m': SAFE_Z,
              'open_width_tolerance_m': 0.001, 'dynamic.open_width_m': 0.022, 'place_back_xyz_m': MAT,
              'fake_tcp_x_m': 0.5, 'fake_tcp_y_m': 0.0, 'fake_tcp_z_m': 0.08,
              'safe_pose': [0.3, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0]}
    params.update(extra)
    node = RobotControllerNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    node.trigger_configure()
    node.trigger_activate()
    return node


def _run(h, goal, timeout=10.0):
    ac = h.action(SafeRetreat, 'robot/safe_retreat')
    phases = []
    gh = wait_future(ac.send_goal_async(goal, feedback_callback=lambda fb: phases.append(fb.feedback.phase)), 5.0)
    assert gh.accepted
    return gh, phases


def test_place_back_parameters_accept_float_arrays(ros):
    node = _node()
    try:
        assert list(node.cfg.place_back_xyz_m) == pytest.approx(MAT)              # 음수 y 그대로
        assert node.cfg.named_poses['safe'][0] == pytest.approx((0.3, 0.0, 0.3))
        assert node.cfg.open_width_m == pytest.approx(0.022)
    finally:
        node.destroy_node()
    node = _node(place_back_xyz_m=[])
    try:
        assert node.cfg.place_back_xyz_m is None
    finally:
        node.destroy_node()


def test_static_place_back_on_fake_backend(ros):
    node = _node()
    h = Harness(node)
    try:
        gh, phases = _run(h, SafeRetreat.Goal(cycle_id='t', step_id=1, keep_sync=False, place_back=True))
        res = wait_future(gh.get_result_async(), 10.0)
        assert (res.result.result, res.result.reason_code) == (OK, ''), res.result
        assert res.result.stone_placed_back is True
        assert [p for i, p in enumerate(phases) if i == 0 or phases[i - 1] != p] == ['RETREAT', 'PLACE_BACK']
        pos, _ = node.backend.read_tcp_pose()
        assert list(pos) == pytest.approx([MAT[0], MAT[1], SAFE_Z], abs=1e-6)
        # place-back 없는 기본 goal은 그대로: 반납 보고 없음
        gh, _ = _run(h, SafeRetreat.Goal(cycle_id='t', step_id=2))
        res = wait_future(gh.get_result_async(), 10.0)
        assert res.result.result == OK and res.result.stone_placed_back is False
    finally:
        h.close()


def test_keep_sync_place_back_routes_through_keep_sync_retreat(ros):
    node = _node()
    h = Harness(node)
    try:
        # 계획 속도 없음(DynamicPlace 전): 움직이지 않고 SYNC_NOT_AVAILABLE
        before = node.backend.read_tcp_pose()[0].copy()
        gh, _ = _run(h, SafeRetreat.Goal(cycle_id='t', step_id=1, keep_sync=True, place_back=True))
        res = wait_future(gh.get_result_async(), 10.0)
        assert (res.result.result, res.result.reason_code) == (PRECONDITION, 'SYNC_NOT_AVAILABLE')
        assert list(node.backend.read_tcp_pose()[0]) == pytest.approx(list(before))
        # 직전 DynamicPlace 계획 속도가 있다(노드가 DynamicPlace 끝에 남기는 값을 직접 넣는다)
        updates = node.place_transport.updates
        node._sync_velocity = ((0.0, 0.0), time.monotonic())
        gh, phases = _run(h, SafeRetreat.Goal(cycle_id='t', step_id=2, keep_sync=True, place_back=True))
        res = wait_future(gh.get_result_async(), 10.0)
        assert (res.result.result, res.result.reason_code) == (OK, ''), res.result
        assert res.result.stone_placed_back is True
        assert node.place_transport.updates > updates                   # 동기 유지 후퇴가 transport로 올렸다
        assert [p for i, p in enumerate(phases) if i == 0 or phases[i - 1] != p] == ['RETREAT', 'PLACE_BACK']
    finally:
        h.close()


def test_place_back_unset_and_stop_on_real_node(ros):
    node = _node(place_back_xyz_m=[])
    h = Harness(node)
    try:
        gh, _ = _run(h, SafeRetreat.Goal(cycle_id='t', step_id=1, place_back=True))
        res = wait_future(gh.get_result_async(), 10.0)
        assert (res.result.result, res.result.reason_code) == (PRECONDITION, 'PLACE_BACK_POSE_UNSET')
        # goal이 반납 자세를 주면 그 값을 쓴다. 실행 중 Stop은 CANCELED·STOPPED
        goal = SafeRetreat.Goal(cycle_id='t', step_id=2, place_back=True,
                                place_back_pose=Pose(position=Point(x=0.35, y=0.1, z=0.05)))
        gh, _ = _run(h, goal)
        time.sleep(0.1)
        assert h.call(Stop, 'robot/stop', requester=Stop.Request.ORCHESTRATOR, reason_code='T').stopped
        res = wait_future(gh.get_result_async(), 10.0)
        assert (res.result.result, res.result.reason_code) == (CANCELED, 'STOPPED')
        assert res.result.stone_placed_back is False
    finally:
        h.close()
