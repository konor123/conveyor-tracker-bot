"""robot_controller_node: robot/status 그리퍼 상태, 재활성 뒤 지령, Stop 진단 기록.

backend:=fake(시뮬레이션)로 띄운다. 실제 로봇·그리퍼에
연결하지 않는다.
"""
import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from drop_interfaces.action import MoveToPose, PickStone, SafeRetreat  # noqa: E402
from drop_interfaces.msg import RobotStatus  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_ros import DOMAIN, Harness, wait_future  # noqa: E402
from geometry_msgs.msg import Point, Pose, Quaternion  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from robot_control.robot_controller_node import RobotControllerNode  # noqa: E402

SAFE_Z = 0.20
PARAMS = {'backend': 'fake', 'fake_move_time_s': 0.1, 'pose_tolerance_m': 0.002, 'grip_min_width_m': 0.010,
          'grip_max_width_m': 0.020, 'grip_min_current_ma': 10.0, 'safe_z_m': SAFE_Z,
          'open_width_tolerance_m': 0.001, 'dynamic.open_width_m': 0.022, 'fake_tcp_z_m': SAFE_Z,
          'place_back_xyz_m': [0.35, 0.15, 0.01]}


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.try_shutdown()


def _node(**extra):
    params = {**PARAMS, **extra}
    node = RobotControllerNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    node.trigger_configure()
    node.trigger_activate()
    return node


def _run(h, typ, name, goal, timeout=10.0):
    gh = wait_future(h.action(typ, name).send_goal_async(goal), 5.0)
    assert gh.accepted
    return wait_future(gh.get_result_async(), timeout).result


def _status_after(h, box, n, pred, what):
    h.wait_for(lambda: len(box) > n and pred(box[-1][1]), timeout=0.5, what=what)
    return box[-1][1]


def test_status_reports_measured_gripper_state(ros):
    """robot/status 그리퍼 필드는 마지막 skill의 측정값: 시작 개방, PickStone(150 mA) 파지 뒤 holding·폭·전류,
    SafeRetreat 반납(개방 폭 확인) 뒤 개방. 2026-11-13 전에는 holding=false·gripper_open=false 고정이라 실제 controller로
    PRECHECK `GRIPPER_NOT_OPEN`·MOVE_STANDBY `DROPPED`였다."""
    h = Harness(_node())
    try:
        box = h.record(RobotStatus, 'robot/status')
        first = _status_after(h, box, 0, lambda m: True, '첫 status')
        assert (first.holding, first.gripper_open, first.gripper_alive) == (False, True, True)
        n = len(box)
        res = _run(h, PickStone, 'robot/pick_stone',
                   PickStone.Goal(target_position=Point(x=0.35, y=0.15, z=0.01), approach_height=0.05,
                                  grip_current_limit=150))
        assert (res.result, res.holding) == (0, True), res
        st = _status_after(h, box, n, lambda m: m.holding, '파지 status')
        assert (st.gripper_open, st.gripper_width, st.gripper_current) == (False, pytest.approx(0.0175),
                                                                          pytest.approx(40.0))
        n = len(box)
        res = _run(h, SafeRetreat, 'robot/safe_retreat', SafeRetreat.Goal(place_back=True))
        assert (res.result, res.stone_placed_back) == (0, True), res
        st = _status_after(h, box, n, lambda m: not m.holding, '반납 status')
        assert (st.gripper_open, st.gripper_width, st.gripper_current) == (True, pytest.approx(0.022), 0.0)
    finally:
        h.close()


def test_gripper_state_kept_when_open_not_confirmed():
    """DynamicPlace가 commit했어도(`released`) 개방을 확인하지 못했으면(`OPEN_NOT_CONFIRMED`·`OPEN_WIDTH`·폭 없음)
    쥔 상태를 바꾸지 않는다. 확인한 개방이면 측정 폭으로 개방 상태다."""
    import math
    from types import SimpleNamespace
    from robot_control.skill_logic import Result, SkillResult
    held = {'holding': True, 'width_m': 0.0175, 'current_ma': 150.0}
    for reason, width in (('OPEN_NOT_CONFIRMED', 0.022), ('OPEN_WIDTH', 0.03), ('', math.nan)):
        ns = SimpleNamespace(_gripper=dict(held))
        res = SkillResult(Result.FAILED if reason else Result.OK, reason, {'open_width_m': width}, released=True)
        assert RobotControllerNode._update_gripper(ns, 'DynamicPlace', res) is False
        assert ns._gripper == held, reason
    ns = SimpleNamespace(_gripper=dict(held))
    res = SkillResult(Result.OK, '', {'open_width_m': 0.022}, released=True)
    assert RobotControllerNode._update_gripper(ns, 'DynamicPlace', res) is True
    assert ns._gripper == {'holding': False, 'width_m': 0.022, 'current_ma': 0.0}


def test_reactivation_without_stop_accepts_motion(ros):
    """정지 유지가 없을 때 비활성 → 재활성 뒤 ready=true이면 이동 지령이 실제로 실행된다(비활성화가 멈춘 시뮬레이션
    backend를 재활성이 푼다). 정지 유지 중 재활성은 그대로 유지한다."""
    node = _node()
    h = Harness(node)
    try:
        box = h.record(RobotStatus, 'robot/status')
        assert node.trigger_deactivate() is not None and node.trigger_activate() is not None
        n = len(box)
        st = _status_after(h, box, n, lambda m: True, '재활성 뒤 status')
        assert st.ready
        target = Pose(position=Point(x=0.45, y=-0.05, z=SAFE_Z), orientation=Quaternion(w=1.0))
        res = _run(h, MoveToPose, 'robot/move_to_pose', MoveToPose.Goal(target_pose=target))
        assert (res.result, res.reason_code) == (0, ''), res
        pos, _ = node.backend.read_tcp_pose()
        assert list(pos) == pytest.approx([0.45, -0.05, SAFE_Z])
    finally:
        h.close()


def test_stop_history_diagnostic_parameter(ros):
    """받은 Stop은 요청자·사유·수신 steady 시각으로 `stop_history_json`에 쌓인다(작업자 해제 전 원인 확인, 판정에 쓰지 않음)."""
    node = _node()
    h = Harness(node)
    try:
        t0 = time.monotonic()
        assert h.call(Stop, 'robot/stop', requester=Stop.Request.SUPERVISOR, reason_code='HB_TEST').stopped
        assert h.call(Stop, 'robot/stop', requester=Stop.Request.ORCHESTRATOR, reason_code='L4_TEST').stopped
        hist = json.loads(node.get_parameter('stop_history_json').value)
        assert [(e['requester'], e['reason_code']) for e in hist] == [
            (Stop.Request.SUPERVISOR, 'HB_TEST'), (Stop.Request.ORCHESTRATOR, 'L4_TEST')]
        assert all(t0 <= e['received_steady_s'] <= time.monotonic() for e in hist)
        # 확인할 수 없는 정지를 감시 노드가 계속 재시도해도 기록은 최근 STOP_HISTORY_MAX건으로 유한하다
        from robot_control.robot_controller_node import STOP_HISTORY_MAX
        for _ in range(STOP_HISTORY_MAX):
            h.call(Stop, 'robot/stop', requester=Stop.Request.SUPERVISOR, reason_code='RETRY')
        hist = json.loads(node.get_parameter('stop_history_json').value)
        assert len(hist) == STOP_HISTORY_MAX and {e['reason_code'] for e in hist} == {'RETRY'}
    finally:
        h.close()
