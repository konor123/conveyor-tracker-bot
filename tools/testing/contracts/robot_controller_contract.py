"""robot_controller_node 외부 계약(PLAN.md 7.4절 상세 범위, SPEC-010 v0.2).

skill별 수락·feedback·결과 코드, cancel, deadline 자체 중단, `Stop`, 단일 지령 권한, lifecycle 비활성,
`robot/status`(10 Hz, Reliable/Volatile depth 1, frame_id=robot_base), PlaceStone·DynamicPlace phase 끝 feedback 키(TC-IF-005-01).
대상은 모든 skill이 0.2 s보다 오래 걸리고 2 s 안에 OK로 끝나며 cancel을 받아들이는 조건으로 띄운다.

사용: test/contract/test_<대상>_robot_controller_contract.py에서
    from robot_controller_contract import *     # noqa: F401,F403
    @pytest.fixture(scope='module')
    def robot_target(): return Target(...)
"""
import time

import pytest
from action_msgs.msg import GoalStatus
from drop_interfaces.action import DropTask, DynamicPlace, MoveToPose, PickStone, PlaceStone, SafeRetreat
from drop_interfaces.msg import RobotStatus
from drop_interfaces.srv import Stop
from rcl_interfaces.msg import Parameter as RosParam, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import get_action_names_and_types

import common
import drop_ros

__all__ = ['robot_contract', 'TestRobotControllerContract']

SKILLS = {
    'robot/pick_stone': PickStone,
    'robot/move_to_pose': MoveToPose,
    'robot/place_stone': PlaceStone,
    'robot/dynamic_place': DynamicPlace,
    'robot/safe_retreat': SafeRetreat,
}
PLACE_PHASE_KEYS = {
    ('robot/place_stone', None): (
        ('APPROACH', ('plan_valid', 'pose_error_m')),
        ('DESCEND', ('target_error_m', 'rim_clearance_m', 'height_error_m')),
        ('RELEASE', ('open_width_m', 'sync_held')),
        ('RETREAT', ('tcp_z_m', 'outside_workspace', 'stone_in_gripper')),
    ),
    ('robot/dynamic_place', DropTask.Goal.MODE_SYNCHRONIZED): (
        ('APPROACH', ('plan_valid', 'pose_error_m')),
        ('SYNCHRONIZE', ('e_xy_m', 'v_rel_mps')),
        ('DESCEND', ('sync_ok', 'rim_clearance_m', 'height_error_m')),
        ('RELEASE', ('open_width_m', 'sync_held')),
        ('RETREAT', ('tcp_z_m', 'outside_workspace', 'stone_in_gripper')),
    ),
    ('robot/dynamic_place', DropTask.Goal.MODE_PREDICTED): (
        ('APPROACH', ('plan_valid', 'pose_error_m')),
        ('SYNCHRONIZE', ('entry_time_error_s', 'v_rel_mps')),
        ('DESCEND', ('sync_ok', 'rim_clearance_m', 'height_error_m')),
        ('RELEASE', ('open_width_m', 'sync_held')),
        ('RETREAT', ('tcp_z_m', 'outside_workspace', 'stone_in_gripper')),
    ),
}
RESULT_TIMEOUT_S = 5.0


@pytest.fixture(scope='module')
def robot_contract(robot_target):
    proc, h = common.started(robot_target, '/robot/stop')
    h.status = h.record(RobotStatus, 'robot/status', steady=True)
    h.clients = {name: h.action(typ, name) for name, typ in SKILLS.items()}
    h.target = robot_target
    yield h
    common.finished(proc, h)


def _goal(h, name, deadline_s=None):
    g = SKILLS[name].Goal(cycle_id='contract', step_id=1)
    if deadline_s is not None:
        g.deadline = drop_ros.to_time(h.now() + deadline_s)
    # 대상별 입력: 실제 노드가 실행에 필요로 하는 값(파지 전류, DynamicPlace 계획·mode)을 대상 시험 모듈이 채운다.
    # mock은 비워 둔다. hook은 판정 조건을 바꾸지 않는다.
    hook = getattr(h, 'goal_hook', None)
    if hook is not None:
        hook(name, g)
    return g


def _send(h, name, goal, feedback=None):
    gh = drop_ros.wait_future(h.clients[name].send_goal_async(goal, feedback_callback=feedback), 5.0)
    return gh


def _result(gh):
    return drop_ros.wait_future(gh.get_result_async(), RESULT_TIMEOUT_S)


STOP_LATCHED = 101     # SPEC-010 3.4절


def _release(h, reason: str):
    """작업자 해제(SPEC-010 4.5절). 파라미터 설정 결과를 돌려준다."""
    p = RosParam(name='stop_release_reason',
                 value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=reason))
    res = h.call(SetParameters, f'/{h.target.node}/set_parameters', parameters=[p])
    return res.results[0]


def _ctrl_hb(h, since: float):
    return [m for t, m in h.heartbeats if t >= since and m.node_name == 'robot_controller_node']


class TestRobotControllerContract:
    def test_robot_heartbeat(self, robot_contract):
        common.check_heartbeat(robot_contract, 'robot_controller_node')

    def test_robot_names_and_types(self, robot_contract):
        h = robot_contract
        common.check_services(h, {'/robot/stop': 'drop_interfaces/srv/Stop'})
        common.check_data_qos(h, '/robot/status', depth=1)
        actions = dict(get_action_names_and_types(h.client))
        for name, typ in SKILLS.items():
            assert actions.get('/' + name) == [f'drop_interfaces/action/{typ.__name__}'], name

    @pytest.mark.realtime_pc            # 1.5 s 동안 10 Hz·간격 0.3 s: CI는 기록만, 판정은 PC(TEST_SPEC.md 0.3절)
    def test_robot_status_stream(self, robot_contract):
        h = robot_contract
        window = 1.5
        t0 = time.monotonic()
        h.sleep(window + 0.05)
        received = [(t, m) for t, m in h.status if t0 <= t < t0 + window]
        got = [m for _, m in received]
        assert len(got) >= 3, len(got)
        assert all(m.header.frame_id == 'robot_base' for m in got)
        stamps = [t for t, _ in received]
        # Header stays ROS time; it is not the cadence clock.
        assert all(0 <= m.header.stamp.nanosec < 1_000_000_000 for m in got)
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        assert 0.8 * common.HEARTBEAT_HZ <= rate <= 1.2 * common.HEARTBEAT_HZ, rate
        assert max(gaps) < 0.3, max(gaps)

    def test_robot_status_stamp_uses_ros_time(self, robot_contract):
        h = robot_contract
        h.sleep(0.2)
        assert h.status
        stamp = drop_ros.from_time(h.status[-1][1].header.stamp)
        assert abs(h.now() - stamp) <= 0.3  # separate from steady cadence

    @pytest.mark.parametrize('name', sorted(SKILLS))
    def test_robot_skill_accepts_feedback_and_ok(self, robot_contract, name):
        h = robot_contract
        phases = []
        gh = _send(h, name, _goal(h, name, deadline_s=10.0), lambda fb: phases.append(fb.feedback.phase))
        assert gh.accepted
        res = _result(gh)
        assert res.status == GoalStatus.STATUS_SUCCEEDED
        assert res.result.result == common.RESULT_OK
        assert phases and all(phases)                       # phase 이름이 빈 문자열이 아님

    @pytest.mark.req('REQ-IF-005')
    @pytest.mark.tc('TC-IF-005-01')
    @pytest.mark.parametrize(('name', 'mode'), sorted(PLACE_PHASE_KEYS, key=lambda x: (x[0], x[1] or 0)))
    def test_robot_place_feedback_keys(self, robot_contract, name, mode):
        h = robot_contract
        feedbacks = []
        goal = _goal(h, name, deadline_s=10.0)
        if mode is not None:
            goal.mode = mode
        gh = _send(h, name, goal, lambda fb: feedbacks.append(fb.feedback))
        assert gh.accepted
        res = _result(gh)
        assert res.status == GoalStatus.STATUS_SUCCEEDED
        assert res.result.result == common.RESULT_OK
        expected = PLACE_PHASE_KEYS[(name, mode)]
        done_fbs = [fb for fb in feedbacks if fb.phase_done]
        assert [fb.phase for fb in done_fbs] == [ph for ph, _ in expected]
        for ph, req_keys in expected:
            assert any(fb.phase == ph and not fb.phase_done for fb in feedbacks), ph
            done_fb = next(fb for fb in done_fbs if fb.phase == ph)
            assert len(done_fb.metric_keys) == len(done_fb.metric_values), ph
            missing = set(req_keys) - set(done_fb.metric_keys)
            assert not missing, f'{name}({mode}) {ph}: missing {sorted(missing)}'

    def test_robot_deadline_self_abort(self, robot_contract):
        h = robot_contract
        t = h.now()
        gh = _send(h, 'robot/move_to_pose', _goal(h, 'robot/move_to_pose', deadline_s=0.2))
        assert gh.accepted
        res = _result(gh)
        assert res.result.result == common.RESULT_TIMEOUT
        assert h.now() - t <= 0.2 + 0.15

    def test_robot_cancel(self, robot_contract):
        h = robot_contract
        gh = _send(h, 'robot/pick_stone', _goal(h, 'robot/pick_stone'))
        assert gh.accepted
        time.sleep(0.05)
        t = time.monotonic()
        drop_ros.wait_future(gh.cancel_goal_async(), 2.0)
        res = _result(gh)
        assert res.status == GoalStatus.STATUS_CANCELED
        assert res.result.result == common.RESULT_CANCELED
        assert res.result.reason_code == 'CANCELED'            # Stop(STOPPED)과 구분(SPEC-010 3.2절)
        assert time.monotonic() - t <= 0.5                     # SPEC-110 cancel 응답 0.5 s

    def test_robot_single_command_authority(self, robot_contract):
        h = robot_contract
        first = _send(h, 'robot/place_stone', _goal(h, 'robot/place_stone'))
        assert first.accepted
        second = _send(h, 'robot/move_to_pose', _goal(h, 'robot/move_to_pose'))
        assert not second.accepted
        _result(first)

    def test_robot_stop_ends_active_goal(self, robot_contract):
        h = robot_contract
        gh = _send(h, 'robot/dynamic_place', _goal(h, 'robot/dynamic_place'))
        assert gh.accepted
        time.sleep(0.05)
        r = h.call(Stop, 'robot/stop', requester=Stop.Request.ORCHESTRATOR, reason_code='CONTRACT')
        assert r.stopped and r.result == common.RESULT_OK
        res = _result(gh)
        assert (res.result.result, res.result.reason_code) == (common.RESULT_CANCELED, 'STOPPED')
        assert res.status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED)
        assert _release(h, 'contract: stop test done').successful

    @pytest.mark.req('REQ-MCK-008', 'REQ-ROB-014')
    @pytest.mark.tc('TC-MCK-008-01')
    def test_robot_stop_latches_until_release(self, robot_contract):
        h = robot_contract
        r = h.call(Stop, 'robot/stop', requester=Stop.Request.SUPERVISOR, reason_code='CONTRACT_IDLE')
        assert r.stopped and r.result == common.RESULT_OK
        t0 = time.monotonic()
        h.wait_for(lambda: len(_ctrl_hb(h, t0)) >= 2 and len([1 for t, _ in h.status if t >= t0]) >= 2,
                   2.0, '정지 유지 뒤 heartbeat·status')
        assert all(m.error_code == STOP_LATCHED for m in _ctrl_hb(h, t0)), [m.error_code for m in _ctrl_hb(h, t0)]
        assert not any(m.ready for t, m in h.status if t >= t0)
        # 유지 중 goal: 수락하고 지령 없이 CANCELED(STOPPED), phase feedback 0회
        fbs = []
        gh = _send(h, 'robot/place_stone', _goal(h, 'robot/place_stone'), feedback=fbs.append)
        assert gh.accepted
        res = _result(gh)
        assert (res.result.result, res.result.reason_code) == (common.RESULT_CANCELED, 'STOPPED')
        assert fbs == []
        # 빈 사유는 거부하고 유지한다
        bad = _release(h, '')
        assert not bad.successful and 'REASON_REQUIRED' in bad.reason
        t1 = time.monotonic()
        h.wait_for(lambda: len(_ctrl_hb(h, t1)) >= 1, 1.0, '거부 뒤 heartbeat')
        assert _ctrl_hb(h, t1)[-1].error_code == STOP_LATCHED
        # 사유 있는 해제 뒤 정상
        assert _release(h, 'contract: operator checked').successful
        t2 = time.monotonic()
        h.wait_for(lambda: len(_ctrl_hb(h, t2)) >= 2 and len([1 for t, _ in h.status if t >= t2]) >= 2,
                   2.0, '해제 뒤 heartbeat·status')
        assert all(m.error_code == 0 for m in _ctrl_hb(h, t2))
        assert all(m.ready for t, m in h.status if t >= t2)
        gh = _send(h, 'robot/move_to_pose', _goal(h, 'robot/move_to_pose'))
        assert gh.accepted
        assert _result(gh).result.result == common.RESULT_OK

    def test_robot_inactive_rejects_goals(self, robot_contract):
        h = robot_contract
        assert h.deactivate(h.target.node)
        try:
            h.sleep(0.2)
            n = len(h.status)
            gh = _send(h, 'robot/move_to_pose', _goal(h, 'robot/move_to_pose'))
            assert not gh.accepted
            h.sleep(0.3)
            assert len(h.status) == n
        finally:
            assert h.activate(h.target.node)
