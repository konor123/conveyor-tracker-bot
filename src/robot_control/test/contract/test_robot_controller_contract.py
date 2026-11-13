"""실제 robot_controller_node(backend:=fake)가 공통 robot_controller 계약을 통과한다(TEST_SPEC.md TC-ROB-008-04).

mock과 같은 `tools/testing/contracts/robot_controller_contract.py`를 그대로 돌린다. 실제 노드에 필요한 입력은
이 모듈의 발행기가 시험 클라이언트 노드에서 DDS로 준다(계획 `drop/intercept_plan`, track `objects/tracked`,
벨트 `belt/measured_motion`). goal마다 새 plan_id로 해제 시각을 고정한다(planner와 같은 규약, SPEC-400 4.5절).
시뮬레이션 backend만 쓰며 `/dsr01`에 연결하지 않는다. 시험 프로세스 안 발행기의 순간 지연(drop_ros.py 참고)에
걸리지 않도록 계획·관측 나이와 주기 간격 한도를 이 대상에서만 넓힌다. 판정 조건은 바꾸지 않는다.
"""
import sys
import threading

import pytest

pytest.importorskip('rclpy')

import robot_controller_contract as shared  # noqa: E402
from drop_interfaces.action import DropTask  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_interfaces.msg import (BeltMotion, InterceptPlan, ObjectDetection, TrackedObject,  # noqa: E402
                                 TrackedObjectArray)
from drop_ros import Target, to_time  # noqa: E402
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from robot_controller_contract import TestRobotControllerContract  # noqa: E402,F401
from std_msgs.msg import Header  # noqa: E402

pytestmark = [
    pytest.mark.req('REQ-ROB-008'), pytest.mark.tc('TC-ROB-008-04'),
    pytest.mark.req('REQ-ROB-007'), pytest.mark.tc('TC-ROB-007-01'),
    pytest.mark.req('REQ-IF-005'), pytest.mark.tc('TC-IF-005-01'),
    pytest.mark.level('C'),
]

SAFE_Z_M = 0.12
Z_REL_M = SAFE_Z_M - 0.012            # 해제 TCP z
LEAD_S = 1.3                          # goal → 해제 시각
CAP_V = (0.02, 0.0)
TRACK_ID, EPOCH = 7, 0
PARAMS = {
    'backend': 'fake', 'fake_move_time_s': 0.3, 'pose_tolerance_m': 0.002,
    'grip_min_width_m': 0.010, 'grip_max_width_m': 0.020, 'grip_min_current_ma': 10.0,
    'open_width_tolerance_m': 0.001, 'safe_z_m': SAFE_Z_M,
    'fake_tcp_x_m': 0.40, 'fake_tcp_y_m': 0.0, 'fake_tcp_z_m': SAFE_Z_M, 'fake_open_delay_s': 0.05,
    'dynamic.fingertip_below_tcp_m': 0.15, 'dynamic.open_width_m': 0.022,
    'dynamic.approach_above_m': 0.005, 'dynamic.sync_hold_s': 0.1, 'dynamic.entry_lead_s': 0.5,
    'dynamic.release_hold_s': 0.05, 'dynamic.z_max_speed_mps': 0.2, 'dynamic.z_max_accel_mps2': 1.0,
    'dynamic.retreat_speed_mps': 0.1,
    # 시험 프로세스 발행기·WSL 부하 여유(이 대상 한정)
    'dynamic.tau_max_s': 1.0, 'sync.max_plan_age_s': 1.0, 'sync.max_tcp_age_s': 1.0,
    'sync.max_loop_gap_s': 0.2, 'dynamic.commit_late_tol_s': 0.2, 'fake_cmd_hold_s': 0.5,
    # 정적 skill 계약 goal(원점 자세)이 TCP를 원점 근처로 옮기므로 XY 상자를 넓힌다
    'sync.min_xy_m': '[-1.0,-1.0]', 'sync.max_xy_m': '[1.0,1.0]',
    # SafeRetreat place-back(REQ-ROB-016): 매트 반납 위치(시험 값)
    'place_back_xyz_m': '[0.40,-0.10,0.06]',
}
LAUNCH = ('import sys, rclpy; from rclpy.executors import MultiThreadedExecutor; '
          'from robot_control.robot_controller_node import RobotControllerNode; '
          'rclpy.init(args=sys.argv); n = RobotControllerNode(); n.trigger_configure(); n.trigger_activate(); '
          'e = MultiThreadedExecutor(num_threads=4); e.add_node(n); e.spin()')


@pytest.fixture(scope='module')
def robot_target():
    args = []
    for k, v in PARAMS.items():
        args += ['-p', f'{k}:={v}']
    return Target(cmd=[sys.executable, '-c', LAUNCH], node='robot_controller_node', ros_args=args)


class PlanFeeder:
    """계획·track·벨트 30 Hz 발행기. `goal_hook`이 DynamicPlace goal마다 새 계획을 연다."""

    def __init__(self, h):
        self.h = h
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.plan_pub = h.client.create_publisher(InterceptPlan, 'drop/intercept_plan', qos)
        self.trk_pub = h.client.create_publisher(TrackedObjectArray, 'objects/tracked', qos)
        self.belt_pub = h.client.create_publisher(BeltMotion, 'belt/measured_motion', qos)
        self.lock = threading.Lock()
        self.plan_id = 100
        self.revision = 0
        self.plan = None
        self.timer = h.client.create_timer(1 / 30., self.publish)

    def goal_hook(self, name, goal):
        if name == 'robot/pick_stone':
            goal.grip_current_limit = 50
            goal.approach_height = 0.05
        elif name == 'robot/dynamic_place':
            if goal.mode == DropTask.Goal.MODE_STATIC:
                goal.mode = DropTask.Goal.MODE_SYNCHRONIZED
            goal.plan_id, goal.target_epoch = self.new_plan(), EPOCH

    def new_plan(self) -> int:
        status = self.h.status[-1][1] if self.h.status else None
        tcp = (status.tcp_pose.position.x, status.tcp_pose.position.y) if status else (0.4, 0.0)
        t0 = self.h.now()
        with self.lock:
            self.plan_id += 1
            release_xy = (tcp[0] + CAP_V[0] * LEAD_S + 0.002, tcp[1] + CAP_V[1] * LEAD_S)
            self.plan = (self.plan_id, t0 + LEAD_S, release_xy)
        self.publish()
        return self.plan_id

    def publish(self):
        now = self.h.now()
        stamp = to_time(now)
        with self.lock:
            plan = self.plan
            if plan is not None and now <= plan[1] + 1.2:
                self.revision += 1
                plan_id, release_s, (x, y) = plan
                self.plan_pub.publish(InterceptPlan(
                    header=Header(stamp=stamp, frame_id='robot_base'), plan_id=plan_id, revision=self.revision,
                    target_track_id=TRACK_ID, target_epoch=EPOCH, source_state_stamp=stamp,
                    release_time=to_time(release_s), valid_until=to_time(release_s + 1.2),
                    target_tcp_pose=Pose(position=Point(x=x, y=y, z=Z_REL_M), orientation=Quaternion(w=1.0)),
                    target_velocity=Vector3(x=CAP_V[0], y=CAP_V[1]), position_std=0.0003, time_margin=5.0,
                    status=InterceptPlan.OK))
        self.trk_pub.publish(TrackedObjectArray(tracks=[TrackedObject(
            header=Header(stamp=stamp, frame_id='conveyor_frame'), track_id=TRACK_ID, epoch=EPOCH,
            object_class=ObjectDetection.CLASS_CAP, status=TrackedObject.OBSERVED, last_observed_stamp=stamp,
            observation_count=100, velocity=Vector3(x=CAP_V[0], y=CAP_V[1]), valid_until=to_time(now + 0.3))]))
        self.belt_pub.publish(BeltMotion(header=Header(stamp=stamp, frame_id='conveyor_frame'),
                                         direction=Vector3(x=1.0), speed=CAP_V[0], steady=True))

    def close(self):
        self.h.client.destroy_timer(self.timer)


@pytest.fixture(scope='module')
def robot_contract(robot_target):
    source = shared.robot_contract.__wrapped__(robot_target)
    h = next(source)
    feeder = PlanFeeder(h)
    h.goal_hook = feeder.goal_hook
    h.wait_for(lambda: len(h.status) >= 2, 5.0, 'robot/status')
    try:
        yield h
    finally:
        feeder.close()
        try:
            next(source)
        except StopIteration:
            pass


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-ROB-016-05')
def test_real_node_safe_retreat_place_back(robot_contract):
    """공통 계약 밖의 실제 노드 확인: SafeRetreat(place_back=true)는 RETREAT → PLACE_BACK, 측정 개방 뒤 반납 보고."""
    h = robot_contract
    # 공통 계약의 비활성화 시험(on_deactivate의 backend.stop)이 시뮬레이션 backend를 정지 상태로 남기고 재활성으로는
    # 풀리지 않는다. 작업자 절차(Stop → 사유 있는 해제)로 정지를 풀고 시작한다
    assert h.call(Stop, 'robot/stop', requester=Stop.Request.ORCHESTRATOR, reason_code='CONTRACT_SETUP').stopped
    assert shared._release(h, 'contract: place-back setup').successful
    phases = []
    goal = shared._goal(h, 'robot/safe_retreat', deadline_s=10.0)
    goal.place_back = True
    gh = shared._send(h, 'robot/safe_retreat', goal, lambda fb: phases.append(fb.feedback.phase))
    assert gh.accepted
    res = shared._result(gh)
    assert (res.result.result, res.result.reason_code) == (0, ''), res.result
    assert res.result.stone_placed_back is True
    assert [p for i, p in enumerate(phases) if i == 0 or phases[i - 1] != p] == ['RETREAT', 'PLACE_BACK']
