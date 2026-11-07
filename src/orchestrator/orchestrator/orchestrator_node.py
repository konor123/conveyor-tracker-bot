"""orchestrator_node(SPEC-100 4.8절): DropTask 액션 서버. ROS service·action으로 SkillPort를 구현해 CycleRunner를 돌린다.

판단(상태 전이·판정·중단 등급)은 cycle_runner가 하고, 이 노드는 ROS 호출·구독 캐시·발행만 한다.
"""
from __future__ import annotations

import math
import os
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from drop_interfaces.action import DropTask, DynamicPlace, MoveToPose, PickStone, PlaceStone, SafeRetreat
from drop_interfaces.msg import (BeltMotion, Heartbeat, ObjectDetection, ObjectDetectionArray, RobotStatus,
                                 StepRecord, TaskStatus, TrackedObject, TrackedObjectArray)
from drop_interfaces.srv import ComputeIntercept, DetectObjects, SelectTarget, Stop, VerifyDrop
from geometry_msgs.msg import Point, Pose
from lifecycle_msgs.msg import State as LcState
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from . import step_judges as sj
from .abort_policy import Event
from .common import Mode, State
from .cycle_runner import CapObservation, CycleRunner, DetectResult, DropGoal, OrchestratorCfg
from .watchdogs import HeartbeatMonitor

DATA_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
HB_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
STEP_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
RESULT_NAMES = ('SUCCESS', 'FAILURE', 'UNKNOWN', 'CANCELED', 'SAFE_ABORT', 'PROTECTIVE_STOP', 'NOT_ATTEMPTED')
POLL_S = 0.005
HEARTBEAT_ROLE_EVENT = {'robot_controller_node': Event.CONTROLLER_HEARTBEAT_LOST}


def _t(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _to_time(sec: float):
    from builtin_interfaces.msg import Time
    ns = int(round(sec * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def metrics_of(fb) -> dict[str, float]:
    return dict(zip(fb.metric_keys, fb.metric_values))


def meas_from_metrics(cls, m: dict[str, float]):
    """phase 끝 metric → 판정 측정값 dataclass. 불리언 필드는 0.5 기준. 없는 키는 판정이 실패하는 값으로 둔다."""
    import dataclasses
    kw = {}
    for f in dataclasses.fields(cls):
        v = m.get(f.name)
        if f.type in ('bool', bool):
            kw[f.name] = bool(v is not None and v >= 0.5)
        else:
            kw[f.name] = float(v) if v is not None else math.inf
    return cls(**kw)


class RosClock:
    def __init__(self, node: Node):
        self._node = node

    def ros_now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def steady_now(self) -> float:
        return time.monotonic()


class RosSkillPort:
    """SkillPort(cycle_runner.py)의 ROS 구현. 메서드는 작업 스레드에서 불리고 deadline(ROS 시각)까지 기다린다."""

    def __init__(self, node: 'OrchestratorNode'):
        self.n = node
        self.clock = node.clock
        self._goal = None              # 진행 중 skill goal handle
        self._result_future = None
        self._feedback: list = []
        self._mode = Mode.STATIC
        self._fault: Event | None = None
        self.drop_goal_handle = None
        self.target = None             # (track_id, epoch)
        self.static_cap = None         # MODE_STATIC 목표(robot_base Point)
        self.plan_id = 0

    # ------------------------------------------------------------ 공통 대기
    def reset(self, goal_handle) -> None:
        self._goal = self._result_future = None
        self._feedback = []
        self._fault = None
        self.drop_goal_handle = goal_handle
        self.target = self.static_cap = None

    def raise_fault(self, event: Event) -> None:
        if self._fault is None and self.drop_goal_handle is not None:
            self._fault = event
            self.n.get_logger().warning(f'비동기 장애: {event.name}')

    def pending_fault(self) -> Event | None:
        return self._fault

    def _wait(self, pred, deadline: float | None) -> bool:
        while True:
            if pred():
                return True
            if self._fault is not None:
                return False
            if deadline is not None and self.clock.ros_now() >= deadline:
                return False
            time.sleep(POLL_S)

    def _call(self, typ, name: str, deadline: float, **fields):
        cli = self.n.clients[name]
        if not self._wait(cli.service_is_ready, deadline):
            return None
        fut = cli.call_async(typ.Request(cycle_id=self.n.cycle_id, step_id=self.n.step_id,
                                         deadline=_to_time(deadline), **fields))
        return fut.result() if self._wait(fut.done, deadline) else None

    def _send(self, name: str, goal, deadline: float):
        ac = self.n.actions[name]
        if not self._wait(ac.server_is_ready, deadline):
            return None
        goal.cycle_id, goal.step_id, goal.deadline = self.n.cycle_id, self.n.step_id, _to_time(deadline)
        self._feedback = []
        fut = ac.send_goal_async(goal, feedback_callback=lambda fb: self._feedback.append(
            (self.clock.ros_now(), fb.feedback)))
        if not self._wait(fut.done, deadline):
            return None
        gh = fut.result()
        if not gh.accepted:
            return False
        self._goal, self._result_future = gh, gh.get_result_async()
        return gh

    def _result(self, deadline: float):
        """진행 중 goal 결과. deadline까지 없으면 None(goal은 cancel 대상으로 남는다)."""
        if self._result_future is None or not self._wait(self._result_future.done, deadline):
            return None
        r = self._result_future.result().result
        self._goal = self._result_future = None
        return r

    def _run_action(self, name, goal, deadline):
        gh = self._send(name, goal, deadline)
        if not gh:
            return None
        return self._result(deadline)

    # ------------------------------------------------------------ SkillPort
    def status(self) -> sj.PrecheckMeas:
        n, p = self.n, self.n.node_cfg
        ages = [n.hb.age(name) for name in p['required_nodes']]
        active = all(n.lifecycle.get(name) == LcState.PRIMARY_STATE_ACTIVE for name in p['required_nodes'])
        rs, belt = n.robot_status, n.belt
        calib = n.detections.calibration_id if n.detections is not None else ''
        want = n.get_parameter('calibration_id').value
        return sj.PrecheckMeas(
            nodes_active=active, heartbeat_age_max_s=max(ages) if ages else math.inf,
            calibration_valid=bool(calib) and (not want or calib == want),
            robot_ready=bool(rs and rs.ready), protective_stop=bool(rs is None or rs.protective_stop),
            gripper_alive=bool(rs and rs.gripper_alive),
            gripper_response_s=rs.gripper_response_s if rs else math.inf,
            gripper_error_bits=rs.gripper_error_bits if rs else 0xFFFF,
            gripper_open=bool(rs and rs.gripper_open),
            belt_steady=bool(belt and belt.steady), belt_speed_mps=belt.speed if belt else math.inf)

    def detect_stones(self, deadline):
        r = self._call(DetectObjects, 'vision/detect_objects', deadline, region=ObjectDetection.REGION_MAT,
                       object_class=ObjectDetection.CLASS_STONE, min_count=1, stable_window=0.0)
        if r is None:
            return None
        dets = list(r.detections)
        if not dets:
            return DetectResult(sj.DetectMeas(0, 0.0, False, math.inf, 0.0), self.clock.ros_now(), None)
        d = dets[0]
        others = [math.dist((d.position.x, d.position.y), (o.position.x, o.position.y)) for o in dets[1:]]
        std = r.position_std[0] if r.position_std else math.inf
        meas = sj.DetectMeas(stone_count=len(dets), confidence=d.confidence, position_valid=d.position_valid,
                             window_std_m=std, clearance_m=min(others) if others else 1.0)
        return DetectResult(meas, self.clock.ros_now(), d.position)

    def pick_stone(self, target, deadline):
        p = self.n.node_cfg
        goal = PickStone.Goal(target_position=target, approach_height=float(p['pick_approach_height_m']),
                              grip_current_limit=int(p['pick_grip_current_ma']))
        r = self._run_action('robot/pick_stone', goal, deadline)
        if r is None:
            return None
        left = self._call(DetectObjects, 'vision/detect_objects', deadline + 1.0,
                          region=ObjectDetection.REGION_MAT, object_class=ObjectDetection.CLASS_STONE)
        radius = float(p['stone_on_mat_radius_m'])
        on_mat = bool(left and any(math.dist((d.position.x, d.position.y), (target.x, target.y)) < radius
                                   for d in left.detections)) and not r.holding
        grip_err = r.result == sj.Result.FAULT and r.reason_code.startswith('GRIPPER')
        return sj.PickMeas(grip_width_m=r.grip_width, grip_current_ma=r.grip_current, stone_on_mat=on_mat,
                           robot_error=r.result == sj.Result.FAULT and not grip_err, gripper_error=grip_err)

    def move_to_pose(self, name, deadline):
        r = self._run_action('robot/move_to_pose', MoveToPose.Goal(named_pose=name), deadline)
        if r is None:
            return None
        if name == 'standby':
            rs = self.n.robot_status
            return sj.StandbyMeas(pose_error_m=r.pose_error if r.result == 0 else math.inf,
                                  holding=bool(rs and rs.holding))
        return r.result == 0

    def _cap_obs(self) -> CapObservation:
        n, b = self.n, self.n.budget
        track_id, _ = self.target
        tr = next((t for t in (n.tracked.tracks if n.tracked else []) if t.track_id == track_id), None)
        belt = n.belt
        caps = [d for d in (n.detections.detections if n.detections else [])
                if d.object_class == ObjectDetection.CLASS_CAP]
        comps = {k[:-2]: float(v) for k, v in b.items() if k.endswith('_s') and k != 'retreat_margin_s'}
        t_req = sum(comps.values())
        now = self.clock.ros_now()
        if tr is None or tr.velocity.x <= 0:
            t_avail, dyn = 0.0, now
        else:
            t_avail = (float(b['x_exit_m']) - tr.position.x) / tr.velocity.x
            dyn = now + t_avail - float(b['retreat_margin_s'])
        meas = sj.WaitCapMeas(
            track_observed=bool(tr and tr.status == TrackedObject.OBSERVED),
            observation_count=tr.observation_count if tr else 0,
            speed_std_mps=belt.speed_std if belt else math.inf, belt_steady=bool(belt and belt.steady),
            stone_inside=any(d.stone_inside for d in caps),
            stone_inside_valid=bool(caps) and all(d.stone_inside_valid for d in caps),
            t_available_s=t_avail, t_required_s=t_req, budget_components=comps)
        return CapObservation(meas, dyn)

    def select_target(self, pass_index, deadline):
        retry = float(self.n.node_cfg['select_retry_s'])
        while True:
            r = self._call(SelectTarget, 'tracking/select_target', deadline,
                           object_class=ObjectDetection.CLASS_CAP, min_observation_count=1)
            if r is None:
                return None
            if r.result == 0:
                self.target = (r.track_id, r.epoch)
                return self._cap_obs()
            if not self._wait(lambda: False, min(deadline, self.clock.ros_now() + retry)) and \
                    (self._fault is not None or self.clock.ros_now() >= deadline):
                return None

    def observe_cap(self, deadline):
        seq = self.n.tracked_seq
        if not self._wait(lambda: self.n.tracked_seq != seq, deadline):
            return None
        return self._cap_obs()

    def detect_cap(self, deadline):
        retry = float(self.n.node_cfg['select_retry_s'])
        while True:
            r = self._call(DetectObjects, 'vision/detect_objects', deadline, region=ObjectDetection.REGION_BELT,
                           object_class=ObjectDetection.CLASS_CAP, min_count=1, stable_window=0.0)
            if r is None:
                return None
            if r.result == 0 and r.detections:
                d = r.detections[0]
                self.static_cap = d.position
                belt = self.n.belt
                meas = sj.WaitCapMeas(
                    track_observed=True, observation_count=int(self.n.cfg_dict['judge']['wait_cap']
                                                                ['observation_count_min_count']),
                    speed_std_mps=0.0, belt_steady=bool(belt is None or belt.steady),
                    stone_inside=d.stone_inside, stone_inside_valid=d.stone_inside_valid,
                    t_available_s=math.inf, t_required_s=0.0)
                return CapObservation(meas, None)
            self._wait(lambda: False, min(deadline, self.clock.ros_now() + retry))
            if self._fault is not None or self.clock.ros_now() >= deadline:
                return None

    def compute_intercept(self, deadline):
        track_id, epoch = self.target
        r = self._call(ComputeIntercept, 'planner/compute_intercept', deadline, target_track_id=track_id,
                       target_epoch=epoch, mode=int(self._mode_goal))
        if r is None:
            return None
        plan = r.plan
        self.plan_id = plan.plan_id
        return sj.PlanMeas(plan_ok=r.result == 0 and plan.status == plan.OK, epoch_match=plan.target_epoch == epoch,
                           time_margin_s=plan.time_margin, position_std_m=plan.position_std,
                           valid_until=_t(plan.valid_until), now=self.clock.ros_now())

    def place_start(self, mode, deadline):
        self._mode = mode
        c = self.n.cfg
        open_width = c.release.d_stone_max_m + c.release.delta_open_m
        if mode is Mode.STATIC:
            pose = Pose()
            if self.static_cap is not None:
                pose.position = Point(x=self.static_cap.x, y=self.static_cap.y, z=self.static_cap.z)
            pose.orientation.w = 1.0
            goal, name = PlaceStone.Goal(target_pose=pose, release_height=c.descend.h_clear_m,
                                         open_width=open_width), 'robot/place_stone'
        else:
            track_id, epoch = self.target
            goal = DynamicPlace.Goal(plan_id=self.plan_id, target_epoch=epoch, mode=int(mode),
                                     e_xy_max=c.sync.e_xy_max_m, v_rel_max=c.sync.v_rel_max_mps,
                                     tau_max=float(self.n.cfg_dict['watchdog']['obs_freshness_s']),
                                     position_std_max=c.plan.position_std_max_m, rim_clearance=c.descend.h_clear_m,
                                     open_width=open_width)
            name = 'robot/dynamic_place'
        if not self._send(name, goal, deadline):
            return None
        m = self._phase_done('APPROACH', deadline)
        return None if m is None else meas_from_metrics(sj.ApproachMeas, m)

    def _phase_done(self, phase, deadline):
        def done():
            return any(fb.phase == phase and fb.phase_done for _, fb in self._feedback) or (
                self._result_future is not None and self._result_future.done())
        if not self._wait(done, deadline):
            return None
        fb = next((fb for _, fb in self._feedback if fb.phase == phase and fb.phase_done), None)
        return metrics_of(fb) if fb is not None else {}

    def place_feedback(self, phase, deadline):
        m = self._phase_done(phase, deadline)
        if m is None:
            return None
        if phase == 'SYNCHRONIZE':
            if self._mode is Mode.PREDICTED:
                return meas_from_metrics(sj.EntryMeas, m)
            return [sj.SyncSample(t, fb.e_xy, fb.v_rel) for t, fb in self._feedback if fb.phase == 'SYNCHRONIZE']
        if phase == 'DESCEND':
            return meas_from_metrics(sj.StaticPlaceMeas if self._mode is Mode.STATIC else sj.DescendMeas, m)
        if phase == 'RELEASE':
            return meas_from_metrics(sj.ReleaseMeas, m)
        if phase == 'RETREAT':
            r = self._result(deadline)
            if r is None or r.result != 0:
                return None
            return meas_from_metrics(sj.RetreatMeas, m)
        raise ValueError(phase)

    def verify(self, deadline):
        track_id = self.target[0] if self.target else 0
        r = self._call(VerifyDrop, 'vision/verify_drop', deadline, cap_track_id=track_id,
                       hold_time=float(self.n.node_cfg['verify_hold_s']))
        if r is None or r.result != 0:
            return None
        return sj.VerifyMeas(responded=True, verdict=r.verdict)

    def safe_retreat(self, keep_sync, place_back, deadline):
        r = self._run_action('robot/safe_retreat', SafeRetreat.Goal(keep_sync=keep_sync, place_back=place_back),
                             deadline)
        return r is not None and r.result == 0

    def stop(self):
        r = self._call_unguarded(Stop, 'robot/stop', 2.0, requester=Stop.Request.ORCHESTRATOR,
                                 reason_code='ORCHESTRATOR_L4')
        return bool(r and r.stopped)

    def _call_unguarded(self, typ, name, timeout_s, **fields):
        """장애 중에도 보내야 하는 호출(Stop). steady 제한시간."""
        cli = self.n.clients[name]
        end = time.monotonic() + timeout_s
        while not cli.service_is_ready() and time.monotonic() < end:
            time.sleep(POLL_S)
        fut = cli.call_async(typ.Request(**fields))
        while not fut.done() and time.monotonic() < end:
            time.sleep(POLL_S)
        return fut.result() if fut.done() else None

    def cancel_active(self, wait_s):
        gh, res = self._goal, self._result_future
        if gh is None:
            return True
        gh.cancel_goal_async()
        end = time.monotonic() + wait_s
        while not res.done() and time.monotonic() < end:
            time.sleep(POLL_S)
        if res.done():
            self._goal = self._result_future = None
            return True
        return False

    def cancel_requested(self):
        gh = self.drop_goal_handle
        return bool(gh is not None and gh.is_cancel_requested)


class OrchestratorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('orchestrator_node', **kwargs)
        self.declare_parameter('config', '')
        self.declare_parameter('calibration_id', '')
        path = self.get_parameter('config').value or self._default_config()
        with open(path, encoding='utf-8') as fh:
            self.cfg_dict = yaml.safe_load(fh)
        self.cfg = OrchestratorCfg.from_dict(self.cfg_dict)
        self.node_cfg = self.cfg_dict['node']
        self.budget = self.cfg_dict['budget']
        self.clock = RosClock(self)
        self.group = ReentrantCallbackGroup()
        g = self.group
        self.hb = HeartbeatMonitor(float(self.cfg_dict['watchdog']['heartbeat_timeout_s']), self.clock)
        self.lifecycle: dict[str, int] = {}
        self.robot_status = self.belt = self.tracked = self.detections = None
        self.tracked_seq = 0
        self.create_subscription(Heartbeat, 'system/heartbeat', self._on_hb, HB_QOS, callback_group=g)
        self.create_subscription(RobotStatus, 'robot/status', self._on_status, DATA_QOS, callback_group=g)
        self.create_subscription(BeltMotion, 'belt/measured_motion', lambda m: setattr(self, 'belt', m),
                                 DATA_QOS, callback_group=g)
        self.create_subscription(TrackedObjectArray, 'objects/tracked', self._on_tracked, DATA_QOS, callback_group=g)
        self.create_subscription(ObjectDetectionArray, 'objects/detections',
                                 lambda m: setattr(self, 'detections', m), DATA_QOS, callback_group=g)
        self.clients = {name: self.create_client(typ, name, callback_group=g) for name, typ in (
            ('vision/detect_objects', DetectObjects), ('vision/verify_drop', VerifyDrop),
            ('tracking/select_target', SelectTarget), ('planner/compute_intercept', ComputeIntercept),
            ('robot/stop', Stop))}
        self.actions = {name: ActionClient(self, typ, name, callback_group=g) for name, typ in (
            ('robot/pick_stone', PickStone), ('robot/move_to_pose', MoveToPose), ('robot/place_stone', PlaceStone),
            ('robot/dynamic_place', DynamicPlace), ('robot/safe_retreat', SafeRetreat))}
        self.status_pub = self.create_publisher(TaskStatus, 'task/status', STATUS_QOS)
        self.step_pub = self.create_publisher(StepRecord, 'task/step_record', STEP_QOS)
        self.port = RosSkillPort(self)
        self.runner = CycleRunner(self.port, self.clock, self.cfg, on_state=self._on_state,
                                  on_record=self._on_record)
        self.state = State.IDLE
        self.cycle_id = ''
        self.step_id = 0
        self.cycle_count = 0
        self.abort_level = 0
        self._busy = threading.Lock()
        self._t_cycle_end = math.inf
        ActionServer(self, DropTask, 'drop_task', execute_callback=self._execute, goal_callback=self._on_goal,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=g)
        self.create_timer(float(self.node_cfg['status_period_s']), self._publish_status, callback_group=g)
        self.create_timer(0.05, self._watch, callback_group=g)

    @staticmethod
    def _default_config() -> str:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('orchestrator'), 'config', 'orchestrator.yaml')

    # ------------------------------------------------------------ 구독·감시
    def _on_hb(self, m: Heartbeat):
        self.hb.beat(m.node_name)
        self.lifecycle[m.node_name] = m.lifecycle_state

    def _on_status(self, m: RobotStatus):
        self.robot_status = m

    def _on_tracked(self, m):
        self.tracked = m
        self.tracked_seq += 1

    def _watch(self):
        """비동기 장애(SPEC-100 4.8절 pending_fault). 사이클 중에만 latch한다."""
        rs = self.robot_status
        if rs is not None and rs.protective_stop:
            self.port.raise_fault(Event.PROTECTIVE_STOP)
        elif rs is not None and not rs.connected:
            self.port.raise_fault(Event.ROBOT_COMM_LOST)
        for name in self.node_cfg['required_nodes']:
            if name in self.lifecycle and self.hb.lost(name):
                self.port.raise_fault(HEARTBEAT_ROLE_EVENT.get(name, Event.HEARTBEAT_LOST))

    # ------------------------------------------------------------ 발행
    def _on_state(self, state: State):
        self.state = state
        self.step_id += 1
        self._publish_status()

    def _on_record(self, rec):
        msg = StepRecord(cycle_id=self.cycle_id, step_seq=rec.step_seq, step_name=rec.step_name,
                         t_start=_to_time(rec.t_start), t_end=_to_time(rec.t_end), result=int(rec.result),
                         reason_code=rec.reason_code, attempt=rec.attempt, abort_level=rec.abort_level,
                         metric_keys=list(rec.metric_keys), metric_values=[float(v) for v in rec.metric_values])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.abort_level = max(self.abort_level, rec.abort_level)
        self.step_pub.publish(msg)

    def _publish_status(self):
        rs = self.robot_status
        msg = TaskStatus(cycle_id=self.cycle_id, state=self.state.name, abort_level=self.abort_level,
                         command_owner='orchestrator', robot_ready=bool(rs and rs.ready),
                         protective_stop=bool(rs and rs.protective_stop), gripper_holding=bool(rs and rs.holding),
                         time_remaining=max(0.0, self._t_cycle_end - self.clock.ros_now())
                         if math.isfinite(self._t_cycle_end) else 0.0)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)

    # ------------------------------------------------------------ DropTask
    def _on_goal(self, goal):
        if not self._busy.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._run_cycle(goal_handle)
        finally:
            self._busy.release()

    def _run_cycle(self, goal_handle):
        g = goal_handle.request
        self.cycle_count += 1
        self.cycle_id = f'{time.strftime("%Y%m%d-%H%M%S")}-{self.cycle_count:04d}'
        self.step_id = 0
        self.abort_level = 0
        self.port.reset(goal_handle)
        self.port._mode_goal = Mode(g.mode)
        # 시작 전 캐시(heartbeat·로봇 상태)를 기다린다. 부족하면 PRECHECK가 판정한다
        end = time.monotonic() + float(self.node_cfg['startup_wait_s'])
        while time.monotonic() < end and not (
                self.robot_status is not None
                and all(n in self.lifecycle for n in self.node_cfg['required_nodes'])):
            time.sleep(0.05)
        self._t_cycle_end = self.clock.ros_now() + g.timeout
        out = self.runner.run(DropGoal(mode=Mode(g.mode), max_belt_speed_mps=g.max_belt_speed, timeout_s=g.timeout,
                                       max_passes=g.max_passes))
        self._t_cycle_end = math.inf
        self.state = out.states[-1] if out.states else State.IDLE
        self._publish_status()
        res = DropTask.Result(result=RESULT_NAMES.index(out.result) if out.result in RESULT_NAMES else 2,
                              reason_code=out.reason_code, release_error_valid=False, stone_offset_valid=False,
                              cycle_time=out.cycle_time_s, passes_skipped=out.passes_skipped)
        self.get_logger().info(f'{self.cycle_id}: {out.result} {out.reason_code} '
                               f'({" → ".join(s.name for s in out.states)})')
        self.port.drop_goal_handle = None
        if out.result == 'CANCELED' and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return res


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
