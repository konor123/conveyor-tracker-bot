"""orchestrator_node(SPEC-100 4.8절): DropTask 액션 서버. ROS service·action으로 SkillPort를 구현해 CycleRunner를 돌린다.

판단(상태 전이·판정·중단 등급)은 cycle_runner가 하고, 이 노드는 ROS 호출·구독 캐시·발행만 한다.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time

import rclpy
import yaml
from drop_interfaces.action import DropTask, DynamicPlace, MoveToPose, PickStone, PlaceStone, SafeRetreat
from drop_interfaces.msg import (BeltMotion, Heartbeat, ObjectDetection, ObjectDetectionArray, RobotStatus,
                                 StepRecord, TaskStatus, TrackedObject, TrackedObjectArray)
from drop_interfaces.srv import ComputeIntercept, DetectObjects, SelectTarget, Stop, VerifyDrop
from geometry_msgs.msg import Point, Pose
from lifecycle_msgs.msg import State as LcState
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from . import step_judges as sj
from .abort_policy import HARD_FAULTS, Event
from .ros_qos import no_pub_events, no_sub_events
from .common import AbortLevel, Mode, State
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
        self._pending_send = None
        self._abandoned_send = False
        self._cancelled_goal = None
        self._send_lock = threading.RLock()
        self._wait_guard = None
        self._cleanup_wait = False
        self._feedback_token = object()
        self._goal = None              # 진행 중 skill goal handle
        self._result_future = None
        self._feedback: list = []
        self._mode = Mode.STATIC
        # 장애 latch는 감시 타이머 스레드(_watch)와 실행 스레드(pending_fault)가 함께 건드리므로 lock으로 묶는다
        self._fault_lock = threading.Lock()
        self._fault: Event | None = None
        self._triggered_faults: set[Event] = set()
        self._released = False         # RELEASE 응답을 받았는가(REQ-ORC-010)
        self.drop_goal_handle = None
        self.target = None             # (track_id, epoch)
        self._skipped_tracks: set[int] = set()
        self.static_cap = None         # MODE_STATIC 목표(robot_base Point)
        self.plan_id = 0

    # ------------------------------------------------------------ 공통 대기
    def reset(self, goal_handle) -> None:
        self._wait_guard = None
        self._cleanup_wait = False
        self._feedback_token = object()
        self._feedback = []
        with self._fault_lock:
            self._fault = None
            self._triggered_faults = set()
            self._released = False
            self.drop_goal_handle = goal_handle
        self.target = self.static_cap = None
        self._skipped_tracks = set()

    def raise_fault(self, event: Event) -> None:
        """사이클 중 비동기 장애를 latch한다. latch된 비하드 장애는 하드 이상으로만 바뀐다(REQ-ORC-010).

        원인을 모르는 ROBOT_STOPPED는 뒤이어 확인된 구체적 하드 이상(보호정지·heartbeat 손실 등)으로 바뀐다(REQ-ORC-011).
        """
        with self._fault_lock:
            if self.drop_goal_handle is None or event in self._triggered_faults:
                return
            generic = self._fault is Event.ROBOT_STOPPED and event in HARD_FAULTS
            if self._fault is not None and not generic and not (event in HARD_FAULTS and self._fault not in HARD_FAULTS):
                return
            self._triggered_faults.add(event)
            self._fault = event
        self.n.get_logger().warning(f'비동기 장애: {event.name}')

    def _interrupting(self) -> bool:
        """latch된 장애가 진행 중인 대기를 끝내는가. 해제 뒤에는 하드 이상만 끝낸다(후퇴·판정은 계속, SPEC-110 4.1)."""
        with self._fault_lock:
            return self._fault is not None and (self._fault in HARD_FAULTS or not self._released)

    def pending_fault(self) -> Event | None:
        self.n._watch()
        with self._fault_lock:
            ev, self._fault = self._fault, None
        return ev

    def set_wait_guard(self, guard, cleanup=False) -> None:
        self._wait_guard = guard
        self._cleanup_wait = cleanup

    def _wait(self, pred, deadline: float | None) -> bool:
        # A frozen ROS clock must not make a transport call unbounded.
        budget = max(0., deadline - self.clock.ros_now()) if deadline is not None else 1.
        # ROS deadlines can move relative to steady time after a backward clock
        # correction. Use the same finite safety factor as the cycle watchdog;
        # cleanup/cycle guards below still enforce their independent hard caps.
        factor = float(self.n.cfg_dict['watchdog']['cycle_steady_factor_ratio'])
        steady_end = time.monotonic() + budget * factor
        while True:
            context = getattr(self.n, 'context', None)
            if context is not None and not context.ok():
                self.raise_fault(Event.ROBOT_COMM_LOST)
                return False
            if self.cancel_requested() and not self._released and not self._cleanup_wait:
                self.raise_fault(Event.USER_CANCEL)
                return False
            event = self._wait_guard() if self._wait_guard is not None else None
            if event is not None:
                self.raise_fault(event)
                return False
            if self._interrupting():
                return False
            if pred():
                return True
            if time.monotonic() >= steady_end:
                return False
            if deadline is not None and self.clock.ros_now() >= deadline:
                return False
            time.sleep(POLL_S)

    def _call(self, typ, name: str, deadline: float, **fields):
        cli = self.n.srv_clients[name]
        if not self._wait(cli.service_is_ready, deadline):
            return None
        fut = cli.call_async(typ.Request(cycle_id=self.n.cycle_id, step_id=self.n.step_id,
                                         deadline=_to_time(deadline), **fields))
        return fut.result() if self._wait(fut.done, deadline) else None

    def _send(self, name: str, goal, deadline: float):
        self._accept_pending()
        if self._abandoned_send and self._result_future is not None and self._result_future.done():
            self.cancel_active(0.)
        if self._pending_send is not None or self._goal is not None:
            return None  # unresolved prior motion forbids a replacement
        ac = self.n.actions[name]
        if not self._wait(ac.server_is_ready, deadline):
            return None
        goal.cycle_id, goal.step_id, goal.deadline = self.n.cycle_id, self.n.step_id, _to_time(deadline)
        self._feedback = []
        token = self._feedback_token = object()
        def feedback(fb):
            if token is self._feedback_token:
                self._feedback.append((self.clock.ros_now(), fb.feedback))
        with self._send_lock:
            fut = self._pending_send = ac.send_goal_async(goal, feedback_callback=feedback)
            self._abandoned_send = False
        if not self._wait(fut.done, deadline):
            with self._send_lock:
                self._abandoned_send = True
            if hasattr(fut, 'add_done_callback'):
                fut.add_done_callback(lambda done: self._accept_pending())
            return None
        self._accept_pending()
        return self._goal if self._goal is not None else False

    def _accept_pending(self):
        with self._send_lock:
            fut = self._pending_send
            if fut is None or not fut.done():
                return
            try:
                gh = fut.result()
            except Exception as exc:
                self.n.get_logger().warning(f'action acceptance failed: {exc}')
                return  # acceptance remains unknown; replacement is forbidden
            if gh is None:
                return  # no acceptance decision
            if gh.accepted:
                try:
                    result_future = gh.get_result_async()
                except Exception as exc:
                    self.n.get_logger().warning(f'action result subscription failed: {exc}')
                    return  # retain acceptance future for a safe retry
                self._goal, self._result_future = gh, result_future
                if self._abandoned_send:
                    try:
                        gh.cancel_goal_async()
                        self._cancelled_goal = gh
                    except Exception as exc:
                        self.n.get_logger().warning(f'late action cancel failed: {exc}')
            self._pending_send = None

    def _result(self, deadline: float):
        """진행 중 goal 결과. deadline까지 없으면 None(goal은 cancel 대상으로 남는다)."""
        if self._result_future is None or not self._wait(self._result_future.done, deadline):
            return None
        r = self._result_future.result().result
        self._goal = self._result_future = None
        self._terminal_fault(r)
        return r

    def _terminal_fault(self, r) -> None:
        """skill 결과가 정지를 알리면 하드 이상을 latch한다(REQ-ORC-011).

        `STOPPED`는 Stop 서비스로 끝났다는 뜻이다(보통 safety_supervisor). 같은 원인을 자체 감시가
        이미 보고 있으면 그 구체적 장애가 남도록 감시를 먼저 돌린다.
        """
        reason = getattr(r, 'reason_code', '')
        if reason == 'PROTECTIVE_STOP':
            self.raise_fault(Event.PROTECTIVE_STOP)
        elif reason == 'STOPPED':
            self.n._watch()
            self.raise_fault(Event.ROBOT_STOPPED)

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
                       object_class=ObjectDetection.CLASS_STONE, min_count=1,
                       stable_window=float(self.n.node_cfg['detect_stable_window_s']))
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
        if pass_index > 1 and self.target is not None:
            self._skipped_tracks.add(self.target[0])
        retry = float(self.n.node_cfg['select_retry_s'])
        while True:
            r = self._call(SelectTarget, 'tracking/select_target', deadline,
                           object_class=ObjectDetection.CLASS_CAP, min_observation_count=1)
            if r is None:
                return None
            if r.result == 0 and r.track_id not in self._skipped_tracks:
                self.target = (r.track_id, r.epoch)
                return self._cap_obs()
            if not self._wait(lambda: False, min(deadline, self.clock.ros_now() + retry)) and \
                    (self._interrupting() or self.clock.ros_now() >= deadline):
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
                           object_class=ObjectDetection.CLASS_CAP, min_count=1,
                           stable_window=float(self.n.node_cfg['detect_stable_window_s']))
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
            if self._interrupting() or self.clock.ros_now() >= deadline:
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
        if fb is None:
            if self._result_future is not None and self._result_future.done():
                self._terminal_fault(self._result_future.result().result)
            return None
        return metrics_of(fb)

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
            with self._fault_lock:
                self._released = True
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
        context = getattr(self.n, 'context', None)
        if context is not None and not context.ok():
            return None
        cli = self.n.srv_clients[name]
        end = time.monotonic() + timeout_s
        while not cli.service_is_ready() and time.monotonic() < end:
            if context is not None and not context.ok():
                return None
            time.sleep(POLL_S)
        if not cli.service_is_ready():
            return None
        fut = cli.call_async(typ.Request(**fields))
        while not fut.done() and time.monotonic() < end:
            if context is not None and not context.ok():
                return None
            time.sleep(POLL_S)
        return fut.result() if fut.done() else None

    def cancel_active(self, wait_s):
        end = time.monotonic() + wait_s
        self._abandoned_send = True
        while self._pending_send is not None and time.monotonic() < end:
            self._accept_pending()
            if self._pending_send is not None:
                time.sleep(POLL_S)
        with self._send_lock:
            if self._pending_send is not None:
                return False
            gh, res = self._goal, self._result_future
            if gh is None:
                return True
            if self._cancelled_goal is not gh:
                try:
                    gh.cancel_goal_async()
                    self._cancelled_goal = gh
                except Exception as exc:
                    self.n.get_logger().warning(f'action cancel failed: {exc}')
                    return False
        while not res.done() and time.monotonic() < end:
            time.sleep(POLL_S)
        if res.done():
            try:
                terminal = res.result()
                if terminal is None or terminal.status not in (4, 5, 6):
                    return False
            except Exception as exc:
                self.n.get_logger().warning(f'action termination unconfirmed: {exc}')
                return False
            self._goal = self._result_future = None
            self._feedback_token = object()
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
        self.declare_parameter('last_trial_json', '')
        self.declare_parameter('resume_reason', '')
        path = self.get_parameter('config').value or self._default_config()
        with open(path, encoding='utf-8') as fh:
            self.cfg_dict = yaml.safe_load(fh)
        self.cfg = OrchestratorCfg.from_dict(self.cfg_dict)
        self.node_cfg = self.cfg_dict['node']
        self.budget = self.cfg_dict['budget']
        self.clock = RosClock(self)
        self.group = ReentrantCallbackGroup()
        self.hb_group = MutuallyExclusiveCallbackGroup()
        self.hb = HeartbeatMonitor(float(self.cfg_dict['watchdog']['heartbeat_timeout_s']), self.clock)
        self.lifecycle: dict[str, int] = {}
        self.robot_status = self.belt = self.tracked = self.detections = None
        self.tracked_seq = 0
        self.create_subscription(Heartbeat, 'system/heartbeat', self._on_hb, HB_QOS, callback_group=self.hb_group,
                                 event_callbacks=no_sub_events())
        self.create_subscription(RobotStatus, 'robot/status', self._on_status, DATA_QOS, callback_group=self.hb_group,
                                 event_callbacks=no_sub_events())
        self.create_subscription(BeltMotion, 'belt/measured_motion', lambda m: setattr(self, 'belt', m),
                                 DATA_QOS, event_callbacks=no_sub_events())
        self.create_subscription(TrackedObjectArray, 'objects/tracked', self._on_tracked, DATA_QOS,
                                 event_callbacks=no_sub_events())
        self.create_subscription(ObjectDetectionArray, 'objects/detections',
                                 lambda m: setattr(self, 'detections', m), DATA_QOS, event_callbacks=no_sub_events())
        self.srv_clients = {name: self.create_client(typ, name) for name, typ in (
            ('vision/detect_objects', DetectObjects), ('vision/verify_drop', VerifyDrop),
            ('tracking/select_target', SelectTarget), ('planner/compute_intercept', ComputeIntercept),
            ('robot/stop', Stop))}
        self.actions = {name: ActionClient(self, typ, name) for name, typ in (
            ('robot/pick_stone', PickStone), ('robot/move_to_pose', MoveToPose), ('robot/place_stone', PlaceStone),
            ('robot/dynamic_place', DynamicPlace), ('robot/safe_retreat', SafeRetreat))}
        self.status_pub = self.create_publisher(TaskStatus, 'task/status', STATUS_QOS, event_callbacks=no_pub_events())
        self.step_pub = self.create_publisher(StepRecord, 'task/step_record', STEP_QOS, event_callbacks=no_pub_events())
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
        self.add_on_set_parameters_callback(self._on_params)
        ActionServer(self, DropTask, 'drop_task', execute_callback=self._execute, goal_callback=self._on_goal,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        self.create_timer(float(self.node_cfg['status_period_s']), self._publish_status)
        # 감시 타이머는 steady 시계(REQ-ABT-009): ROS 시각이 멈추거나 NTP 보정으로 역행해도 돈다
        self.create_timer(0.05, self._watch, callback_group=self.hb_group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_params(self, params):
        for p in params:
            if p.name == 'resume_reason' and str(p.value).strip():
                if self.runner.resume(str(p.value)):
                    self.state = State.IDLE
                    self.abort_level = 0
                    self._publish_status()
        return SetParametersResult(successful=True)

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
                and self.belt is not None
                and self.detections is not None
                and all(self.lifecycle.get(n) == LcState.PRIMARY_STATE_ACTIVE
                        for n in self.node_cfg['required_nodes'])):
            time.sleep(0.05)
        self._t_cycle_end = self.clock.ros_now() + g.timeout
        out = self.runner.run(DropGoal(mode=Mode(g.mode), max_belt_speed_mps=g.max_belt_speed, timeout_s=g.timeout,
                                       max_passes=g.max_passes))
        self._t_cycle_end = math.inf
        if out.abort_level in (AbortLevel.L2, AbortLevel.L3, AbortLevel.L4):
            self.abort_level = max(self.abort_level, int(out.abort_level))
        self.state = out.states[-1] if out.states else State.IDLE
        step_durs = {r.step_name: round(max(0.0, r.t_end - r.t_start), 6) for r in out.records}
        summary = {
            'trial_id': self.cycle_id,
            'mode': f'MODE_{Mode(g.mode).name}',
            'result': out.result,
            'abort_level': int(self.abort_level),
            'reason_code': out.reason_code,
            'passes_skipped': int(out.passes_skipped),
            'cycle_time_s': round(max(0.0, out.cycle_time_s), 6),
            'step_durations_s': step_durs,
        }
        self.set_parameters([Parameter('last_trial_json', value=json.dumps(summary, ensure_ascii=False))])
        self._publish_status()
        time.sleep(0.03)
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
    executor = MultiThreadedExecutor(num_threads=4)
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
