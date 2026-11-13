"""robot_controller_node: skill action·Stop·Heartbeat·RobotStatus의 얇은 ROS 래퍼.

판단은 `skill_logic.py`(정적 skill)와 `dynamic_place.py`(DynamicPlace·동기 유지 후퇴)가 한다. 기본 backend는
`disabled`이며 어떤 파라미터도 실기 연결을 암묵적으로 만들지 않는다. `backend:=fake`는 시뮬레이션 값만 쓴다.
DynamicPlace는 해제 transport가 있을 때만 실행하고(`backend:=fake`의 `SimulatedPlaceTransport`), 없으면
PRECONDITION(`PLANNER_NOT_READY`)으로 닫는다. 계획은 `drop/intercept_plan`, track은 `objects/tracked`, 벨트는
`belt/measured_motion`에서 받는다.
`Stop`은 정지 유지를 걸고, 작업자가 `stop_release_reason`에 사유를 쓸 때만 풀린다(SPEC-010 4.5절, REQ-ROB-014).
`SafeRetreat`는 정적 실행기가 판단하고, `keep_sync`이면 해제 transport의 동기 유지 후퇴를 주입해 쓴다(REQ-ROB-010·016).
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import fields, replace

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from drop_interfaces.action import DynamicPlace, MoveToPose, PickStone, PlaceStone, SafeRetreat
from drop_interfaces.msg import BeltMotion, Heartbeat, InterceptPlan, RobotStatus, TrackedObjectArray
from drop_interfaces.srv import Stop
from geometry_msgs.msg import Point, Pose, Quaternion
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from .belt_sync import PlanSample, SyncConfig
from .dynamic_place import (BeltInfo, DynamicPlaceCfg, DynamicPlaceSequencer, GateLimits, KeepSyncRetreat,
                            PlaceGoal, TrackInfo, run_keep_sync_retreat)
from .fake_backend import SimulatedBackend
from .fake_place_transport import SimulatedPlaceTransport
from .skill_logic import Result, SkillResult, StaticSkillCfg, StaticSkillExecutor
from .stop_latch import ERROR_STOP_LATCHED, StopLatch

HEARTBEAT_PERIOD_S = 0.1
DATA_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
HB_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)
# 계획·track·벨트 구독: 발행자(SPEC-010 6절)와 같은 reliable/volatile depth 5
INPUT_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
POLL_S = 0.01
# 정적 skill 파라미터와 같은 값을 쓰는 DynamicPlace 설정(따로 선언하지 않는다)
SHARED_DYNAMIC_KEYS = {'safe_z_m': 'safe_z_m', 'open_width_tolerance_m': 'open_width_tolerance_m'}
# 빈 배열이 기본값(미설정)인 실수 배열 파라미터. 빈 목록으로 선언하면 rclpy가 BYTE_ARRAY로 정해 실수 배열 덮어쓰기를
# 거부하므로 동적 형식으로 선언한다
ARRAY_PARAMS = ('standby_pose', 'safe_pose', 'place_back_xyz_m')

ACTIONS = {
    'MoveToPose': (MoveToPose, 'robot/move_to_pose'),
    'PickStone': (PickStone, 'robot/pick_stone'),
    'PlaceStone': (PlaceStone, 'robot/place_stone'),
    'DynamicPlace': (DynamicPlace, 'robot/dynamic_place'),
    'SafeRetreat': (SafeRetreat, 'robot/safe_retreat'),
}


def _from_time(t: Time) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


def _float_or_none(node, name: str):
    value = node.get_parameter(name).value
    if value is None:
        return None
    value = float(value)
    return None if value < 0.0 else value


def _array_or_none(node, name: str):
    value = node.get_parameter(name).value
    if value is None or len(value) == 0:
        return None
    return [float(v) for v in value]


def _xyz_or_none(node, name: str):
    """[x, y, z] m. 비어 있으면 미설정. 좌표는 음수도 유효하므로 −1 미설정 규약을 쓰지 않는다."""
    values = _array_or_none(node, name)
    if values is None:
        return None
    if len(values) != 3:
        node.get_logger().error(f'{name}: [x, y, z] 3개 필요, 받은 값 {values}; 미설정으로 둔다')
        return None
    return tuple(values)


def _plan_sample(msg) -> PlanSample:
    p, v = msg.target_tcp_pose.position, msg.target_velocity
    return PlanSample(int(msg.plan_id), int(msg.target_track_id), int(msg.target_epoch), int(msg.revision),
                      _from_time(msg.header.stamp), _from_time(msg.source_state_stamp),
                      _from_time(msg.release_time), _from_time(msg.valid_until), (p.x, p.y), (v.x, v.y),
                      msg.header.frame_id, int(msg.status), float(msg.position_std), float(p.z))


def _pose_parts(pose: Pose):
    return (np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float),
            np.array([pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w], dtype=float))


class RobotControllerNode(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__('robot_controller_node', **kwargs)
        self._declare_parameters()
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        self._lock = threading.Lock()
        self._active = False
        self._active_goal = False
        self._stop_requested = False
        self.stop_latch = StopLatch()
        self._seq = 0
        self.cfg = self._load_cfg()
        self.dyn_cfg, self.sync_cfg = self._load_dynamic_cfg()
        self.backend_name = str(self.get_parameter('backend').value)
        self._goal_abort = None
        self.place_transport = None
        self.backend = self._make_backend()
        self.skill = StaticSkillExecutor(self.backend, self.cfg, self, active=self._active)
        self._inbox = threading.Lock()
        self._plans = deque(maxlen=64)          # (수신 steady 시각, 순번, PlanSample)
        self._plan_seq = 0
        self._tracks: dict[int, TrackInfo] = {}
        self._belt: BeltInfo | None = None
        self._sync_velocity = None              # (계획 속도 XY, steady 시각): SafeRetreat keep_sync 기준
        # 입력 구독(합쳐 약 90 Hz)은 별도 그룹에 둬서 기본 그룹의 status·heartbeat 타이머를 늦추지 않는다
        no_events = SubscriptionEventCallbacks(use_default_callbacks=False)
        inputs = MutuallyExclusiveCallbackGroup()
        self.create_subscription(InterceptPlan, 'drop/intercept_plan', self._on_plan, INPUT_QOS,
                                 event_callbacks=no_events, callback_group=inputs)
        self.create_subscription(TrackedObjectArray, 'objects/tracked', self._on_tracked, INPUT_QOS,
                                 event_callbacks=no_events, callback_group=inputs)
        self.create_subscription(BeltMotion, 'belt/measured_motion', self._on_belt, INPUT_QOS,
                                 event_callbacks=no_events, callback_group=inputs)
        self._servers = []
        self.stop_service = None
        self.status_pub = None
        self.hb_pub = None
        self._timers = []
        self.add_on_set_parameters_callback(self._on_params)

    # ------------------------------------------------------------ 파라미터·설정
    def _declare_parameters(self) -> None:
        defs = {
            'backend': 'disabled',
            'pose_tolerance_m': -1.0,
            'approach_height_m': -1.0,
            'safe_z_m': -1.0,
            'height_tolerance_m': -1.0,
            'open_width_tolerance_m': -1.0,
            'release_rim_clearance_m': -1.0,
            'grip_min_width_m': -1.0,
            'grip_max_width_m': -1.0,
            'grip_min_current_ma': -1.0,
            'move_v_mps': 0.03,
            'move_a_mps2': 0.05,
            'close_pulse': -1,
            'fake_move_time_s': 0.2,
            'fake_grip_width_m': 0.0175,
            'fake_grip_current_ma': 40.0,
            'fake_tcp_x_m': 0.0,
            'fake_tcp_y_m': 0.0,
            'fake_tcp_z_m': 0.1,
            'stop_release_reason': '',
            # backend:=fake 해제 transport(가정값, SPEC-500 3절)
            'fake_tcp_lag_s': 0.029,
            'fake_open_delay_s': 0.1,
            'fake_abort_sync_hold_s': 1.0,
            'fake_cmd_hold_s': 0.1,
        }
        for name, value in defs.items():
            self.declare_parameter(name, value)
        for name in ARRAY_PARAMS:
            self.declare_parameter(name, [], ParameterDescriptor(dynamic_typing=True))
        for f in fields(DynamicPlaceCfg):
            if f.name not in SHARED_DYNAMIC_KEYS:
                self.declare_parameter(f'dynamic.{f.name}', float(f.default))
        for f in fields(SyncConfig):
            value = f.default
            self.declare_parameter(f'sync.{f.name}', [float(v) for v in value] if isinstance(value, tuple)
                                   else float(value))

    def _load_dynamic_cfg(self):
        dyn = {f.name: float(self.get_parameter(f'dynamic.{f.name}').value)
               for f in fields(DynamicPlaceCfg) if f.name not in SHARED_DYNAMIC_KEYS}
        for key, name in SHARED_DYNAMIC_KEYS.items():
            dyn[key] = float(self.get_parameter(name).value)
        sync = {}
        for f in fields(SyncConfig):
            value = self.get_parameter(f'sync.{f.name}').value
            sync[f.name] = tuple(float(v) for v in value) if isinstance(f.default, tuple) else float(value)
        return DynamicPlaceCfg(**dyn), SyncConfig(**sync)

    def _load_cfg(self) -> StaticSkillCfg:
        named = {}
        for name, key in (('standby', 'standby_pose'), ('safe', 'safe_pose')):
            values = _array_or_none(self, key)
            if values and len(values) == 7:
                named[name] = ((values[0], values[1], values[2]),
                               (values[3], values[4], values[5], values[6]))
        return StaticSkillCfg(
            named_poses=named,
            pose_tolerance_m=_float_or_none(self, 'pose_tolerance_m'),
            approach_height_m=_float_or_none(self, 'approach_height_m'),
            safe_z_m=_float_or_none(self, 'safe_z_m'),
            height_tolerance_m=_float_or_none(self, 'height_tolerance_m'),
            open_width_tolerance_m=_float_or_none(self, 'open_width_tolerance_m'),
            release_rim_clearance_m=_float_or_none(self, 'release_rim_clearance_m'),
            grip_min_width_m=_float_or_none(self, 'grip_min_width_m'),
            grip_max_width_m=_float_or_none(self, 'grip_max_width_m'),
            grip_min_current_ma=_float_or_none(self, 'grip_min_current_ma'),
            move_v_mps=float(self.get_parameter('move_v_mps').value),
            move_a_mps2=float(self.get_parameter('move_a_mps2').value),
            place_back_xyz_m=_xyz_or_none(self, 'place_back_xyz_m'),
            open_width_m=_float_or_none(self, 'dynamic.open_width_m'),
        )

    def _make_backend(self):
        if self.backend_name == 'disabled':
            return None
        if self.backend_name == 'fake':
            pose = ((float(self.get_parameter('fake_tcp_x_m').value),
                     float(self.get_parameter('fake_tcp_y_m').value),
                     float(self.get_parameter('fake_tcp_z_m').value)),
                    (0.0, 0.0, 0.0, 1.0))
            backend = SimulatedBackend(
                tcp_pose=pose,
                move_time_s=float(self.get_parameter('fake_move_time_s').value),
                grip_width_m=float(self.get_parameter('fake_grip_width_m').value),
                grip_current_ma=float(self.get_parameter('fake_grip_current_ma').value),
                cancel_requested=self._backend_abort,
            )
            # DynamicPlace 해제 transport도 같은 시뮬레이션 TCP·정지 상태를 쓴다
            self.place_transport = SimulatedPlaceTransport(
                self, robot=backend, lag_s=float(self.get_parameter('fake_tcp_lag_s').value),
                open_delay_s=float(self.get_parameter('fake_open_delay_s').value),
                abort_sync_hold_s=float(self.get_parameter('fake_abort_sync_hold_s').value),
                cmd_hold_s=float(self.get_parameter('fake_cmd_hold_s').value),
                grip_width_m=float(self.get_parameter('fake_grip_width_m').value))
            return backend
        self.get_logger().error(f'알 수 없는 backend={self.backend_name!r}; disabled로 둔다')
        return None

    # ------------------------------------------------------------ lifecycle
    def on_configure(self, state):
        self.state_id = State.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.state_id = State.PRIMARY_STATE_ACTIVE
        self._active = True
        self.skill.active = True
        group = ReentrantCallbackGroup()
        # action 서버는 처음 활성화 때 한 번 만들고 비활성 동안에도 남겨, 비활성 goal을 거부로 응답한다(SPEC-010 4.3절)
        if not self._servers:
            for name, (typ, topic) in ACTIONS.items():
                self._servers.append(ActionServer(
                    self, typ, topic, execute_callback=self._executor_for(name),
                    goal_callback=self._on_goal,
                    cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                    callback_group=group))
        self.stop_service = self.create_service(Stop, 'robot/stop', self._on_stop,
                                                callback_group=group)
        self.status_pub = self.create_publisher(RobotStatus, 'robot/status', DATA_QOS,
                                                event_callbacks=PublisherEventCallbacks(use_default_callbacks=False))
        self.hb_pub = self.create_publisher(Heartbeat, 'system/heartbeat', HB_QOS,
                                            event_callbacks=PublisherEventCallbacks(use_default_callbacks=False))
        # 상태·heartbeat는 steady 시계로 발행한다(REQ-ABT-009): ROS 시각 정지·역행에도 끊기지 않는다
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._timers = [self.create_timer(HEARTBEAT_PERIOD_S, self._on_status, clock=steady),
                        self.create_timer(HEARTBEAT_PERIOD_S, self._on_heartbeat, clock=steady)]
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._active = False
        self.skill.active = False
        if self.backend is not None:
            self.backend.stop()
        if self.stop_service is not None:
            self.destroy_service(self.stop_service)
            self.stop_service = None
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()
        for pub in (self.status_pub, self.hb_pub):
            if pub is not None:
                self.destroy_publisher(pub)
        self.status_pub = self.hb_pub = None
        self.state_id = State.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.state_id = State.PRIMARY_STATE_FINALIZED
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------ goal
    def _on_goal(self, _goal_request):
        with self._lock:
            if not self._active or self._active_goal or self.skill.authority.busy:
                return GoalResponse.REJECT
            # 수락 시점에 실행 중으로 예약한다: 수락과 실행 사이의 작업자 해제를 GOAL_ACTIVE로 막는다
            self._active_goal = True
        return GoalResponse.ACCEPT

    def _executor_for(self, name):
        return lambda goal_handle: self._execute(name, goal_handle)

    def _execute(self, name, goal_handle):
        try:
            return self._execute_reserved(name, goal_handle)
        finally:
            with self._lock:
                self._active_goal = False         # 실행 초입 예외로 빠져도 수락 때의 예약을 푼다

    def _execute_reserved(self, name, goal_handle):
        with self._lock:
            self._active_goal = True
            latched = self.stop_latch.latched
        self._stop_requested = False
        goal = goal_handle.request
        typ = ACTIONS[name][0]
        deadline_s = None
        raw_deadline = _from_time(goal.deadline) if hasattr(goal, 'deadline') else 0.0
        if raw_deadline > 0.0:
            deadline_s = raw_deadline

        def cancel():
            return bool(goal_handle.is_cancel_requested or self._stop_requested or self.stop_latch.latched)

        # 시뮬레이션 backend의 이동도 cancel·deadline에서 끊는다(REQ-ROB-007)
        self._goal_abort = lambda: cancel() or (deadline_s is not None and self.ros_now() >= deadline_s)

        def feedback(phase, metrics, done):
            fb = typ.Feedback(phase=phase)
            if name in ('PlaceStone', 'DynamicPlace'):
                fb.phase_done = bool(done)
                if done:
                    fb.metric_keys = list(metrics.keys())
                    fb.metric_values = [float(v) for v in metrics.values()]
            goal_handle.publish_feedback(fb)

        try:
            if latched:
                # 정지 유지 중: 지령 없이 STOPPED로 끝낸다(SPEC-010 4.5절)
                res = SkillResult(Result.CANCELED, 'STOPPED', {})
            elif name == 'MoveToPose':
                pos, quat = _pose_parts(goal.target_pose)
                res = self.skill.move_to_pose(pos, quat, deadline_s, cancel, feedback,
                                              named_pose=str(goal.named_pose))
            elif name == 'PickStone':
                res = self.skill.pick_stone((goal.target_position.x, goal.target_position.y,
                                             goal.target_position.z),
                                            float(goal.approach_height),
                                            int(goal.grip_current_limit),
                                            deadline_s, cancel, feedback)
            elif name == 'PlaceStone':
                pos, quat = _pose_parts(goal.target_pose)
                res = self.skill.place_stone((pos, quat), float(goal.release_height),
                                             float(goal.open_width), deadline_s, cancel, feedback)
            elif name == 'SafeRetreat':
                runner = (self._keep_sync_runner(goal_handle, deadline_s)
                          if self.place_transport is not None else None)
                res = self.skill.safe_retreat(bool(goal.keep_sync), bool(goal.place_back),
                                              goal.place_back_pose, deadline_s, cancel, feedback,
                                              keep_sync_retreat=runner)
            elif self.place_transport is None:
                res = self.skill.dynamic_place()
            else:
                res = self._dynamic_place(goal, goal_handle, deadline_s)
        except Exception as exc:      # backend 예외도 집계해 성공으로 만들지 않는다
            self.get_logger().error(f'{name} 실행 예외: {exc!r}')
            res = _failed(str(exc))
        finally:
            self._goal_abort = None
            with self._lock:
                self._active_goal = False

        # 실행 중 들어온 Stop도 STOPPED로 낸다: 시작 때 본 값만이 아니라 지금의 정지 유지 상태를 다시 읽는다
        if (self._stop_requested or latched or self.stop_latch.latched) and res.result != Result.OK:
            # Stop으로 선점된 goal은 cancel과 구분해 CANCELED·STOPPED로 끝낸다(SPEC-010 3.2절, REQ-ROB-007)
            res = replace(res, result=Result.CANCELED, reason_code='STOPPED')
        result = typ.Result()
        result.result = int(res.result)
        result.reason_code = res.reason_code
        self._fill_result(name, result, res)
        if res.result == Result.OK:
            goal_handle.succeed()
        elif res.result == Result.CANCELED and goal_handle.is_cancel_requested:
            goal_handle.canceled()        # rclpy는 cancel 요청을 받은 goal만 CANCELED로 끝낼 수 있다
        else:
            goal_handle.abort()
        return result

    @staticmethod
    def _fill_result(name, result, res):
        if name == 'PickStone':
            result.grip_width = float(res.metrics.get('grip_width_m', float('nan')))
            result.grip_current = float(res.metrics.get('grip_current_ma', 0.0))
            result.holding = bool(res.holding)
        elif name == 'MoveToPose':
            result.pose_error = float(res.metrics.get('pose_error_m', float('nan')))
        elif name == 'PlaceStone':
            result.released = bool(res.released)
            result.release_error = float(res.metrics.get('target_error_m', float('nan')))
        elif name == 'SafeRetreat':
            result.stone_placed_back = bool(res.released)       # 측정 개방 폭을 확인한 뒤에만 참(REQ-ROB-016)
        elif name == 'DynamicPlace':
            result.released = bool(res.released)
            for key, field_name in (('gate_e_xy_m', 'gate_e_xy'), ('gate_v_rel_mps', 'gate_v_rel'),
                                    ('gate_t_since_obs_s', 'gate_t_since_obs'),
                                    ('gate_position_std_m', 'gate_position_std')):
                setattr(result, field_name, float(res.metrics.get(key, float('nan'))))

    # ------------------------------------------------------------ DynamicPlace·동기 유지 후퇴
    def _dynamic_place(self, goal, goal_handle, deadline_s):
        """순서기를 주기마다 돌리고 feedback을 옮긴다. 판단은 `dynamic_place.DynamicPlaceSequencer`가 한다."""
        if not self.skill.authority.acquire('dynamic_place'):
            return SkillResult(Result.PRECONDITION, 'BUSY')
        try:
            limits = GateLimits.from_goal({k: getattr(goal, k) for k in
                                           ('e_xy_max', 'v_rel_max', 'tau_max', 'position_std_max',
                                            'rim_clearance')}, self.dyn_cfg.gate_defaults())
            place_goal = PlaceGoal(int(goal.plan_id), int(goal.target_epoch), int(goal.mode), limits,
                                   float(goal.open_width), deadline_s or 0.0)
            reset = getattr(self.place_transport, 'reset_cycle', None)
            if reset is not None:
                reset(float(self.get_parameter('fake_grip_width_m').value))
            seq = DynamicPlaceSequencer(self.place_transport, place_goal, self.dyn_cfg, self.sync_cfg)
            # goal 전에 받은 계획 중 신선한 것만 넘긴다: 오래된 revision은 입력 상태 나이로 PLAN_INVALID가 된다
            plans, last = self._plans_since(None, fresh_s=self.sync_cfg.max_plan_age_s)
            out = None
            period = self.sync_cfg.period_s
            next_t = time.monotonic()
            while True:
                with self._inbox:
                    tracks, belt = dict(self._tracks), self._belt
                out = seq.tick(self.ros_now(), time.monotonic(), plans, tracks, belt,
                               cancel=bool(goal_handle.is_cancel_requested),
                               stop=bool(self._stop_requested or self.stop_latch.latched))
                for fb in out.feedback:
                    goal_handle.publish_feedback(_dynamic_feedback(fb))
                if out.result is not None:
                    break
                if not rclpy.ok():
                    seq.t.abort(False)
                    return SkillResult(Result.FAULT, 'SHUTDOWN')
                next_t += period
                delay = next_t - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                else:
                    next_t = time.monotonic()
                plans, last = self._plans_since(last)
            if seq.plan_velocity_xy is not None:
                self._sync_velocity = (seq.plan_velocity_xy, time.monotonic())
            r = out.result
            return SkillResult(r.result, r.reason_code, {**r.gate, **r.metrics}, released=bool(r.released))
        finally:
            self.skill.authority.release('dynamic_place')

    def _keep_sync_runner(self, goal_handle, deadline_s):
        """SafeRetreat(keep_sync=true) 구간: 마지막 DynamicPlace 계획 속도로 XY를 유지하며 안전 높이까지(REQ-ROB-010).

        단일 지령 권한은 정적 실행기(`safe_retreat`)가 잡는다. 계획 속도가 없거나 오래되면 `KeepSyncRetreat`가
        움직이지 않고 `PRECONDITION`(`SYNC_NOT_AVAILABLE`)으로 끝낸다.
        """
        def run(feedback):
            v = None
            if self._sync_velocity is not None:
                velocity, t = self._sync_velocity
                if time.monotonic() - t <= self.dyn_cfg.keep_sync_ref_max_age_s:
                    v = velocity
            retreat = KeepSyncRetreat(self.place_transport, v, self.dyn_cfg, deadline_s or 0.0,
                                      period_s=self.sync_cfg.period_s)
            return run_keep_sync_retreat(
                retreat, self, time.sleep, cancel=lambda: bool(goal_handle.is_cancel_requested),
                stop=lambda: bool(self._stop_requested or self.stop_latch.latched), feedback=feedback,
                alive=rclpy.ok)
        return run

    def _plans_since(self, last, fresh_s=None):
        now = time.monotonic()
        with self._inbox:
            items = list(self._plans)
            top = self._plan_seq
        if last is None:
            return [p for t, n, p in items if fresh_s is None or now - t <= fresh_s], top
        return [p for t, n, p in items if n > last], top

    def _on_plan(self, msg):
        sample = _plan_sample(msg)
        with self._inbox:
            self._plan_seq += 1
            self._plans.append((time.monotonic(), self._plan_seq, sample))

    def _on_tracked(self, msg):
        tracks = {int(t.track_id): TrackInfo(int(t.track_id), int(t.epoch), int(t.status),
                                             _from_time(t.header.stamp), _from_time(t.last_observed_stamp))
                  for t in msg.tracks}
        with self._inbox:
            self._tracks = tracks

    def _on_belt(self, msg):
        with self._inbox:
            self._belt = BeltInfo(_from_time(msg.header.stamp), bool(msg.steady))

    def _backend_abort(self) -> bool:
        abort = self._goal_abort
        return bool(self._stop_requested or (abort is not None and abort()))

    # ------------------------------------------------------------ Stop·Status·Heartbeat
    def _on_stop(self, request, response):
        self.stop_latch.latch(f'{int(request.requester)}:{request.reason_code}')
        self._stop_requested = True
        res = self.skill.stop(str(request.reason_code))
        if self.place_transport is not None:
            self.place_transport.stop()
        end = time.monotonic() + 2.0
        while self._active_goal and time.monotonic() < end:
            time.sleep(POLL_S)
        self.skill.authority.release('stop')
        stopped = bool(res.stopped) and not self._active_goal
        response.stopped = stopped
        response.result = int(Result.OK) if stopped else int(Result.FAULT)
        response.reason_code = '' if stopped else (res.reason_code or 'STOP_NOT_CONFIRMED')
        return response

    def _on_status(self):
        if not self._active or self.status_pub is None:
            return
        msg = RobotStatus(header=Header(frame_id='robot_base'))
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.backend is None:
            msg.connected = False
            msg.ready = False
            msg.protective_stop = False
            msg.tcp_pose = Pose(orientation=Quaternion(w=1.0))
            msg.in_workspace = False
            msg.holding = False
            msg.gripper_alive = False
        else:
            pos, quat = self.backend.read_tcp_pose()
            msg.connected = True
            msg.ready = self.stop_latch.ready(self._active, protective_stop=False)
            msg.protective_stop = False
            msg.tcp_pose = Pose(position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
                                orientation=Quaternion(x=float(quat[0]), y=float(quat[1]),
                                                       z=float(quat[2]), w=float(quat[3])))
            msg.holding = False
            msg.gripper_alive = True
        self.status_pub.publish(msg)

    def _on_heartbeat(self):
        if not self._active or self.hb_pub is None:
            return
        self._seq += 1
        code = self.stop_latch.heartbeat_code(protective_stop=False)
        msg = Heartbeat(node_name='robot_controller_node', lifecycle_state=self.state_id,
                        seq=self._seq, error_code=code,
                        detail='STOP_LATCHED' if code == ERROR_STOP_LATCHED else '')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.hb_pub.publish(msg)

    def _on_params(self, params):
        """작업자 해제(`stop_release_reason`). 실행 중 goal·보호정지·빈 사유는 거부하고 유지한다."""
        for p in params:
            if p.name != 'stop_release_reason':
                continue
            with self._lock:
                was_latched = self.stop_latch.latched
                code = self.stop_latch.release(str(p.value), goal_active=self._active_goal,
                                               protective_stop=False)
            if code:
                self.get_logger().warning(f'정지 유지 해제 거부: {code}')
                return SetParametersResult(successful=False, reason=code)
            if not was_latched:
                continue                  # 정지 유지 중이 아니면 아무것도 하지 않는다(SPEC-010 4.5절)
            reset = getattr(self.backend, 'reset_stop', None)
            if reset is not None:
                reset()
                reset_place = getattr(self.place_transport, 'reset_stop', None)
                if reset_place is not None:
                    reset_place()
                if self.stop_latch.latched:       # 해제와 reset 사이에 새 Stop이 왔으면 다시 멈춘다
                    self.backend.stop()
                    if self.place_transport is not None:
                        self.place_transport.stop()
            self.get_logger().info(f'정지 유지 해제: {str(p.value).strip()}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ 시각 계약
    def ros_now(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9

    def steady_now(self) -> float:
        return time.monotonic()


def _dynamic_feedback(fb) -> DynamicPlace.Feedback:
    msg = DynamicPlace.Feedback(phase=fb.phase, e_xy=float(fb.e_xy_m), v_rel=float(fb.v_rel_mps),
                                plan_revision=int(fb.plan_revision), phase_done=bool(fb.phase_done))
    if fb.phase_done:
        msg.metric_keys = list(fb.metrics.keys())
        msg.metric_values = [float(v) for v in fb.metrics.values()]
    return msg


def _failed(detail: str):
    from .skill_logic import SkillResult
    return SkillResult(Result.FAULT, 'INTERNAL_ERROR', {}, None)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
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
