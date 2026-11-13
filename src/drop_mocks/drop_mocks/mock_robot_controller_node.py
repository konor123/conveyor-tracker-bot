"""mock_robot_controller_node(SPEC-700 4.5절): skill 액션 5종 + Stop + RobotStatus + Heartbeat 대역.

`Stop`은 실제 노드와 같은 정지 유지를 건다(SPEC-010 4.5절, REQ-MCK-008). 해제는 `stop_release_reason`.
`tcp_lag_s` > 0이면 DynamicPlace 동적 phase(APPROACH~RELEASE)에서 goal의 계획(`drop/intercept_plan`) 목표 궤적을
1차 지연으로 따르는 TCP를 `robot/status`와 SYNCHRONIZE `e_xy`·`v_rel`에 쓴다(REQ-MCK-009, `tcp_lag.py`).
기준 궤적은 그 goal에서 처음 쓴 revision으로 고정한다(revision 사이 잡음이 모델 속도로 번지지 않게).
그 계획을 받지 못했으면 대본 값을 쓴다.
"""
from __future__ import annotations

import json
import threading
import time

import rclpy
from drop_interfaces.action import DynamicPlace, MoveToPose, PickStone, PlaceStone, SafeRetreat
from drop_interfaces.msg import Heartbeat, InterceptPlan, RobotStatus
from drop_interfaces.srv import Stop
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.event_handler import SubscriptionEventCallbacks
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from .ros_common import (DATA_QOS, ERROR_PROTECTIVE_STOP, ERROR_STOP_LATCHED, HEARTBEAT_PERIOD_S, HEARTBEAT_QOS,
                         MockLifecycleNode, from_time, no_pub_events, spin_mock, steady_clock)
from .scenario_engine import GRIPPER_SKILLS, MODE_PREDICTED, RESULT
from .tcp_lag import DEFAULT_TCP_LAG_S, PlanRef, make_lag

ACTIONS = {
    'PickStone': (PickStone, 'robot/pick_stone', ('APPROACH', 'DESCEND', 'GRIP', 'LIFT')),
    'MoveToPose': (MoveToPose, 'robot/move_to_pose', ('MOVE',)),
    'PlaceStone': (PlaceStone, 'robot/place_stone', ('APPROACH', 'DESCEND', 'RELEASE', 'RETREAT')),
    'DynamicPlace': (DynamicPlace, 'robot/dynamic_place',
                     ('APPROACH', 'SYNCHRONIZE', 'DESCEND', 'RELEASE', 'RETREAT')),
    'SafeRetreat': (SafeRetreat, 'robot/safe_retreat', ('RETREAT',)),
}
PLACE_SKILLS = ('PlaceStone', 'DynamicPlace')
WORKSPACE_PHASES = ('APPROACH', 'SYNCHRONIZE', 'DESCEND', 'RELEASE')
POLL_S = 0.005
SYNC_FEEDBACK_PERIOD_S = 0.04
PLAN_CACHE = 16                    # 기억하는 계획 수(plan_id별 마지막 revision)


class MockRobotControllerNode(MockLifecycleNode):
    def __init__(self, **kwargs):
        super().__init__('mock_robot_controller_node', **kwargs)
        # 파라미터 선언도 설정 callback(_on_params)을 부르므로 그 callback이 읽는 상태를 먼저 만든다
        self._lock = threading.Lock()
        self._busy = False
        self._stop_latched = False         # Stop 정지 유지(작업자 해제까지, t0 재설정으로 풀리지 않음)
        self.declare_parameter('stop_count', 0)
        self.declare_parameter('last_stop_requester', 0)
        self.declare_parameter('last_stop_reason', '')
        self._stop_count = 0
        self.declare_parameter('cancel_count', 0)
        self.declare_parameter('cancel_received_steady_s', 0.0)
        self.declare_parameter('stop_history_json', '[]')
        self.declare_parameter('stop_release_reason', '')
        self.declare_parameter('skill_run_count', 0)
        # TCP 1차 지연 시정수 [s]. 0 = 모델 끔(대본 값). 기본 29 ms = OFF-2 speedl 100 Hz 부하 지연(측정 근거 모델)
        self.declare_parameter('tcp_lag_s', DEFAULT_TCP_LAG_S)
        self._plans: dict[int, PlanRef] = {}
        self._model_pose = None            # 동적 phase 동안 지연 모델 TCP 위치(robot_base)
        self._cancel_count = 0
        self._stop_history = []
        self._observer_lock = threading.Lock()
        self._active_skill: str | None = None
        self._active_phase: str | None = None
        self._holding = False
        self._stop_requested = False
        self._pstop_latched = False        # skill 대본으로 시작한 보호정지(노드 재시작까지 유지)
        self._skill_runs = 0
        self._calls = {name: 0 for name in ACTIONS}
        self._seq = 0
        self._last_hb_mono = 0.0
        self.refuse_if_provided(['robot/stop'])
        group = ReentrantCallbackGroup()
        self._servers = [
            ActionServer(self, typ, topic, execute_callback=self._executor_for(name),
                         goal_callback=self._on_goal, cancel_callback=self._on_cancel,
                         callback_group=group)
            for name, (typ, topic, _) in ACTIONS.items()]
        self.create_service(Stop, 'robot/stop', self._on_stop, callback_group=group)
        self.create_subscription(InterceptPlan, 'drop/intercept_plan', self._on_plan, DATA_QOS,
                                 event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False),
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.status_pub = self.create_publisher(RobotStatus, 'robot/status', HEARTBEAT_QOS,
                                                event_callbacks=no_pub_events())
        self.hb_pub = self.create_publisher(Heartbeat, 'system/heartbeat', HEARTBEAT_QOS,
                                            event_callbacks=no_pub_events())
        # 상태·heartbeat는 steady 시계로 발행한다(REQ-ABT-009)
        self.create_timer(HEARTBEAT_PERIOD_S, self._on_status, clock=steady_clock())
        self.create_timer(HEARTBEAT_PERIOD_S, self._on_heartbeat, clock=steady_clock())
        self.autostart()

    def protective_stop(self) -> bool:
        return self._pstop_latched or self.engine.protective_stop(self.scenario_t())

    def _on_plan(self, msg):
        p, v = msg.target_tcp_pose.position, msg.target_velocity
        with self._lock:
            self._plans[int(msg.plan_id)] = PlanRef(from_time(msg.release_time), (p.x, p.y, p.z), (v.x, v.y, v.z))
            while len(self._plans) > PLAN_CACHE:
                self._plans.pop(next(iter(self._plans)))

    # ------------------------------------------------------------ goal 수락
    def _on_goal(self, goal_request):
        with self._lock:
            if not self.active or self._busy:   # 단일 지령 권한: 진행 중이면 거부
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        # Observation only: retain the existing ACCEPT behavior, including hang scripts.
        with self._observer_lock:
            self._cancel_count += 1
            self.set_parameters([
                Parameter('cancel_count', value=self._cancel_count),
                Parameter('cancel_received_steady_s', value=time.monotonic()),
            ])
        return CancelResponse.ACCEPT

    def _executor_for(self, name):
        return lambda goal_handle: self._execute(name, goal_handle)

    def _execute(self, name, goal_handle):
        try:
            return self._run(name, goal_handle)
        finally:
            with self._lock:
                self._active_skill = None
                self._active_phase = None
                self._busy = False

    def _run(self, name, goal_handle):
        typ, _, phases = ACTIONS[name]
        goal = goal_handle.request
        if self._stop_latched:
            # 정지 유지 중: 대본·feedback 없이 STOPPED로 끝낸다(SPEC-010 4.5절)
            goal_handle.abort()
            return typ.Result(result=RESULT['CANCELED'], reason_code='STOPPED')
        with self._observer_lock:
            self._skill_runs += 1
            self.set_parameters([Parameter('skill_run_count', value=self._skill_runs)])
        reply = self.engine.skill(name, self._calls[name])
        self._calls[name] += 1
        t_start = self.scenario_t()
        uses_gripper = name in GRIPPER_SKILLS or (name == 'SafeRetreat' and goal.place_back)
        duration = reply.duration_s + (self.engine.gripper_delay(t_start) if uses_gripper else 0.0)
        if name == 'SafeRetreat' and goal.place_back:
            phases = phases + ('PLACE_BACK',)
        predicted = name == 'DynamicPlace' and getattr(goal, 'mode', 0) == MODE_PREDICTED
        deadline = from_time(goal.deadline)
        sent_phase = -1
        last_sync_fb = -1.0
        tau = float(self.get_parameter('tcp_lag_s').value) if name == 'DynamicPlace' else 0.0
        self._lag = None                   # (TcpLag, PlanRef): goal의 계획을 받은 뒤부터
        with self._lock:
            self._active_skill = name
            self._active_phase = phases[0]
        while True:
            self._advance_lag(tau, goal, phases[max(sent_phase, 0)])
            if time.monotonic() - self._last_hb_mono >= HEARTBEAT_PERIOD_S * 0.5:
                self._on_heartbeat()
                self._on_status()
            if self._stop_requested:
                return self._finish(goal_handle, typ, name, goal, reply, RESULT['CANCELED'], 'STOPPED')
            if self.protective_stop():
                return self._finish(goal_handle, typ, name, goal, reply, RESULT['FAULT'], 'PROTECTIVE_STOP')
            if goal_handle.is_cancel_requested and reply.cancel_ack:
                return self._finish(goal_handle, typ, name, goal, reply, RESULT['CANCELED'], 'CANCELED')
            if not rclpy.ok():
                return typ.Result(result=RESULT['FAULT'], reason_code='SHUTDOWN')
            if reply.hang:
                time.sleep(POLL_S)
                continue
            elapsed = self.scenario_t() - t_start
            if reply.protective_stop_at_ratio is not None and elapsed >= reply.protective_stop_at_ratio * duration:
                self._pstop_latched = True
                self._on_status()
                self._on_heartbeat()
                continue
            if deadline > 0.0 and self.ros_now() >= deadline:
                return self._finish(goal_handle, typ, name, goal, reply, RESULT['TIMEOUT'], 'DEADLINE')
            if elapsed >= duration:
                while sent_phase < len(phases) - 1:
                    if sent_phase >= 0:
                        self._close_phase(goal_handle, typ, name, phases[sent_phase], reply, predicted)
                    sent_phase += 1
                    self._enter_phase(goal_handle, typ, name, phases[sent_phase], reply)
                if sent_phase >= 0:
                    self._close_phase(goal_handle, typ, name, phases[sent_phase], reply, predicted)
                return self._finish(goal_handle, typ, name, goal, reply, reply.result, reply.reason_code)
            idx = min(int(elapsed / duration * len(phases)), len(phases) - 1) if duration > 0 else 0
            while sent_phase < idx:
                if sent_phase >= 0:
                    self._close_phase(goal_handle, typ, name, phases[sent_phase], reply, predicted)
                sent_phase += 1
                self._enter_phase(goal_handle, typ, name, phases[sent_phase], reply)
                last_sync_fb = elapsed
            if name == 'DynamicPlace' and phases[sent_phase] == 'SYNCHRONIZE':
                if elapsed - last_sync_fb >= SYNC_FEEDBACK_PERIOD_S:
                    last_sync_fb = elapsed
                    goal_handle.publish_feedback(
                        self._feedback(typ, name, 'SYNCHRONIZE', reply, phase_done=False, predicted=predicted,
                                       model=self._lag_errors()))
            time.sleep(POLL_S)

    # ------------------------------------------------------------ TCP 1차 지연 모델(REQ-MCK-009)
    def _advance_lag(self, tau, goal, phase):
        """동적 phase에서 지연 모델을 지금 ROS 시각까지 진행하고 status TCP로 쓴다. 그 밖에서는 대본 자세."""
        if not tau > 0.0 or phase not in WORKSPACE_PHASES:      # 0·음수·NaN = 모델 끔
            if self._model_pose is not None:
                with self._lock:
                    self._model_pose = None
            return
        now = self.ros_now()
        if self._lag is None:
            with self._lock:
                ref = self._plans.get(int(goal.plan_id))
            if ref is None:
                return                     # 계획을 아직 못 받음: 대본 값
            start = self._model_pose if self._model_pose is not None else self.engine.robot_status_pose()[:3]
            self._lag = (make_lag(tau, start, now), ref)
        lag, ref = self._lag                # 처음 쓴 revision으로 고정한 기준
        lag.advance(now, ref)
        with self._lock:
            self._model_pose = tuple(float(v) for v in lag.pos)

    def _lag_errors(self):
        """지연 모델의 (e_xy, v_rel) 또는 None(모델 없음)."""
        if getattr(self, '_lag', None) is None:
            return None
        lag, ref = self._lag
        return lag.errors(ref)

    # ------------------------------------------------------------ 결과·feedback
    def _enter_phase(self, goal_handle, typ, name, phase, reply):
        with self._lock:
            self._active_phase = phase
        goal_handle.publish_feedback(self._feedback(typ, name, phase, reply, phase_done=False,
                                                    model=self._lag_errors()))

    def _close_phase(self, goal_handle, typ, name, phase, reply, predicted):
        if name in PLACE_SKILLS:
            goal_handle.publish_feedback(
                self._feedback(typ, name, phase, reply, phase_done=True, predicted=predicted,
                               model=self._lag_errors()))
            if phase == 'RELEASE':
                with self._lock:
                    self._holding = False

    @staticmethod
    def _feedback(typ, name, phase, reply, phase_done: bool = False, predicted: bool = False, model=None):
        """`model` = 지연 모델의 (e_xy, v_rel). 대본의 SYNCHRONIZE `phase_metrics`가 있으면 대본이 우선한다."""
        fb = typ.Feedback(phase=phase)
        sync_ov = reply.phase_metrics.get('SYNCHRONIZE') or {}
        e_xy, v_rel = model if model is not None else (reply.values['gate_e_xy_m'], reply.values['gate_v_rel_mps'])
        if name == 'DynamicPlace':
            fb.e_xy = float(sync_ov.get('e_xy_m', e_xy))
            fb.v_rel = float(sync_ov.get('v_rel_mps', v_rel))
        if name in PLACE_SKILLS:
            fb.phase_done = phase_done
            if phase_done:
                keys, values = reply.metrics_for(phase, predicted=predicted)
                if name == 'DynamicPlace' and phase == 'SYNCHRONIZE' and model is not None:
                    measured = {'e_xy_m': e_xy, 'v_rel_mps': v_rel}
                    values = [float(sync_ov.get(k, measured[k])) if k in measured else v
                              for k, v in zip(keys, values)]
                fb.metric_keys, fb.metric_values = keys, values
        return fb

    def _finish(self, goal_handle, typ, name, goal, reply, result, reason):
        ok = result == RESULT['OK']
        v = reply.values
        res = typ.Result(result=result, reason_code=reason)
        if name == 'PickStone':
            res.grip_width, res.grip_current = v['grip_width_m'], v['grip_current_ma']
            res.holding = v.get('holding', ok)
            with self._lock:
                self._holding = bool(res.holding)
        elif name == 'MoveToPose':
            res.pose_error = v['pose_error_m']
        elif name == 'PlaceStone':
            res.released, res.release_error = v.get('released', ok), v['release_error_m']
        elif name == 'DynamicPlace':
            res.released = v.get('released', ok)
            res.gate_e_xy, res.gate_v_rel = v['gate_e_xy_m'], v['gate_v_rel_mps']
            res.gate_t_since_obs, res.gate_position_std = v['gate_t_since_obs_s'], v['gate_position_std_m']
        elif name == 'SafeRetreat':
            res.stone_placed_back = ok and goal.place_back
            if goal.place_back and ok:
                with self._lock:
                    self._holding = False
        with self._lock:
            self._active_skill = None
            self._active_phase = None
            self._busy = False
        self._on_status()
        self._on_heartbeat()
        if result == RESULT['CANCELED'] and goal_handle.is_cancel_requested and reason == 'CANCELED':
            goal_handle.canceled()
        elif ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return res

    # ------------------------------------------------------------ Stop·Status·Heartbeat
    def _on_stop(self, req, res):
        with self._observer_lock:
            self._stop_history.append({
                'requester': int(req.requester), 'reason_code': str(req.reason_code),
                'received_steady_s': time.monotonic(),
            })
            self.set_parameters([
                Parameter('stop_history_json', value=json.dumps(self._stop_history)),
            ])
        self._stop_latched = True
        self._stop_requested = True
        try:
            end = time.monotonic() + 2.0
            while self._busy and time.monotonic() < end:
                time.sleep(POLL_S)
        finally:
            self._stop_requested = False
        self._stop_count += 1
        self.set_parameters([
            Parameter('stop_count', value=self._stop_count),
            Parameter('last_stop_requester', value=int(req.requester)),
            Parameter('last_stop_reason', value=str(req.reason_code)),
        ])
        self.get_logger().info(f'STOP_CALLED requester={req.requester} reason={req.reason_code}')
        res.stopped = not self._busy
        res.result = RESULT['OK'] if res.stopped else RESULT['FAULT']
        res.reason_code = '' if res.stopped else 'STOP_TIMEOUT'
        return res

    def _on_params(self, params):
        """작업자 해제(`stop_release_reason`). 실행 중 goal·보호정지·빈 사유는 거부하고 유지한다."""
        for p in params:
            if p.name != 'stop_release_reason':
                continue
            reason = str(p.value).strip()
            with self._lock:
                code = ('' if not self._stop_latched else 'REASON_REQUIRED' if not reason else
                        'GOAL_ACTIVE' if self._busy else 'PROTECTIVE_STOP' if self.protective_stop() else '')
                if not code:
                    self._stop_latched = False
            if code:
                return SetParametersResult(successful=False, reason=code)
            self.get_logger().info(f'STOP_RELEASED reason={reason}')
        return super()._on_params(params)

    def _on_status(self):
        if not self.active:
            return
        t = self.scenario_t()
        pstop = self.protective_stop()
        with self._lock:
            holding = self._holding
            in_ws = self._active_skill in PLACE_SKILLS and self._active_phase in WORKSPACE_PHASES
        x, y, z, qx, qy, qz, qw = self.engine.robot_status_pose()
        with self._lock:
            model = self._model_pose
        if model is not None:
            x, y, z = model
        st = RobotStatus(
            connected=True,
            ready=self.active and not pstop and not self._stop_latched,
            protective_stop=pstop,
            tcp_pose=Pose(position=Point(x=x, y=y, z=z),
                          orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)),
            in_workspace=in_ws,
            holding=holding,
            gripper_alive=True,
            gripper_response_s=0.1 + self.engine.gripper_delay(t),
            gripper_error_bits=0,
            gripper_open=not holding,
            gripper_width=0.0175 if holding else 0.022,
            gripper_current=150.0 if holding else 0.0,
        )
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'robot_base'
        self.status_pub.publish(st)

    def _on_heartbeat(self):
        with self._lock:
            self._last_hb_mono = time.monotonic()
            if not self.engine.heartbeat_alive('robot_controller', self.scenario_t()):
                return
            self._seq += 1
            seq = self._seq
        pstop = self.protective_stop()
        code, detail = ((ERROR_PROTECTIVE_STOP, 'PROTECTIVE_STOP') if pstop else
                        (ERROR_STOP_LATCHED, 'STOP_LATCHED') if self._stop_latched else (0, ''))
        hb = Heartbeat(node_name='robot_controller_node', lifecycle_state=self.state_id, seq=seq,
                       error_code=code, detail=detail)
        hb.header.stamp = self.get_clock().now().to_msg()
        self.hb_pub.publish(hb)


def main(args=None):
    spin_mock(MockRobotControllerNode, 'mock_robot_controller_node', num_threads=3, args=args)


if __name__ == '__main__':
    main()
