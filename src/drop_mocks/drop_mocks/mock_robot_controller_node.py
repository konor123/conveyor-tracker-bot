"""mock_robot_controller_node(SPEC-700 4.5절): skill 액션 5종 + Stop + RobotStatus + Heartbeat 대역."""
from __future__ import annotations

import threading
import time

import rclpy
from drop_interfaces.action import DynamicPlace, MoveToPose, PickStone, PlaceStone, SafeRetreat
from drop_interfaces.msg import Heartbeat, RobotStatus
from drop_interfaces.srv import Stop
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.parameter import Parameter

from .ros_common import (ERROR_PROTECTIVE_STOP, HEARTBEAT_PERIOD_S, HEARTBEAT_QOS,
                         MockLifecycleNode, from_time)
from .scenario_engine import GRIPPER_SKILLS, MODE_PREDICTED, RESULT

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


class MockRobotControllerNode(MockLifecycleNode):
    def __init__(self, **kwargs):
        super().__init__('mock_robot_controller_node', **kwargs)
        self.declare_parameter('stop_count', 0)
        self.declare_parameter('last_stop_requester', 0)
        self.declare_parameter('last_stop_reason', '')
        self._stop_count = 0
        self._lock = threading.Lock()
        self._busy = False
        self._active_skill: str | None = None
        self._active_phase: str | None = None
        self._holding = False
        self._stop_requested = False
        self._pstop_latched = False        # skill 대본으로 시작한 보호정지(노드 재시작까지 유지)
        self._calls = {name: 0 for name in ACTIONS}
        self._seq = 0
        self._last_hb_mono = 0.0
        self.refuse_if_provided(['robot/stop'])
        group = ReentrantCallbackGroup()
        self._servers = [
            ActionServer(self, typ, topic, execute_callback=self._executor_for(name),
                         goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=group)
            for name, (typ, topic, _) in ACTIONS.items()]
        self.create_service(Stop, 'robot/stop', self._on_stop, callback_group=group)
        self.status_pub = self.create_publisher(RobotStatus, 'robot/status', HEARTBEAT_QOS)
        self.hb_pub = self.create_publisher(Heartbeat, 'system/heartbeat', HEARTBEAT_QOS)
        self.create_timer(HEARTBEAT_PERIOD_S, self._on_status)
        self.create_timer(HEARTBEAT_PERIOD_S, self._on_heartbeat)
        self.autostart()

    def protective_stop(self) -> bool:
        return self._pstop_latched or self.engine.protective_stop(self.scenario_t())

    # ------------------------------------------------------------ goal 수락
    def _on_goal(self, goal_request):
        with self._lock:
            if not self.active or self._busy:   # 단일 지령 권한: 진행 중이면 거부
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

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
        with self._lock:
            self._active_skill = name
            self._active_phase = phases[0]
        while True:
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
                        self._feedback(typ, name, 'SYNCHRONIZE', reply, phase_done=False, predicted=predicted))
            time.sleep(POLL_S)

    # ------------------------------------------------------------ 결과·feedback
    def _enter_phase(self, goal_handle, typ, name, phase, reply):
        with self._lock:
            self._active_phase = phase
        goal_handle.publish_feedback(self._feedback(typ, name, phase, reply, phase_done=False))

    def _close_phase(self, goal_handle, typ, name, phase, reply, predicted):
        if name in PLACE_SKILLS:
            goal_handle.publish_feedback(
                self._feedback(typ, name, phase, reply, phase_done=True, predicted=predicted))
            if phase == 'RELEASE':
                with self._lock:
                    self._holding = False

    @staticmethod
    def _feedback(typ, name, phase, reply, phase_done: bool = False, predicted: bool = False):
        fb = typ.Feedback(phase=phase)
        if name == 'DynamicPlace':
            sync_ov = reply.phase_metrics.get('SYNCHRONIZE') or {}
            fb.e_xy = float(sync_ov.get('e_xy_m', reply.values['gate_e_xy_m']))
            fb.v_rel = float(sync_ov.get('v_rel_mps', reply.values['gate_v_rel_mps']))
        if name in PLACE_SKILLS:
            fb.phase_done = phase_done
            if phase_done:
                fb.metric_keys, fb.metric_values = reply.metrics_for(phase, predicted=predicted)
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

    def _on_status(self):
        if not self.active:
            return
        t = self.scenario_t()
        pstop = self.protective_stop()
        with self._lock:
            holding = self._holding
            in_ws = self._active_skill in PLACE_SKILLS and self._active_phase in WORKSPACE_PHASES
        x, y, z, qx, qy, qz, qw = self.engine.robot_status_pose()
        st = RobotStatus(
            connected=True,
            ready=self.active and not pstop,
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
        hb = Heartbeat(node_name='robot_controller_node', lifecycle_state=self.state_id, seq=seq,
                       error_code=ERROR_PROTECTIVE_STOP if pstop else 0,
                       detail='PROTECTIVE_STOP' if pstop else '')
        hb.header.stamp = self.get_clock().now().to_msg()
        self.hb_pub.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MockRobotControllerNode()
    except RuntimeError as e:
        get_logger('mock_robot_controller_node').error(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    executor = MultiThreadedExecutor(num_threads=3)
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
