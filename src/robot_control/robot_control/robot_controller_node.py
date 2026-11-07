"""robot_controller_node: 정적 skill action·Stop·Heartbeat·RobotStatus의 얇은 ROS 래퍼.

판단은 `skill_logic.py`가 한다. 기본 backend는 `disabled`이며 어떤 파라미터도
실기 연결을 암묵적으로 만들지 않는다. `backend:=fake`는 시뮬레이션 값만 쓴다.
DynamicPlace는 계획기·추종 제어가 준비될 때까지 PRECONDITION으로 닫는다.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from drop_interfaces.action import DynamicPlace, MoveToPose, PickStone, PlaceStone, SafeRetreat
from drop_interfaces.msg import Heartbeat, RobotStatus
from drop_interfaces.srv import Stop
from geometry_msgs.msg import Point, Pose, Quaternion
from lifecycle_msgs.msg import State
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from .fake_backend import SimulatedBackend
from .skill_logic import Result, StaticSkillCfg, StaticSkillExecutor

HEARTBEAT_PERIOD_S = 0.1
DATA_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
HB_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)
POLL_S = 0.01

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
        self._seq = 0
        self.cfg = self._load_cfg()
        self.backend_name = str(self.get_parameter('backend').value)
        self.backend = self._make_backend()
        self.skill = StaticSkillExecutor(self.backend, self.cfg, self, active=self._active)
        self._servers = []
        self.stop_service = None
        self.status_pub = None
        self.hb_pub = None
        self._timers = []

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
            'standby_pose': [],
            'safe_pose': [],
        }
        for name, value in defs.items():
            self.declare_parameter(name, value)

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
        )

    def _make_backend(self):
        if self.backend_name == 'disabled':
            return None
        if self.backend_name == 'fake':
            pose = ((float(self.get_parameter('fake_tcp_x_m').value),
                     float(self.get_parameter('fake_tcp_y_m').value),
                     float(self.get_parameter('fake_tcp_z_m').value)),
                    (0.0, 0.0, 0.0, 1.0))
            return SimulatedBackend(
                tcp_pose=pose,
                move_time_s=float(self.get_parameter('fake_move_time_s').value),
                grip_width_m=float(self.get_parameter('fake_grip_width_m').value),
                grip_current_ma=float(self.get_parameter('fake_grip_current_ma').value),
                cancel_requested=lambda: self._stop_requested,
            )
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
        for name, (typ, topic) in ACTIONS.items():
            self._servers.append(ActionServer(
                self, typ, topic, execute_callback=self._executor_for(name),
                goal_callback=self._on_goal,
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=group))
        self.stop_service = self.create_service(Stop, 'robot/stop', self._on_stop,
                                                callback_group=group)
        self.status_pub = self.create_publisher(RobotStatus, 'robot/status', DATA_QOS)
        self.hb_pub = self.create_publisher(Heartbeat, 'system/heartbeat', HB_QOS)
        self._timers = [self.create_timer(HEARTBEAT_PERIOD_S, self._on_status),
                        self.create_timer(HEARTBEAT_PERIOD_S, self._on_heartbeat)]
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._active = False
        self.skill.active = False
        if self.backend is not None:
            self.backend.stop()
        for server in self._servers:
            server.destroy()
        self._servers.clear()
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
        return GoalResponse.ACCEPT

    def _executor_for(self, name):
        return lambda goal_handle: self._execute(name, goal_handle)

    def _execute(self, name, goal_handle):
        with self._lock:
            self._active_goal = True
        self._stop_requested = False
        goal = goal_handle.request
        typ = ACTIONS[name][0]
        deadline_s = None
        raw_deadline = _from_time(goal.deadline) if hasattr(goal, 'deadline') else 0.0
        if raw_deadline > 0.0:
            deadline_s = raw_deadline

        def cancel():
            return bool(goal_handle.is_cancel_requested or self._stop_requested)

        def feedback(phase, metrics, done):
            fb = typ.Feedback(phase=phase)
            if name in ('PlaceStone', 'DynamicPlace'):
                fb.phase_done = bool(done)
                if done:
                    fb.metric_keys = list(metrics.keys())
                    fb.metric_values = [float(v) for v in metrics.values()]
            goal_handle.publish_feedback(fb)

        try:
            if name == 'MoveToPose':
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
                res = self.skill.safe_retreat(bool(goal.keep_sync), bool(goal.place_back),
                                              goal.place_back_pose, deadline_s, cancel, feedback)
            else:
                res = self.skill.dynamic_place()
        except Exception as exc:      # backend 예외도 집계해 성공으로 만들지 않는다
            self.get_logger().error(f'{name} 실행 예외: {exc!r}')
            res = _failed(str(exc))
        finally:
            with self._lock:
                self._active_goal = False

        result = typ.Result()
        result.result = int(res.result)
        result.reason_code = res.reason_code
        self._fill_result(name, result, res)
        if res.result == Result.OK:
            goal_handle.succeed()
        elif res.result == Result.CANCELED:
            goal_handle.canceled()
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
            result.stone_placed_back = False

    # ------------------------------------------------------------ Stop·Status·Heartbeat
    def _on_stop(self, request, response):
        self._stop_requested = True
        res = self.skill.stop(str(request.reason_code))
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
            msg.ready = self._active and not self._stop_requested
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
        msg = Heartbeat(node_name='robot_controller_node', lifecycle_state=self.state_id,
                        seq=self._seq, error_code=0, detail='')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.hb_pub.publish(msg)

    # ------------------------------------------------------------ 시각 계약
    def ros_now(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9


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
