"""safety_supervisor_node(PLAN.md 2.1·2.2·5.1·5.3절, SPEC-110): 독립 안전 감시 노드.

Heartbeat·RobotStatus·TaskStatus를 감시하고 L4 조건(보호정지·로봇 통신 단절·robot_controller
heartbeat 손실)을 검출하면 robot/stop(Stop.srv, requester=SUPERVISOR)만 호출한다.
로봇 모션 지령 토픽·액션은 소유하지 않는다(단일 지령 권한).
"""
from __future__ import annotations

import os
import time

import rclpy
import yaml
from drop_interfaces.msg import Heartbeat, RobotStatus, TaskStatus
from drop_interfaces.srv import Stop
from rclpy.clock import Clock, ClockType
from .ros_qos import no_sub_events
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .common import SteadyClock
from .watchdogs import HeartbeatMonitor

ERROR_PROTECTIVE_STOP = 100
HB_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
DATA_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class _MonoClock:
    def steady_now(self) -> float:
        return time.monotonic()


class SupervisorMonitor:
    """순수 L4 감시 판정기. 동일 장애 구간에서는 1회만 정지 사유를 내고 정상 복구 시 해제한다."""

    def __init__(self, heartbeat_timeout_s: float, clock: SteadyClock):
        self._hb = HeartbeatMonitor(heartbeat_timeout_s, clock)
        self._controller_seen = False
        self._hb_error_code = 0
        self._connected = True
        self._protective_stop = False
        self._latched: str | None = None

    def on_heartbeat(self, node_name: str, error_code: int = 0) -> None:
        self._hb.beat(node_name)
        if node_name == 'robot_controller_node':
            self._controller_seen = True
            self._hb_error_code = int(error_code)

    def on_robot_status(self, connected: bool, protective_stop: bool) -> None:
        self._connected = bool(connected)
        self._protective_stop = bool(protective_stop)

    def active_fault(self) -> str | None:
        if self._protective_stop or self._hb_error_code == ERROR_PROTECTIVE_STOP:
            return 'PROTECTIVE_STOP'
        if not self._connected:
            return 'ROBOT_COMM_LOST'
        if self._controller_seen and self._hb.lost('robot_controller_node'):
            return 'CONTROLLER_HEARTBEAT_LOST'
        return None

    def check(self) -> str | None:
        reason = self.active_fault()
        if reason is None:
            self._latched = None
            return None
        if reason == self._latched:
            return None
        self._latched = reason
        return reason


class SafetySupervisorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('safety_supervisor_node', **kwargs)
        self.declare_parameter('config', '')
        path = self.get_parameter('config').value or self._default_config()
        with open(path, encoding='utf-8') as fh:
            cfg = yaml.safe_load(fh)
        hb_timeout = float(cfg['watchdog']['heartbeat_timeout_s'])
        self.steady_clock = _MonoClock()
        self.monitor = SupervisorMonitor(hb_timeout, self.steady_clock)
        self.task_status: TaskStatus | None = None
        self._stop_future = None
        self._stop_reason = None
        self._stop_confirmed = False
        self._next_stop_try = 0.
        self.stop_calls: list[tuple[str, float]] = []
        self._stop_cli = self.create_client(Stop, 'robot/stop')
        self.create_subscription(Heartbeat, 'system/heartbeat', self._on_hb, HB_QOS, event_callbacks=no_sub_events())
        self.create_subscription(RobotStatus, 'robot/status', self._on_status, DATA_QOS,
                                 event_callbacks=no_sub_events())
        self.create_subscription(TaskStatus, 'task/status', self._on_task, STATUS_QOS, event_callbacks=no_sub_events())
        # 감시 타이머는 steady 시계(REQ-ABT-009)
        self.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _default_config() -> str:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('orchestrator'), 'config', 'orchestrator.yaml')

    def _on_hb(self, msg: Heartbeat) -> None:
        self.monitor.on_heartbeat(msg.node_name, msg.error_code)
        if msg.error_code == ERROR_PROTECTIVE_STOP:
            self._tick()

    def _on_status(self, msg: RobotStatus) -> None:
        self.monitor.on_robot_status(msg.connected, msg.protective_stop)
        if msg.protective_stop or not msg.connected:
            self._tick()

    def _on_task(self, msg: TaskStatus) -> None:
        self.task_status = msg

    def _tick(self) -> None:
        # Fault detection and delivery confirmation have separate lifetimes.
        self.monitor.check()
        reason = self.monitor.active_fault()
        now = self.steady_clock.steady_now()
        if self._stop_future is not None:
            if not self._stop_future.done():
                if now >= getattr(self, '_stop_warn_at', now + 1.):
                    self.get_logger().warning('Stop response pending; no overlapping RPC permitted')
                    self._stop_warn_at = now + 1.
                return  # never overlap Stop RPCs, including after fault recovery
            try:
                response = self._stop_future.result()
                self._stop_confirmed = bool(response and response.stopped)
                if not self._stop_confirmed:
                    self.get_logger().warning('Stop rejected: stopped=false')
            except Exception as exc:
                self.get_logger().warning(f'Stop failed: {exc}')
                self._stop_confirmed = False
            self._stop_future = None
            self._next_stop_try = now + 0.1
        if reason is None:
            self._stop_reason, self._stop_confirmed = None, False
            return
        if reason != self._stop_reason:
            self._stop_reason, self._stop_confirmed = reason, False
        if not self._stop_confirmed and now >= self._next_stop_try:
            self._request_stop(reason)
            self._next_stop_try = now + 0.1

    def _request_stop(self, reason: str) -> None:
        self.stop_calls.append((reason, time.monotonic()))
        if not self._stop_cli.service_is_ready():
            self.get_logger().warning('Stop service unavailable; retry pending')
            return
        req = Stop.Request(requester=Stop.Request.SUPERVISOR, reason_code=reason)
        try:
            self._stop_future = self._stop_cli.call_async(req)
            self._stop_warn_at = self.steady_clock.steady_now() + 1.
        except Exception as exc:
            self.get_logger().warning(f'Stop send failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
