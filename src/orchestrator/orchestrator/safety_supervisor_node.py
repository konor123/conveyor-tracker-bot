"""safety_supervisor_node: 독립 안전 감시 노드.

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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .common import SteadyClock
from .node_shutdown import ShutdownGuard
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
    # 진단 상태의 기본값(생성자를 거치지 않는 시험 대역도 같은 경로를 쓴다)
    _t_start = 0.0
    _last_tick = None
    _ctrl_last_seq = None
    _ctrl_beats = 0
    _ctrl_lost_at = None
    _stop_rejects = 0           # 같은 장애 구간에서 이어진 `stopped=false` 응답 수
    STOP_RETRY_S = 0.1          # 서비스 부재·호출 예외 뒤 재시도 간격, `stopped=false` 뒤 첫 재시도 간격
    STOP_RETRY_MAX_S = 1.0      # `stopped=false`가 이어질 때 두 배씩 늘리는 재시도 간격의 상한
    STOP_REJECT_WARN_S = 5.0    # 연속 거부 경고 간격(첫 거부는 바로 경고)
    _reject_warn_at = 0.0

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
        # 감시 타이머는 steady 시계
        self.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        # 진단(판정에 쓰지 않음): controller heartbeat 손실을 낸 순간의 상태와, 다시 받았을 때 seq가 이어지는지.
        # seq가 이어지면 controller는 계속 냈고 전달·처리가 늦었다. 건너뛰면 controller 쪽 발행이 멈췄다.
        self._t_start = self.steady_clock.steady_now()
        self._last_tick = None
        self._ctrl_last_seq = None
        self._ctrl_beats = 0
        self._ctrl_lost_at = None

    @staticmethod
    def _default_config() -> str:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('orchestrator'), 'config', 'orchestrator.yaml')

    def _on_hb(self, msg: Heartbeat) -> None:
        self.monitor.on_heartbeat(msg.node_name, msg.error_code)
        if msg.node_name == 'robot_controller_node':
            if self._ctrl_lost_at is not None:
                silent = self.steady_clock.steady_now() - self._ctrl_lost_at
                prev = self._ctrl_last_seq
                missing = (int(msg.seq) - prev - 1) if prev is not None else -1
                self.get_logger().warning(
                    f'controller heartbeat 재수신: 손실 판정 {silent:.3f} s 뒤, seq {prev}→{msg.seq} '
                    f'(빠진 seq {missing}), 발행 뒤 수신까지 {self._stamp_age(msg)}')
                self._ctrl_lost_at = None
            self._ctrl_last_seq = int(msg.seq)
            self._ctrl_beats += 1
        if msg.error_code == ERROR_PROTECTIVE_STOP:
            self._tick()

    def _stamp_age(self, msg) -> str:
        """진단: 메시지 header stamp부터 지금까지의 ROS 시간. 작으면 발행이 늦었고, 크면 전달·처리가 늦었다."""
        try:
            st = msg.header.stamp
            age = self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9)
        except Exception:
            return 'n/a'
        return f'{age:.3f} s'

    def _on_status(self, msg: RobotStatus) -> None:
        self.monitor.on_robot_status(msg.connected, msg.protective_stop)
        if msg.protective_stop or not msg.connected:
            self._tick()

    def _on_task(self, msg: TaskStatus) -> None:
        self.task_status = msg

    def _tick(self) -> None:
        # Fault detection and delivery confirmation have separate lifetimes.
        latched = self.monitor.check()
        reason = self.monitor.active_fault()
        now = self.steady_clock.steady_now()
        tick_gap = None if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        if latched == 'CONTROLLER_HEARTBEAT_LOST':
            self._ctrl_lost_at = now
            age = self.monitor._hb.age('robot_controller_node')
            gap = f'{tick_gap:.3f}' if tick_gap is not None else 'n/a'
            self.get_logger().warning(
                f'CONTROLLER_HEARTBEAT_LOST: 마지막 수신 {age:.3f} s 전, 받은 개수 {self._ctrl_beats}, '
                f'마지막 seq {self._ctrl_last_seq}, 직전 감시 주기 간격 {gap} s, 기동 뒤 {now - self._t_start:.3f} s')
        if self._stop_future is not None:
            if not self._stop_future.done():
                if now >= getattr(self, '_stop_warn_at', now + 1.):
                    self.get_logger().warning('Stop response pending; no overlapping RPC permitted')
                    self._stop_warn_at = now + 1.
                return  # never overlap Stop RPCs, including after fault recovery
            delay = self.STOP_RETRY_S
            try:
                response = self._stop_future.result()
                self._stop_confirmed = bool(response and response.stopped)
                if self._stop_confirmed:
                    self._stop_rejects = 0
                else:
                    # 전달됐고 controller는 정지 유지를 걸었지만 정지를 확인하지 못했다(예: 실기 연결 없는 backend).
                    # 확인될 때까지 재시도하되 간격을 두 배씩 늘려 상한 1 s로 두고, 경고는 처음과 5 s마다 남긴다.
                    self._stop_rejects += 1
                    delay = min(self.STOP_RETRY_S * 2 ** (self._stop_rejects - 1), self.STOP_RETRY_MAX_S)
                    if self._stop_rejects == 1 or now >= self._reject_warn_at:
                        self.get_logger().warning(
                            f'Stop rejected: stopped=false (사유 {self._stop_reason}, 연속 {self._stop_rejects}회, '
                            f'다음 재시도 {delay:.1f} s 뒤)')
                        self._reject_warn_at = now + self.STOP_REJECT_WARN_S
            except Exception as exc:
                self.get_logger().warning(f'Stop failed: {exc}')
                self._stop_confirmed = False
            self._stop_future = None
            self._next_stop_try = now + delay
        if reason is None:
            self._stop_reason, self._stop_confirmed, self._stop_rejects = None, False, 0
            return
        if reason != self._stop_reason:
            self._stop_reason, self._stop_confirmed, self._stop_rejects = reason, False, 0
            self._next_stop_try = min(self._next_stop_try, now)    # 새 사유는 늘어난 간격을 기다리지 않는다
        if not self._stop_confirmed and now >= self._next_stop_try:
            self._request_stop(reason)
            self._next_stop_try = now + self.STOP_RETRY_S

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
    guard = ShutdownGuard()            # rclpy.init 전
    rclpy.init(args=args)
    node = None
    try:
        node = SafetySupervisorNode()
        rclpy.spin(node)
    except BaseException as exc:
        if not guard.is_shutdown(exc):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        guard.close()


if __name__ == '__main__':
    main()
