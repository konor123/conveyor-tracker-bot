"""mock 노드 공통: 시나리오 로드, 시각 변환, QoS, lifecycle 상태(SPEC-700 2, 4.6절)."""
from __future__ import annotations

import os
import time

import rclpy
from builtin_interfaces.msg import Time
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .node_shutdown import ShutdownGuard
from .scenario_engine import ScenarioEngine
from .stall_watch import STALL_DUMP_ENV, StallWatch, stall_dump_s

DATA_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
HEARTBEAT_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)
HEARTBEAT_PERIOD_S = 0.1
EXCLUSIVE_WATCH_PERIOD_S = 0.5     # 기동 뒤 같은 역할 서버 감시 주기(REQ-MCK-007)
ERROR_PROTECTIVE_STOP = 100        # SPEC-010 3.4절
ERROR_STOP_LATCHED = 101           # SPEC-010 3.4·4.5절: Stop 정지 유지


def to_time(sec: float) -> Time:
    ns = int(round(sec * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def from_time(t: Time) -> float:
    return t.sec + t.nanosec * 1e-9


def steady_clock() -> Clock:
    """heartbeat·상태 발행 타이머용 steady 시계(SPEC-110 REQ-ABT-009). ROS 시각 정지·역행과 무관하게 돈다."""
    return Clock(clock_type=ClockType.STEADY_TIME)


def no_pub_events() -> PublisherEventCallbacks:
    """기본 QoS 이벤트 핸들러를 붙이지 않는다(SPEC-010 6절: QoS 불일치는 계약 시험으로 검출, 핸들러는 executor 부하)."""
    return PublisherEventCallbacks(use_default_callbacks=False)


def repo_path(value: str) -> str:
    """상대 경로는 저장소 루트 기준(SPEC-000 REQ-SYS-005, 원본 구현 calibration.repo_paths와 같은 규칙).

    루트: DROP_REPO_ROOT → 이 파일에서 위로 올라가며 src·records를 함께 가진 첫 폴더 → 현재 폴더.
    """
    value = os.path.expanduser(value)
    if not value or os.path.isabs(value):
        return value
    root = os.environ.get('DROP_REPO_ROOT')
    if not root:
        d = os.path.dirname(os.path.abspath(__file__))
        while True:
            if os.path.isdir(os.path.join(d, 'src')) and os.path.isdir(os.path.join(d, 'records')):
                root = d
                break
            parent = os.path.dirname(d)
            if parent == d:
                root = os.getcwd()
                break
            d = parent
    return os.path.normpath(os.path.join(os.path.expanduser(root), value))


def scenario_file(value: str) -> str:
    """시나리오 이름 → 설치된 scenarios/<이름>.yaml. 경로를 주면 상대 경로는 저장소 루트 기준(SPEC-000 REQ-SYS-005)."""
    if value.endswith('.yaml') or os.sep in value:
        return repo_path(value)
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('drop_mocks'), 'scenarios', value + '.yaml')


class MockLifecycleNode(LifecycleNode):
    """시나리오 파라미터·lifecycle 상태·deadline 검사를 가진 mock 노드 기반."""

    def __init__(self, name: str, **kwargs):
        super().__init__(name, **kwargs)
        self.declare_parameter('scenario', 'nominal_20mms')
        self.declare_parameter('seed', -1)
        self.declare_parameter('t0_s', 0.0)
        self.declare_parameter('duration_scale_ratio', -1.0)
        self.declare_parameter('drop_role', '')
        self.declare_parameter('drop_at_s', -1.0)
        self.declare_parameter('autostart', True)
        self.declare_parameter('exclusive_check_s', 1.0)
        self.conflict: str | None = None
        self.stall_watch: StallWatch | None = None     # spin_mock이 환경 변수를 보고 붙인다
        path = scenario_file(self.get_parameter('scenario').value)
        self.engine = ScenarioEngine.from_file(path, int(self.get_parameter('seed').value))
        scale = float(self.get_parameter('duration_scale_ratio').value)
        if scale > 0.0:
            self.engine._scale = scale
        drop_role = str(self.get_parameter('drop_role').value)
        drop_at = float(self.get_parameter('drop_at_s').value)
        if drop_role or drop_at >= 0.0:
            self._set_heartbeat_stop(drop_role or None, drop_at if drop_at >= 0.0 else None)
        if int(self.get_parameter('seed').value) < 0:
            self.set_parameters([Parameter('seed', value=self.engine.seed)])
        t0 = float(self.get_parameter('t0_s').value)
        self.t0 = t0 if t0 > 0.0 else self.ros_now()
        self.declare_parameter(
            'assumed_params', self.engine.assumed_params() or [''],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY, read_only=True))
        self.add_on_set_parameters_callback(self._on_params)
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        self.get_logger().info(
            f'scenario={self.engine.name} seed={self.engine.seed} t0_s={self.t0:.3f} file={path}')

    def _set_heartbeat_stop(self, role: str | None = None, t_s: float | None = None) -> None:
        ev = next((e for e in self.engine.events if e.get('type') == 'heartbeat_stop'), None)
        if ev is None:
            ev = {'type': 'heartbeat_stop', 'role': role or 'vision', 't_s': 0.0 if t_s is None else float(t_s)}
            self.engine.events.append(ev)
        else:
            if role is not None:
                ev['role'] = role
            if t_s is not None:
                ev['t_s'] = float(t_s)
        self.engine.events.sort(key=lambda e: e['t_s'])

    def _on_params(self, params):
        t0_updated = False
        for p in params:
            if p.name == 'seed' and int(p.value) >= 0:
                self.engine.seed = int(p.value)
            elif p.name == 't0_s':
                v = float(p.value)
                self.t0 = v if v > 0.0 else self.ros_now()
                t0_updated = True
                if hasattr(self, '_last_k'):
                    self._last_k = None
                if hasattr(self, 'verify_calls'):
                    self.verify_calls = 0
                if hasattr(self, '_pstop_latched'):
                    self._pstop_latched = False
                if hasattr(self, '_holding'):
                    self._holding = False
                if hasattr(self, '_calls'):
                    for k in self._calls:
                        self._calls[k] = 0
            elif p.name == 'duration_scale_ratio' and float(p.value) > 0.0:
                self.engine._scale = float(p.value)
            elif p.name == 'drop_role':
                self._set_heartbeat_stop(role=str(p.value))
            elif p.name == 'drop_at_s' and float(p.value) >= 0.0:
                self._set_heartbeat_stop(t_s=float(p.value))
        if t0_updated and self.active:
            if hasattr(self, '_on_status'):
                self._on_status()
            if hasattr(self, '_on_heartbeat'):
                self._on_heartbeat()
        return SetParametersResult(successful=True)

    def refuse_if_provided(self, services: list[str]) -> None:
        """같은 역할의 서버가 있으면 기동을 거부하고, 기동 뒤에도 감시한다(REQ-MCK-007, 실제 노드와 동시 기동 금지).

        기동할 때 `exclusive_check_s` 동안 그래프를 보고, 그 뒤에는 `EXCLUSIVE_WATCH_PERIOD_S`마다 자기 서버 외의
        서버가 생겼는지 본다. discovery 지연(수 s)으로 기동 확인 창에서 놓친 서버도 이 감시로 잡는다.
        발견하면 `conflict`를 남긴다. `spin_mock`은 이를 보고 executor를 멈춘 뒤 종료 코드 1로 끝낸다.
        """
        # count_services는 remap을 반영하지 않으므로 remap까지 푼 이름으로 센다
        services = [self.resolve_service_name(s) for s in services]
        end = time.monotonic() + float(self.get_parameter('exclusive_check_s').value)
        while True:
            found = [s for s in services if self.count_services(s) > 0]
            if found:
                raise RuntimeError(f'REAL_NODE_PRESENT: 이미 제공 중인 서비스 {found}')
            if time.monotonic() >= end:
                break
            time.sleep(0.05)
        self._exclusive_services = list(services)
        self.create_timer(EXCLUSIVE_WATCH_PERIOD_S, self._watch_exclusive,
                          callback_group=MutuallyExclusiveCallbackGroup(), clock=steady_clock())

    def _watch_exclusive(self) -> None:
        if self.conflict is not None:
            return
        found = [s for s in self._exclusive_services if self.count_services(s) > 1]
        if found:
            self.conflict = f'REAL_NODE_PRESENT: 기동 뒤 같은 역할 서버 발견 {found}'
            self.get_logger().error(self.conflict)

    def stall_beat(self) -> None:
        """heartbeat 콜백 진입 표시(REQ-MCK-012). 진단이 꺼져 있으면 아무것도 하지 않는다."""
        if self.stall_watch is not None:
            self.stall_watch.beat()

    def autostart(self) -> None:
        if self.get_parameter('autostart').value:
            self.trigger_configure()
            self.trigger_activate()

    # ------------------------------------------------------------ 시각
    def ros_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def scenario_t(self) -> float:
        return self.ros_now() - self.t0

    def deadline_passed(self, deadline: Time) -> bool:
        d = from_time(deadline)
        return d > 0.0 and self.ros_now() >= d

    @property
    def active(self) -> bool:
        return self.state_id == State.PRIMARY_STATE_ACTIVE

    # ------------------------------------------------------------ lifecycle
    def on_configure(self, state):
        self.state_id = State.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.state_id = State.PRIMARY_STATE_ACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.state_id = State.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.state_id = State.PRIMARY_STATE_FINALIZED
        return TransitionCallbackReturn.SUCCESS


def spin_mock(node_cls, name: str, num_threads: int, args=None) -> None:
    """대역 노드 실행 공통. 동시 기동 거부(기동 시·기동 뒤 모두)는 종료 코드 1로 끝낸다(REQ-MCK-007).

    종료 신호는 `ShutdownGuard`로 가린다(REQ-SYS-007): 신호로 생긴 예외는 traceback 없이 종료 코드 0이다.
    """
    guard = ShutdownGuard()            # rclpy.init 전
    rclpy.init(args=args)
    try:
        node = node_cls()
    except BaseException as e:
        if guard.is_shutdown(e):
            # 기동 확인 창(`exclusive_check_s`) 안에서 launch가 끝나도 실행 중 종료와 같이 조용히 끝낸다
            rclpy.try_shutdown()
            guard.close()
            return
        if not isinstance(e, RuntimeError):
            raise
        get_logger(name).error(str(e))
        rclpy.try_shutdown()
        guard.close()
        raise SystemExit(1)
    limit = stall_dump_s()
    if limit > 0.0:
        node.stall_watch = StallWatch(node.get_logger(), limit)
        node.get_logger().info(f'{STALL_DUMP_ENV}={limit:.3f}: heartbeat 경로 정지 진단 켬')
    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(node)
    try:
        while rclpy.ok() and node.conflict is None:
            executor.spin_once(timeout_sec=0.1)
    except BaseException as exc:
        if not guard.is_shutdown(exc):
            raise
    finally:
        if node.stall_watch is not None:
            node.stall_watch.stop()
        conflict = node.conflict
        node.destroy_node()
        rclpy.try_shutdown()
        guard.close()
    if conflict is not None:
        raise SystemExit(1)
