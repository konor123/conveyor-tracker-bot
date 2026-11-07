"""mock 노드 공통: 시나리오 로드, 시각 변환, QoS, lifecycle 상태(SPEC-700 2, 4.6절)."""
from __future__ import annotations

import os
import time

from builtin_interfaces.msg import Time
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .scenario_engine import ScenarioEngine

DATA_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
HEARTBEAT_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)
HEARTBEAT_PERIOD_S = 0.1
ERROR_PROTECTIVE_STOP = 100        # SPEC-010 코드 표 확정 전 임시값(SPEC-700 4.5절)


def to_time(sec: float) -> Time:
    ns = int(round(sec * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def from_time(t: Time) -> float:
    return t.sec + t.nanosec * 1e-9


def scenario_file(value: str) -> str:
    if value.endswith('.yaml') or os.sep in value:
        return value
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
        """같은 역할의 서버가 이미 있으면 기동을 거부한다(REQ-MCK-007, 실제 노드와 동시 기동 금지).

        discovery가 끝날 시간을 주려고 `exclusive_check_s` 동안 그래프를 본다.
        """
        end = time.monotonic() + float(self.get_parameter('exclusive_check_s').value)
        while True:
            found = [s for s in services if self.count_services(s) > 0]
            if found:
                raise RuntimeError(f'REAL_NODE_PRESENT: 이미 제공 중인 서비스 {found}')
            if time.monotonic() >= end:
                return
            time.sleep(0.05)

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
