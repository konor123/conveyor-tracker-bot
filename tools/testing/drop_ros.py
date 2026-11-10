"""ROS 수준(C·I0) 시험 공통 도구: 시험용 클라이언트 노드, 대상 프로세스 기동, lifecycle 조작.

계약 시험(PLAN.md 7.4절)은 대상을 `Target`으로 받는다. 환경 변수 `DROP_CONTRACT_<ROLE>`(예:
`DROP_CONTRACT_VISION="ros2 run vision vision_node"`)가 있으면 패키지 기본 대상 대신 그 명령을 쓴다.
"""
from __future__ import annotations

import os
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass, field

import rclpy
from builtin_interfaces.msg import Time
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.action import ActionClient
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter

DOMAIN = 100 + os.getpid() % 100
RECORD_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)


@dataclass
class Target:
    """계약 시험 대상. `cmd`는 `--ros-args` 앞까지의 실행 명령, `node`는 lifecycle 노드 이름."""

    cmd: list[str]
    node: str
    ros_args: list[str] = field(default_factory=list)

    @classmethod
    def from_env(cls, role: str, default: 'Target') -> 'Target':
        value = os.environ.get(f'DROP_CONTRACT_{role.upper()}')
        if not value:
            return default
        return cls(cmd=shlex.split(value), node=os.environ.get(f'DROP_CONTRACT_{role.upper()}_NODE', default.node))


def to_time(sec: float) -> Time:
    ns = int(round(sec * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def from_time(t: Time) -> float:
    return t.sec + t.nanosec * 1e-9


RCLPY_INTERNAL_TOPICS = ('/parameter_events', '/rosout')


def own_qos_event_handlers(node) -> list[tuple[str, int]]:
    """노드가 직접 만든 구독·발행 중 QoS 이벤트 핸들러가 붙은 것(토픽, 개수). rclpy 내부 발행자는 뺀다(REQ-IF-006)."""
    pubs = [p for p in node.publishers if p.topic_name not in RCLPY_INTERNAL_TOPICS]
    return [(e.topic_name, len(e.event_handlers)) for e in pubs + list(node.subscriptions) if e.event_handlers]


def wait_future(fut, timeout: float):
    end = time.monotonic() + timeout
    while not fut.done():
        if time.monotonic() > end:
            raise TimeoutError('future가 제한시간 안에 끝나지 않음')
        time.sleep(0.005)
    return fut.result()


class SteadyRosClock:
    """Publish controlled ROS time for schedule tests, independent of host NTP jumps.

    The injected clock_jump scenario still changes message stamps; this clock only
    supplies a stable scenario timeline. Close after all subscribing nodes stop.

    Every node with use_sim_time handles each /clock message as a Python callback in
    the shared executor. At 200 Hz with three nodes this saturated 2~4 vCPU CI runners
    (frame timer woke at about 6 Hz and ROS time lagged steady time by up to 0.4 s,
    CI run 42762185513). 50 Hz keeps the quantization (20 ms) below one 30 Hz frame.
    """

    PERIOD_S = 0.02

    def __init__(self, period_s: float = PERIOD_S):
        self.period_s = period_s
        from rosgraph_msgs.msg import Clock
        self._message = Clock
        self.node = rclpy.create_node(f'drop_test_clock_{os.getpid()}')
        self.pub = self.node.create_publisher(Clock, '/clock', 10)
        self.ros_origin = time.time()
        self.steady_origin = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def now(self):
        return self.ros_origin + time.monotonic() - self.steady_origin

    def _run(self):
        while not self._stop.is_set():
            self.pub.publish(self._message(clock=to_time(self.now())))
            self._stop.wait(self.period_s)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        assert not self._thread.is_alive()
        self.node.destroy_node()


class Harness:
    """노드 여러 개 + 시험용 클라이언트 노드를 백그라운드 executor로 돌린다."""

    def __init__(self, *nodes, use_sim_time=False):
        self.client = rclpy.create_node(
            f'drop_test_client_{os.getpid()}_{id(self)}',
            parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
        self.nodes = list(nodes)
        self.exec = MultiThreadedExecutor()
        for n in self.nodes + [self.client]:
            self.exec.add_node(n)
        self._thread = threading.Thread(target=self.exec.spin, daemon=True)
        self._thread.start()

    def now(self) -> float:
        return self.client.get_clock().now().nanoseconds * 1e-9

    def record(self, typ, topic, steady: bool = False):
        """수신 (수신 시각, 메시지) 목록. 목록은 계속 채워진다.

        수신 시각은 ROS 시각이다. `steady=True`면 `time.monotonic()`으로 남긴다(시계 점프와 무관한 주기 측정용).
        """
        box = []
        clock = time.monotonic if steady else self.now
        self.client.create_subscription(typ, topic, lambda m: box.append((clock(), m)), RECORD_QOS,
                                       event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))
        return box

    def call(self, typ, name, timeout=5.0, **fields):
        cli = self.client.create_client(typ, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'서비스 없음: {name}')
        return wait_future(cli.call_async(typ.Request(**fields)), timeout)

    def action(self, typ, name, timeout=5.0) -> ActionClient:
        ac = ActionClient(self.client, typ, name)
        if not ac.wait_for_server(timeout_sec=timeout):
            raise TimeoutError(f'action 서버 없음: {name}')
        return ac

    def wait_until(self, t_ros: float) -> None:
        while self.now() < t_ros:
            time.sleep(0.002)

    def sleep(self, dt: float) -> None:
        self.wait_until(self.now() + dt)

    # ------------------------------------------------------------ lifecycle
    def lifecycle_state(self, node: str, timeout=5.0) -> int:
        return self.call(GetState, f'/{node}/get_state', timeout=timeout).current_state.id

    def transition(self, node: str, transition_id: int) -> bool:
        return self.call(ChangeState, f'/{node}/change_state',
                         transition=Transition(id=transition_id)).success

    def deactivate(self, node: str) -> bool:
        return self.transition(node, Transition.TRANSITION_DEACTIVATE)

    def activate(self, node: str) -> bool:
        return self.transition(node, Transition.TRANSITION_ACTIVATE)

    # ------------------------------------------------------------ graph
    def publisher_qos(self, topic: str) -> list:
        return [i.qos_profile for i in self.client.get_publishers_info_by_topic(topic)]

    def service_types(self) -> dict[str, list[str]]:
        return dict(self.client.get_service_names_and_types())

    def wait_for(self, pred, timeout=5.0, what='조건') -> None:
        end = time.monotonic() + timeout
        while not pred():
            if time.monotonic() > end:
                raise TimeoutError(f'{what}이(가) {timeout} s 안에 충족되지 않음')
            time.sleep(0.01)

    def close(self) -> None:
        self.exec.shutdown()
        for n in self.nodes + [self.client]:
            n.destroy_node()


def spawn(target: Target, extra_ros_args: list[str] | None = None) -> subprocess.Popen:
    """대상을 시험 도메인(`DOMAIN`)에서 별도 프로세스로 띄운다."""
    args = list(target.ros_args) + list(extra_ros_args or [])
    cmd = list(target.cmd) + (['--ros-args'] + args if args else [])
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    return subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop(proc: subprocess.Popen) -> None:
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
