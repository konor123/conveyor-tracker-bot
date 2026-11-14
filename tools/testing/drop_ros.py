"""ROS 수준(C·I0) 시험 공통 도구: 시험용 클라이언트 노드, 대상 프로세스 기동, lifecycle 조작.

계약 시험은 대상을 `Target`으로 받는다. 환경 변수 `DROP_CONTRACT_<ROLE>`(예:
`DROP_CONTRACT_VISION="ros2 run vision vision_node"`)가 있으면 패키지 기본 대상 대신 그 명령을 쓴다.
"""
from __future__ import annotations

import os
import shlex
import subprocess
import sys
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
SERVICE_DISCOVERY_S = 15.0     # Harness.call의 서비스 발견 대기 상한 [s]
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
    """노드가 직접 만든 구독·발행 중 QoS 이벤트 핸들러가 붙은 것(토픽, 개수). rclpy 내부 발행자는 뺀다."""
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

    The publisher runs in its own process. Inside the test process it stalled with
    that process on CI runners, so every node's ROS time froze and the mock world
    published stale frames 0.27 s late (CI run 42781388215: publish, middleware and
    callback lateness all 0.27 s). CLOCK_MONOTONIC is shared between processes, so
    the child uses the same origins. 50 Hz keeps the quantization (20 ms) below one
    30 Hz frame and the /clock callback load low (CI run 42762185513 at 200 Hz).

    `jump(dt)` moves the published time by dt (negative = backward) for clock
    regression tests; the child reads the offsets from its stdin.
    """

    PERIOD_S = 0.02

    def __init__(self, period_s: float = PERIOD_S, domain_id: int = DOMAIN):
        self.period_s = period_s
        self.ros_origin = time.time()
        self.steady_origin = time.monotonic()
        self._offset = 0.0
        self._proc = subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), 'clock', repr(self.ros_origin),
             repr(self.steady_origin), repr(period_s)],
            env=dict(os.environ, ROS_DOMAIN_ID=str(domain_id)), stdin=subprocess.PIPE, text=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def now(self):
        return self.ros_origin + self._offset + time.monotonic() - self.steady_origin

    def jump(self, dt: float) -> None:
        """발행하는 ROS 시각을 dt만큼 옮긴다(음수 = 역행). 자식은 다음 발행부터 반영한다."""
        self._offset += dt
        self._proc.stdin.write(f'{dt!r}\n')
        self._proc.stdin.flush()

    def close(self):
        self._proc.terminate()
        self._proc.wait(timeout=5)


def _clock_main(ros_origin: float, steady_origin: float, period_s: float) -> None:
    """SteadyRosClock 자식 프로세스: ROS_DOMAIN_ID 환경 변수의 domain에 /clock을 낸다. stdin 한 줄 = 시각 이동량 [s]."""
    from rosgraph_msgs.msg import Clock
    offset = [0.0]

    def read_jumps():
        for line in sys.stdin:
            if line.strip():
                offset[0] += float(line)
    threading.Thread(target=read_jumps, daemon=True).start()
    rclpy.init()
    node = rclpy.create_node(f'drop_test_clock_{os.getpid()}')
    pub = node.create_publisher(Clock, '/clock', 10)
    try:
        while rclpy.ok():
            pub.publish(Clock(clock=to_time(ros_origin + offset[0] + time.monotonic() - steady_origin)))
            time.sleep(period_s)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


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

    def record(self, typ, topic, steady: bool = False, info: bool = False):
        """수신 (수신 시각, 메시지) 목록. 목록은 계속 채워진다.

        수신 시각은 ROS 시각이다. `steady=True`면 `time.monotonic()`으로 남긴다(시계 점프와 무관한 주기 측정용).
        `info=True`면 (콜백 시각, 메시지, 미들웨어 정보, 콜백 때 시스템 시각 [ns])를 남긴다. 미들웨어 정보는
        rclpy `MessageInfo`의 `source_timestamp`(발행)·`received_timestamp`(미들웨어 수신) [ns, 시스템 시각]로,
        시험 프로세스의 Python 콜백이 늦게 돌아도 바뀌지 않는다. 시스템 시계는 보정으로 단조 시계와 어긋날 수
        있으므로(부하 걸린 WSL에서 2.5 s 어긋남을 초당 약 1 s씩 따라잡는 것을 관측) 절대값을 단조 시각에
        옮기지 말고 `콜백 시각 − (콜백 때 시스템 시각 − 미들웨어 시각)`처럼 짧은 구간의 차이로만 쓴다.
        """
        box = []
        clock = time.monotonic if steady else self.now
        if info:
            def cb(m, i):
                box.append((clock(), m, i, time.time_ns()))
        else:
            def cb(m):
                box.append((clock(), m))
        self.client.create_subscription(typ, topic, cb, RECORD_QOS,
                                       event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))
        return box

    def call(self, typ, name, timeout=5.0, **fields):
        """서비스를 한 번 부른다. `timeout`은 응답 대기다. 새 client의 끝점 매칭은 서버 프로세스의 발견 처리에
        달려 공유 CI 러너에서 5 s를 넘긴 적이 있어(2026-11-13 run 43398439752·43403812431, 모듈 내내 떠 있던
        `robot_controller_node`의 `change_state`) 서비스 발견은 `SERVICE_DISCOVERY_S`까지 기다린다."""
        cli = self.client.create_client(typ, name)
        if not cli.wait_for_service(timeout_sec=max(timeout, SERVICE_DISCOVERY_S)):
            # 원인 구분용: 서버가 그래프에 없는지(노드 종료) 있는데 매칭이 늦은지
            servers = [n for n, _ in self.client.get_service_names_and_types() if n == name]
            nodes = sorted(f'{ns.rstrip("/")}/{n}' for n, ns in self.client.get_node_names_and_namespaces())
            raise TimeoutError(f'서비스 없음: {name} (그래프의 같은 이름 서비스 {len(servers)}개, 노드 {nodes})')
        return wait_future(cli.call_async(typ.Request(**fields)), timeout)

    def action(self, typ, name, timeout=5.0) -> ActionClient:
        """action client를 만들고 서버 발견을 `SERVICE_DISCOVERY_S`까지 기다린다(`call`과 같은 이유, 2026-11-14 CI
        run 43453467009의 `robot/move_to_pose` 5 s 초과)."""
        ac = ActionClient(self.client, typ, name)
        if not ac.wait_for_server(timeout_sec=max(timeout, SERVICE_DISCOVERY_S)):
            nodes = sorted(f'{ns.rstrip("/")}/{n}' for n, ns in self.client.get_node_names_and_namespaces())
            raise TimeoutError(f'action 서버 없음: {name} (노드 {nodes})')
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


if __name__ == '__main__' and sys.argv[1:2] == ['clock']:
    _clock_main(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
