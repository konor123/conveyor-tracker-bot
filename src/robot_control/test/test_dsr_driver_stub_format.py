"""Doosan 드라이버 형식 확인(SPEC-500 REQ-ROB-011, TEST_SPEC.md TC-ROB-011-01). 실기 미검증 코드의 C 시험.

`mode:=virtual`은 DRCF 에뮬레이터(docker)가 있어야 하는데 시험 PC에 없다. 그래서 실제 `dsr_msgs2` 형식으로 연 최소
stub(`fakes/dsr_driver_stub.py`)에 프로젝트 client(`DsrRegisterIO`·`DsrMotionClient`·`DoosanBackend`·
`DoosanSpeedlTransport`)를 그대로 붙여 서비스·토픽 이름, 형식, 필드 이름, 배열 길이, 단위(값)를 확인한다.
rclpy는 기본 설정에서 메시지 필드 검사를 하지 않으므로(`ROS_PYTHON_CHECK_FIELDS`) 이 시험 동안만 생성 모듈의 검사를 켠다.
제어기 수락·이동·시간·시리얼·그리퍼는 확인하지 않는다(SPEC-500 4.6절). `dsr_msgs2`를 찾을 수 없으면 skip이다.
`dsr_msgs2`는 수집 때가 아니라 시험 안에서 가져온다(같은 프로세스에서 도는 `test_doosan_stream.py`의 import 검사).
skip은 모듈 수준 `pytest.skip`이 아니라 `skipif` 표지로 한다. ROS가 설치된 PC·CI에서는 launch_testing pytest 플러그인이
수집 때 모듈을 미리 import하므로, 모듈 수준 skip이 같은 폴더의 다른 시험 수집까지 멈춘다.
"""
from __future__ import annotations

import importlib
import importlib.util
import math
import re
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
SKIP_REASON = ('dsr_msgs2 없음: Doosan 메시지 패키지가 있는 PC에서 판정한다. virtual 모드는 DRCF 에뮬레이터(docker)가 '
               '시험 PC에 없어 실제 형식 stub로 대체한다(SPEC-500 REQ-ROB-011)')
HAVE_DSR_MSGS2 = importlib.util.find_spec('dsr_msgs2') is not None

from drop_ros import DOMAIN  # noqa: E402
from fakes.clock import FakeClock  # noqa: E402
from fakes.dsr_driver_stub import SPEEDL_DEPTH, DsrDriverStub  # noqa: E402
from rclpy.context import Context  # noqa: E402

from robot_control.doosan_backend import DoosanBackend, DoosanBackendCfg  # noqa: E402
from robot_control.doosan_stream import DoosanSpeedlTransport  # noqa: E402
from robot_control.dsr_motion_client import DsrMotionClient  # noqa: E402
from robot_control.dsr_register_io import DsrRegisterIO, RosServiceCaller  # noqa: E402
from robot_control.dynamic_place import MotionRef  # noqa: E402

pytestmark = [pytest.mark.skipif(not HAVE_DSR_MSGS2, reason=SKIP_REASON),
              pytest.mark.req('REQ-ROB-011'), pytest.mark.tc('TC-ROB-011-01'), pytest.mark.level('C')]

# 드라이버 표면(SPEC-500 4.6절 표). 이름은 static_pick_place.doosan_client·tools/bench/stream_bench.py·OFF-2 기록과 같다.
SERVICES = {
    '/dsr01/drl/drl_start': 'dsr_msgs2/srv/DrlStart',
    '/dsr01/drl/get_drl_state': 'dsr_msgs2/srv/GetDrlState',
    '/dsr01/plc/set_output_register_int': 'dsr_msgs2/srv/SetOutputRegisterInt',
    '/dsr01/plc/get_output_register_int': 'dsr_msgs2/srv/GetOutputRegisterInt',
    '/dsr01/system/get_robot_state': 'dsr_msgs2/srv/GetRobotState',
    '/dsr01/motion/move_line': 'dsr_msgs2/srv/MoveLine',
    '/dsr01/motion/move_stop': 'dsr_msgs2/srv/MoveStop',
    '/dsr01/aux_control/get_current_posx': 'dsr_msgs2/srv/GetCurrentPosx',
}
SPEEDL = ('/dsr01/speedl_stream', 'dsr_msgs2/msg/SpeedlStream')
DOWN = (0.0, 1.0, 0.0, 0.0)                     # 그리퍼 아래(ZYZ b = 180°)
TIMEOUT_S = 5.0                                 # 공유 PC 부하·DDS 발견 여유(시간 판정은 이 시험의 대상이 아님)


class MonoClock:
    @staticmethod
    def steady_now() -> float:
        return time.monotonic()

    @staticmethod
    def ros_now() -> float:
        return time.time()


def wait_for(pred, timeout_s: float = 10.0) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


def foreign_driver_endpoints(node) -> list[str]:
    """stub 밖에서 드라이버 이름(`/dsr01/...` 서비스, `speedl_stream` 구독)을 쓰는 노드. 발견된 것만 본다."""
    found = []
    for name, ns in node.get_node_names_and_namespaces():
        if (name, ns) == ('dsr_driver_stub', '/'):
            continue
        try:
            services = node.get_service_names_and_types_by_node(name, ns)
        except Exception:  # noqa: BLE001 - 조사하는 사이 사라진 노드
            continue
        found += [f'{ns}|{name}: {srv}' for srv, _ in services if srv.startswith('/dsr01/')]
    found += [f'{s.node_namespace}|{s.node_name}: {SPEEDL[0]}' for s in node.get_subscriptions_info_by_topic(SPEEDL[0])
              if (s.node_name, s.node_namespace) != ('dsr_driver_stub', '/')]
    return found


@pytest.fixture
def checked_fields(monkeypatch):
    """생성 메시지 모듈의 필드 검사를 켠다(ROS_PYTHON_CHECK_FIELDS=1과 같음, 이 시험 동안만)."""
    for name in ('dsr_msgs2.msg', 'dsr_msgs2.srv', 'std_msgs.msg'):      # 생성 모듈을 모두 불러온 뒤 켠다
        importlib.import_module(name)
    patched = 0
    for name, module in list(sys.modules.items()):
        if re.match(r'(dsr_msgs2|std_msgs)\.(msg|srv)\._', name) and hasattr(module, 'ros_python_check_fields'):
            monkeypatch.setattr(module, 'ros_python_check_fields', '1')
            patched += 1
    assert patched > 0


@pytest.fixture
def rig(checked_fields):
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN)       # 시험 전용 도메인: 실제 /dsr01 드라이버와 섞이지 않는다
    stub = DsrDriverStub(context, registers={30: 0, 31: 127, 32: 128, 39: 250, 40: 255})
    caller = RosServiceCaller('robot_control_dsr_io', context=context)
    try:
        assert wait_for(lambda: set(SERVICES) <= {n for n, _ in caller.node.get_service_names_and_types()}), \
            'stub 서비스가 helper 노드에 보이지 않음'
        # 실제 드라이버가 같은 도메인에 보이면 요청을 하나도 보내지 않고 멈춘다(로봇 지령 경로 차단)
        foreign = foreign_driver_endpoints(caller.node)
        assert not foreign, f'stub가 아닌 드라이버 이름 사용 노드: {foreign}'
        yield stub, caller, context
    finally:
        caller.close()
        stub.close()
        rclpy.try_shutdown(context=context)


def test_field_checks_reject_malformed_requests(checked_fields):
    """필드 검사가 켜져 있어야 '수락'이 형식 확인이 된다(빈 통과 방지)."""
    from dsr_msgs2.msg import SpeedlStream
    from dsr_msgs2.srv import GetOutputRegisterInt, MoveLine
    for bad in (dict(pos=[0.0] * 5), dict(vel=[30.0, 20.0, 0.0]), dict(sync_typ=1), dict(sync_type=1.0),
                dict(vel=[30, 20])):
        with pytest.raises(AssertionError):
            MoveLine.Request(**bad)
    with pytest.raises(AssertionError):
        SpeedlStream(vel=[0.0] * 6, acc=[300.0, 60.0, 0.0], time=0.02)
    with pytest.raises(AssertionError):
        GetOutputRegisterInt.Request(address=-1, timeout_ms=1000)       # uint16


def test_driver_surface_names_and_types(rig):
    stub, caller, _ = rig
    assert stub.service_names == SERVICES
    graph = dict(caller.node.get_service_names_and_types())
    assert {n: graph[n] for n in SERVICES} == {n: [t] for n, t in SERVICES.items()}
    assert wait_for(lambda: (SPEEDL[0], [SPEEDL[1]]) in caller.node.get_topic_names_and_types())
    subs = caller.node.get_subscriptions_info_by_topic(SPEEDL[0])
    assert [(s.node_name, s.topic_type) for s in subs] == [('dsr_driver_stub', SPEEDL[1])]


def test_register_io_requests_match_driver_format(rig):
    stub, caller, _ = rig
    io = DsrRegisterIO(caller, call_timeout_s=TIMEOUT_S, drl_start_timeout_s=TIMEOUT_S)   # 형식은 dsr_msgs2에서 가져옴
    assert io.robot_state() == 1 and io.drl_state() == 3 and io.robot_standby()
    code = 'set_output_register_int(36, 1)\n'
    assert io.drl_start(code) is True
    (req,) = stub.requests_of('drl/drl_start')
    assert (type(req).__name__, req.robot_system, req.code) == ('DrlStart_Request', 0, code)

    assert io.set(10, 127) is True and io.set(24, 0) is True
    assert [(r.address, r.value) for r in stub.requests_of('plc/set_output_register_int')] == [(10, 127), (24, 0)]

    # DRL이 쓴 바이트 0·127·128·250·255: 128 이상은 int32 응답이 음수다
    assert [io.get(a) for a in (30, 31, 32, 39, 40)] == [0, 127, 128, 250, 255]
    raw = [r.value for r in stub.responses_of('plc/get_output_register_int')]
    assert raw[0] == 0 and raw[1] == 127 << 24 and all(v < 0 for v in raw[2:])
    assert io.get(10) == 127                                   # PC가 쓴 값(stub 가정: 같은 표현)
    reqs = stub.requests_of('plc/get_output_register_int')
    assert [r.address for r in reqs] == [30, 31, 32, 39, 40, 10] and {r.timeout_ms for r in reqs} == {1000}
    assert stub.rejected == []


def test_motion_client_and_static_backend_over_stub(rig):
    stub, caller, _ = rig
    client = DsrMotionClient(caller, call_timeout_s=TIMEOUT_S)
    assert client.robot_state() == 1 and client.drl_state() == 3
    assert client.current_posx() == [500.0, 0.0, 300.0, 0.0, 180.0, 0.0]
    backend = DoosanBackend(client, None, MonoClock(), time.sleep, DoosanBackendCfg())

    out = backend.move_line([0.5, 0.0, 0.25], DOWN)
    assert out.completed and out.reason_code == '' and out.pose_error_m <= 1e-6
    (req,) = stub.requests_of('motion/move_line')
    assert type(req).__name__ == 'MoveLine_Request'
    for arr, n in ((req.pos, 6), (req.vel, 2), (req.acc, 2)):
        assert isinstance(arr, np.ndarray) and arr.dtype == np.float64 and arr.shape == (n,)
    assert req.pos[:3].tolist() == pytest.approx([500.0, 0.0, 250.0])           # mm
    assert req.pos[4] == pytest.approx(180.0)                                    # deg(ZYZ b), 자세는 아래 왕복으로 확인
    assert req.vel.tolist() == pytest.approx([30.0, math.degrees(0.35)])       # mm/s, deg/s
    assert req.acc.tolist() == pytest.approx([50.0, math.degrees(0.7)])        # mm/s², deg/s²
    assert (req.time, req.radius, req.ref, req.mode, req.blend_type, req.sync_type) == (0.0, 0.0, 0, 0, 0, 1)
    assert {r.ref for r in stub.requests_of('aux_control/get_current_posx')} == {0}

    pos, quat = backend.read_tcp_pose()
    assert pos.tolist() == pytest.approx([0.5, 0.0, 0.25], abs=1e-9)
    assert abs(float(np.dot(quat, DOWN))) == pytest.approx(1.0, abs=1e-9)

    assert backend.stop() is True and backend.is_stopped()
    assert [r.stop_mode for r in stub.requests_of('motion/move_stop')] == [2]
    backend.reset_stop()
    assert stub.rejected == []

    stub.fail.add('motion/move_line')                          # 드라이버 거부(success=false) → 지령 거절
    assert backend.move_line([0.5, 0.0, 0.26], DOWN).reason_code == 'MOVE_REJECTED'
    stub.fail.add('aux_control/get_current_posx')              # 읽기 실패 → 값을 만들지 않음
    assert all(math.isnan(v) for v in backend.read_tcp_pose()[0])


def test_speedl_stream_messages_match_driver_format(rig):
    stub, caller, context = rig
    from dsr_msgs2.msg import SpeedlStream
    client = DsrMotionClient(caller, call_timeout_s=TIMEOUT_S)
    node = rclpy.create_node('speedl_publisher', context=context, use_global_arguments=False,
                             start_parameter_services=False)
    try:
        pub = node.create_publisher(SpeedlStream, SPEEDL[0], SPEEDL_DEPTH)  # tools/bench/stream_bench.py와 같은 depth
        assert wait_for(lambda: pub.get_subscription_count() >= 1 and stub.node.count_publishers(SPEEDL[0]) >= 1), \
            'stub 구독과 발행기가 서로 보이지 않음'
        clock = FakeClock()
        transport = DoosanSpeedlTransport(pub.publish, SpeedlStream, clock, lambda: None, None,
                                          move_stop=client.move_stop)
        assert transport.update(MotionRef(clock.ros_now(), (0.5, 0.0, 0.2), (0.02, 0.0, 0.0), (0.02, 0.0, -0.005)))
        assert transport.stop()
        assert wait_for(lambda: len(stub.speedl) >= 2, TIMEOUT_S)
        first, zero = stub.speedl[:2]
        for m in (first, zero):
            assert type(m).__name__ == 'SpeedlStream'
            assert m.vel.dtype == np.float64 and m.vel.shape == (6,) and m.acc.shape == (2,)
            assert m.acc.tolist() == pytest.approx([300.0, 60.0])            # mm/s², deg/s²
            assert m.time == pytest.approx(0.02)                             # 2주기 [s]
        assert first.vel.tolist() == pytest.approx([20.0, 0.0, -5.0, 0.0, 0.0, 0.0])   # mm/s, deg/s
        assert zero.vel.tolist() == [0.0] * 6
        assert [r.stop_mode for r in stub.requests_of('motion/move_stop')] == [2]
        assert stub.rejected == []
    finally:
        node.destroy_node()


def test_stub_rejects_out_of_range_values(rig):
    """stub 검사가 실제로 거부할 수 있어야 '거부 없이 수락'이 의미를 가진다."""
    stub, caller, _ = rig
    from dsr_msgs2.srv import MoveLine, MoveStop
    base = dict(pos=[500.0, 0.0, 250.0, 0.0, 180.0, 0.0], vel=[30.0, 20.0], acc=[50.0, 40.0], time=0.0, radius=0.0,
                ref=0, mode=0, blend_type=0)
    assert caller.call(MoveLine, '/dsr01/motion/move_line', TIMEOUT_S, sync_type=2, **base).success is False
    assert caller.call(MoveStop, '/dsr01/motion/move_stop', TIMEOUT_S, stop_mode=4).success is False
    assert [t for t, _ in stub.rejected] == ['motion/move_line', 'motion/move_stop']


def test_guard_detects_other_nodes_using_driver_names(rig):
    """실제 드라이버 차단 검사(`rig`)가 빈 통과가 아님: stub가 아닌 노드의 `/dsr01` 서비스·`speedl_stream` 구독을 찾는다."""
    _, caller, context = rig
    from dsr_msgs2.msg import SpeedlStream
    from dsr_msgs2.srv import MoveStop
    other = rclpy.create_node('other_driver', context=context, use_global_arguments=False,
                              start_parameter_services=False)
    try:
        other.create_service(MoveStop, '/dsr01/motion/move_stop', lambda req, res: res)
        other.create_subscription(SpeedlStream, SPEEDL[0], lambda m: None, SPEEDL_DEPTH)
        expected = {'/|other_driver: /dsr01/motion/move_stop', f'/|other_driver: {SPEEDL[0]}'}
        assert wait_for(lambda: expected <= set(foreign_driver_endpoints(caller.node)))
    finally:
        other.destroy_node()
