"""전용 helper 노드 서비스 호출(SPEC-500 REQ-GRP-017, TEST_SPEC.md TC-GRP-017-02). rclpy와 표준 서비스 형식만 쓴다.

`dsr_msgs2`가 없으므로 `std_srvs`(SetBool·Trigger) 서버로 호출 경로만 확인한다. Doosan 드라이버 동작은 검증하지 않는다.
시간 경계는 공유 러너 부하를 견디도록 넉넉하게 둔다.
"""
import os
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
std_srvs = pytest.importorskip('std_srvs.srv')

from drop_ros import DOMAIN  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

from robot_control.dsr_register_io import DsrError, RosServiceCaller  # noqa: E402

NS = f'/wp4_dsr_{os.getpid()}'
SetBool, Trigger = std_srvs.SetBool, std_srvs.Trigger


@pytest.fixture
def ctx():
    context = Context()
    rclpy.init(args=['--ros-args', '-r', '__node:=renamed_by_launch'], context=context, domain_id=DOMAIN)
    yield context
    rclpy.try_shutdown(context=context)


class Servers:
    """가짜 서비스 서버: ok(성공), fail(success=false), slow(1.0 s 뒤 응답)."""

    def __init__(self, context):
        self.node = rclpy.create_node('fake_dsr_servers', context=context, use_global_arguments=False)
        group = ReentrantCallbackGroup()
        self.node.create_service(SetBool, f'{NS}/ok', self._ok, callback_group=group)
        self.node.create_service(SetBool, f'{NS}/fail', self._fail, callback_group=group)
        self.node.create_service(Trigger, f'{NS}/slow', self._slow, callback_group=group)
        self.executor = MultiThreadedExecutor(num_threads=3, context=context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @staticmethod
    def _ok(req, res):
        res.success, res.message = True, f'data={req.data}'
        return res

    @staticmethod
    def _fail(req, res):
        res.success, res.message = False, 'refused'
        return res

    @staticmethod
    def _slow(req, res):
        time.sleep(1.0)
        res.success = True
        return res

    def close(self):
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()


@pytest.mark.req('REQ-GRP-017')
@pytest.mark.tc('TC-GRP-017-02')
@pytest.mark.level('C')
def test_helper_node_calls_from_inside_an_executor_callback(ctx):
    servers = Servers(ctx)
    caller = RosServiceCaller('robot_control_dsr_io', context=ctx)
    # 호출하는 쪽: MultiThreadedExecutor가 돌리는 노드의 콜백(action 실행 콜백과 같은 자리)에서 동기 호출
    main = rclpy.create_node('main_like_controller', context=ctx)
    mt = MultiThreadedExecutor(num_threads=2, context=ctx)
    mt.add_node(main)
    out = {}
    done = threading.Event()

    def in_callback():
        timer.cancel()
        try:
            t0 = time.monotonic()
            out['ok'] = caller.call(SetBool, f'{NS}/ok', 3.0, data=True)
            out['dt'] = time.monotonic() - t0
            try:
                caller.call(SetBool, f'{NS}/fail', 3.0, data=True)
            except DsrError as exc:          # 실패 응답 판단은 RegisterIO 몫: caller는 응답을 그대로 준다
                out['fail_exc'] = exc
            out['fail'] = caller.call(SetBool, f'{NS}/fail', 3.0, data=True)
        finally:
            done.set()
    timer = main.create_timer(0.05, in_callback)
    spin = threading.Thread(target=mt.spin, daemon=True)
    spin.start()
    try:
        assert done.wait(10.0), '콜백 안 호출이 끝나지 않음'
        assert out['ok'].success is True and out['ok'].message == 'data=True'
        assert out['dt'] < 3.0
        assert 'fail_exc' not in out and out['fail'].success is False
        assert caller.node.get_name() == 'robot_control_dsr_io'      # 전역 __node 이름 바꾸기가 걸리지 않는다
        assert main.get_name() == 'renamed_by_launch'
    finally:
        mt.shutdown(timeout_sec=1.0)
        main.destroy_node()
        caller.close()
        servers.close()
    assert not caller._thread.is_alive()


@pytest.mark.req('REQ-GRP-017')
@pytest.mark.tc('TC-GRP-017-02')
@pytest.mark.level('C')
def test_timeout_and_missing_service_raise_without_values(ctx):
    servers = Servers(ctx)
    caller = RosServiceCaller('robot_control_dsr_io', context=ctx)
    try:
        caller.call(SetBool, f'{NS}/ok', 3.0, data=False)               # 연결 확인
        t0 = time.monotonic()
        with pytest.raises(DsrError) as err:
            caller.call(Trigger, f'{NS}/slow', 0.3)
        dt = time.monotonic() - t0
        assert err.value.reason == 'SERVICE_TIMEOUT'
        assert 0.3 <= dt < 0.9, dt                                       # steady 제한시간, 1.0 s 응답을 기다리지 않음
        client = caller._clients[(Trigger, f'{NS}/slow')]
        time.sleep(1.2)                                                  # 늦은 응답이 와도 대기 요청으로 남지 않는다
        assert not getattr(client, '_pending_requests', {})
        t0 = time.monotonic()
        with pytest.raises(DsrError) as err:
            caller.call(SetBool, f'{NS}/missing', 0.5, data=True)
        assert err.value.reason == 'SERVICE_UNAVAILABLE'
        assert time.monotonic() - t0 < 1.5
        assert caller.call(SetBool, f'{NS}/ok', 3.0, data=True).success  # 시간 초과 뒤에도 계속 쓸 수 있다
    finally:
        caller.close()
        servers.close()
