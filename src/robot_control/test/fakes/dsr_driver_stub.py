"""doosan-robot2 드라이버 표면의 최소 ROS stub(SPEC-500 REQ-ROB-011, TEST_SPEC.md TC-ROB-011-01).

`mode:=virtual`(DRCF 에뮬레이터) 대신 쓴다. 실제 `dsr_msgs2` 형식으로 프로젝트가 쓰는 서비스 8종과 `speedl_stream`
토픽을 열어, 이름·형식·필드·배열 길이·값을 확인할 수 있게 받은 요청을 그대로 남긴다. 제어기는 흉내 내지 않는다.

- 상태는 고정값(`robot_state`, `drl_state`)이다. `move_line`을 받으면(base·절대) 그 `pos`를 곧바로 현재 posx로 둔다
  (이동·시간·상태 전이 없음). `move_stop`·`drl_start`는 받기만 한다.
- 레지스터 읽기 응답은 OFF-2 관찰 표현(하위 1바이트를 최상위 바이트 자리, 부호 있는 int32, `fakes/dsr.py`와 같음)이고,
  PC가 쓴 값도 같은 표현으로 돌려준다(가정, OFF-2 확인 항목).
- 요청 검사는 srv 주석의 열거값과 유한값뿐이다(제어기 수락 조건이 아님). 어긋나면 `success=false`로 거부하고
  `rejected`에 남긴다. `fail`에 넣은 서비스는 상태를 바꾸지 않고 `success=false`를 준다.

rclpy·`dsr_msgs2`·`std_msgs`는 생성할 때 가져온다(없으면 시험이 먼저 skip한다).
"""
from __future__ import annotations

import math
import threading

from fakes.dsr import encode_register_value

# 서비스 이름(<ns> 뒤) → `dsr_msgs2/srv` 형식. 이름은 `static_pick_place.doosan_client`·`tools/bench/stream_bench.py`·
# OFF-2 기록(`/dsr01/plc/*`)과 같다.
SERVICES = {
    'drl/drl_start': 'DrlStart',
    'drl/get_drl_state': 'GetDrlState',
    'plc/set_output_register_int': 'SetOutputRegisterInt',
    'plc/get_output_register_int': 'GetOutputRegisterInt',
    'system/get_robot_state': 'GetRobotState',
    'motion/move_line': 'MoveLine',
    'motion/move_stop': 'MoveStop',
    'aux_control/get_current_posx': 'GetCurrentPosx',
}
SPEEDL_TOPIC, SPEEDL_DEPTH = 'speedl_stream', 10          # 드라이버 구독 depth(stream-candidates 1.3절)
SOLUTION_SPACE = 2.0                                       # get_current_posx 응답 7번째 값(stub 고정값)


def _finite(values) -> bool:
    return all(math.isfinite(float(v)) for v in values)


class DsrDriverStub:
    def __init__(self, context, namespace: str = 'dsr01', robot_state: int = 1, drl_state: int = 3,
                 posx=(500.0, 0.0, 300.0, 0.0, 180.0, 0.0), registers: dict | None = None):
        import rclpy
        from dsr_msgs2 import msg, srv
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from std_msgs.msg import Float64MultiArray
        self._f64array = Float64MultiArray
        self.ns = '/' + namespace.strip('/')
        self.robot_state = int(robot_state)
        self.drl_state = int(drl_state)
        self.posx = [float(v) for v in posx]
        self.registers: dict[int, int] = dict(registers or {})
        self.requests: list[tuple[str, object]] = []        # (서비스 끝 이름, 받은 요청 객체)
        self.responses: list[tuple[str, object]] = []       # (서비스 끝 이름, 보낸 응답 객체)
        self.speedl: list = []                              # 받은 SpeedlStream
        self.rejected: list[tuple[str, str]] = []           # (이름, 이유)
        self.fail: set[str] = set()
        self._lock = threading.Lock()
        self.node = rclpy.create_node('dsr_driver_stub', context=context, use_global_arguments=False,
                                      start_parameter_services=False)
        group = ReentrantCallbackGroup()
        self.service_names = {}
        for tail, type_name in SERVICES.items():
            handler = getattr(self, '_' + tail.split('/')[1])
            name = f'{self.ns}/{tail}'
            self.node.create_service(getattr(srv, type_name), name, self._wrap(tail, handler), callback_group=group)
            self.service_names[name] = f'dsr_msgs2/srv/{type_name}'
        self.topic_name = f'{self.ns}/{SPEEDL_TOPIC}'
        self.node.create_subscription(msg.SpeedlStream, self.topic_name, self._on_speedl, SPEEDL_DEPTH,
                                      callback_group=group)
        self.executor = MultiThreadedExecutor(num_threads=3, context=context)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, name='dsr_driver_stub', daemon=True)
        self._thread.start()

    # ------------------------------------------------------------ 공통
    def _wrap(self, tail, handler):
        def callback(req, res):
            with self._lock:
                self.requests.append((tail, req))
                if tail in self.fail:                       # 지정한 실패 응답: 상태를 바꾸지 않는다
                    res.success = False
                else:
                    reason = handler(req, res)
                    if reason:
                        self.rejected.append((tail, reason))
                    res.success = not reason
                self.responses.append((tail, res))
            return res
        return callback

    def requests_of(self, tail: str) -> list:
        with self._lock:
            return [r for t, r in self.requests if t == tail]

    def responses_of(self, tail: str) -> list:
        with self._lock:
            return [r for t, r in self.responses if t == tail]

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self._thread.join(timeout=2.0)

    # ------------------------------------------------------------ 서비스(반환 = 거부 이유, 빈 문자열이면 수락)
    def _drl_start(self, req, res) -> str:
        if req.robot_system != 0:
            return f'robot_system={req.robot_system}(real 모드는 0)'
        return '' if req.code.strip() else 'code 비어 있음'

    def _get_drl_state(self, req, res) -> str:
        res.drl_state = self.drl_state
        return ''

    def _set_output_register_int(self, req, res) -> str:
        # 실제 제어기처럼 최상위 바이트만 저장한다(2026-11-14 현장)
        self.registers[int(req.address)] = (int(req.value) >> 24) & 255
        return ''

    def _get_output_register_int(self, req, res) -> str:
        res.value = encode_register_value(self.registers.get(int(req.address), 0))
        return '' if req.timeout_ms > 0 else 'timeout_ms 0'

    def _get_robot_state(self, req, res) -> str:
        res.robot_state = self.robot_state
        return ''

    def _move_line(self, req, res) -> str:
        if not _finite([*req.pos, *req.vel, *req.acc, req.time, req.radius]):
            return '비유한 값'
        if req.ref not in (0, 1, 2) or req.mode not in (0, 1) or req.blend_type not in (0, 1) \
                or req.sync_type not in (0, 1):
            return f'열거값 ref={req.ref} mode={req.mode} blend_type={req.blend_type} sync_type={req.sync_type}'
        if req.time < 0.0 or req.radius < 0.0 or min(req.vel) <= 0.0 or min(req.acc) <= 0.0:
            return f'범위 time={req.time} radius={req.radius} vel={list(req.vel)} acc={list(req.acc)}'
        if req.ref == 0 and req.mode == 0:
            self.posx = [float(v) for v in req.pos]
        return ''

    def _move_stop(self, req, res) -> str:
        return '' if req.stop_mode in (0, 1, 2, 3) else f'stop_mode={req.stop_mode}'

    def _get_current_posx(self, req, res) -> str:
        if not (req.ref in (0, 2) or 101 <= req.ref <= 200):
            return f'ref={req.ref}'
        res.task_pos_info = [self._f64array(data=[*self.posx, SOLUTION_SPACE])]
        return ''

    # ------------------------------------------------------------ 토픽
    def _on_speedl(self, m) -> None:
        with self._lock:
            self.speedl.append(m)
            if not _finite([*m.vel, *m.acc, m.time]) or m.time <= 0.0:
                self.rejected.append((SPEEDL_TOPIC, f'vel={list(m.vel)} acc={list(m.acc)} time={m.time}'))
