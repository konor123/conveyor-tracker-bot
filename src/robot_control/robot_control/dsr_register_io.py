"""Doosan 드라이버 서비스로 제어기 출력 레지스터를 읽고 쓰는 `resident_transport.RegisterIO` 구현(REQ-GRP-017).

**실기 미검증.** `robot_controller_node`에 연결하지 않았다(SPEC-500 4.6절). 서비스 이름은 OFF-2 드라이버
(`doosan-robot2`, `records/sessions/OFF-2-2026-11-07.md`)와 `static_pick_place.doosan_client`가 쓴 것과 같다.

- `<ns>/plc/get_output_register_int(address, timeout_ms)`: 응답 `value`는 DRL이 쓴 값의 하위 1바이트가 최상위
  바이트 자리에 온다(OFF-2 관찰). `decode_register_byte`로 `(value >> 24) & 255`만 쓴다
  (`doosan_client.decode_register_byte`와 같은 해석). PC가 쓴 값(seq 10)을 읽을 때도 같은 표현인지는 OFF-2 확인 항목이다.
- `<ns>/plc/set_output_register_int(address, value)`: PC → DRL 값은 0~127만 보낸다(REQ-GRP-016).
- `<ns>/drl/drl_start(robot_system=0, code)`: 로봇 STANDBY(`system/get_robot_state` = 1)와 DRL 정지
  (`drl/get_drl_state` ∈ {STOP 1, LAST 3})를 확인한 뒤에만 부른다.

서비스 호출은 `ServiceCaller`가 한다. `RosServiceCaller`는 전용 helper 노드와 그 노드만 도는 executor 스레드를 써서,
호출한 쪽 노드(예: MultiThreadedExecutor가 돌리는 action 콜백)에서 `spin_until_future_complete`를 부르지 않는다.
제한시간은 steady 시계로 재고, 시간 초과·서비스 없음·실패 응답은 값을 지어내지 않고 `DsrError`로 알린다.
서비스 형식(`dsr_msgs2.srv`)은 처음 호출할 때 가져오거나 주입한다. 순수 시험은 가짜 caller로 돈다.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Mapping, Protocol

ROBOT_STATE_STANDBY = 1
DRL_STATE_PLAY, DRL_STATE_STOP, DRL_STATE_HOLD, DRL_STATE_LAST = 0, 1, 2, 3
DRL_IDLE_STATES = (DRL_STATE_STOP, DRL_STATE_LAST)      # LAST = 드라이버 연결 뒤 실행한 프로그램 없음
PC_VALUE_MAX = 127                                      # PC → DRL 레지스터 값(REQ-GRP-016)
SERVICES = {
    'get_register': ('GetOutputRegisterInt', 'plc/get_output_register_int'),
    'set_register': ('SetOutputRegisterInt', 'plc/set_output_register_int'),
    'drl_start': ('DrlStart', 'drl/drl_start'),
    'drl_state': ('GetDrlState', 'drl/get_drl_state'),
    'robot_state': ('GetRobotState', 'system/get_robot_state'),
}


class DsrError(RuntimeError):
    """Doosan 서비스 호출 실패. `reason`: SERVICE_UNAVAILABLE / SERVICE_TIMEOUT / SERVICE_NO_RESPONSE /
    SERVICE_ERROR / SERVICE_FAILED / ROBOT_NOT_STANDBY / DRL_BUSY."""

    def __init__(self, reason: str, detail: str = ''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason
        self.detail = detail


def decode_register_byte(value: int) -> int:
    """제어기 출력 레지스터 응답 값 → DRL이 쓴 바이트(OFF-2 관찰, `doosan_client.decode_register_byte`와 같음)."""
    return (int(value) >> 24) & 255


class ServiceCaller(Protocol):
    def call(self, srv_type, name: str, timeout_s: float, **fields): ...


def dsr_service_types() -> dict:
    """`dsr_msgs2.srv` 형식. 순수 시험에서는 부르지 않는다(가짜 형식을 주입)."""
    from dsr_msgs2 import srv
    return {key: getattr(srv, type_name) for key, (type_name, _) in SERVICES.items()}


class RosServiceCaller:
    """전용 helper 노드 + 전용 executor 스레드로 서비스를 부른다.

    전역 ROS 인자를 쓰지 않으므로(`use_global_arguments=False`) launch의 `__node` 이름 바꾸기가 이 노드에 걸리지
    않는다. 시간 초과면 대기 중 요청을 지우고 `DsrError`(`SERVICE_TIMEOUT`)다. `close()`로 노드와 스레드를 닫는다.
    """

    def __init__(self, node_name: str = 'robot_control_dsr_io', context=None,
                 clock: Callable[[], float] = time.monotonic):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        self._clock = clock
        self.node = rclpy.create_node(node_name, context=context, use_global_arguments=False,
                                      start_parameter_services=False)
        self._executor = SingleThreadedExecutor(context=context)
        self._executor.add_node(self.node)
        self._clients: dict = {}
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._spin, name=f'{node_name}_executor', daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:          # noqa: BLE001 - 종료(ExternalShutdown 등) 때 스레드만 끝낸다
            pass

    def call(self, srv_type, name: str, timeout_s: float, **fields):
        deadline = self._clock() + float(timeout_s)
        with self._lock:
            client = self._clients.get((srv_type, name))
            if client is None:
                client = self.node.create_client(srv_type, name)
                self._clients[(srv_type, name)] = client
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=max(0.0, deadline - self._clock())):
                raise DsrError('SERVICE_UNAVAILABLE', name)
        future = client.call_async(srv_type.Request(**fields))
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        while not future.done():
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                remove = getattr(client, 'remove_pending_request', None)
                if remove is not None:
                    remove(future)
                future.cancel()
                raise DsrError('SERVICE_TIMEOUT', f'{name} {float(timeout_s):.3f} s')
            done.wait(min(remaining, 0.005))
        try:
            res = future.result()
        except Exception as exc:   # noqa: BLE001
            raise DsrError('SERVICE_ERROR', f'{name}: {exc!r}') from exc
        if res is None:
            raise DsrError('SERVICE_NO_RESPONSE', name)
        return res

    def close(self) -> None:
        self._executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self._thread.join(timeout=2.0)


class DsrRegisterIO:
    """`RegisterIO`(get·set·drl_start) over Doosan 서비스. **실기 미검증.**

    - `get(address)`: 응답 `value`를 `decode_register_byte`로 푼 0~255. 실패 응답·시간 초과는 예외(값을 만들지 않음).
    - `set(address, value)`: 0~127 정수만. 실패 응답이면 False, 시간 초과·서비스 없음은 예외(썼는지 모름).
    - `drl_start(code)`: STANDBY가 아니면 `DsrError`(`ROBOT_NOT_STANDBY`), DRL이 STOP·LAST가 아니면(PLAY·HOLD 등)
      `DsrError`(`DRL_BUSY`)이며 둘 다 `drl_start`를 부르지 않는다. 실패 응답이면 False.
    """

    def __init__(self, caller: ServiceCaller, namespace: str = 'dsr01', srv_types: Mapping | None = None,
                 call_timeout_s: float = 1.0, drl_start_timeout_s: float = 5.0, register_timeout_ms: int = 1000):
        if not call_timeout_s > 0.0 or not drl_start_timeout_s > 0.0:
            raise ValueError('제한시간은 양수')
        self.caller = caller
        self.ns = '/' + namespace.strip('/')
        self._types = dict(srv_types) if srv_types is not None else None
        self.call_timeout_s = float(call_timeout_s)
        self.drl_start_timeout_s = float(drl_start_timeout_s)
        self.register_timeout_ms = int(register_timeout_ms)

    def _call(self, key: str, timeout_s: float | None = None, **fields):
        if self._types is None:
            self._types = dsr_service_types()
        return self.caller.call(self._types[key], f'{self.ns}/{SERVICES[key][1]}',
                                self.call_timeout_s if timeout_s is None else timeout_s, **fields)

    @staticmethod
    def _succeeded(res) -> bool:
        return bool(getattr(res, 'success', True))

    # ------------------------------------------------------------ RegisterIO
    def get(self, address: int) -> int:
        res = self._call('get_register', address=int(address), timeout_ms=self.register_timeout_ms)
        if not self._succeeded(res):
            raise DsrError('SERVICE_FAILED', f'get_output_register_int({address})')
        return decode_register_byte(res.value)

    def set(self, address: int, value: int) -> bool:
        if type(value) is not int or not 0 <= value <= PC_VALUE_MAX:
            raise ValueError(f'register value must be an integer in 0..{PC_VALUE_MAX}: {value!r}')
        res = self._call('set_register', address=int(address), value=value)
        return self._succeeded(res)

    def drl_start(self, code: str) -> bool:
        state = self.robot_state()
        if state != ROBOT_STATE_STANDBY:
            raise DsrError('ROBOT_NOT_STANDBY', f'robot_state={state}')
        drl = self.drl_state()
        if drl not in DRL_IDLE_STATES:
            raise DsrError('DRL_BUSY', f'drl_state={drl}')
        res = self._call('drl_start', self.drl_start_timeout_s, robot_system=0, code=code)
        return self._succeeded(res)

    # ------------------------------------------------------------ 상태
    def robot_state(self) -> int:
        res = self._call('robot_state')
        if not self._succeeded(res):
            raise DsrError('SERVICE_FAILED', 'get_robot_state')
        return int(res.robot_state)

    def drl_state(self) -> int:
        res = self._call('drl_state')
        if not self._succeeded(res):
            raise DsrError('SERVICE_FAILED', 'get_drl_state')
        return int(res.drl_state)

    def robot_standby(self) -> bool:
        """`GripperSession.start(robot_standby)`에 넘길 값."""
        return self.robot_state() == ROBOT_STATE_STANDBY
