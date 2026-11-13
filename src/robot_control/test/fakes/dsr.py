"""Doosan 드라이버 서비스 대역(`dsr_register_io.ServiceCaller` 모양). 실제 `/dsr01`에 연결하지 않는다.

레지스터 읽기 응답은 OFF-2 관찰처럼 값의 하위 1바이트를 최상위 바이트 자리(부호 있는 int32)에 담는다.
PC가 쓴 값을 읽을 때도 같은 표현이라고 가정한다(실기 미확인, OFF-2). 레지스터 저장소는 `regs`(get·set·drl_start를
가진 객체, 예: `FakeResidentController`)이거나 내부 dict다.
"""
from __future__ import annotations

from types import SimpleNamespace

from robot_control.dsr_register_io import DsrError


def encode_register_value(byte: int) -> int:
    raw = (int(byte) & 255) << 24
    return raw - (1 << 32) if raw >= (1 << 31) else raw


class FakeDsrServices:
    def __init__(self, regs=None, robot_state: int = 1, drl_state: int = 3):
        self.regs = regs
        self.store: dict[int, int] = {}
        self.robot_state = robot_state
        self.drl_state = drl_state
        self.calls: list[tuple[str, str, dict]] = []      # (형식 이름, 서비스 이름, 요청 필드)
        self.fail: set[str] = set()                         # 실패 응답(success=False)을 낼 서비스 끝 이름
        self.timeout: set[str] = set()                      # 시간 초과를 낼 서비스 끝 이름
        self.raw_override: dict[int, int] = {}              # 주소별 원시 응답 값

    def call(self, srv_type, name: str, timeout_s: float, **fields):
        self.calls.append((getattr(srv_type, '__name__', str(srv_type)), name, dict(fields)))
        tail = name.rsplit('/', 1)[-1]
        if tail in self.timeout:
            raise DsrError('SERVICE_TIMEOUT', f'{name} {timeout_s} s')
        ok = tail not in self.fail
        if tail == 'get_output_register_int':
            address = fields['address']
            if address in self.raw_override:
                return SimpleNamespace(value=self.raw_override[address], success=ok)
            value = self.regs.get(address) if self.regs is not None else self.store.get(address, 0)
            return SimpleNamespace(value=encode_register_value(value), success=ok)
        if tail == 'set_output_register_int':
            if ok:
                if self.regs is not None:
                    self.regs.set(fields['address'], fields['value'])
                else:
                    self.store[fields['address']] = fields['value']
            return SimpleNamespace(success=ok)
        if tail == 'drl_start':
            if ok and self.regs is not None:
                self.regs.drl_start(fields['code'])
            return SimpleNamespace(success=ok)
        if tail == 'get_robot_state':
            return SimpleNamespace(robot_state=self.robot_state, success=ok)
        if tail == 'get_drl_state':
            return SimpleNamespace(drl_state=self.drl_state, success=ok)
        raise AssertionError(f'unexpected service {name}')

    def names(self) -> list[str]:
        return [n for _, n, _ in self.calls]


def fake_types() -> dict:
    """`dsr_msgs2.srv` 대신 쓰는 이름표 형식."""
    from robot_control.dsr_register_io import SERVICES
    return {key: type(type_name, (), {}) for key, (type_name, _) in SERVICES.items()}
