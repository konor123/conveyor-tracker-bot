"""상주 루프 DRL 본문(`drl/gripper_resident.drl`)을 그대로 실행하는 가짜 제어기 환경. 시간 모델은 없다.

DRL이 부르는 `flange_serial_*`, `get/set_output_register_int`, `wait`를 제공하고 그리퍼 Modbus 응답을 흉내 낸다.
PC 쪽 레지스터 쓰기는 루프 조건(레지스터 25) 읽기 횟수에 맞춘 대본으로 넣는다.
"""
from __future__ import annotations

from robot_control.gripper_adapter import crc16_modbus, encode7


class ResidentDrlEnv:
    def __init__(self, model=35074, mode=5, torque=1, error=0, goal_current=100, pos=600,
                 fail_current_write=False, fail_position_write=False, max_loops=50):
        self.words = {0: model, 3: 12, 5: mode << 8, 256: torque, 259: error, 275: goal_current,
                      287: 6, 290: pos, 291: 0}
        self.fail_current_write = fail_current_write
        self.fail_position_write = fail_position_write
        self.max_loops = max_loops
        self.reg: dict[int, int] = {}
        self.sets: list[tuple[int, int]] = []          # DRL의 set_output_register_int 순서
        self.modbus_writes: list[tuple[int, int, int]] = []   # (기능 코드, 주소, 값)
        self.script: dict[int, dict[int, int]] = {}    # 루프 조건 읽기 횟수 → PC가 쓰는 레지스터
        self._loops = 0
        self._req = None

    # ------------------------------------------------------------ PC 대본
    def command(self, at_loop: int, seq: int, pulse: int, current: int, arm: bool = False) -> None:
        hi_p, lo_p = encode7(pulse)
        hi_c, lo_c = encode7(current)
        self.script.setdefault(at_loop, {}).update({11: hi_p, 12: lo_p, 13: hi_c, 14: lo_c, 10: seq,
                                                    24: 1 if arm else 0})

    # ------------------------------------------------------------ DRL 전역
    def get_output_register_int(self, address):
        if address == 25:
            self._loops += 1
            for a, v in self.script.pop(self._loops, {}).items():
                self.reg[a] = v
            if self._loops >= self.max_loops:
                return 1
        return self.reg.get(address, 0)

    def set_output_register_int(self, address, value):
        self.sets.append((address, int(value)))
        self.reg[address] = int(value)

    def flange_serial_write(self, data):
        self._req = bytes(data)

    def flange_serial_read(self, timeout):
        req, self._req = self._req, None
        if req is None:
            return 0, b''
        addr = req[2] * 256 + req[3]
        if req[1] == 3:
            body = bytes([1, 3, 2 * req[5]]) + b''.join(
                self.words.get(addr + i, 0).to_bytes(2, 'big') for i in range(req[5]))
        elif req[1] == 6:
            value = req[4] * 256 + req[5]
            self.modbus_writes.append((6, addr, value))
            if addr == 275 and self.fail_current_write:
                return 0, b''
            self.words[addr] = value
            body = req[:6]
        elif req[1] == 16:
            value = req[7] * 256 + req[8]
            self.modbus_writes.append((16, addr, value))
            if self.fail_position_write:
                return 0, b''
            self.words[290] = value
            body = req[:6]
        else:
            return 0, b''
        out = body + crc16_modbus(body)
        return len(out), out

    def run(self, code: str) -> None:
        env = {'flange_serial_open': lambda **kw: None, 'flange_serial_close': lambda: None,
               'flange_serial_write': self.flange_serial_write, 'flange_serial_read': self.flange_serial_read,
               'get_output_register_int': self.get_output_register_int,
               'set_output_register_int': self.set_output_register_int, 'wait': lambda s: None,
               'DR_EIGHTBITS': 8, 'DR_PARITY_NONE': 0, 'DR_STOPBITS_ONE': 1}
        exec(compile(code, 'gripper_resident.drl', 'exec'), env)
