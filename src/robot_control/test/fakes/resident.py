"""상주 루프 DRL과 그리퍼의 대역(시간 모델 포함). 실제 하드웨어 측정이 아니다.

`drl/gripper_resident.drl`의 루프 순서를 따라 한다: seq가 바뀌면 (전류 쓰기) + 위치 쓰기,
예약(arm) 중에는 Modbus 읽기를 멈추고 `spin_s`마다 seq만 확인, 그 밖에는 상태 읽기 1왕복.
Modbus 왕복마다 `rtt_s`가 걸린다. 쓰기 프레임이 그리퍼에 닿는 시점은 왕복 시작 뒤 `write_effect_s`다.
"""
from __future__ import annotations

from collections import defaultdict

from robot_control.resident_transport import ResidentRegs


class FakeResidentController:
    def __init__(self, clock, rtt_s: float = 0.58, spin_s: float = 0.005, write_effect_s: float | None = None,
                 pulse_rate: float = 1500.0, present_pulse: int = 600, stone_pulse: int | None = None,
                 init_status: int = 1, flush_s: float = 0.5):
        self.clock = clock
        self.rtt_s = float(rtt_s)
        self.spin_s = float(spin_s)
        self.write_effect_s = self.rtt_s if write_effect_s is None else float(write_effect_s)
        self.pulse_rate = float(pulse_rate)
        self.stone_pulse = stone_pulse
        self.init_status = init_status
        self.flush_s = float(flush_s)
        self.regs = ResidentRegs()
        self.reg: dict[int, int] = defaultdict(int)
        self.running = False
        self.drl_starts = 0
        self.modbus_round_trips = 0
        self.motion_starts: list[float] = []         # 그리퍼가 목표를 받은 시각
        self._pos = float(present_pulse)
        self._pos_t = clock.steady_now()
        self._target: float | None = None
        self._goal_current = -1
        self._done_seq = 0
        self._t = 0.0
        self._pending = None

    # ------------------------------------------------------------ RegisterIO
    def get(self, address: int) -> int:
        self._advance()
        return int(self.reg[address])

    def set(self, address: int, value: int) -> bool:
        self._advance()
        self.reg[address] = int(value)
        return True

    def drl_start(self, code: str) -> bool:
        self._advance()
        self.drl_starts += 1
        r = self.regs
        self.reg[r.init] = self.reg[r.arm] = self.reg[r.stop] = 0
        self._done_seq = self.reg[r.seq]
        self._t = self.clock.steady_now()
        self.running = True

        def publish_init():
            self.reg[r.init] = self.init_status
            self.reg[r.mode], self.reg[r.firmware], self.reg[r.error] = 5, 12, 0
            self.reg[r.done_seq] = self._done_seq
            if self.init_status == 1:
                self._publish_state(self._t)
            else:
                self.running = False

        self.modbus_round_trips += 3                 # ident, RAM, 상태
        self._pending = (self._t + self.flush_s + 3 * self.rtt_s, publish_init)
        return True

    # ------------------------------------------------------------ 그리퍼 물리
    def pulse_at(self, t: float) -> int:
        self._move_to(t)
        return int(round(self._pos))

    @property
    def stalled(self) -> bool:
        return self.stone_pulse is not None and self._target is not None and self._pos >= self.stone_pulse \
            and self._target > self.stone_pulse

    def _move_to(self, t: float) -> None:
        if self._target is not None and t > self._pos_t:
            step = self.pulse_rate * (t - self._pos_t)
            if self._target >= self._pos:
                limit = self._target if self.stone_pulse is None else min(self._target, max(self.stone_pulse, self._pos))
                self._pos = min(limit, self._pos + step)
            else:
                self._pos = max(self._target, self._pos - step)
        self._pos_t = max(self._pos_t, t)

    def _publish_state(self, t: float) -> None:
        r = self.regs
        pos = self.pulse_at(t)
        current = max(self._goal_current, 0) if self.stalled else 6
        self.reg[r.st_pos_hi], self.reg[r.st_pos_hi + 1] = (pos >> 8) & 255, pos & 255
        self.reg[r.st_cur_hi], self.reg[r.st_cur_hi + 1] = (current >> 8) & 255, current & 255
        self.reg[r.alive] = (self.reg[r.alive] + 1) & 255

    # ------------------------------------------------------------ 루프
    def _advance(self) -> None:
        now = self.clock.steady_now()
        while self.running:
            if self._pending is not None:
                t_end, apply = self._pending
                if t_end > now:
                    return
                self._pending = None
                self._t = t_end
                apply()
                continue
            self._begin(self._t)

    def _payload(self, hi: int) -> int:
        return (self.reg[hi] & 127) * 128 + (self.reg[hi + 1] & 127)

    def _begin(self, t: float) -> None:
        r = self.regs
        if self.reg[r.stop] == 1:
            self.running = False
            return
        seq = self.reg[r.seq] & 127
        cur = self._payload(r.cur_hi)
        if seq != self._done_seq:
            pos = self._payload(r.pos_hi)
            n_before = 1 if cur != self._goal_current else 0
            effect_t = t + n_before * self.rtt_s + self.write_effect_s

            def execute():
                self._move_to(effect_t)
                self._goal_current, self._target = cur, float(pos)
                self.motion_starts.append(effect_t)
                self._done_seq = seq
                self.reg[r.done_seq], self.reg[r.arm] = seq, 0

            self.modbus_round_trips += n_before + 1
            self._pending = (t + (n_before + 1) * self.rtt_s, execute)
        elif self.reg[r.arm] == 1:
            if cur != self._goal_current:
                self.modbus_round_trips += 1

                def prewrite():
                    self._goal_current = cur
                self._pending = (t + self.rtt_s, prewrite)
            else:
                def tick():
                    self.reg[r.alive] = (self.reg[r.alive] + 1) & 255
                self._pending = (t + self.spin_s, tick)
        else:
            self.modbus_round_trips += 1
            self._pending = (t + self.rtt_s, lambda: self._publish_state(t + self.rtt_s))
