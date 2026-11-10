"""cycle.drl 단위 시험. DRL 본문을 가짜 제어기 함수(flange_serial_*, movel, 레지스터) 위에서 그대로 실행한다.

실기 측정이 아니다. 시간은 Modbus 왕복 0.58 s(2026-11-09 측정)와 이동 사다리꼴 모델로 센 값이다.
"""
import math
import sys
import types

import pytest


class _Req:
    def __init__(self, **kw):
        self.__dict__.update(kw)


_mod = sys.modules.get('dsr_msgs2.srv') or types.ModuleType('dsr_msgs2.srv')
for _n in ('DrlStart', 'GetDrlState', 'GetOutputRegisterInt', 'GetCurrentPosx'):
    if not hasattr(_mod, _n):
        setattr(_mod, _n, type(_n, (), {'Request': _Req}))
sys.modules.setdefault('dsr_msgs2', types.ModuleType('dsr_msgs2'))
sys.modules['dsr_msgs2.srv'] = _mod

from robot_control.gripper_adapter import crc16_modbus  # noqa: E402
from static_pick_place import doosan_client as dc  # noqa: E402
from static_pick_place.plan import DOWN_QUAT_XYZW, PickPlaceParams, plan_pick_place  # noqa: E402

STONE_TOP = (0.5772, -0.0384, -0.0165)
CAP_RIM = (0.5878, 0.0895, 0.0841)
MEASURED_GRIPPER_S, MEASURED_CYCLE_S = 19.76, 36.35      # 2026-11-09 3회차(pp-run3.jsonl)
MOVE_OVERHEAD_S = 0.144                                  # 같은 로그의 구간별 측정 − 사다리꼴 모델


class FakeController:
    """DRL이 부르는 제어기 함수와 그리퍼. 왕복마다 `rtt_s`, 응답이 없으면 읽기 대기 시간만큼 시계가 간다."""

    def __init__(self, rtt_s=0.58, stone=659, pos=600, model=35074, mode=5, torque=1, error=0,
                 goal_current=100, respond=True, fail_move_at=None, pulse_rate=1500.0):
        self.rtt_s, self.stone, self.pos, self.model, self.mode = rtt_s, stone, float(pos), model, mode
        self.torque, self.error, self.goal_current, self.respond = torque, error, goal_current, respond
        self.fail_move_at, self.pulse_rate = fail_move_at, pulse_rate
        self.t = self.gripper_s = self.move_s = 0.0
        self.target = None
        self.pos_t = 0.0
        self.reg, self.moves, self.round_trips, self.position_writes = {}, [], 0, 0
        self._req = None

    # ---- 그리퍼 물리
    def _advance(self):
        if self.target is not None:
            step = self.pulse_rate * (self.t - self.pos_t)
            if self.target >= self.pos:
                limit = self.target if self.stone is None else min(self.target, max(self.stone, self.pos))
                self.pos = min(limit, self.pos + step)
            else:
                self.pos = max(self.target, self.pos - step)
        self.pos_t = self.t

    def _stalled(self):
        return self.stone is not None and self.target is not None and self.target > self.stone <= self.pos

    def _words(self, addr, count):
        self._advance()
        pos = int(round(self.pos))
        table = {0: self.model, 5: self.mode << 8, 256: self.torque, 259: self.error, 275: self.goal_current,
                 287: (self.goal_current - 1) if self._stalled() else 6, 290: pos & 0xFFFF, 291: pos >> 16}
        return [table.get(addr + i, 0) for i in range(count)]

    # ---- DRL 전역
    def flange_serial_write(self, data):
        self._req = bytes(data)

    def flange_serial_read(self, timeout):
        dt = self.rtt_s if (self._req is not None and self.respond) else timeout
        self.t += dt
        self.gripper_s += dt
        req, self._req = self._req, None
        if req is None or not self.respond:
            return 0, b''
        self.round_trips += 1
        addr = req[2] * 256 + req[3]
        if req[1] == 3:
            body = bytes([1, 3, 2 * req[5]]) + b''.join(w.to_bytes(2, 'big') for w in self._words(addr, req[5]))
        else:
            self._advance()
            if req[1] == 6 and addr == 256:
                self.torque = req[5]
            elif req[1] == 6 and addr == 275:
                self.goal_current = req[4] * 256 + req[5]
            elif req[1] == 16 and addr == 282:
                self.target = float(req[7] * 256 + req[8])
                self.position_writes += 1
            body = req[:6]
        out = body + crc16_modbus(body)
        return len(out), out

    def movel(self, pos, vel=None, acc=None):
        index = len(self.moves)
        if self.fail_move_at == index:
            return -1
        prev = self.moves[-1][0] if self.moves else [373.0, 0.0, 405.0]
        d = math.dist(prev[:3], pos[:3])
        dt = (d / vel[0] + vel[0] / acc[0]) if d >= vel[0] ** 2 / acc[0] else 2 * math.sqrt(d / acc[0])
        self.t += dt + MOVE_OVERHEAD_S
        self.move_s += dt + MOVE_OVERHEAD_S
        self.moves.append((list(pos), list(vel), list(acc)))
        return 0

    def run(self, code):
        env = {'flange_serial_open': lambda **kw: None, 'flange_serial_close': lambda: None,
               'flange_serial_write': self.flange_serial_write, 'flange_serial_read': self.flange_serial_read,
               'set_output_register_int': self.reg.__setitem__, 'movel': self.movel,
               'posx': lambda *a: list(a), 'DR_EIGHTBITS': 8, 'DR_PARITY_NONE': 0, 'DR_STOPBITS_ONE': 1}
        exec(compile(code, 'cycle.drl', 'exec'), env)
        return self.reg[15]


def steps(**kw):
    return plan_pick_place(STONE_TOP, CAP_RIM, PickPlaceParams(**kw))


def run(ctrl=None, **kw):
    ctrl = ctrl or FakeController()
    return ctrl, ctrl.run(dc.render_cycle_drl(steps(**kw), DOWN_QUAT_XYZW))


@pytest.mark.level('U')
def test_cycle_runs_all_steps_in_one_program_and_skips_the_first_open():
    plan = steps()
    code = dc.render_cycle_drl(plan, DOWN_QUAT_XYZW)
    assert '__' not in code
    body = '\n'.join(line.split('#')[0] for line in code.splitlines())
    assert 'exit(' not in body and 'while' not in body        # 스스로 끝난다(2026-11-09 PLAY로 남은 DRL)

    ctrl = FakeController()
    assert ctrl.run(code) == 250
    moves = [s for s in plan if s.kind == 'move']
    assert len(ctrl.moves) == len(moves) == 7
    for (pos, vel, acc), s in zip(ctrl.moves, moves):
        assert pos[:3] == pytest.approx([x * 1000.0 for x in s.pos_m], abs=1e-3)
        assert abs(pos[4]) == pytest.approx(180.0)            # 수직 아래
        assert (vel[0], acc[0]) == pytest.approx((s.v_mps * 1000.0, s.a_mps2 * 1000.0))
    assert ctrl.reg[14] == len(plan)
    assert ctrl.position_writes == 2                          # 파지와 해제뿐, 이미 열린 첫 열기는 쓰지 않는다
    assert (ctrl.reg[16] * 256 + ctrl.reg[17], ctrl.reg[18] * 256 + ctrl.reg[19], ctrl.reg[20]) == (659, 149, 2)
    assert (ctrl.reg[21] * 256 + ctrl.reg[22], ctrl.reg[23]) == (600, 1)
    assert int(round(ctrl.pos)) == 600 and ctrl.goal_current == 150


@pytest.mark.level('U')
def test_cycle_gripper_time_is_well_below_the_measured_one_shot_time():
    ctrl, result = run()
    assert result == 250
    assert ctrl.gripper_s < 0.45 * MEASURED_GRIPPER_S
    assert ctrl.move_s == pytest.approx(16.59, abs=0.1)       # 이동 모델이 3회차 측정 합과 맞는다
    assert ctrl.t < 0.7 * MEASURED_CYCLE_S


@pytest.mark.level('U')
def test_first_open_is_sent_when_the_gripper_is_not_open():
    ctrl, result = run(FakeController(pos=0))
    assert result == 250 and ctrl.position_writes == 3


@pytest.mark.level('U')
def test_empty_grasp_stops_before_lifting():
    ctrl, result = run(FakeController(stone=None))
    assert result == 248
    assert len(ctrl.moves) == 2 and ctrl.reg[20] == 1         # 공깃돌 위·하강까지만, 끝점 도달
    assert ctrl.reg[14] == 4


@pytest.mark.level('U')
@pytest.mark.parametrize('kw, code', [({'respond': False}, 241), ({'model': 1}, 246), ({'mode': 3}, 247),
                                      ({'error': 4}, 244)])
def test_gripper_not_ready_means_the_arm_never_moves(kw, code):
    ctrl, result = run(FakeController(**kw))
    assert result == code and ctrl.moves == [] and ctrl.position_writes == 0


@pytest.mark.level('U')
def test_move_failure_stops_the_program():
    ctrl, result = run(FakeController(fail_move_at=2))
    assert result == 249 and len(ctrl.moves) == 2
    assert ctrl.position_writes == 1                          # 파지까지만, 해제 열기는 하지 않는다


@pytest.mark.level('U')
def test_render_rejects_out_of_range_values():
    from dataclasses import replace
    plan = steps()
    for bad in (replace(plan[0], pulse=751), replace(plan[0], current_ma=151), replace(plan[1], v_mps=0.3),
                replace(plan[1], a_mps2=0.0), replace(plan[0], kind='other')):
        with pytest.raises(ValueError):
            dc.render_cycle_drl([bad, *plan[1:]], DOWN_QUAT_XYZW)
    with pytest.raises(ValueError):
        dc.render_cycle_drl([], DOWN_QUAT_XYZW)
    with pytest.raises(ValueError):
        dc.render_cycle_drl(plan, DOWN_QUAT_XYZW, read_timeout_s=1.0)


class _FakeRobot(dc.DoosanClient):
    """레지스터 15(결과)·14(진행)를 호출마다 대본에서 하나씩 돌려준다."""

    def __init__(self, reg15, reg14, payload, drl_state=1):
        self.node, self.ns, self._clients = None, 'dsr01', {}
        self.script = {15: list(reg15), 14: list(reg14)}
        self.payload, self.drl_state, self.calls = payload, drl_state, []

    def _call(self, srv_type, name, timeout_s=None, **f):
        self.calls.append((name, f))
        r = types.SimpleNamespace(success=True)
        if name == 'drl/get_drl_state':
            st = self.drl_state
            r.drl_state = (st.pop(0) if len(st) > 1 else st[0]) if isinstance(st, list) else st
        elif name == 'plc/get_output_register_int':
            a = f['address']
            if a in self.script:
                v = self.script[a].pop(0) if len(self.script[a]) > 1 else self.script[a][0]
            else:
                v = self.payload[a - 16]
            r.value = v << 24
        return r


@pytest.mark.level('U')
def test_run_cycle_reports_each_step_and_waits_for_program_stop(monkeypatch):
    monkeypatch.setattr(dc.time, 'sleep', lambda s: None)
    plan = steps()
    payload = [2, 147, 0, 149, 2, 2, 88, 1]                   # 파지 659·149 mA·막힘, 열기 600·도달
    robot = _FakeRobot([250, 0, 0, 0, 0, 250], [0, 1, 4, 10, 10], payload, drl_state=[1, 0, 1])
    seen = []
    out = robot.run_cycle(plan, DOWN_QUAT_XYZW, timeout_s=5.0, on_step=lambda n, s: seen.append(n))
    assert (out.grasp_pulse, out.grasp_current_ma, out.grasp_status, out.open_pulse, out.open_status) \
        == (659, 149, 2, 600, 1)
    assert seen == [s.name for s in plan]                     # 지난 실행의 250은 무시하고 모든 단계를 보고한다
    assert [n for n, _ in out.step_times_s] == seen
    assert sum(1 for n, _ in robot.calls if n == 'drl/drl_start') == 1
    assert robot.drl_state == [1]                             # STOP을 본 뒤에 돌아왔다


@pytest.mark.level('U')
def test_run_cycle_failure_names_the_step_and_busy_drl_is_rejected(monkeypatch):
    monkeypatch.setattr(dc.time, 'sleep', lambda s: None)
    plan = steps()
    with pytest.raises(dc.RobotError) as e:
        _FakeRobot([0, 0, 248], [0, 4, 4], [0] * 8).run_cycle(plan, DOWN_QUAT_XYZW, timeout_s=5.0)
    assert e.value.reason == 'GRASP_EMPTY' and 'close_grasp' in str(e.value)
    with pytest.raises(dc.RobotError) as e:
        _FakeRobot([0], [0], [0] * 8, drl_state=0).run_cycle(plan, DOWN_QUAT_XYZW, timeout_s=5.0)
    assert e.value.reason == 'DRL_BUSY'


@pytest.mark.level('U')
def test_stop_after_runs_only_the_leading_steps():
    from static_pick_place.pick_place import cycle_timeout_s, truncate_steps
    from static_pick_place.plan import PlanError
    plan = steps()
    assert [s.name for s in truncate_steps(plan, 'over_stone')] == ['open_before_pick', 'over_stone']
    with pytest.raises(PlanError):
        truncate_steps(plan, 'nope')

    ctrl = FakeController()
    assert ctrl.run(dc.render_cycle_drl(truncate_steps(plan, 'open_before_pick'), DOWN_QUAT_XYZW)) == 250
    assert ctrl.moves == [] and ctrl.position_writes == 0     # 그리퍼 확인만, 팔은 움직이지 않는다

    ctrl = FakeController()
    assert ctrl.run(dc.render_cycle_drl(truncate_steps(plan, 'close_grasp'), DOWN_QUAT_XYZW)) == 250
    assert len(ctrl.moves) == 2 and ctrl.reg[20] == 2
    assert cycle_timeout_s(plan, PickPlaceParams()) > 60.0
