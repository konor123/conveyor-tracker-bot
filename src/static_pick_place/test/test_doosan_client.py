"""doosan_client.py 단위 시험. dsr_msgs2·ROS 없이 서비스 호출을 가짜로 바꿔 확인한다."""
import sys
import types

import numpy as np
import pytest


class _Req:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _srv(name):
    return type(name, (), {'Request': _Req})


_mod = types.ModuleType('dsr_msgs2.srv')
for _n in ('DrlStart', 'GetDrlState', 'GetOutputRegisterInt', 'GetCurrentPosx', 'MoveLine', 'Ikin',
           'GetLastAlarm', 'GetRobotMode', 'GetRobotState'):
    setattr(_mod, _n, _srv(_n))
sys.modules.setdefault('dsr_msgs2', types.ModuleType('dsr_msgs2'))
sys.modules['dsr_msgs2.srv'] = _mod

from static_pick_place import doosan_client as dc  # noqa: E402
from static_pick_place.plan import DOWN_QUAT_XYZW  # noqa: E402


def test_render_drl_fills_template_and_has_no_exit():
    code = dc.render_gripper_drl(750, 150, True)
    assert 'TARGET = 750' in code and 'CURRENT = 150' in code and 'GRASP = True' in code
    assert not any(k in code for k in ('__PULSE__', '__CURRENT__', '__GRASP__'))
    body = '\n'.join(line.split('#')[0] for line in code.splitlines())
    assert 'exit(' not in body                       # exit()는 DRL을 PLAY로 남겼다(2026-11-09)
    assert 'GRASP = False' in dc.render_gripper_drl(600, 100, False)


@pytest.mark.parametrize('pulse, current', [(-1, 100), (751, 100), (600, 0), (600, 151), (600.0, 100)])
def test_render_drl_rejects_outside_envelope(pulse, current):
    with pytest.raises(ValueError):
        dc.render_gripper_drl(pulse, current, False)


class _FakeRobot(dc.DoosanClient):
    """서비스 이름별 응답을 대본으로 돌려준다. 레지스터 15는 호출마다 대본 값을 하나씩 소비한다."""

    def __init__(self, reg15, payload, drl_state=1, posx=(373.0, 0.0, 405.0, 0.0, 180.0, 0.0)):
        self.node, self.ns, self._clients = None, 'dsr01', {}
        self.reg15, self.payload, self.drl_state, self.posx = list(reg15), payload, drl_state, list(posx)
        self.calls = []

    def _call(self, srv_type, name, timeout_s=None, **f):
        self.calls.append((name, f))
        r = types.SimpleNamespace(success=True)
        if name == 'drl/get_drl_state':
            st = self.drl_state
            r.drl_state = (st.pop(0) if len(st) > 1 else st[0]) if isinstance(st, list) else st
        elif name == 'plc/get_output_register_int':
            v = self.reg15.pop(0) if f['address'] == 15 and len(self.reg15) > 1 else (
                self.reg15[0] if f['address'] == 15 else self.payload[f['address'] - 16])
            r.value = v << 24
        elif name == 'aux_control/get_current_posx':
            r.task_pos_info = [types.SimpleNamespace(data=self.posx + [2.0])]
        elif name == 'motion/move_line':
            self.posx = list(f['pos'])
        return r


def test_gripper_ignores_previous_result_until_reset():
    payload = [2, 238, 2, 200, 0, 150, 2, 0]          # 목표 750, 위치 712(막힘), 150 mA, 상태 2
    # 지난 실행의 250이 남아 있다 → DRL이 0으로 지운 뒤 250을 받아야 한다
    robot = _FakeRobot([250, 250, 0, 0, 250], payload)
    g = robot.gripper(750, 150, grasp=True, timeout_s=2.0)
    assert (g.target_pulse, g.position_pulse, g.current_ma, g.status) == (750, 712, 150, 2)
    starts = [i for i, (n, _) in enumerate(robot.calls) if n == 'drl/drl_start']
    assert len(starts) == 1


def test_gripper_failure_marker_and_busy_drl():
    with pytest.raises(dc.RobotError) as e:
        _FakeRobot([0, 247], [0] * 8).gripper(600, 100, timeout_s=2.0)
    assert e.value.reason == 'GRIPPER_MODE_NOT_5'
    with pytest.raises(dc.RobotError) as e:
        _FakeRobot([0], [0] * 8, drl_state=0).gripper(600, 100, timeout_s=2.0)
    assert e.value.reason == 'DRL_BUSY'


def test_move_line_sends_doosan_units_and_checks_reached():
    robot = _FakeRobot([0], [0] * 8)
    reached = robot.move_line((0.6795, -0.0161, 0.2844), DOWN_QUAT_XYZW, 0.020, 0.040)
    name, f = [c for c in robot.calls if c[0] == 'motion/move_line'][0]
    assert f['pos'][:3] == pytest.approx([679.5, -16.1, 284.4])
    assert abs(f['pos'][4]) == pytest.approx(180.0)                  # 수직 아래
    assert f['vel'][0] == pytest.approx(20.0) and f['acc'][0] == pytest.approx(40.0)
    assert (f['ref'], f['mode']) == (0, 0)
    assert np.allclose(reached, (0.6795, -0.0161, 0.2844))


def test_gripper_returns_only_after_drl_program_stopped():
    """결과 250을 쓴 직후 DRL은 아직 PLAY다. 그때 보낸 move_line이 무시됐다(2026-11-09 1회차)."""
    robot = _FakeRobot([0, 250], [2, 88, 2, 88, 0, 6, 1, 0], drl_state=[1, 0, 0, 1])
    g = robot.gripper(600, 100, timeout_s=2.0)
    assert g.status == 1
    assert robot.drl_state == [1]                      # 시작 전 1회 + 종료 확인 때 0, 0, 1을 모두 소비


def test_drl_read_timeout_and_grasp_needs_motion():
    code = dc.render_gripper_drl(750, 150, True, 0.05)
    assert 'READ_TIMEOUT = 0.25' in dc.render_gripper_drl(750, 150, True, 0.25)
    assert 'abs(pos - last) <= 2 and (abs(pos - start) > 10 or signed * 5 >= CURRENT * 4)' in code
    for bad in (0.001, 1.0):
        with pytest.raises(ValueError):
            dc.render_gripper_drl(600, 100, False, bad)


def test_tcp_pose_reads_again_when_the_driver_answers_all_zeros():
    robot = _FakeRobot([0], [0] * 8)
    answers = [[0.0] * 6, [0.0] * 6, [373.0, 0.0, 405.0, 0.0, 180.0, 0.0]]
    real_call = robot._call

    def call(srv_type, name, timeout_s=None, **f):
        if name == 'aux_control/get_current_posx':
            return types.SimpleNamespace(success=True, task_pos_info=[types.SimpleNamespace(data=answers.pop(0) + [2.0])])
        return real_call(srv_type, name, timeout_s, **f)
    robot._call = call
    pos, _ = robot.tcp_pose()
    assert np.allclose(pos, [0.373, 0.0, 0.405])
    answers[:] = [[0.0] * 6] * 3
    with pytest.raises(dc.RobotError) as e:
        robot.tcp_pose()
    assert e.value.reason == 'POSE_READ_FAILED'
