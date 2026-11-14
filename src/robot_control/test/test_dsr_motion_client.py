"""Doosan 단위 팔 client. **실기 미검증** 코드의 U 시험.

서비스 호출은 가짜 caller가 받는다. 실제 `dsr_msgs2` 형식으로 보내는 확인은(C)이다.
"""
from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from robot_control.dsr_motion_client import MOTION_SERVICES, DsrMotionClient
from robot_control.dsr_register_io import DsrError



class FakeCaller:
    def __init__(self):
        self.calls: list[tuple[str, str, float, dict]] = []
        self.success: dict[str, bool] = {}
        self.posx_data = [500.0, 0.0, 300.0, 0.0, 180.0, 0.0, 2.0]   # 6개 자세 + solution space
        self.timeout: set[str] = set()

    def call(self, srv_type, name, timeout_s, **fields):
        self.calls.append((srv_type.__name__, name, timeout_s, dict(fields)))
        tail = name.split('/', 2)[-1]
        if tail in self.timeout:
            raise DsrError('SERVICE_TIMEOUT', name)
        ok = self.success.get(tail, True)
        if tail == 'aux_control/get_current_posx':
            info = [] if self.posx_data is None else [SimpleNamespace(data=list(self.posx_data))]
            return SimpleNamespace(task_pos_info=info, success=ok)
        if tail == 'system/get_robot_state':
            return SimpleNamespace(robot_state=1, success=ok)
        if tail == 'drl/get_drl_state':
            return SimpleNamespace(drl_state=3, success=ok)
        return SimpleNamespace(success=ok)


def types():
    return {key: type(type_name, (), {}) for key, (type_name, _) in MOTION_SERVICES.items()}


def client(**kw):
    caller = FakeCaller()
    return DsrMotionClient(caller, srv_types=types(), **kw), caller


def test_move_line_async_fills_every_field_in_doosan_units():
    c, caller = client()
    posx = np.array([500.0, 0.0, 250.0, 0.0, 180.0, 0.0])                # motion_adapter 출력 형태(numpy)
    assert c.move_line_async(posx, [np.float64(30.0), 20], [50, np.float64(40.1)]) is True
    (type_name, name, timeout_s, f), = caller.calls
    assert (type_name, name, timeout_s) == ('MoveLine', '/dsr01/motion/move_line', 1.0)
    assert set(f) == {'pos', 'vel', 'acc', 'time', 'radius', 'ref', 'mode', 'blend_type', 'sync_type'}
    assert f['pos'] == [500.0, 0.0, 250.0, 0.0, 180.0, 0.0] and f['vel'] == [30.0, 20.0] and f['acc'] == [50.0, 40.1]
    assert all(type(v) is float for v in f['pos'] + f['vel'] + f['acc'])   # numpy·정수도 float로 보낸다
    assert (f['time'], f['radius'], f['ref'], f['mode'], f['blend_type'], f['sync_type']) == (0.0, 0.0, 0, 0, 0, 1)
    assert type(f['time']) is float and type(f['sync_type']) is int


@pytest.mark.parametrize('posx, vel, acc', [
    ([0.0] * 5, [30.0, 20.0], [50.0, 40.0]),
    ([0.0] * 6, [30.0], [50.0, 40.0]),
    ([0.0] * 6, [30.0, 20.0], [50.0, 40.0, 1.0]),
    ([0.0] * 5 + [math.nan], [30.0, 20.0], [50.0, 40.0]),
    ([0.0] * 6, [math.inf, 20.0], [50.0, 40.0]),
])
def test_move_line_async_rejects_bad_lengths_and_nonfinite_without_sending(posx, vel, acc):
    c, caller = client()
    with pytest.raises(ValueError):
        c.move_line_async(posx, vel, acc)
    assert caller.calls == []


def test_move_stop_uses_soft_stop_and_reports_response():
    c, caller = client()
    assert c.move_stop() is True
    assert caller.calls[-1][:2] == ('MoveStop', '/dsr01/motion/move_stop') and caller.calls[-1][3] == {'stop_mode': 2}
    assert type(caller.calls[-1][3]['stop_mode']) is int
    caller.success['motion/move_stop'] = False
    caller.success['motion/move_line'] = False
    assert c.move_stop() is False
    assert c.move_line_async([0.0] * 6, [30.0, 20.0], [50.0, 40.0]) is False


def test_current_posx_takes_first_six_values_of_first_array():
    c, caller = client(namespace='/robotA/')
    assert c.current_posx() == [500.0, 0.0, 300.0, 0.0, 180.0, 0.0]
    assert caller.calls[-1][:2] == ('GetCurrentPosx', '/robotA/aux_control/get_current_posx')
    assert caller.calls[-1][3] == {'ref': 0}


@pytest.mark.parametrize('data', [None, [1.0] * 5, [1.0, 2.0, math.nan, 4.0, 5.0, 6.0, 2.0]])
def test_current_posx_bad_response_raises(data):
    c, caller = client()
    caller.posx_data = data
    with pytest.raises(DsrError) as err:
        c.current_posx()
    assert err.value.reason == 'SERVICE_BAD_RESPONSE'


def test_states_and_failures():
    c, caller = client()
    assert c.robot_state() == 1 and c.drl_state() == 3
    assert [n for _, n, _, _ in caller.calls] == ['/dsr01/system/get_robot_state', '/dsr01/drl/get_drl_state']
    for tail, read in (('system/get_robot_state', c.robot_state), ('drl/get_drl_state', c.drl_state),
                       ('aux_control/get_current_posx', c.current_posx)):
        caller.success[tail] = False
        with pytest.raises(DsrError) as err:
            read()
        assert err.value.reason == 'SERVICE_FAILED'
    caller.timeout.add('motion/move_line')
    with pytest.raises(DsrError) as err:                       # 시간 초과는 수락 여부를 모른다: 예외
        c.move_line_async([0.0] * 6, [30.0, 20.0], [50.0, 40.0])
    assert err.value.reason == 'SERVICE_TIMEOUT'


def test_config_validation():
    with pytest.raises(ValueError):
        DsrMotionClient(FakeCaller(), call_timeout_s=0.0)
    with pytest.raises(ValueError):
        DsrMotionClient(FakeCaller(), stop_mode=4)
