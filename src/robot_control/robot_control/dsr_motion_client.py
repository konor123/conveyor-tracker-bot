"""Doosan 단위 팔 client(`doosan_backend.DoosanMotionClient`) over Doosan 드라이버 서비스. **실기 미검증.**

`robot_controller_node`에 연결하지 않는다. 서비스 이름은 OFF-2 드라이버(`doosan-robot2`)와 `static_pick_place.doosan_client`·`tools/bench/stream_bench.py`가 쓴 것과 같다.

- `<ns>/motion/move_line`: `pos` [mm×3, deg ZYZ×3], `vel` [mm/s, deg/s], `acc` [mm/s², deg/s²], `time` 0, `radius` 0,
  `ref` 0(DR_BASE), `mode` 0(절대), `blend_type` 0, `sync_type` 1(ASYNC: 지령을 받으면 이동 완료를 기다리지 않고 응답).
  배열 길이(6·2·2)와 유한값은 보내기 전에 확인한다. rclpy는 기본 설정에서 메시지 필드 검사를 하지 않아 잘못된
  길이·이름이 드라이버에 그대로 갈 수 있기 때문이다.
- `<ns>/motion/move_stop(stop_mode)`: 기본 2(soft stop, `tools/bench/stream_bench.py`와 같음).
- `<ns>/aux_control/get_current_posx(ref=0)`: `task_pos_info[0].data`의 앞 6개(7번째는 solution space).
- `<ns>/system/get_robot_state`, `<ns>/drl/get_drl_state`.

서비스 호출은 `dsr_register_io.ServiceCaller`(노드에서는 `RosServiceCaller` 하나를 `DsrRegisterIO`와 함께 쓴다)가 한다.
`move_line`·`move_stop`의 실패 응답은 False, 읽기의 실패 응답은 `DsrError`(`SERVICE_FAILED`), 비었거나 짧거나 비유한
위치 응답은 `DsrError`(`SERVICE_BAD_RESPONSE`), 시간 초과·서비스 없음은 caller의 `DsrError`다(값을 지어내지 않는다).
"""
from __future__ import annotations

import math
from typing import Mapping

from .dsr_register_io import DsrError, ServiceCaller

MOTION_SERVICES = {
    'move_line': ('MoveLine', 'motion/move_line'),
    'move_stop': ('MoveStop', 'motion/move_stop'),
    'current_posx': ('GetCurrentPosx', 'aux_control/get_current_posx'),
    'robot_state': ('GetRobotState', 'system/get_robot_state'),
    'drl_state': ('GetDrlState', 'drl/get_drl_state'),
}
REF_BASE = 0
MOVE_MODE_ABS = 0
SYNC_TYPE_ASYNC = 1
STOP_MODE_SOFT = 2                    # MoveStop.srv: 0 QSTOP_STO, 1 QSTOP, 2 soft stop, 3 HOLD


def dsr_motion_service_types() -> dict:
    """`dsr_msgs2.srv` 형식. 순수 시험에서는 부르지 않는다(가짜 형식을 주입)."""
    from dsr_msgs2 import srv
    return {key: getattr(srv, type_name) for key, (type_name, _) in MOTION_SERVICES.items()}


def _floats(values, n: int, name: str) -> list[float]:
    out = [float(v) for v in values]
    if len(out) != n or not all(math.isfinite(v) for v in out):
        raise ValueError(f'{name}: 유한한 실수 {n}개 필요: {values!r}')
    return out


class DsrMotionClient:
    """`DoosanMotionClient` 구현. **실기 미검증.**"""

    def __init__(self, caller: ServiceCaller, namespace: str = 'dsr01', srv_types: Mapping | None = None,
                 call_timeout_s: float = 1.0, stop_mode: int = STOP_MODE_SOFT):
        if not call_timeout_s > 0.0:
            raise ValueError('제한시간은 양수')
        if type(stop_mode) is not int or not 0 <= stop_mode <= 3:
            raise ValueError(f'stop_mode는 0~3 정수: {stop_mode!r}')
        self.caller = caller
        self.ns = '/' + namespace.strip('/')
        self._types = dict(srv_types) if srv_types is not None else None
        self.call_timeout_s = float(call_timeout_s)
        self.stop_mode = stop_mode

    def _call(self, key: str, **fields):
        if self._types is None:
            self._types = dsr_motion_service_types()
        return self.caller.call(self._types[key], f'{self.ns}/{MOTION_SERVICES[key][1]}', self.call_timeout_s,
                                **fields)

    @staticmethod
    def _succeeded(res) -> bool:
        return bool(getattr(res, 'success', True))

    def _read(self, key: str, **fields):
        res = self._call(key, **fields)
        if not self._succeeded(res):
            raise DsrError('SERVICE_FAILED', MOTION_SERVICES[key][1])
        return res

    # ------------------------------------------------------------ 지령
    def move_line_async(self, posx, vel, acc) -> bool:
        """비동기 직선 이동(base, 절대). 반환 = 드라이버가 받았는지(`success`). 도착은 호출한 쪽이 측정으로 판정한다."""
        pos = _floats(posx, 6, 'posx')
        v = _floats(vel, 2, 'vel')
        a = _floats(acc, 2, 'acc')
        res = self._call('move_line', pos=pos, vel=v, acc=a, time=0.0, radius=0.0, ref=REF_BASE,
                         mode=MOVE_MODE_ABS, blend_type=0, sync_type=SYNC_TYPE_ASYNC)
        return self._succeeded(res)

    def move_stop(self) -> bool:
        return self._succeeded(self._call('move_stop', stop_mode=self.stop_mode))

    # ------------------------------------------------------------ 읽기
    def current_posx(self) -> list[float]:
        res = self._read('current_posx', ref=REF_BASE)
        info = list(getattr(res, 'task_pos_info', None) or [])
        data = [float(v) for v in info[0].data] if info else []
        if len(data) < 6 or not all(math.isfinite(v) for v in data[:6]):
            raise DsrError('SERVICE_BAD_RESPONSE', f'get_current_posx: {data!r}')
        return data[:6]

    def robot_state(self) -> int:
        return int(self._read('robot_state').robot_state)

    def drl_state(self) -> int:
        return int(self._read('drl_state').drl_state)
