"""노드 시험용 simulated backend. 실제 로봇·그리퍼에 연결하지 않는다.

`backend:=fake`일 때만 쓰이며, 반환값은 하드웨어 측정이 아니라 시뮬레이션 값이다.
기본은 `backend:=disabled`다.
"""
from __future__ import annotations

import time

import numpy as np

from .skill_logic import GripOutcome, MoveOutcome, OpenOutcome


class SimulatedBackend:
    def __init__(self, tcp_pose=((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0)),
                 move_time_s: float = 0.2, grip_width_m: float = 0.0175,
                 grip_current_ma: float = 40.0, close_width_m: float | None = None,
                 cancel_requested=None):
        self.tcp_pos = np.array(tcp_pose[0], dtype=float)
        self.tcp_quat = np.array(tcp_pose[1], dtype=float)
        self.move_time_s = max(0.0, float(move_time_s))
        self.grip_width_m = float(grip_width_m)
        self.grip_current_ma = float(grip_current_ma)
        self.close_width_m = self.grip_width_m if close_width_m is None else float(close_width_m)
        self.cancel_requested = cancel_requested or (lambda: False)
        self._stopped = False
        self._open_width_m = 0.106

    def move_line(self, target_pos_m, target_quat_xyzw) -> MoveOutcome:
        target = np.asarray(target_pos_m, dtype=float)
        start = self.tcp_pos.copy()
        steps = max(1, int(self.move_time_s / 0.01))
        for i in range(1, steps + 1):
            if self.cancel_requested() or self._stopped:
                return MoveOutcome(False, float('inf'), 'CANCELED')
            time.sleep(self.move_time_s / steps)
            self.tcp_pos = start + (target - start) * (i / steps)
        self.tcp_pos = target
        return MoveOutcome(True, 0.0001)

    def read_tcp_pose(self):
        return self.tcp_pos.copy(), self.tcp_quat.copy()

    def gripper_open(self, width_m: float) -> OpenOutcome:
        if self._stopped:
            return OpenOutcome(False, None, 'STOPPED')
        self._open_width_m = float(width_m)
        return OpenOutcome(True, float(width_m))

    def gripper_close(self, current_limit_ma: int) -> GripOutcome:
        if self._stopped:
            return GripOutcome(None, None, None, 'STOPPED')
        return GripOutcome(self.close_width_m, self.grip_current_ma,
                           True, '')

    def stop(self) -> bool:
        self._stopped = True
        return True

    def is_stopped(self) -> bool:
        return self._stopped

    def reset_stop(self) -> None:
        self._stopped = False
