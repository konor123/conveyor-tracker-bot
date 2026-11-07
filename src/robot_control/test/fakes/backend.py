"""정적 skill 판단 시험용 fake backend. 하드웨어 연결을 만들지 않는다."""
from __future__ import annotations

import numpy as np

from robot_control.skill_logic import GripOutcome, MoveOutcome, OpenOutcome


class FakeStaticBackend:
    def __init__(self, tcp_pose=((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))):
        self.tcp_pos = np.array(tcp_pose[0], dtype=float)
        self.tcp_quat = np.array(tcp_pose[1], dtype=float)
        self.move_error_m = 0.0005
        self.move_ok = True
        self.grip = GripOutcome(0.0175, 40.0, True)
        self.open = OpenOutcome(True, 0.021)
        self.stop_sent = False
        self.stopped = True
        self.calls: list[tuple[str, object]] = []

    def move_line(self, target_pos_m, target_quat_xyzw) -> MoveOutcome:
        self.calls.append(('move_line', np.asarray(target_pos_m, dtype=float).copy()))
        if self.move_ok:
            self.tcp_pos = np.asarray(target_pos_m, dtype=float).copy()
        return MoveOutcome(self.move_ok, self.move_error_m)

    def read_tcp_pose(self):
        return self.tcp_pos.copy(), self.tcp_quat.copy()

    def gripper_open(self, width_m: float) -> OpenOutcome:
        self.calls.append(('gripper_open', width_m))
        return self.open

    def gripper_close(self, current_limit_ma: int) -> GripOutcome:
        self.calls.append(('gripper_close', current_limit_ma))
        return self.grip

    def stop(self) -> bool:
        self.calls.append(('stop', None))
        self.stop_sent = True
        return True

    def is_stopped(self) -> bool:
        return self.stopped
