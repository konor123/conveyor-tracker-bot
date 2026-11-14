"""Constant-velocity prediction without ROS or a tracking-package dependency."""
from dataclasses import dataclass
from typing import ClassVar

import numpy as np


@dataclass(frozen=True)
class TrackState:
    OBSERVED: ClassVar[int] = 0
    COASTING: ClassVar[int] = 1
    LOST: ClassVar[int] = 2
    track_id: int
    epoch: int
    status: int
    t_state: float
    pos: np.ndarray
    vel: np.ndarray
    cov: np.ndarray
    valid_until: float
    last_observed: float
    z: float = 0.          # 뚜껑 테두리 윗면 높이(conveyor_frame, m). 해제 높이 계산에 쓴다


def predict(track: TrackState, t: float) -> tuple[np.ndarray, np.ndarray]:
    dt = t - track.t_state
    h = np.array([[1., 0., dt, 0.], [0., 1., 0., dt]])
    return (np.asarray(track.pos) + np.asarray(track.vel) * dt,
            h @ np.asarray(track.cov) @ h.T)
