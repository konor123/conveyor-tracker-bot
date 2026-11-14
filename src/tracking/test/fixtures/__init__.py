"""Deterministic trajectories, independent of ROS and wall clocks."""
from dataclasses import dataclass
import numpy as np
from tracking.association import Det


@dataclass
class Frame:
    t: float
    truth: np.ndarray
    dets: list
    dropped: bool


class TrajectoryFixture:
    def __init__(self, seed=0, duration=60., hz=30., meas_std=.0003,
                 speed=.02, step=None, occlusion=None, drop_fraction=0.,
                 reverse=False, sigma_accel=0.):
        rng = np.random.default_rng(seed)
        dt = 1 / hz
        count = round(duration * hz) + 1
        dropped = set(rng.choice(np.arange(1, count),
                                round((count - 1) * drop_fraction), replace=False))
        state = np.array([-.2, 0., speed, 0.])
        self.frames = []
        for k in range(count):
            t = k * dt
            if k:
                if step and t >= step[0] and (k - 1) * dt < step[0]:
                    state[2] = step[1]
                a = rng.normal(0, sigma_accel, 2)
                state[:2] += state[2:] * dt + .5 * a * dt**2
                state[2:] += a * dt
            z = state[:2] + rng.normal(0, meas_std, 2)
            hidden = occlusion is not None and occlusion[0] <= t <= occlusion[1]
            self.frames.append(Frame(t, state.copy(), [] if hidden else [Det(z)], k in dropped))
        if reverse:
            self.frames[10], self.frames[11] = self.frames[11], self.frames[10]

    def __iter__(self):
        return iter(self.frames)
