"""Belt estimate from the most-observed live cap track."""
from dataclasses import dataclass
import numpy as np

SOURCE_CAP = 0


@dataclass(frozen=True)
class BeltMotion:
    t_state: float
    direction: np.ndarray
    speed: float
    speed_std: float
    window_sec: float
    source: int = SOURCE_CAP
    steady: bool = True
    last_change_stamp: float | None = None

    @property
    def velocity(self):
        return self.speed * self.direction


def estimate_belt_motion(tracks, previous=None, *, v_min_mps=.002,
                         window_max_s=2., steady=True, last_change_stamp=None):
    live = [t for t in tracks if t.status in (0, 1) and t.object_class == 2]
    if not live:
        return previous
    track = max(live, key=lambda t: t.observation_count)
    v = track.velocity[:2]
    speed = float(np.linalg.norm(v))
    direction = v/speed if speed > v_min_mps else np.array([1., 0.])
    std = float(np.sqrt(max(0., direction @ track.state_cov[2:, 2:] @ direction)))
    return BeltMotion(track.t_state, direction.copy(), speed, std,
                      min(track.t_state-track.created_at, window_max_s),
                      steady=steady, last_change_stamp=last_change_stamp)
