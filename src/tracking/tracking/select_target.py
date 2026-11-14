"""Pure SelectTarget decision; service deadlines belong to the node."""
from dataclasses import dataclass

OK, PRECONDITION = 0, 3


@dataclass(frozen=True)
class SelectResult:
    result: int
    reason: str
    track_id: int | None = None
    epoch: int = 0
    t_enter_window: float | None = None


def select_target(tracks, belt_motion, *, epoch, object_class=2,
                  min_observation_count=30, v_min_mps=.002, x_reach_start_m=-.05):
    if belt_motion is not None and belt_motion.speed <= v_min_mps:
        return SelectResult(PRECONDITION, 'BELT_STOPPED', epoch=epoch)
    candidates = [t for t in tracks if t.object_class == object_class and t.status == 0
                  and t.observation_count >= min_observation_count
                  and t.velocity[0] > v_min_mps and t.position[0] < x_reach_start_m]
    if not candidates:
        return SelectResult(PRECONDITION, 'NO_TARGET', epoch=epoch)
    track = min(candidates, key=lambda t: x_reach_start_m-t.position[0])
    entry = track.t_state + (x_reach_start_m-track.position[0])/track.velocity[0]
    return SelectResult(OK, '', track.track_id, epoch, float(entry))
