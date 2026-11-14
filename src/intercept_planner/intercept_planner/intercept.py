"""Pure request validation and intercept planning."""
from dataclasses import dataclass, field
from enum import IntEnum
import math

import numpy as np

from .prediction import TrackState, predict
from .time_budget import evaluate_budget


class Result(IntEnum):
    OK = 0
    FAILED = 1
    TIMEOUT = 2
    PRECONDITION = 3


class Status(IntEnum):
    OK = 0
    TIME_INSUFFICIENT = 1
    UNREACHABLE = 2
    UNCERTAIN = 3
    EXPIRED = 4


class Mode(IntEnum):
    STATIC = 0
    PREDICTED = 1
    SYNCHRONIZED = 2


@dataclass(frozen=True)
class PlanRequest:
    target_track_id: int
    target_epoch: int
    mode: int
    deadline: float
    earliest_release: float = 0.


@dataclass(frozen=True)
class PlannerParams:
    x_exit_m: float = .30
    margin_s: float = 1.
    v_min_mps: float = .002
    t_transit_s: float = 1.5
    t_sync_s: float = .5
    t_descend_s: float = .8
    t_release_s: float = .3
    t_clear_s: float = .9
    x_reach_start_m: float = -.05
    workspace_min_m: tuple = (.20, -.40, .00)
    workspace_max_m: tuple = (.70, .40, .40)
    release_above_rim_m: float = .1656   # 해제 TCP z − 테두리 z. 정지 투입 성공 높이
    tool_quat_xyzw: tuple = (1., 0., 0., 0.)
    tcp_offset_m: tuple = (0., 0.)
    position_std_max_m: float = .0005
    T_base_conveyor: np.ndarray = field(default_factory=lambda: np.eye(4))
    # The adapter supplies the current stream epoch even if no track exists.
    current_epoch: int = 0

    @property
    def components(self) -> dict[str, float]:
        return {name: getattr(self, name) for name in (
            't_transit_s', 't_sync_s', 't_descend_s', 't_release_s', 't_clear_s')}

    @classmethod
    def from_mapping(cls, cfg: dict, *, current_epoch: int = 0) -> 'PlannerParams':
        values = {key: value for section in ('budget', 'reach', 'target', 'uncertainty')
                  for key, value in cfg[section].items()}
        return cls(**values, T_base_conveyor=np.asarray(cfg['T_base_conveyor'], dtype=float),
                   current_epoch=current_epoch)


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    orientation: np.ndarray


@dataclass(frozen=True)
class InterceptPlan:
    target_track_id: int
    target_epoch: int
    source_state_stamp: float
    release_time: float
    valid_until: float
    target_tcp_pose: Pose
    target_velocity: np.ndarray
    position_std: float
    time_margin: float
    status: Status
    reason: str
    plan_id: int = 0
    revision: int = 1


@dataclass(frozen=True)
class Plan:
    """ComputeIntercept-like envelope; rejected requests have no plan payload."""
    result: Result
    reason_code: str
    plan: InterceptPlan | None = None


def compute_plan(track: TrackState | None, now: float, req: PlanRequest,
                 p: PlannerParams) -> Plan:
    return _compute_plan(track, now, req, p)


def _compute_plan(track: TrackState | None, now: float, req: PlanRequest,
                  p: PlannerParams, *, fixed_release: float | None = None) -> Plan:
    if now > req.deadline:
        return Plan(Result.TIMEOUT, '')
    if req.mode not in (Mode.PREDICTED, Mode.SYNCHRONIZED):
        return Plan(Result.PRECONDITION, 'MODE_UNSUPPORTED')
    if req.target_epoch != p.current_epoch:
        return Plan(Result.FAILED, 'EPOCH_MISMATCH')
    if track is None or track.track_id != req.target_track_id:
        return Plan(Result.FAILED, 'TRACK_NOT_FOUND')
    if track.epoch != p.current_epoch:
        return Plan(Result.FAILED, 'EPOCH_MISMATCH')

    # Invalid tracks and belts get a diagnostic payload, without dividing by
    # their velocity or trying to construct a physically meaningful target.
    def invalid(status: Status, reason: str) -> Plan:
        release = now if fixed_release is None else fixed_release
        payload = InterceptPlan(track.track_id, track.epoch, track.t_state,
            release, release + p.t_release_s + p.t_clear_s,
            Pose(np.zeros(3), np.asarray(p.tool_quat_xyzw, dtype=float)),
            np.zeros(3), 0., -sum(p.components.values()), status, reason)
        return Plan(Result.FAILED, reason, payload)

    if track.status == TrackState.LOST or now > track.valid_until:
        return invalid(Status.EXPIRED, 'TRACK_LOST')
    pos_now, _ = predict(track, now)
    budget = evaluate_budget(float(pos_now[0]), float(track.vel[0]), p.x_exit_m,
                             p.components, p.margin_s, v_min=p.v_min_mps)
    if budget.reason in ('BELT_STOPPED', 'BELT_REVERSED'):
        return invalid(Status.TIME_INSUFFICIENT, budget.reason)
    lead = p.t_transit_s + p.t_sync_s + p.t_descend_s
    reach_time = track.t_state
    if track.pos[0] < p.x_reach_start_m:
        reach_time += (p.x_reach_start_m - track.pos[0]) / track.vel[0]
    release = (max(now + lead, req.earliest_release, reach_time)
               if fixed_release is None else fixed_release)
    required = max(budget.t_required, release - now + p.t_release_s + p.t_clear_s)
    margin = budget.t_available - required
    pos_rel, cov_rel = predict(track, release)
    transform = np.asarray(p.T_base_conveyor)
    base = transform @ np.array([pos_rel[0], pos_rel[1], track.z, 1.])
    # 해제 높이는 절대값이 아니라 테두리 위 상대 높이다: 벨트 평면·컨베이어 배치가 바뀌어도 따라간다.
    target = np.array([base[0] - p.tcp_offset_m[0], base[1] - p.tcp_offset_m[1],
                       base[2] + p.release_above_rim_m])
    velocity = transform[:3, :3] @ np.array([*track.vel, 0.])
    std = math.sqrt(max(0., float(np.linalg.eigvalsh(cov_rel)[-1])))
    status, reason = Status.OK, ''
    if margin < p.margin_s:
        status, reason = Status.TIME_INSUFFICIENT, 'TIME_MARGIN'
    elif (np.any(target < p.workspace_min_m) or np.any(target > p.workspace_max_m)
          or pos_rel[0] > p.x_exit_m):
        status, reason = Status.UNREACHABLE, 'UNREACHABLE'
    elif std > p.position_std_max_m:
        status, reason = Status.UNCERTAIN, 'UNCERTAIN'
    payload = InterceptPlan(track.track_id, track.epoch, track.t_state, release,
        release + p.t_release_s + p.t_clear_s,
        Pose(target, np.asarray(p.tool_quat_xyzw, dtype=float)), velocity, std, margin,
        status, reason)
    return Plan(Result.OK if status == Status.OK else Result.FAILED, reason, payload)
