"""XY-only tracking core. No robot/ROS/gripper I/O; SI units, robot_base.

Uses InterceptPlan's release-time position, not its source-state position.
Inactive output requires the caller's verified stop path; zero is NOT proof of stop.
An UNCERTAIN revision (belt `steady=false` pulse) is discarded, not latched: the last OK
plan is followed at constant velocity for at most `uncertain_hold_s` (REQ-ROB-013).
"""
from dataclasses import dataclass
import math
import numpy as np
from .motion_adapter import tracking_command


@dataclass(frozen=True)
class SyncConfig:
    kp: float = 4.0
    max_speed_mps: float = .10
    max_accel_mps2: float = .30
    period_s: float = .01
    max_loop_gap_s: float = .03
    max_plan_age_s: float = .25
    max_tcp_age_s: float = .25
    max_position_std_m: float = .00325
    release_position_std_max_m: float = .0005
    uncertain_hold_s: float = .6
    min_xy_m: tuple = (.20, -.40)
    max_xy_m: tuple = (.75, .40)

    def __post_init__(self):
        vals = (self.kp, self.max_speed_mps, self.max_accel_mps2, self.period_s,
                self.max_loop_gap_s, self.max_plan_age_s, self.max_tcp_age_s,
                self.max_position_std_m, self.release_position_std_max_m, self.uncertain_hold_s)
        if any(not math.isfinite(x) or x <= 0 for x in vals):
            raise ValueError('configuration must be finite and positive')
        if self.release_position_std_max_m > self.max_position_std_m:
            raise ValueError('release uncertainty limit exceeds plan limit')
        if self.period_s > self.max_loop_gap_s:
            raise ValueError('period exceeds loop gap')
        if not _xy(self.min_xy_m) or not _xy(self.max_xy_m) or any(a >= b for a,b in zip(self.min_xy_m,self.max_xy_m)):
            raise ValueError('invalid workspace')


@dataclass(frozen=True)
class PlanSample:
    plan_id: int
    track_id: int
    epoch: int
    revision: int
    generated_s: float
    source_stamp_s: float
    release_s: float
    valid_until_s: float
    release_xy_m: tuple
    velocity_xy_mps: tuple
    frame_id: str = 'robot_base'
    status: int = 0
    position_std_m: float = .0003
    release_z_m: float = float('nan')


@dataclass(frozen=True)
class SyncOutput:
    active: bool
    reason: str
    velocity_xy_mps: tuple = (0., 0.)
    target_xy_m: tuple = (0., 0.)
    error_xy_m: tuple = (0., 0.)


STATUS_OK, STATUS_UNCERTAIN = 0, 3      # InterceptPlan.OK, InterceptPlan.UNCERTAIN


def _xy(v):
    try:
        return len(v) == 2 and all(math.isfinite(x) for x in v)
    except (TypeError, ValueError):
        return False


class BeltSynchronizer:
    def __init__(self, cfg=None):
        self.cfg = cfg or SyncConfig()
        self.arm(None, None, None)
        self.stop('NOT_ARMED')

    def arm(self, plan_id, track_id, epoch):
        """Only owner of the skill may explicitly start/rearm a selected plan."""
        self.identity = (plan_id, track_id, epoch)
        self.plan = None
        self.fault = None
        self.received_s = None
        self.last_revision = None
        self.uncertain_since_s = None
        self.last_steady_s = self.last_ros_s = None
        self.previous = np.zeros(3)

    def stop(self, reason='STOP_REQUESTED'):
        self.fault = reason
        self.previous = np.zeros(3)
        return SyncOutput(False, reason)

    def release_uncertainty_ok(self, ros_now_s, steady_now_s):
        """Diagnostic only: NOT release authorization (height/speed/gripper gates absent)."""
        p = self.plan
        if (self.fault or p is None or self.holding
                or not all(math.isfinite(x) for x in (ros_now_s, steady_now_s))):
            return False
        return (0 <= p.position_std_m <= self.cfg.release_position_std_max_m
                and 0 <= ros_now_s-p.source_stamp_s <= self.cfg.max_plan_age_s
                and 0 <= steady_now_s-self.received_s <= self.cfg.max_plan_age_s
                and ros_now_s <= p.valid_until_s)

    @property
    def holding(self):
        """True while discarded UNCERTAIN revisions keep the last OK plan on hold."""
        return self.uncertain_since_s is not None

    def _inside(self, xy):
        return all(a <= x <= b for a,x,b in zip(self.cfg.min_xy_m,xy,self.cfg.max_xy_m))

    def accept(self, p, ros_now_s, steady_now_s):
        if self.fault:
            return False
        if (p.plan_id,p.track_id,p.epoch) != self.identity:
            self.stop('PLAN_IDENTITY_CHANGED')
            return False
        # Replayed/duplicate messages (accepted or discarded) must not refresh the receive watchdog.
        if self.last_revision is not None and p.revision <= self.last_revision:
            return False
        nums = (ros_now_s, steady_now_s, p.generated_s, p.source_stamp_s,
                p.release_s, p.valid_until_s, p.position_std_m)
        valid = (all(math.isfinite(x) for x in nums) and p.revision >= 0
                 and p.frame_id == 'robot_base'
                 and _xy(p.release_xy_m) and _xy(p.velocity_xy_mps)
                 and p.source_stamp_s <= p.generated_s <= ros_now_s
                 and 0 <= ros_now_s-p.source_stamp_s <= self.cfg.max_plan_age_s
                 and p.valid_until_s >= ros_now_s
                 and (self.last_ros_s is None or ros_now_s >= self.last_ros_s)
                 and (self.received_s is None or steady_now_s >= self.received_s))
        if valid and p.status == STATUS_UNCERTAIN:
            # Discard without latching. The hold starts at the first UNCERTAIN revision;
            # later or duplicate ones do not extend it, only an accepted OK revision ends it.
            self.last_revision = p.revision
            if self.uncertain_since_s is None:
                self.uncertain_since_s = steady_now_s
            return False
        if not valid or p.status != STATUS_OK or not 0 <= p.position_std_m <= self.cfg.max_position_std_m:
            self.stop('PLAN_INVALID')
            return False
        self.plan, self.received_s = p, steady_now_s
        self.last_revision, self.uncertain_since_s = p.revision, None
        return True

    def step(self, ros_now_s, steady_now_s, tcp_xy_m, tcp_stamp_s, tcp_frame):
        if self.fault:
            return SyncOutput(False,self.fault)
        if not all(math.isfinite(x) for x in (ros_now_s,steady_now_s,tcp_stamp_s)) or not _xy(tcp_xy_m):
            return self.stop('NONFINITE') if self.plan is not None else SyncOutput(False,'WAIT_PLAN')
        if self.holding and steady_now_s-self.uncertain_since_s > self.cfg.uncertain_hold_s + 1e-12:
            return self.stop('UNCERTAIN_HOLD')
        if self.plan is None:
            return SyncOutput(False,'WAIT_PLAN')
        dt = self.cfg.period_s if self.last_steady_s is None else steady_now_s-self.last_steady_s
        if self.last_ros_s is not None and ros_now_s < self.last_ros_s:
            return self.stop('CLOCK_BACKWARD')
        if dt <= 0 or dt > self.cfg.max_loop_gap_s + 1e-12:
            return self.stop('LOOP_GAP')
        p = self.plan
        if ros_now_s > p.valid_until_s:
            return self.stop('PLAN_EXPIRED')
        # During an UNCERTAIN hold the bounded hold limit replaces the plan-age watchdog.
        if not self.holding and (not 0 <= steady_now_s-self.received_s <= self.cfg.max_plan_age_s
                                 or not 0 <= ros_now_s-p.source_stamp_s <= self.cfg.max_plan_age_s):
            return self.stop('PLAN_STALE')
        if tcp_frame != 'robot_base' or not 0 <= ros_now_s-tcp_stamp_s <= self.cfg.max_tcp_age_s:
            return self.stop('TCP_STALE')
        target = np.array(p.release_xy_m) + np.array(p.velocity_xy_mps)*(ros_now_s-p.release_s)
        if not self._inside(target) or not self._inside(tcp_xy_m):
            return self.stop('WORKSPACE')
        cmd = tracking_command([*target,0.], [*p.velocity_xy_mps,0.], [*tcp_xy_m,0.],
                               self.cfg.kp,self.cfg.max_speed_mps,self.cfg.max_accel_mps2,self.previous,dt)
        # Reject even a one-tick projected command outside the approved XY box.
        if not self._inside(np.array(tcp_xy_m)+cmd[:2]*dt):
            return self.stop('WORKSPACE')
        self.previous = cmd
        self.last_ros_s,self.last_steady_s = ros_now_s,steady_now_s
        return SyncOutput(True,'OK',tuple(cmd[:2]),tuple(target),tuple(target-np.array(tcp_xy_m)))
