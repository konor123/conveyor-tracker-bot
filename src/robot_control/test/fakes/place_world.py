"""DynamicPlace 순서기 U 시험용 합성 세계와 실행 도구. 실제 시간·ROS·로봇을 쓰지 않는다.

뚜껑은 `v`로 등속 이동하고, planner처럼 주기마다 같은 `release_time`으로 revision을 올린 계획을 낸다
(SPEC-400 4.5절). TCP 목표 규약은 `InterceptPlan.target_tcp_pose`와 같다(파지 오프셋 반영).
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from robot_control.belt_sync import PlanSample, SyncConfig
from robot_control.dynamic_place import (MODE_PREDICTED, MODE_SYNCHRONIZED, TRACK_OBSERVED, BeltInfo,
                                         DynamicPlaceCfg, DynamicPlaceSequencer, GateLimits, PlaceGoal,
                                         TrackInfo)
from robot_control.fake_place_transport import SimulatedPlaceTransport

from .clock import FakeClock

LIMITS = GateLimits(e_xy_max_m=.00325, v_rel_max_mps=.005, tau_max_s=.3, position_std_max_m=.0005,
                    rim_clearance_m=.002)
CFG = DynamicPlaceCfg(safe_z_m=.25, fingertip_below_tcp_m=.16, open_width_m=.022, open_width_tolerance_m=.001)
PERIOD_S = .01


@dataclass
class PlaceWorld:
    clock: FakeClock
    t0: float
    plan_id: int = 5
    track_id: int = 7
    epoch: int = 2
    release_in_s: float = 2.0
    release_xy: tuple = (.50, 0.)
    z_rel: float = .20
    v: tuple = (.02, 0.)
    std: float = .0003
    valid_extra_s: float = 1.2
    status: int = 0
    steady: bool | None = True
    track_status: int = TRACK_OBSERVED
    track_epoch: int | None = None
    track_age_s: float = .03
    obs_age_s: float = .03
    revision: int = 0
    plan_every: int = 3                      # 약 33 Hz(영상 30 Hz마다 revision)
    override: dict = field(default_factory=dict)
    ticks: int = 0

    @property
    def release_s(self):
        return self.t0 + self.release_in_s

    def cap_xy(self, t):
        return np.array(self.release_xy) + np.array(self.v) * (t - self.release_s)

    def plan(self, **kw) -> PlanSample:
        now = self.clock.ros_now()
        self.revision += 1
        p = PlanSample(self.plan_id, self.track_id, self.epoch, self.revision, now, now, self.release_s,
                       self.release_s + self.valid_extra_s, tuple(self.release_xy), tuple(self.v),
                       status=self.status, position_std_m=self.std, release_z_m=self.z_rel)
        return replace(p, **{**self.override, **kw})

    def plans(self):
        self.ticks += 1
        return [self.plan()] if (self.ticks - 1) % self.plan_every == 0 else []

    def tracks(self):
        now = self.clock.ros_now()
        if self.track_status is None:
            return {}
        epoch = self.epoch if self.track_epoch is None else self.track_epoch
        return {self.track_id: TrackInfo(self.track_id, epoch, self.track_status, now - self.track_age_s,
                                         now - self.obs_age_s)}

    def belt(self):
        now = self.clock.ros_now()
        return None if self.steady is None else BeltInfo(now - .03, self.steady)


@dataclass
class Rig:
    mode: int = MODE_SYNCHRONIZED
    cfg: DynamicPlaceCfg = CFG
    sync_cfg: SyncConfig = field(default_factory=SyncConfig)
    limits: GateLimits = LIMITS
    deadline_in_s: float = 0.
    goal_kw: dict = field(default_factory=dict)
    transport_kw: dict = field(default_factory=dict)
    world_kw: dict = field(default_factory=dict)
    start_xyz: tuple = (.455, 0., .24)

    def __post_init__(self):
        self.clock = FakeClock()
        self.world = PlaceWorld(self.clock, self.clock.ros_now(), **self.world_kw)
        kw = dict(lag_s=.029, open_delay_s=.1)
        kw.update(self.transport_kw)
        self.transport = SimulatedPlaceTransport(self.clock, pos=self.start_xyz, **kw)
        deadline = self.clock.ros_now() + self.deadline_in_s if self.deadline_in_s else 0.
        self.goal = replace(PlaceGoal(self.world.plan_id, self.world.epoch, self.mode, self.limits,
                                      open_width_m=0., deadline_s=deadline), **self.goal_kw)
        self.seq = DynamicPlaceSequencer(self.transport, self.goal, self.cfg, self.sync_cfg)
        self.feedback = []          # (t_ros, PlaceFeedback)
        self.result = None
        self.result_t = None

    def tick(self, cancel=False, stop=False, plans=None):
        self.clock.advance(PERIOD_S)
        now = self.clock.ros_now()
        out = self.seq.tick(now, self.clock.steady_now(), self.world.plans() if plans is None else plans,
                            self.world.tracks(), self.world.belt(), cancel=cancel, stop=stop)
        self.feedback += [(now, f) for f in out.feedback]
        if out.result is not None and self.result is None:
            self.result, self.result_t = out.result, now
        return out

    def run(self, hook=None, max_s=8.):
        """`hook(rig)`이 'cancel'·'stop'을 돌려주면 그 tick에 요청한다."""
        n = int(max_s / PERIOD_S)
        for _ in range(n):
            req = hook(self) if hook is not None else None
            self.tick(cancel=req == 'cancel', stop=req == 'stop')
            if self.result is not None:
                return self.result
        raise AssertionError(f'결과 없음: phase={self.seq.phase}')

    def done_phases(self):
        return [f.phase for _, f in self.feedback if f.phase_done]

    def t_rel(self):
        return self.clock.ros_now() - self.world.t0


__all__ = ['PlaceWorld', 'Rig', 'CFG', 'LIMITS', 'PERIOD_S', 'MODE_PREDICTED', 'MODE_SYNCHRONIZED']
