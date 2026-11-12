"""DynamicPlace 순서기와 해제 게이트(순수 Python). 노드는 감싸기만 한다(PLAN.md 0.2절, SPEC-500 4.5절).

transport와의 경계는 속도 지령이 아니라 **해제 확정(commit)** 이다.
  - `update(ref)`: 따라갈 기준(목표·피드포워드·REQ-ROB-006 지령). 스트림 transport는 매 주기 반영하고,
    한 DRL 프로그램 transport는 commit 뒤에는 반영하지 않는다(`capabilities.mid_course_update`).
  - `commit_release(permit)`: 되돌릴 수 없는 해제 확정, goal당 최대 1회. 스트림 + 상주 루프에서는
    `GripperSession.fire_open`, 한 DRL 프로그램에서는 프로그램 시작(인자: 해제 시각·자세·속도·개방 폭).
  - `poll()`: TCP·속도·시각·그리퍼 폭·해제 여부·장애. `abort(keep_sync)`: 멈춤 또는 동기 유지한 채 멈춤.
해제 게이트(PLAN.md 4.4절)는 commit 직전 한 번, `capabilities.commit_lead_s` 앞으로 외삽한 값으로 판정한다.
두 transport가 같은 함수를 쓰고 horizon만 다르다. `mid_course_update`가 없는 transport는 MODE_PREDICTED만 한다.

시간은 float 초다. 계획·관측·deadline은 ROS 시각, 주기·보류 감시는 steady 시각으로 받는다. 길이 m, 속도 m/s.
"L3 요청"은 결과 `FAILED`·`released=false`(`abort_request=3`)다. 중단 등급은 orchestrator가 정한다(PLAN.md 5절).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Protocol, Sequence

import numpy as np

from .belt_sync import BeltSynchronizer, PlanSample, SyncConfig
from .motion_adapter import tracking_command
from .skill_logic import Result

MODE_STATIC, MODE_PREDICTED, MODE_SYNCHRONIZED = 0, 1, 2          # DropTask.Goal.MODE_*
TRACK_OBSERVED, TRACK_COASTING, TRACK_LOST = 0, 1, 2              # TrackedObject.*
PHASES = ('APPROACH', 'SYNCHRONIZE', 'DESCEND', 'RELEASE', 'RETREAT')
DYNAMIC_PHASES = frozenset(PHASES[1:])
EPS = 1e-12
TERMINAL_GATE_REASONS = frozenset({'GATE_NONFINITE', 'GATE_EPOCH', 'GATE_EXPIRED', 'GATE_RIM_CLEARANCE'})
GATE_KEYS = ('gate_e_xy_m', 'gate_v_rel_mps', 'gate_t_since_obs_s', 'gate_position_std_m',
             'gate_rim_clearance_m', 'gate_horizon_s', 'gate_ok')
# 추종 코어 멈춤 이유 → 결과 이유. 계획 식별·epoch·만료는 SPEC-010 4.2절대로 PLAN_INVALID다.
_PLAN_INVALID = frozenset({'PLAN_INVALID', 'PLAN_IDENTITY_CHANGED', 'PLAN_EXPIRED'})
# 이 이유로 멈추면 XY 동기를 유지하지 않는다(위치·피드백 자체를 믿을 수 없음).
_NO_KEEP_SYNC = frozenset({'WORKSPACE', 'NONFINITE', 'TCP_STALE'})


def _finite(*values) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False


def _xy(v) -> np.ndarray:
    return np.asarray(v, dtype=float).reshape(-1)[:2]


def _norm(v) -> float:
    return float(np.linalg.norm(np.asarray(v, dtype=float)))


# ================================================================== 해제 게이트
@dataclass(frozen=True)
class GateLimits:
    e_xy_max_m: float
    v_rel_max_mps: float
    tau_max_s: float
    position_std_max_m: float
    rim_clearance_m: float          # h_clear: 핑거 끝 − 테두리 하한

    @classmethod
    def from_goal(cls, goal: Mapping[str, float], defaults: 'GateLimits') -> 'GateLimits':
        """`DynamicPlace.Goal` 값이 유한한 양수면 그 값, 아니면(0·음수·NaN) 기본값."""
        def pick(key, default):
            try:
                value = float(goal.get(key))
            except (TypeError, ValueError):
                return default
            return value if math.isfinite(value) and value > 0.0 else default
        return cls(pick('e_xy_max', defaults.e_xy_max_m), pick('v_rel_max', defaults.v_rel_max_mps),
                   pick('tau_max', defaults.tau_max_s), pick('position_std_max', defaults.position_std_max_m),
                   pick('rim_clearance', defaults.rim_clearance_m))

    def valid(self) -> bool:
        values = (self.e_xy_max_m, self.v_rel_max_mps, self.tau_max_s, self.position_std_max_m,
                  self.rim_clearance_m)
        return _finite(*values) and all(v > 0.0 for v in values)


@dataclass(frozen=True)
class TrackInfo:
    track_id: int
    epoch: int
    status: int
    stamp_s: float                  # t_state(ROS)
    last_observed_s: float


@dataclass(frozen=True)
class BeltInfo:
    stamp_s: float                  # ROS
    steady: bool


@dataclass(frozen=True)
class GateInputs:
    """지금(`t_now_s`) 값. 게이트가 `horizon_s`만큼 등속으로 외삽한다.

    XY는 TCP 목표 규약(`InterceptPlan.target_tcp_pose` = 뚜껑 중심 − 파지 오프셋)으로 받는다. 공깃돌 중심 = TCP +
    오프셋, 뚜껑 중심 = 목표 + 오프셋이므로 둘의 차이는 오프셋과 무관하다(SPEC-400 REQ-PLN-004).
    """
    t_now_s: float
    stone_xy_m: tuple
    stone_v_xy_mps: tuple
    cap_xy_m: tuple
    cap_v_xy_mps: tuple
    fingertip_z_m: float
    fingertip_vz_mps: float
    rim_z_m: float
    position_std_m: float
    valid_until_s: float
    goal_epoch: int
    plan_epoch: int
    track: TrackInfo | None
    belt: BeltInfo | None
    plan_hold: bool                 # 추종 코어가 UNCERTAIN revision 보류 중


@dataclass(frozen=True)
class GateDecision:
    ok: bool
    reason: str
    terminal: bool                  # True면 기다려도 열리지 않는다
    metrics: dict


def release_gate(inputs: GateInputs, limits: GateLimits, horizon_s: float) -> GateDecision:
    """PLAN.md 4.4절 해제 게이트. 해제 순간(`t_now + horizon`) 예측으로 판정하고 첫 위반 이유를 낸다."""
    i, h = inputs, float(horizon_s)
    t_rel = i.t_now_s + h
    stone_v, cap_v = _xy(i.stone_v_xy_mps), _xy(i.cap_v_xy_mps)
    stone = _xy(i.stone_xy_m) + stone_v * h
    cap = _xy(i.cap_xy_m) + cap_v * h
    e_xy, v_rel = _norm(stone - cap), _norm(stone_v - cap_v)
    clearance = i.fingertip_z_m + i.fingertip_vz_mps * h - i.rim_z_m
    track = i.track
    t_since_obs = t_rel - track.last_observed_s if track is not None else math.nan
    metrics = {'gate_e_xy_m': e_xy, 'gate_v_rel_mps': v_rel, 'gate_t_since_obs_s': t_since_obs,
               'gate_position_std_m': float(i.position_std_m), 'gate_rim_clearance_m': clearance,
               'gate_horizon_s': h}
    tau = limits.tau_max_s
    checks = (
        (not _finite(t_rel, e_xy, v_rel, clearance, i.position_std_m, i.valid_until_s) or h < 0.0,
         'GATE_NONFINITE'),
        (i.goal_epoch != i.plan_epoch or (track is not None and track.epoch != i.goal_epoch), 'GATE_EPOCH'),
        (t_rel > i.valid_until_s + EPS, 'GATE_EXPIRED'),
        (clearance < limits.rim_clearance_m - EPS, 'GATE_RIM_CLEARANCE'),
        (i.belt is None or i.t_now_s - i.belt.stamp_s > tau + EPS, 'GATE_BELT_STALE'),
        (not i.belt.steady if i.belt is not None else False, 'GATE_BELT_UNSTEADY'),
        (bool(i.plan_hold), 'GATE_PLAN_UNCERTAIN'),
        (track is None or i.t_now_s - track.stamp_s > tau + EPS, 'GATE_TRACK_STALE'),
        (track is not None and track.status not in (TRACK_OBSERVED, TRACK_COASTING), 'GATE_TRACK_LOST'),
        (track is not None and track.status == TRACK_COASTING and not t_since_obs <= tau + EPS, 'GATE_COASTING'),
        (not i.position_std_m <= limits.position_std_max_m + EPS, 'GATE_POSITION_STD'),
        (not e_xy <= limits.e_xy_max_m + EPS, 'GATE_E_XY'),
        (not v_rel <= limits.v_rel_max_mps + EPS, 'GATE_V_REL'),
    )
    reason = next((name for failed, name in checks if failed), '')
    metrics['gate_ok'] = 0.0 if reason else 1.0
    return GateDecision(not reason, reason, reason in TERMINAL_GATE_REASONS, metrics)


# ================================================================== transport 계약
@dataclass(frozen=True)
class TransportCaps:
    mid_course_update: bool         # commit 뒤에도 update()를 반영(스트림). False = 프로그램 실행형
    commit_lead_s: float            # commit_release() → 공깃돌이 손가락을 떠나는 시각까지
    keep_sync_retreat: bool         # 계획 속도로 XY를 유지하며 올라갈 수 있음(REQ-ROB-010)


@dataclass(frozen=True)
class MotionRef:
    t_s: float                      # ROS
    target_xyz_m: tuple
    velocity_ff_mps: tuple          # 계획 속도(피드포워드)
    command_mps: tuple              # REQ-ROB-006 식·제한을 지난 지령(스트림 transport가 그대로 보냄)


@dataclass(frozen=True)
class ReleasePermit:
    t_commit_s: float
    t_release_s: float              # = t_commit + commit_lead_s
    release_xyz_m: tuple            # t_release의 TCP 목표
    velocity_mps: tuple             # 계획 속도
    open_width_m: float
    hold_s: float                   # 해제 뒤 동기 유지
    retreat_z_m: float
    retreat_speed_mps: float
    gate: dict                      # 이 commit을 허가한 게이트 값


@dataclass(frozen=True)
class TransportSample:
    t_s: float                      # TCP를 읽은 ROS 시각
    pos_m: tuple
    vel_mps: tuple
    frame_id: str
    gripper_width_m: float
    released: bool                  # 손가락이 열려 공깃돌이 떨어짐
    committed: bool
    fault: str = ''


class PlaceTransport(Protocol):
    capabilities: TransportCaps

    def update(self, ref: MotionRef) -> bool: ...
    def commit_release(self, permit: ReleasePermit) -> bool: ...
    def poll(self) -> TransportSample: ...
    def abort(self, keep_sync: bool) -> bool: ...
    def stop(self) -> bool: ...
    def is_stopped(self) -> bool: ...


# ================================================================== 설정·goal·출력
@dataclass(frozen=True)
class DynamicPlaceCfg:
    # goal 임계값이 0일 때 쓰는 해제 게이트 기본값(SPEC-100 sync.*, watchdog.obs_freshness_s, descend.h_clear_m)
    e_xy_max_m: float = .00325
    v_rel_max_mps: float = .005
    tau_max_s: float = .3
    position_std_max_m: float = .0005
    rim_clearance_m: float = .002
    approach_above_m: float = .03
    approach_tol_m: float = .002
    approach_timeout_s: float = 2.25
    plan_wait_s: float = .5
    sync_hold_s: float = .3
    sync_timeout_s: float = 2.
    sync_feedback_period_s: float = .03
    entry_lead_s: float = 1.3
    descend_tol_m: float = .0005
    descend_timeout_s: float = 1.5
    z_kp_hz: float = 10.
    z_max_speed_mps: float = .05
    z_max_accel_mps2: float = .3
    release_wait_max_s: float = 2.
    release_hold_s: float = .3
    release_timeout_s: float = 1.
    commit_late_tol_s: float = .02
    retreat_speed_mps: float = .05
    retreat_margin_m: float = .002
    retreat_timeout_s: float = 3.
    sync_speed_tol_mps: float = .001
    keep_sync_ref_max_age_s: float = 2.
    release_above_rim_m: float = .1656
    # 실측 기록이 있을 때만 채운다. 음수 = 미설정 → PRECONDITION(SPEC-500 3절)
    fingertip_below_tcp_m: float = -1.
    safe_z_m: float = -1.
    open_width_m: float = -1.
    open_width_tolerance_m: float = -1.

    def __post_init__(self):
        unset_ok = {'fingertip_below_tcp_m', 'safe_z_m', 'open_width_m', 'open_width_tolerance_m'}
        for name, value in self.__dict__.items():
            if not _finite(value) or (name not in unset_ok and value <= 0.0):
                raise ValueError(f'{name}: 유한한 양수 필요')

    def gate_defaults(self) -> GateLimits:
        return GateLimits(self.e_xy_max_m, self.v_rel_max_mps, self.tau_max_s, self.position_std_max_m,
                          self.rim_clearance_m)


@dataclass(frozen=True)
class PlaceGoal:
    plan_id: int
    target_epoch: int
    mode: int
    limits: GateLimits
    open_width_m: float = 0.        # 0 = 설정값
    deadline_s: float = 0.          # ROS, 0 = 없음


@dataclass(frozen=True)
class PlaceFeedback:
    phase: str
    phase_done: bool
    metrics: dict
    e_xy_m: float = math.nan
    v_rel_mps: float = math.nan
    plan_revision: int = 0


@dataclass(frozen=True)
class PlaceResult:
    result: Result
    reason_code: str = ''
    released: bool = False
    gate: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    abort_request: int = 0          # 3 = L3 요청(해제 전 실패)


@dataclass(frozen=True)
class TickOut:
    feedback: tuple
    result: PlaceResult | None


def _nan_gate() -> dict:
    return {k: (0.0 if k == 'gate_ok' else math.nan) for k in GATE_KEYS}


def _z_command(z_target, z, v_prev, dt, kp, v_max, a_max) -> float:
    v = max(-v_max, min(v_max, kp * (z_target - z)))
    dv = max(-a_max * dt, min(a_max * dt, v - v_prev))
    return v_prev + dv


# ================================================================== 동기 유지 후퇴
class KeepSyncRetreat:
    """동기 유지 후퇴(REQ-ROB-010): XY는 계획 속도 그대로, z는 안전 높이까지 올린다.

    판정은 보낸 지령이 아니라 `poll()` 표본의 XY 속도로 한다. 안전 높이에 닿기 전 모든 표본이 계획 속도
    ±`sync_speed_tol_mps` 안이어야 OK다. 어긋나도 안전 높이까지는 올리고 `SYNC_SPEED`로 끝낸다. 처음부터 안전
    높이 이상이면 올릴 것이 없으므로 지령 없이 OK다. `keep_sync_retreat`가 없는
    transport와 계획 속도가 없는 경우는 움직이지 않고 `PRECONDITION`(`SYNC_NOT_AVAILABLE`)이다.
    """

    def __init__(self, transport: PlaceTransport, velocity_xy_mps, cfg: DynamicPlaceCfg,
                 deadline_s: float = 0., period_s: float = .01):
        self.t = transport
        self.v = None if velocity_xy_mps is None or not _finite(*velocity_xy_mps) else _xy(velocity_xy_mps)
        self.cfg = cfg
        self.deadline_s = float(deadline_s)
        self.period_s = period_s
        self.errors: list[float] = []
        self.result: PlaceResult | None = None
        self._t0 = None
        self._vz = 0.0
        self._steady = None

    @property
    def target_z(self) -> float:
        return self.cfg.safe_z_m + self.cfg.retreat_margin_m

    def step(self, ros, steady, sample: TransportSample) -> dict | None:
        """표본 하나로 오차를 기록하고 기준을 보낸다. 안전 높이에 닿으면 phase 끝 측정값을 돌려준다."""
        pos, vel = np.asarray(sample.pos_m, float), np.asarray(sample.vel_mps, float)
        if pos[2] >= self.cfg.safe_z_m - EPS:
            return {'tcp_z_m': float(pos[2]), 'outside_workspace': 1.0,
                    'stone_in_gripper': 0.0 if sample.released else 1.0,
                    'sync_speed_error_max_mps': max(self.errors, default=0.0)}
        self.errors.append(_norm(vel[:2] - self.v))
        dt = self.period_s if self._steady is None else max(steady - self._steady, 1e-6)
        self._steady = steady
        self._vz = _z_command(self.target_z, pos[2], self._vz, dt, self.cfg.z_kp_hz, self.cfg.retreat_speed_mps,
                              self.cfg.z_max_accel_mps2)
        self.t.update(MotionRef(ros, (pos[0], pos[1], self.target_z), (*self.v, 0.0), (*self.v, self._vz)))
        return None

    def speed_ok(self) -> bool:
        """안전 높이 전 표본이 모두 기준 안. 표본이 없으면(처음부터 안전 높이) 올린 것이 없으므로 참."""
        return max(self.errors, default=0.0) <= self.cfg.sync_speed_tol_mps + EPS

    def tick(self, ros_now_s, steady_now_s, cancel=False, stop=False) -> TickOut:
        if self.result is not None:
            return TickOut((), self.result)
        fb = []
        if self._t0 is None:
            if not self.t.capabilities.keep_sync_retreat or self.v is None:
                return self._end(Result.PRECONDITION, 'SYNC_NOT_AVAILABLE', fb)
            if self.cfg.safe_z_m < 0.0:
                return self._end(Result.PRECONDITION, 'SAFE_Z_UNSET', fb)
            self._t0 = ros_now_s
            fb.append(PlaceFeedback('RETREAT', False, {}))
        if stop:
            self.t.stop()
            return self._end(Result.CANCELED, 'STOPPED', fb)
        if cancel:
            self.t.abort(True)
            return self._end(Result.CANCELED, 'CANCELED', fb)
        if 0.0 < self.deadline_s <= ros_now_s:
            self.t.abort(True)
            return self._end(Result.TIMEOUT, 'DEADLINE', fb)
        sample = self.t.poll()
        if sample.fault:
            return self._end(Result.FAULT, sample.fault, fb)
        done = self.step(ros_now_s, steady_now_s, sample)
        if done is not None:
            if self.errors:                          # 올렸을 때만 멈춘다(지령을 보낸 적이 없으면 그대로 둔다)
                self.t.abort(False)
            fb.append(PlaceFeedback('RETREAT', True, done))
            ok = self.speed_ok()
            return self._end(Result.OK if ok else Result.FAILED, '' if ok else 'SYNC_SPEED', fb, done)
        if ros_now_s - self._t0 >= self.cfg.retreat_timeout_s:
            self.t.abort(False)
            return self._end(Result.FAILED, 'RETREAT_TIMEOUT', fb, {'tcp_z_m': float(sample.pos_m[2])})
        return TickOut(tuple(fb), None)

    def _end(self, result, reason, fb, metrics=None) -> TickOut:
        self.result = PlaceResult(result, reason, metrics=dict(metrics or {}))
        return TickOut(tuple(fb), self.result)


# ================================================================== 순서기
class DynamicPlaceSequencer:
    """APPROACH → SYNCHRONIZE → DESCEND → RELEASE → RETREAT. `tick()`을 주기(`sync_cfg.period_s`)마다 부른다.

    계획 수락·추종은 `BeltSynchronizer`(REQ-ROB-013), 해제는 `release_gate`(REQ-ROB-009)로 판정한다.
    goal에 track 번호가 없으므로 goal의 plan_id·epoch와 맞는 첫 계획의 track으로 무장한다.
    """

    def __init__(self, transport: PlaceTransport, goal: PlaceGoal, cfg: DynamicPlaceCfg | None = None,
                 sync_cfg: SyncConfig | None = None):
        self.t = transport
        self.goal = goal
        self.cfg = cfg or DynamicPlaceCfg()
        self.sync_cfg = sync_cfg or SyncConfig()
        self.sync = BeltSynchronizer(self.sync_cfg)
        self.caps = transport.capabilities
        self.phase = 'APPROACH'
        self.armed = False
        self.track_id = None
        self.committed = False
        self.gate: GateDecision | None = None
        self.result: PlaceResult | None = None
        self._start = None
        self._phase_t = None
        self._last_sync_fb = -math.inf
        self._run_t = None
        self._sync_ok = True
        self._sync_held = True
        self._vz = 0.0
        self._v_prev = np.zeros(3)
        self._steady = None
        self._frozen: PlanSample | None = None
        self._commit_t = None
        self._released_t = None
        self._final_reason = ''
        self._retreat: KeepSyncRetreat | None = None
        self._open_width = goal.open_width_m if _finite(goal.open_width_m) and goal.open_width_m > 0 \
            else self.cfg.open_width_m

    # ------------------------------------------------------------ 조회
    @property
    def plan(self) -> PlanSample | None:
        return self._frozen or self.sync.plan

    @property
    def plan_revision(self) -> int:
        return int(self.plan.revision) if self.plan is not None else 0

    @property
    def release_xy(self):
        return tuple(self.plan.release_xy_m) if self.plan is not None else None

    @property
    def plan_velocity_xy(self):
        """SafeRetreat(keep_sync)에 넘길 계획 속도. 계획을 받은 적이 없으면 None."""
        return tuple(self.plan.velocity_xy_mps) if self.plan is not None else None

    # ------------------------------------------------------------ 시작 조건
    def _precondition(self) -> str:
        c, g = self.cfg, self.goal
        if g.mode not in (MODE_PREDICTED, MODE_SYNCHRONIZED):
            return 'MODE_UNSUPPORTED'
        if g.mode == MODE_SYNCHRONIZED and not self.caps.mid_course_update:
            return 'MODE_NEEDS_MID_COURSE_UPDATE'
        if not g.limits.valid():
            return 'GATE_LIMITS_INVALID'
        for value, reason in ((c.fingertip_below_tcp_m, 'FINGERTIP_OFFSET_UNSET'), (c.safe_z_m, 'SAFE_Z_UNSET'),
                              (self._open_width, 'OPEN_WIDTH_UNSET'),
                              (c.open_width_tolerance_m, 'OPEN_WIDTH_TOLERANCE_UNSET')):
            if value < 0.0:
                return reason
        if c.release_above_rim_m - c.fingertip_below_tcp_m < g.limits.rim_clearance_m - EPS:
            return 'RIM_CLEARANCE_UNREACHABLE'
        return ''

    # ------------------------------------------------------------ 주기 실행
    def tick(self, ros_now_s: float, steady_now_s: float, plans: Sequence[PlanSample] = (),
             tracks: Mapping[int, TrackInfo] | None = None, belt: BeltInfo | None = None,
             cancel: bool = False, stop: bool = False) -> TickOut:
        if self.result is not None:
            return TickOut((), self.result)
        fb: list[PlaceFeedback] = []
        ros, steady = float(ros_now_s), float(steady_now_s)
        if self._start is None:
            reason = self._precondition()
            if reason:
                return self._end(fb, Result.PRECONDITION, reason)
            self._start = ros
            self._enter(fb, 'APPROACH', ros)
        if stop:
            self.t.stop()
            return self._end(fb, Result.CANCELED, 'STOPPED')
        if cancel:
            return self._end(fb, Result.CANCELED, 'CANCELED', abort=self._keep_sync())
        if 0.0 < self.goal.deadline_s <= ros:
            return self._end(fb, Result.TIMEOUT, 'DEADLINE', abort=self._keep_sync())
        if not self.committed:
            failed = self._ingest(fb, plans, ros, steady)
            if failed is not None:
                return failed
            if self.sync.plan is None:
                if ros - self._start >= self.cfg.plan_wait_s - EPS:
                    return self._fail(fb, 'NO_PLAN', keep_sync=False)
                if self.armed and self.sync.fault:
                    return self._fail_sync(fb, self.sync.fault)
                return TickOut(tuple(fb), None)
        sample = self.t.poll()
        if sample.fault:
            return self._end(fb, Result.FAULT, sample.fault, abort=False)
        # poll()은 tick 시각을 읽은 뒤 TCP를 읽는다. 한 주기 안에서 늦게 찍힌 표본 시각은 미래가 아니므로 그
        # 시각으로 판정한다. 그보다 앞선 표본 시각은 그대로 두어 TCP_STALE로 잡는다(REQ-ROB-013).
        if 0.0 < sample.t_s - ros <= self.sync_cfg.period_s + EPS:
            ros = sample.t_s
        dt = self.sync_cfg.period_s if self._steady is None else max(steady - self._steady, 1e-6)
        self._steady = steady
        if self.committed:
            return self._tick_committed(fb, ros, steady, sample, dt)
        return self._tick_open(fb, ros, steady, sample, tracks or {}, belt)

    # ------------------------------------------------------------ commit 전
    def _ingest(self, fb, plans, ros, steady):
        for p in plans:
            if not self.armed:
                if p.plan_id != self.goal.plan_id:
                    continue                      # 다른 계획(이전 사이클 캐시 등)은 무장 전에는 무시한다
                if p.epoch != self.goal.target_epoch:
                    return self._fail(fb, 'PLAN_INVALID', keep_sync=self._keep_sync())
                self.sync.arm(p.plan_id, p.track_id, p.epoch)
                self.armed, self.track_id = True, p.track_id
            if self.sync.accept(p, ros, steady) and not _finite(p.release_z_m):
                self.sync.stop('PLAN_INVALID')
            if self.sync.fault:
                return self._fail_sync(fb, self.sync.fault)
        return None

    def _tick_open(self, fb, ros, steady, s, tracks, belt):
        c, g = self.cfg, self.goal
        pos, vel = np.asarray(s.pos_m, float), np.asarray(s.vel_mps, float)
        out = self.sync.step(ros, steady, tuple(pos[:2]), s.t_s, s.frame_id)
        if not out.active:
            return self._fail_sync(fb, out.reason)
        p = self.sync.plan
        target = np.asarray(out.target_xy_m, float)
        v_plan = _xy(p.velocity_xy_mps)
        e_xy, v_rel = _norm(pos[:2] - target), _norm(vel[:2] - v_plan)
        good = e_xy <= g.limits.e_xy_max_m + EPS and v_rel <= g.limits.v_rel_max_mps + EPS
        z_rel = float(p.release_z_m)
        z_app = z_rel + c.approach_above_m
        t_commit = p.release_s - self.caps.commit_lead_s
        z_t = z_rel
        if self.phase == 'APPROACH':
            z_t = z_app
            err = _norm([pos[0] - target[0], pos[1] - target[1], pos[2] - z_app])
            if err <= c.approach_tol_m + EPS:
                fb.append(self._fb('APPROACH', True, {'plan_valid': 1.0, 'pose_error_m': err}, e_xy, v_rel))
                self._enter(fb, 'SYNCHRONIZE', ros, e_xy, v_rel)
                self._last_sync_fb = ros
            elif ros - self._phase_t >= c.approach_timeout_s - EPS:
                return self._fail(fb, 'APPROACH_TIMEOUT', keep_sync=False)
        elif self.phase == 'SYNCHRONIZE':
            z_t = z_app
            if ros - self._last_sync_fb >= c.sync_feedback_period_s - 1e-9:
                self._last_sync_fb = ros
                fb.append(self._fb('SYNCHRONIZE', False, {}, e_xy, v_rel))
            if g.mode == MODE_SYNCHRONIZED:
                self._run_t = (self._run_t if self._run_t is not None else ros) if good else None
                hold = 0.0 if self._run_t is None else ros - self._run_t
                if self._run_t is not None and hold >= c.sync_hold_s - 1e-9:
                    fb.append(self._fb('SYNCHRONIZE', True, {'elapsed_s': ros - self._phase_t, 'hold_s': hold,
                                                             'e_xy_m': e_xy, 'v_rel_mps': v_rel}, e_xy, v_rel))
                    self._enter(fb, 'DESCEND', ros, e_xy, v_rel)
                elif ros - self._phase_t >= c.sync_timeout_s - EPS:
                    return self._fail(fb, 'SYNC_TIMEOUT')
            else:
                t_entry = p.release_s - c.entry_lead_s
                if ros >= t_entry - 1e-9 and v_rel <= g.limits.v_rel_max_mps + EPS:
                    fb.append(self._fb('SYNCHRONIZE', True, {'entry_time_error_s': ros - t_entry,
                                                             'v_rel_mps': v_rel}, e_xy, v_rel))
                    self._enter(fb, 'DESCEND', ros, e_xy, v_rel)
                elif ros - max(self._phase_t, t_entry) >= c.sync_timeout_s - EPS:
                    return self._fail(fb, 'SYNC_TIMEOUT')
        elif self.phase == 'DESCEND':
            # 프로그램형 transport는 하강을 프로그램 안에서 한다: commit 전에는 접근 높이에서 추종만 한다
            z_t = z_rel if self.caps.mid_course_update else z_app
            self._sync_ok = self._sync_ok and good
            height = pos[2] - z_rel
            if abs(height) <= c.descend_tol_m + EPS:
                return self._descend_done(fb, ros, pos, e_xy, v_rel)
            if ros - self._phase_t >= c.descend_timeout_s - EPS:
                return self._fail(fb, 'DESCEND_TIMEOUT')
            if g.mode == MODE_PREDICTED and ros >= t_commit - 1e-9:
                if self.caps.mid_course_update:
                    return self._fail(fb, 'RELEASE_TIME_MISSED')
                failed = self._try_commit(fb, ros, s, target, v_plan, z_rel, tracks, belt, final=True)
                if failed is not None:
                    return failed
        elif self.phase == 'RELEASE':
            if g.mode == MODE_PREDICTED:
                if ros >= t_commit - 1e-9:
                    if ros > t_commit + c.commit_late_tol_s + EPS:
                        return self._fail(fb, 'RELEASE_TIME_MISSED')
                    failed = self._try_commit(fb, ros, s, target, v_plan, z_rel, tracks, belt, final=True)
                    if failed is not None:
                        return failed
            else:
                window_end = (ros - self._phase_t >= c.release_wait_max_s - EPS
                              or ros + self.caps.commit_lead_s + c.release_hold_s > p.valid_until_s + EPS)
                failed = self._try_commit(fb, ros, s, target, v_plan, z_rel, tracks, belt, final=window_end)
                if failed is not None:
                    return failed
        if not self.committed or self.caps.mid_course_update:
            self._send(ros, target, v_plan, z_t, np.asarray(out.velocity_xy_mps, float), pos, vel)
        return TickOut(tuple(fb), None)

    def _descend_done(self, fb, ros, pos, e_xy, v_rel):
        c = self.cfg
        p = self.plan
        clearance = (pos[2] - c.fingertip_below_tcp_m) - (p.release_z_m - c.release_above_rim_m)
        fb.append(self._fb('DESCEND', True, {'sync_ok': 1.0 if self._sync_ok else 0.0, 'rim_clearance_m': clearance,
                                             'height_error_m': float(pos[2] - p.release_z_m)}, e_xy, v_rel))
        if not self.committed and not self._sync_ok:
            return self._fail(fb, 'SYNC_LOST')     # orchestrator가 DESCEND를 실패로 볼 값이면 해제하지 않는다
        self._enter(fb, 'RELEASE', ros, e_xy, v_rel)
        return TickOut(tuple(fb), None)

    def _try_commit(self, fb, ros, s, target, v_plan, z_rel, tracks, belt, final):
        """게이트를 판정하고 열리면 commit 1회. 닫혀 있으면 terminal이거나 `final`일 때만 실패로 끝낸다."""
        c, g, p = self.cfg, self.goal, self.sync.plan
        h = self.caps.commit_lead_s
        pos, vel = np.asarray(s.pos_m, float), np.asarray(s.vel_mps, float)
        if self.caps.mid_course_update:
            stone, stone_v = pos[:2], vel[:2]
            tip_z, tip_vz = pos[2] - c.fingertip_below_tcp_m, vel[2]
        else:                                        # 프로그램이 TCP를 목표 궤적에 놓는다(사전 측정 없음)
            stone, stone_v = target, v_plan
            tip_z, tip_vz = z_rel - c.fingertip_below_tcp_m, 0.0
        self.gate = release_gate(GateInputs(
            ros, tuple(stone), tuple(stone_v), tuple(target), tuple(v_plan), tip_z, tip_vz,
            z_rel - c.release_above_rim_m, p.position_std_m, p.valid_until_s, g.target_epoch, p.epoch,
            tracks.get(self.track_id), belt, self.sync.holding), g.limits, h)
        if not self.gate.ok:
            if self.gate.terminal or final:
                return self._fail(fb, self.gate.reason)
            return None
        permit = ReleasePermit(ros, ros + h, (*(target + v_plan * h), z_rel), (*v_plan, 0.0), self._open_width,
                               c.release_hold_s, c.safe_z_m + c.retreat_margin_m, c.retreat_speed_mps,
                               dict(self.gate.metrics))
        if not self.t.commit_release(permit):
            return self._fail(fb, 'COMMIT_REJECTED')
        self.committed, self._commit_t, self._frozen = True, ros, p
        return None

    def _send(self, ros, target, v_plan, z_t, cmd_xy, pos, vel):
        c = self.cfg
        dt = self.sync_cfg.period_s
        self._vz = _z_command(z_t, pos[2], self._vz, dt, c.z_kp_hz, c.z_max_speed_mps, c.z_max_accel_mps2)
        cmd = (float(cmd_xy[0]), float(cmd_xy[1]), self._vz)
        self._v_prev = np.array(cmd)
        self.t.update(MotionRef(ros, (float(target[0]), float(target[1]), z_t), (*v_plan, 0.0), cmd))

    # ------------------------------------------------------------ commit 뒤
    def _tick_committed(self, fb, ros, steady, s, dt):
        c, g, p = self.cfg, self.goal, self._frozen
        pos, vel = np.asarray(s.pos_m, float), np.asarray(s.vel_mps, float)
        v_plan = _xy(p.velocity_xy_mps)
        target = _xy(p.release_xy_m) + v_plan * (ros - p.release_s)
        e_xy, v_rel = _norm(pos[:2] - target), _norm(vel[:2] - v_plan)
        good = e_xy <= g.limits.e_xy_max_m + EPS and v_rel <= g.limits.v_rel_max_mps + EPS
        if self.phase == 'RETREAT':
            done = self._retreat.step(ros, steady, s)
            if done is not None:
                fb.append(self._fb('RETREAT', True, done, e_xy, v_rel))
                self.t.abort(False)
                reason = self._final_reason or ('' if self._retreat.speed_ok() else 'SYNC_SPEED')
                return self._end(fb, Result.FAILED if reason else Result.OK, reason, metrics=done)
            if ros - self._phase_t >= c.retreat_timeout_s - EPS:
                self.t.abort(False)
                return self._end(fb, Result.FAILED, 'RETREAT_TIMEOUT')
            return TickOut(tuple(fb), None)
        if self.phase == 'DESCEND':                  # 프로그램형: 프로그램이 내려가는 것을 지켜본다
            self._sync_ok = self._sync_ok and good
            if abs(pos[2] - p.release_z_m) <= c.descend_tol_m + EPS:
                self._descend_done(fb, ros, pos, e_xy, v_rel)
            elif ros - self._phase_t >= c.descend_timeout_s - EPS:
                self._final_reason = self._final_reason or 'DESCEND_TIMEOUT'
                self._enter(fb, 'RELEASE', ros)
        if self.phase == 'RELEASE':
            self._sync_held = self._sync_held and good
            if s.released and self._released_t is None:
                self._released_t = ros
            width = float(s.gripper_width_m)
            confirmed = self._released_t is not None and ros - self._released_t >= c.release_hold_s - 1e-9
            # 개방 확인 제한은 예정 해제 시각(commit + commit_lead_s)부터 센다
            if confirmed or ros - self._commit_t - self.caps.commit_lead_s >= c.release_timeout_s - EPS:
                fb.append(self._fb('RELEASE', True, {'open_width_m': width,
                                                     'sync_held': 1.0 if self._sync_held else 0.0}, e_xy, v_rel))
                if not confirmed:
                    self._final_reason = self._final_reason or 'OPEN_NOT_CONFIRMED'
                elif abs(width - self._open_width) > c.open_width_tolerance_m + EPS:
                    self._final_reason = self._final_reason or 'OPEN_WIDTH'
                elif not self._sync_held:
                    self._final_reason = self._final_reason or 'SYNC_LOST'
                self._enter(fb, 'RETREAT', ros)
                self._retreat = KeepSyncRetreat(self.t, tuple(v_plan), c, period_s=self.sync_cfg.period_s)
                return TickOut(tuple(fb), None)
            if self.caps.mid_course_update:          # 개방 지연 + 여유 동안 동기 유지
                cmd = tracking_command([*target, 0.0], [*v_plan, 0.0], [pos[0], pos[1], 0.0], self.sync_cfg.kp,
                                       self.sync_cfg.max_speed_mps, self.sync_cfg.max_accel_mps2,
                                       [self._v_prev[0], self._v_prev[1], 0.0], dt)
                self._send(ros, target, v_plan, p.release_z_m, cmd[:2], pos, vel)
        return TickOut(tuple(fb), None)

    # ------------------------------------------------------------ 공통
    def _keep_sync(self) -> bool:
        return self.committed or self.phase in DYNAMIC_PHASES

    def _fb(self, phase, done, metrics, e_xy=math.nan, v_rel=math.nan) -> PlaceFeedback:
        return PlaceFeedback(phase, done, {k: float(v) for k, v in metrics.items()}, float(e_xy), float(v_rel),
                             self.plan_revision)

    def _enter(self, fb, phase, ros, e_xy=math.nan, v_rel=math.nan):
        self.phase, self._phase_t = phase, ros
        fb.append(self._fb(phase, False, {}, e_xy, v_rel))

    def _fail_sync(self, fb, reason):
        mapped = 'PLAN_INVALID' if reason in _PLAN_INVALID else reason
        keep = self._keep_sync() and reason not in _NO_KEEP_SYNC
        return self._fail(fb, mapped, keep_sync=keep)

    def _fail(self, fb, reason, keep_sync=None):
        keep = self._keep_sync() if keep_sync is None else keep_sync
        return self._end(fb, Result.FAILED, reason, abort=keep)

    def _end(self, fb, result, reason, abort=None, metrics=None) -> TickOut:
        if abort is not None:
            self.t.abort(bool(abort))
        released = self.committed
        abort_request = 3 if result == Result.FAILED and not released else 0
        gate = dict(self.gate.metrics) if self.gate is not None else _nan_gate()
        self.result = PlaceResult(result, reason, released, gate, dict(metrics or {}), abort_request)
        return TickOut(tuple(fb), self.result)
