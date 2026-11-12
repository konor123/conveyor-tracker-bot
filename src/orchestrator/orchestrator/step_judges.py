"""단계별 성공 판정(SPEC-100 4.2~4.4절). 입력은 측정값, 출력은 Judgement. 길이 m, 시간 s.

reason "WAIT"는 판정 보류(같은 단계에서 관측 계속)이며 result는 FAILED로 둔다. 실행기는 이를 실패로 처리하지 않는다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence

from .common import FromDict, Judgement, Result, ge, le

WAIT = 'WAIT'
PASS_SKIP = 'PASS_SKIP'
VERDICT_SUCCESS, VERDICT_FAILURE, VERDICT_UNKNOWN = 0, 1, 2      # DropTask 결과 코드와 같은 값

METRIC_KEYS: dict[str, frozenset[str]] = {
    'PRECHECK': frozenset({'nodes_active', 'heartbeat_age_max_s', 'calibration_valid', 'robot_ready',
                           'protective_stop', 'gripper_alive', 'gripper_response_s',
                           'gripper_error_bits', 'gripper_open', 'belt_steady', 'belt_speed_mps'}),
    'DETECT_STONE': frozenset({'stone_count', 'confidence', 'position_valid', 'window_std_m',
                               'clearance_m'}),
    'PICK_STONE': frozenset({'grip_width_m', 'grip_current_ma', 'stone_on_mat', 'robot_error',
                             'gripper_error'}),
    'MOVE_STANDBY': frozenset({'pose_error_m', 'holding'}),
    # 시간 예산 성분 budget_<성분>_s 가 추가된다
    'WAIT_CAP': frozenset({'track_observed', 'observation_count', 'speed_std_mps', 'belt_steady',
                           'stone_inside', 'stone_inside_valid', 't_available_s', 't_required_s',
                           'time_margin_s', 't_to_window_s'}),
    'PLAN_INTERCEPT': frozenset({'plan_ok', 'epoch_match', 'time_margin_s', 'position_std_m',
                                 'valid_for_s'}),
    'APPROACH': frozenset({'plan_valid', 'pose_error_m'}),
    'SYNCHRONIZE': frozenset({'elapsed_s', 'hold_s', 'e_xy_m', 'v_rel_mps'}),
    'SYNCHRONIZE_PREDICTED': frozenset({'entry_time_error_s', 'v_rel_mps'}),
    'DESCEND': frozenset({'sync_ok', 'rim_clearance_m', 'height_error_m'}),
    'DESCEND_STATIC': frozenset({'target_error_m', 'rim_clearance_m', 'height_error_m'}),
    'RELEASE': frozenset({'open_width_m', 'sync_held'}),
    'RETREAT': frozenset({'tcp_z_m', 'outside_workspace', 'stone_in_gripper'}),
    'VERIFY': frozenset({'responded', 'verdict'}),
}


def _metrics(**kw) -> dict[str, float]:
    return {k: float(v) for k, v in kw.items()}


def _judge(checks, metrics, ok_detail: str = '') -> Judgement:
    """checks: (실패 조건, 이유 코드[, detail]) 열. 처음 걸리는 것이 판정이다."""
    for c in checks:
        if c[0]:
            return Judgement(Result.FAILED, c[1], metrics, c[2] if len(c) > 2 else '')
    return Judgement(Result.OK, '', metrics, ok_detail)


# ---------------------------------------------------------------- PRECHECK
@dataclass(frozen=True)
class PrecheckMeas:
    nodes_active: bool
    heartbeat_age_max_s: float
    calibration_valid: bool
    robot_ready: bool
    protective_stop: bool
    gripper_alive: bool
    gripper_response_s: float
    gripper_error_bits: int
    gripper_open: bool
    belt_steady: bool
    belt_speed_mps: float
    calibration_mismatch: bool = False  # vision과 orchestrator의 calibration_id가 둘 다 있고 다름. 이유 코드만 가른다


@dataclass(frozen=True)
class PrecheckParams(FromDict):
    heartbeat_age_max_s: float
    gripper_response_max_s: float
    max_belt_speed_mps: float          # DropTask.max_belt_speed


def judge_precheck(m: PrecheckMeas, p: PrecheckParams) -> Judgement:
    metrics = _metrics(nodes_active=m.nodes_active, heartbeat_age_max_s=m.heartbeat_age_max_s,
                       calibration_valid=m.calibration_valid, robot_ready=m.robot_ready,
                       protective_stop=m.protective_stop, gripper_alive=m.gripper_alive,
                       gripper_response_s=m.gripper_response_s, gripper_error_bits=m.gripper_error_bits,
                       gripper_open=m.gripper_open, belt_steady=m.belt_steady,
                       belt_speed_mps=m.belt_speed_mps)
    return _judge([
        (not m.nodes_active, 'NODE_INACTIVE'),
        (not le(m.heartbeat_age_max_s, p.heartbeat_age_max_s), 'HEARTBEAT_STALE'),
        (m.calibration_mismatch, 'CALIBRATION_MISMATCH'),
        (not m.calibration_valid, 'CALIBRATION_INVALID'),
        (m.protective_stop, 'PROTECTIVE_STOP'),
        (not m.robot_ready, 'ROBOT_NOT_READY'),
        (not m.gripper_alive, 'GRIPPER_DEAD'),
        (m.gripper_error_bits != 0, 'GRIPPER_ERROR'),
        (not le(m.gripper_response_s, p.gripper_response_max_s), 'GRIPPER_SLOW'),
        (not m.gripper_open, 'GRIPPER_NOT_OPEN'),
        (not m.belt_steady, 'BELT_UNSTEADY'),
        (not le(m.belt_speed_mps, p.max_belt_speed_mps), 'BELT_TOO_FAST'),
    ], metrics)


# ---------------------------------------------------------------- DETECT_STONE
@dataclass(frozen=True)
class DetectMeas:
    stone_count: int
    confidence: float
    position_valid: bool
    window_std_m: float          # 안정 창 위치 표준편차
    clearance_m: float           # 주변 물체까지 최소 거리(핑거 간섭)


@dataclass(frozen=True)
class DetectParams(FromDict):
    confidence_min_ratio: float
    window_std_max_m: float
    clearance_min_m: float


def judge_detect_stone(m: DetectMeas, p: DetectParams) -> Judgement:
    metrics = _metrics(stone_count=m.stone_count, confidence=m.confidence,
                       position_valid=m.position_valid, window_std_m=m.window_std_m,
                       clearance_m=m.clearance_m)
    return _judge([
        (m.stone_count < 1, 'NO_STONE'),
        (not m.position_valid, 'POSITION_INVALID'),
        (not ge(m.confidence, p.confidence_min_ratio), 'LOW_CONFIDENCE'),
        (not le(m.window_std_m, p.window_std_max_m), 'POSITION_UNSTABLE'),
        (not ge(m.clearance_m, p.clearance_min_m), 'CLEARANCE'),
    ], metrics)


# ---------------------------------------------------------------- PICK_STONE
@dataclass(frozen=True)
class PickMeas:
    grip_width_m: float          # Present Position → pulse 실측표 변환
    grip_current_ma: float
    stone_on_mat: bool           # 매트 ROI에 해당 공깃돌이 남아 있음
    robot_error: bool
    gripper_error: bool


@dataclass(frozen=True)
class PickParams(FromDict):
    d_stone_min_m: float         # 팔각기둥: 면-면 거리
    d_stone_max_m: float         # 팔각기둥: 꼭짓점-꼭짓점 거리
    width_tol_m: float
    grip_current_min_ma: float
    detection_age_max_s: float   # 선행조건(실행기에서 사용)


def judge_pick_stone(m: PickMeas, p: PickParams) -> Judgement:
    metrics = _metrics(grip_width_m=m.grip_width_m, grip_current_ma=m.grip_current_ma,
                       stone_on_mat=m.stone_on_mat, robot_error=m.robot_error,
                       gripper_error=m.gripper_error)
    lo, hi = p.d_stone_min_m - p.width_tol_m, p.d_stone_max_m + p.width_tol_m
    return _judge([
        (m.robot_error, 'ROBOT_ERROR'),
        (m.gripper_error, 'GRIPPER_ERROR'),
        (m.stone_on_mat, 'STONE_ON_MAT'),
        (not (ge(m.grip_width_m, lo) and le(m.grip_width_m, hi)), 'GRIP_WIDTH'),
        (not ge(m.grip_current_ma, p.grip_current_min_ma), 'GRIP_CURRENT_LOW'),
    ], metrics)


# ---------------------------------------------------------------- MOVE_STANDBY
@dataclass(frozen=True)
class StandbyMeas:
    pose_error_m: float
    holding: bool


@dataclass(frozen=True)
class StandbyParams(FromDict):
    pose_error_max_m: float


def judge_move_standby(m: StandbyMeas, p: StandbyParams) -> Judgement:
    metrics = _metrics(pose_error_m=m.pose_error_m, holding=m.holding)
    return _judge([
        (not m.holding, 'DROPPED'),
        (not le(m.pose_error_m, p.pose_error_max_m), 'POSE_ERROR'),
    ], metrics)


# ---------------------------------------------------------------- WAIT_CAP
@dataclass(frozen=True)
class WaitCapMeas:
    track_observed: bool         # SelectTarget track이 OBSERVED
    observation_count: int
    speed_std_mps: float
    belt_steady: bool
    stone_inside: bool
    stone_inside_valid: bool
    t_available_s: float
    t_required_s: float
    budget_components: dict[str, float] = field(default_factory=dict)
    t_to_window_s: float = 0.0   # 도달 구간 진입까지 남은 시간((x_reach_start − x) / v, REQ-ORC-018), STATIC은 0


@dataclass(frozen=True)
class WaitCapParams(FromDict):
    observation_count_min_count: int
    speed_std_max_mps: float
    margin_s: float
    plan_lead_s: float = math.inf  # 도달 구간 진입 이 시간 전부터 계획한다(예측 구간 축소, REQ-ORC-013)


def judge_wait_cap(m: WaitCapMeas, p: WaitCapParams) -> Judgement:
    margin = m.t_available_s - m.t_required_s
    metrics = _metrics(track_observed=m.track_observed, observation_count=m.observation_count,
                       speed_std_mps=m.speed_std_mps, belt_steady=m.belt_steady,
                       stone_inside=m.stone_inside, stone_inside_valid=m.stone_inside_valid,
                       t_available_s=m.t_available_s, t_required_s=m.t_required_s,
                       time_margin_s=margin, t_to_window_s=m.t_to_window_s)
    metrics.update({f'budget_{k}_s': float(v) for k, v in m.budget_components.items()})
    return _judge([
        (not m.track_observed, WAIT, 'TRACK_NOT_OBSERVED'),
        (m.observation_count < p.observation_count_min_count, WAIT, 'OBSERVATION_COUNT'),
        (not le(m.speed_std_mps, p.speed_std_max_mps), WAIT, 'SPEED_STD'),
        (not m.belt_steady, WAIT, 'BELT_UNSTEADY'),
        (not m.stone_inside_valid, PASS_SKIP, 'CAP_STATE_UNKNOWN'),
        (m.stone_inside, 'CAP_NOT_EMPTY'),
        (not le(m.t_to_window_s, p.plan_lead_s), WAIT, 'TOO_EARLY'),
        (not ge(margin, p.margin_s), PASS_SKIP, 'TIME_MARGIN'),
    ], metrics)


# ---------------------------------------------------------------- PLAN_INTERCEPT
@dataclass(frozen=True)
class PlanMeas:
    plan_ok: bool                # ComputeIntercept status == OK
    epoch_match: bool
    time_margin_s: float
    position_std_m: float
    valid_until: float           # ROS 시각
    now: float                   # ROS 시각


@dataclass(frozen=True)
class PlanParams(FromDict):
    time_margin_min_s: float
    position_std_max_m: float
    t_approach_s: float


def judge_plan_intercept(m: PlanMeas, p: PlanParams) -> Judgement:
    valid_for = m.valid_until - m.now
    metrics = _metrics(plan_ok=m.plan_ok, epoch_match=m.epoch_match, time_margin_s=m.time_margin_s,
                       position_std_m=m.position_std_m, valid_for_s=valid_for)
    return _judge([
        (not m.plan_ok, 'PLAN_STATUS'),
        (not m.epoch_match, 'EPOCH_MISMATCH'),
        (not ge(m.time_margin_s, p.time_margin_min_s), 'TIME_MARGIN'),
        (not le(m.position_std_m, p.position_std_max_m), 'POSITION_STD'),
        (not (valid_for > p.t_approach_s + 1e-9), 'PLAN_EXPIRES'),    # valid_until > now + T_approach
    ], metrics)


# ---------------------------------------------------------------- APPROACH
@dataclass(frozen=True)
class ApproachMeas:
    plan_valid: bool
    pose_error_m: float          # 해제점 위 접근 높이 기준


@dataclass(frozen=True)
class ApproachParams(FromDict):
    pose_error_max_m: float


def judge_approach(m: ApproachMeas, p: ApproachParams) -> Judgement:
    metrics = _metrics(plan_valid=m.plan_valid, pose_error_m=m.pose_error_m)
    return _judge([
        (not m.plan_valid, 'PLAN_INVALID'),
        (not le(m.pose_error_m, p.pose_error_max_m), 'POSE_ERROR'),
    ], metrics)


# ---------------------------------------------------------------- SYNCHRONIZE
@dataclass(frozen=True)
class SyncSample:
    t: float
    e_xy_m: float
    v_rel_mps: float


@dataclass(frozen=True)
class SyncParams(FromDict):
    e_xy_max_m: float
    v_rel_max_mps: float
    hold_s: float
    timeout_s: float
    release_position_std_max_m: float = 0.0005   # 해제 게이트 기준, 계획 단계 기준과 독립(REQ-ORC-014)


def judge_synchronize(samples: Sequence[SyncSample], p: SyncParams) -> Judgement:
    """e_xy ≤ e_xy_max, v_rel ≤ v_rel_max가 hold_s 연속이면 OK. 위반하면 다시 센다. timeout_s 안에 못 채우면 TIMEOUT."""
    if not samples:
        return Judgement(Result.FAILED, WAIT, _metrics(elapsed_s=0.0, hold_s=0.0, e_xy_m=0.0,
                                                       v_rel_mps=0.0))
    t0 = samples[0].t
    run_start = None
    for s in samples:
        good = le(s.e_xy_m, p.e_xy_max_m) and le(s.v_rel_mps, p.v_rel_max_mps)
        if good:
            run_start = s.t if run_start is None else run_start
        else:
            run_start = None
        hold = 0.0 if run_start is None else s.t - run_start
        metrics = _metrics(elapsed_s=s.t - t0, hold_s=hold, e_xy_m=s.e_xy_m, v_rel_mps=s.v_rel_mps)
        if run_start is not None and ge(hold, p.hold_s):
            return Judgement(Result.OK, '', metrics)
        if ge(s.t - t0, p.timeout_s):
            return Judgement(Result.TIMEOUT, 'TIMEOUT', metrics)
    return Judgement(Result.FAILED, WAIT, metrics)


@dataclass(frozen=True)
class EntryMeas:
    entry_time_error_s: float    # 계획 속도 일치 구간 진입 시각 − 계획 시각
    v_rel_mps: float


@dataclass(frozen=True)
class EntryParams(FromDict):
    entry_time_tol_s: float
    v_rel_max_mps: float


def judge_predicted_entry(m: EntryMeas, p: EntryParams) -> Judgement:
    """MODE_PREDICTED의 SYNCHRONIZE: 계획 속도 일치 구간 진입 판정."""
    metrics = _metrics(entry_time_error_s=m.entry_time_error_s, v_rel_mps=m.v_rel_mps)
    return _judge([
        (not le(abs(m.entry_time_error_s), p.entry_time_tol_s), 'ENTRY_TIME'),
        (not le(m.v_rel_mps, p.v_rel_max_mps), 'V_REL'),
    ], metrics)


# ---------------------------------------------------------------- DESCEND
@dataclass(frozen=True)
class DescendMeas:
    sync_ok: bool                # 하강 중 동기 게이트 계속 만족
    rim_clearance_m: float       # 핑거 끝 − 테두리
    height_error_m: float        # |z − z_release|


@dataclass(frozen=True)
class DescendParams(FromDict):
    height_tol_m: float
    h_clear_m: float


def judge_descend(m: DescendMeas, p: DescendParams) -> Judgement:
    metrics = _metrics(sync_ok=m.sync_ok, rim_clearance_m=m.rim_clearance_m,
                       height_error_m=m.height_error_m)
    return _judge([
        (not m.sync_ok, 'SYNC_LOST'),
        (not ge(m.rim_clearance_m, p.h_clear_m), 'RIM_CLEARANCE'),
        (not le(abs(m.height_error_m), p.height_tol_m), 'HEIGHT_ERROR'),
    ], metrics)


@dataclass(frozen=True)
class StaticPlaceMeas:
    target_error_m: float        # 정지 뚜껑 목표 기준 XY 오차
    rim_clearance_m: float
    height_error_m: float


@dataclass(frozen=True)
class StaticPlaceParams(FromDict):
    target_error_max_m: float
    height_tol_m: float
    h_clear_m: float


def judge_static_place(m: StaticPlaceMeas, p: StaticPlaceParams) -> Judgement:
    """MODE_STATIC의 DESCEND."""
    metrics = _metrics(target_error_m=m.target_error_m, rim_clearance_m=m.rim_clearance_m,
                       height_error_m=m.height_error_m)
    return _judge([
        (not le(m.target_error_m, p.target_error_max_m), 'TARGET_ERROR'),
        (not ge(m.rim_clearance_m, p.h_clear_m), 'RIM_CLEARANCE'),
        (not le(abs(m.height_error_m), p.height_tol_m), 'HEIGHT_ERROR'),
    ], metrics)


# ---------------------------------------------------------------- RELEASE
@dataclass(frozen=True)
class ReleaseMeas:
    open_width_m: float
    sync_held: bool              # 개방 지연(p95) + 여유 동안 동기 유지


@dataclass(frozen=True)
class ReleaseParams(FromDict):
    d_stone_max_m: float
    delta_open_m: float
    open_tol_m: float


def judge_release(m: ReleaseMeas, p: ReleaseParams) -> Judgement:
    metrics = _metrics(open_width_m=m.open_width_m, sync_held=m.sync_held)
    return _judge([
        (not ge(m.open_width_m, p.d_stone_max_m + p.delta_open_m - p.open_tol_m), 'OPEN_WIDTH'),
        (not m.sync_held, 'SYNC_LOST'),
    ], metrics)


# ---------------------------------------------------------------- RETREAT
@dataclass(frozen=True)
class RetreatMeas:
    tcp_z_m: float
    outside_workspace: bool
    stone_in_gripper: bool


@dataclass(frozen=True)
class RetreatParams(FromDict):
    safe_z_m: float


def judge_retreat(m: RetreatMeas, p: RetreatParams) -> Judgement:
    metrics = _metrics(tcp_z_m=m.tcp_z_m, outside_workspace=m.outside_workspace,
                       stone_in_gripper=m.stone_in_gripper)
    return _judge([
        (not ge(m.tcp_z_m, p.safe_z_m), 'Z_LOW'),
        (m.stone_in_gripper, 'STONE_IN_GRIPPER'),
        (not m.outside_workspace, 'IN_WORKSPACE'),
    ], metrics)


# ---------------------------------------------------------------- VERIFY
@dataclass(frozen=True)
class VerifyMeas:
    responded: bool              # VerifyDrop이 제한시간 안에 응답
    verdict: int                 # VERDICT_*


@dataclass(frozen=True)
class VerifyParams(FromDict):
    timeout_s: float


def judge_verify(m: VerifyMeas, p: VerifyParams) -> Judgement:
    """보조 판정. 응답이 없으면 결과 UNKNOWN(reason VERIFY_TIMEOUT)."""
    if not m.responded:
        return Judgement(Result.TIMEOUT, 'VERIFY_TIMEOUT',
                         _metrics(responded=False, verdict=VERDICT_UNKNOWN), 'UNKNOWN')
    name = {VERDICT_SUCCESS: 'SUCCESS', VERDICT_FAILURE: 'FAILURE'}.get(m.verdict, 'UNKNOWN')
    return Judgement(Result.OK, '', _metrics(responded=True, verdict=m.verdict), name)
