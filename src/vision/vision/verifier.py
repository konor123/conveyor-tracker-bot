"""투입 자동 확인(SPEC-200 4.6절, PLAN.md 4.5절). 결과는 SUCCESS 또는 UNKNOWN이며 FAILURE를 내지 않는다.

ROS와 무관한 순수 모듈. 입력은 후퇴 완료 뒤 프레임별 병뚜껑 관측(track id로 연결된 검출)이고,
선택한 뚜껑에서 `stone_inside`가 `hold_time_s` 동안 끊기지 않으면 SUCCESS다. 실패 원인(튕겨 나감,
그리퍼에 붙음, 이동 중 떨어뜨림)은 관찰자가 판정한다. 시각은 영상 노출 시각 [s], 길이는 robot_base [m]이다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

SUCCESS, FAILURE, UNKNOWN = 0, 1, 2      # VerifyDrop.Response 상수와 같은 값. FAILURE는 내지 않는다


@dataclass(frozen=True)
class CapObs:
    """한 프레임의 병뚜껑 하나. `stone_inside_valid=False`는 가림 등으로 판정 불가."""
    track_id: int
    stone_inside_valid: bool
    stone_inside: bool = False                              # 4.3절: 공깃돌 중심이 뚜껑 안쪽 원 안
    stone_offset_xy_m: tuple[float, float] | None = None    # 뚜껑 중심 → 안에 든 공깃돌 중심 XY


@dataclass(frozen=True)
class FrameObs:
    stamp_s: float
    caps: tuple[CapObs, ...] = ()


@dataclass(frozen=True)
class VerifyParams:
    hold_time_s: float = 1.0       # 유지 확인 시간(VerifyDrop.hold_time, PLAN.md 4.5절)
    max_gap_s: float = 0.2         # 판정 가능한 관측 사이 최대 간격. 넘으면 연속 유지가 아니다(제안값)


@dataclass(frozen=True)
class Verdict:
    verdict: int                   # SUCCESS / UNKNOWN
    reason: str                    # SUCCESS면 "", 아니면 NOT_OBSERVED / NOT_INSIDE / HOLD_SHORT
    stone_inside: bool             # 선택한 뚜껑의 마지막 판정 가능 관측
    stone_offset_m: float          # SUCCESS 구간 평균 오프셋의 크기(안착 오프셋, 기록용)
    stone_offset_valid: bool
    hold_s: float                  # 가장 긴 연속 유지 구간 [s]


def verify(frames: Sequence[FrameObs], cap_track_id: int, p: VerifyParams) -> Verdict:
    if not (math.isfinite(p.hold_time_s) and p.hold_time_s > 0 and math.isfinite(p.max_gap_s) and p.max_gap_s > 0):
        raise ValueError('hold_time_s and max_gap_s must be positive')
    obs = []                                  # (시각, 뚜껑 관측) — 선택한 뚜껑을 판정할 수 있는 프레임만
    for f in sorted(frames, key=lambda f: f.stamp_s):
        cap = next((c for c in f.caps if c.track_id == cap_track_id), None)
        if cap is not None and cap.stone_inside_valid and (not obs or f.stamp_s > obs[-1][0]):
            obs.append((f.stamp_s, cap))
    if not obs:
        return Verdict(UNKNOWN, 'NOT_OBSERVED', False, 0.0, False, 0.0)
    best, run = [], []
    for t, cap in obs:
        if not cap.stone_inside or (run and t - run[-1][0] > p.max_gap_s):
            run = []
        if cap.stone_inside:
            run.append((t, cap))
            if run[-1][0] - run[0][0] > (best[-1][0] - best[0][0] if best else -1.0):
                best = list(run)
    last_inside = obs[-1][1].stone_inside
    if not best:
        return Verdict(UNKNOWN, 'NOT_INSIDE', last_inside, 0.0, False, 0.0)
    hold = best[-1][0] - best[0][0]
    if hold < p.hold_time_s - 1e-9:
        return Verdict(UNKNOWN, 'HOLD_SHORT', last_inside, 0.0, False, hold)
    offsets = [c.stone_offset_xy_m for _, c in best if c.stone_offset_xy_m is not None]
    if offsets:
        mx = sum(o[0] for o in offsets) / len(offsets)
        my = sum(o[1] for o in offsets) / len(offsets)
        return Verdict(SUCCESS, '', last_inside, math.hypot(mx, my), True, hold)
    return Verdict(SUCCESS, '', last_inside, 0.0, False, hold)
