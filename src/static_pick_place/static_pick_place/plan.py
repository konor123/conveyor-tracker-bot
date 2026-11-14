"""정지 pick-and-place 계획: 검출 → 경유점. ROS와 무관한 순수 모듈.

모든 값은 robot_base(= Doosan base_link) 기준 m. 그리퍼는 수직 아래(Rz(0) Ry(180°) Rz(0)),
TCP는 등록하지 않은 Doosan 기본 플랜지이므로 높이는 플랜지 기준이다.
기본값은 2026-11-09 LEGION 현장 시험에서 얻었다.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

CLASS_STONE, CLASS_CAP = 1, 2
REGION_MAT, REGION_BELT = 1, 2
DOWN_QUAT_XYZW = (0.0, 1.0, 0.0, 0.0)      # 플랜지 z축이 robot_base −z를 향함


class PlanError(ValueError):
    def __init__(self, reason: str, detail: str = ''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason


@dataclass(frozen=True)
class PickPlaceParams:
    # 높이(플랜지) — 2026-11-09 실측
    travel_z_m: float = 0.2844                    # 매트·벨트 위 이동 높이. 11-07 Hand-Eye·11-09 시험에서 도달 확인
    grasp_above_stone_top_m: float = 0.1620       # 플랜지 z − 공깃돌 윗면 z. 파지 성공 높이 144.4 mm − 돌 윗면 −17.6 mm
    release_above_rim_m: float = 0.1656           # 플랜지 z − 뚜껑 테두리 z. 투입 성공 160.6 mm(244.4 − 83.8)에서 사용자 지시로 5 mm 올림
    # 속도
    travel_v_mps: float = 0.100
    vertical_v_mps: float = 0.050
    travel_a_mps2: float = 0.300
    vertical_a_mps2: float = 0.150
    # 그리퍼(하드웨어 pulse, 0 = 완전 열림). 현장 닫힘 끝점 750, 작은 대상이라 600까지만 연다
    open_pulse: int = 600
    close_pulse: int = 750
    grip_current_ma: int = 150
    gripper_read_timeout_s: float = 0.5           # DRL 플랜지 RS-485 응답 대기. 0.25 s는 가끔, 0.15 s 이하는 항상 응답을 놓침(11-09)
    gripper_timeout_s: float = 15.0
    # 대기 자세(끝난 뒤 카메라 시야에서 비킨다). 2026-11-09 드라이버 연결 시 자세
    park_x_m: float = 0.373
    park_y_m: float = 0.0
    park_z_m: float = 0.405
    # 작업 공간(플랜지 XY)과 도달 여유
    x_min_m: float = 0.30
    x_max_m: float = 0.75
    y_min_m: float = -0.25
    y_max_m: float = 0.25
    z_min_m: float = 0.10
    j3_min_rad: float = 0.1745                    # 역기구학 J3(10°)보다 작으면 팔을 다 편 경계(11-09: 벨트 x≈0.81 m에서 0°)


@dataclass(frozen=True)
class Step:
    name: str
    kind: str                                     # 'move' | 'gripper'
    pos_m: tuple[float, float, float] | None = None
    v_mps: float = 0.0
    a_mps2: float = 0.0
    pulse: int = 0
    current_ma: int = 0
    grasp: bool = False                           # gripper: 막혀 멈추면 성공(파지)


def _usable(d) -> bool:
    return d.position_valid and not d.blob.occluded_partial


def choose_stone(detections: Sequence, p: PickPlaceParams):
    """매트 위 공깃돌 중 작업 공간 안에서 로봇 베이스에 가장 가까운 것."""
    cands = [d for d in detections if d.region == REGION_MAT and d.object_class == CLASS_STONE and _usable(d)
             and _in_workspace(d.position, p)]
    return min(cands, key=lambda d: np.hypot(d.position[0], d.position[1])) if cands else None


def choose_cap(detections: Sequence, p: PickPlaceParams):
    """벨트 위 빈 병뚜껑(stone_inside 판정 가능하고 false). 여러 개면 베이스에 가장 가까운 것."""
    cands = [d for d in detections if d.region == REGION_BELT and d.object_class == CLASS_CAP and _usable(d)
             and d.stone_inside_valid and not d.stone_inside and _in_workspace(d.position, p)]
    return min(cands, key=lambda d: np.hypot(d.position[0], d.position[1])) if cands else None


def _in_workspace(pos, p: PickPlaceParams) -> bool:
    return p.x_min_m <= pos[0] <= p.x_max_m and p.y_min_m <= pos[1] <= p.y_max_m


def plan_pick_place(stone_top_m, rim_top_m, p: PickPlaceParams) -> list[Step]:
    """stone_top_m: 공깃돌 윗면 중심, rim_top_m: 뚜껑 테두리 윗면 중심(둘 다 robot_base [m])."""
    s = np.asarray(stone_top_m, float)
    c = np.asarray(rim_top_m, float)
    for name, q in (('stone', s), ('cap', c)):
        if q.shape != (3,) or not np.isfinite(q).all():
            raise PlanError('BAD_TARGET', name)
        if not (p.x_min_m <= q[0] <= p.x_max_m and p.y_min_m <= q[1] <= p.y_max_m):
            raise PlanError('OUT_OF_WORKSPACE', f'{name} xy=({q[0]:.4f}, {q[1]:.4f})')
    z_grasp = s[2] + p.grasp_above_stone_top_m
    z_release = c[2] + p.release_above_rim_m
    for name, z in (('grasp', z_grasp), ('release', z_release)):
        if not p.z_min_m <= z < p.travel_z_m:
            raise PlanError('BAD_HEIGHT', f'{name} z={z:.4f} (허용 {p.z_min_m:.3f}~{p.travel_z_m:.4f})')

    def move(name, x, y, z, vertical):
        return Step(name, 'move', (float(x), float(y), float(z)),
                    p.vertical_v_mps if vertical else p.travel_v_mps,
                    p.vertical_a_mps2 if vertical else p.travel_a_mps2)

    return [
        Step('open_before_pick', 'gripper', pulse=p.open_pulse, current_ma=p.grip_current_ma),
        move('over_stone', s[0], s[1], p.travel_z_m, False),
        move('descend_grasp', s[0], s[1], z_grasp, True),
        Step('close_grasp', 'gripper', pulse=p.close_pulse, current_ma=p.grip_current_ma, grasp=True),
        move('lift_stone', s[0], s[1], p.travel_z_m, True),
        move('over_cap', c[0], c[1], p.travel_z_m, False),
        move('descend_release', c[0], c[1], z_release, True),
        Step('open_release', 'gripper', pulse=p.open_pulse, current_ma=p.grip_current_ma),
        move('retreat', c[0], c[1], p.travel_z_m, True),
        move('park', p.park_x_m, p.park_y_m, p.park_z_m, False),        # z가 오르기만 하는 대각선
    ]


def target_points(stone, cap) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """pipeline Detection → (공깃돌 윗면, 테두리 윗면). 평면 높이는 vision HeightParams가 이미 반영했다
    (h_stone_offset = 공깃돌 높이, h_rim = 벨트면 → 테두리 윗면)."""
    return tuple(stone.position), tuple(cap.position)
