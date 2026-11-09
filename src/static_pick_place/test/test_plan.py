"""plan.py 단위 시험. 기대 높이는 2026-11-09 현장 성공 값(파지 플랜지 144.4 mm, 해제 244.4 mm)."""
from dataclasses import dataclass

import pytest

from static_pick_place.plan import (CLASS_CAP, CLASS_STONE, REGION_BELT, REGION_MAT, PickPlaceParams, PlanError,
                                    choose_cap, choose_stone, plan_pick_place)

P = PickPlaceParams()
STONE_TOP = (0.6795, -0.0161, -0.0176)      # 11-09 가운데 공깃돌 윗면
RIM_TOP = (0.6122, 0.0868, 0.0838)          # 11-09 재배치 뒤 뚜껑 테두리


@pytest.mark.level('U')
def test_plan_reproduces_site_heights_and_order():
    steps = plan_pick_place(STONE_TOP, RIM_TOP, P)
    assert [s.name for s in steps] == ['open_before_pick', 'over_stone', 'descend_grasp', 'close_grasp',
                                       'lift_stone', 'over_cap', 'descend_release', 'open_release', 'retreat',
                                       'park']
    by = {s.name: s for s in steps}
    assert by['descend_grasp'].pos_m == pytest.approx((0.6795, -0.0161, 0.1444), abs=1e-4)
    # 투입 성공 높이 244.4 mm에서 사용자 지시로 5 mm 올림
    assert by['descend_release'].pos_m == pytest.approx((0.6122, 0.0868, 0.2494), abs=1e-4)
    assert by['park'].pos_m == (P.park_x_m, P.park_y_m, P.park_z_m)
    assert by['over_stone'].pos_m[2] == by['over_cap'].pos_m[2] == P.travel_z_m
    assert by['close_grasp'].grasp and by['close_grasp'].pulse == 750 and by['close_grasp'].current_ma == 150
    assert by['open_before_pick'].pulse == by['open_release'].pulse == 600
    # 수직 구간만 느린 속도
    assert {s.v_mps for s in steps if s.name.startswith(('descend', 'lift', 'retreat'))} == {P.vertical_v_mps}
    assert {s.v_mps for s in steps if s.name.startswith(('over', 'park'))} == {P.travel_v_mps}


@pytest.mark.level('U')
@pytest.mark.parametrize('stone, cap, reason', [
    ((0.808, -0.0817, -0.0176), RIM_TOP, 'OUT_OF_WORKSPACE'),     # 11-09 벨트 원래 위치: 도달 경계
    (STONE_TOP, (0.808, -0.0817, 0.080), 'OUT_OF_WORKSPACE'),
    ((0.68, 0.0, 0.20), RIM_TOP, 'BAD_HEIGHT'),                  # 파지 높이가 이동 높이 이상
    ((0.68, 0.0, -0.10), RIM_TOP, 'BAD_HEIGHT'),                 # 너무 낮음(z_min 아래)
    ((float('nan'), 0.0, 0.0), RIM_TOP, 'BAD_TARGET'),
])
def test_plan_rejects(stone, cap, reason):
    with pytest.raises(PlanError) as e:
        plan_pick_place(stone, cap, P)
    assert e.value.reason == reason


@dataclass
class _Blob:
    occluded_partial: bool = False


@dataclass
class _Det:
    region: int
    object_class: int
    position: tuple
    position_valid: bool = True
    stone_inside: bool = False
    stone_inside_valid: bool = True
    blob: _Blob = None

    def __post_init__(self):
        self.blob = self.blob or _Blob()


@pytest.mark.level('U')
def test_choose_stone_nearest_valid_on_mat():
    dets = [_Det(REGION_MAT, CLASS_STONE, (0.70, 0.05, -0.02)),
            _Det(REGION_MAT, CLASS_STONE, (0.63, 0.00, -0.02)),
            _Det(REGION_MAT, CLASS_STONE, (0.50, 0.00, -0.02), blob=_Blob(True)),        # 가림
            _Det(REGION_MAT, CLASS_STONE, (0.45, 0.00, -0.02), position_valid=False),
            _Det(REGION_BELT, CLASS_STONE, (0.40, 0.00, 0.08)),                         # 벨트 위 돌
            _Det(REGION_MAT, CLASS_STONE, (0.90, 0.00, -0.02))]                         # 작업 공간 밖
    assert choose_stone(dets, P).position == (0.63, 0.00, -0.02)
    assert choose_stone([], P) is None


@pytest.mark.level('U')
def test_choose_cap_only_empty_and_judgeable():
    full = _Det(REGION_BELT, CLASS_CAP, (0.55, 0.0, 0.08), stone_inside=True)
    unknown = _Det(REGION_BELT, CLASS_CAP, (0.56, 0.0, 0.08), stone_inside_valid=False)
    empty = _Det(REGION_BELT, CLASS_CAP, (0.61, 0.08, 0.08))
    assert choose_cap([full, unknown], P) is None
    assert choose_cap([full, unknown, empty], P) is empty
