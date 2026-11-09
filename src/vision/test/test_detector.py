"""detector.py 합성 영상 시험(TEST_SPEC 3.5절 TC-VIS-001~003, 007)."""
import numpy as np
import pytest

from synth_images import (BELT_BGR, CAP_INNER_RATIO, CAP_OUTER_PX, H, MAT_BGR, STONE_BGR, STONE_FLAT_PX, W,
                          Canvas, octagon_eq_diameter)
from vision.detector import (CLASS_CAP, CLASS_STONE, REGION_BELT, REGION_MAT, Blob, ColorModel, InsideParams,
                             Roi, detect, stone_inside)

FULL = ((0.0, 0.0), (W - 1.0, 0.0), (W - 1.0, H - 1.0), (0.0, H - 1.0))
MAT = Roi(REGION_MAT, FULL)
BELT = Roi(REGION_BELT, FULL)
# 합성 배율(0.55 mm/px)에 맞춘 크기 범위. 실영상 기본값은 detector.default_classes()
STONE = ColorModel(diameter_min_px=20.0, diameter_max_px=45.0, circularity_min=0.70)
CAP = ColorModel(diameter_min_px=45.0, diameter_max_px=75.0, circularity_min=0.75,
                 hsv_lo=(90, 120, 70), hsv_hi=(118, 255, 255))
CLASSES = {CLASS_STONE: STONE, CLASS_CAP: CAP}
INSIDE = InsideParams(inner_diameter_ratio=CAP_INNER_RATIO)


def _random_layout(rng, n, min_gap, margin):
    pts = []
    while len(pts) < n:
        p = rng.uniform([margin, margin], [W - margin, H - margin])
        if all(np.hypot(*(p - q)) >= min_gap for q in pts):
            pts.append(p)
    return np.array(pts)


def _match(dets, truth, cls):
    err = []
    found = [d for d in dets if d.object_class == cls]
    for t in truth:
        d = min(found, key=lambda b: np.hypot(b.u - t[0], b.v - t[1]))
        err.append(np.hypot(d.u - t[0], d.v - t[1]))
    return np.array(err), found


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-001')
@pytest.mark.tc('TC-VIS-001-01')
def test_stone_center_accuracy_on_mat():
    errs, derr = [], []
    colors = list(STONE_BGR.values())
    for seed in range(20):
        rng = np.random.default_rng(seed)
        truth = _random_layout(rng, 5, 60.0, 30.0)
        c = Canvas(MAT_BGR)
        for k, (u, v) in enumerate(truth):
            c.octagon(u, v, STONE_FLAT_PX, colors[k % len(colors)], rot_rad=rng.uniform(0, np.pi / 4))
        dets = detect(c.image(rng=rng), MAT, {CLASS_STONE: STONE})
        e, found = _match(dets, truth, CLASS_STONE)
        assert len(found) == len(truth)
        errs.extend(e)
        derr.extend(abs(d.diameter_px - octagon_eq_diameter(STONE_FLAT_PX)) for d in found)
    errs = np.array(errs)
    assert len(errs) == 100
    print(f'stone center err p95={np.percentile(errs, 95):.3f} px max={errs.max():.3f}, '
          f'diameter err max={max(derr):.3f} px')
    assert np.percentile(errs, 95) <= 0.2
    assert max(derr) <= 1.0


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-001')
@pytest.mark.tc('TC-VIS-001-02')
def test_cap_center_accuracy_on_belt():
    errs = []
    for seed in range(20):
        rng = np.random.default_rng(100 + seed)
        truth = _random_layout(rng, 5, 90.0, 40.0)
        c = Canvas(BELT_BGR)
        for u, v in truth:
            c.cap(u, v)
        dets = detect(c.image(rng=rng), BELT, CLASSES)
        e, found = _match(dets, truth, CLASS_CAP)
        assert len(found) == len(truth)
        assert not [d for d in dets if d.object_class == CLASS_STONE]   # 빈 뚜껑 안쪽 바닥을 공깃돌로 보지 않는다
        errs.extend(e)
    errs = np.array(errs)
    print(f'cap center err p95={np.percentile(errs, 95):.3f} px max={errs.max():.3f}')
    assert np.percentile(errs, 95) <= 0.2


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-002')
@pytest.mark.tc('TC-VIS-002-01')
def test_mixed_scene_classification():
    wrong, total = 0, 0
    colors = list(STONE_BGR.items())
    for seed in range(200):
        rng = np.random.default_rng(1000 + seed)
        pts = _random_layout(rng, 5, 80.0, 45.0)
        c = Canvas(BELT_BGR)
        kinds = rng.permutation(['stone', 'stone', 'cap', 'cap', 'marker'])
        for kind, (u, v) in zip(kinds, pts):
            if kind == 'stone':
                name, bgr = colors[rng.integers(len(colors))]
                c.octagon(u, v, STONE_FLAT_PX, bgr, rot_rad=rng.uniform(0, np.pi / 4))
            elif kind == 'cap':
                c.cap(u, v)
            else:                       # 흰 사각 표식(ArUco 테두리 크기)은 어느 class도 아니다
                c.square(u, v, 50.0, (250, 250, 250))
        dets = detect(c.image(rng=rng), BELT, CLASSES)
        for kind, (u, v) in zip(kinds, pts):
            near = [d for d in dets if np.hypot(d.u - u, d.v - v) < 10.0]
            expect = {'stone': [CLASS_STONE], 'cap': [CLASS_CAP], 'marker': []}[kind]
            total += 1
            wrong += sorted(d.object_class for d in near) != expect
        wrong += len(dets) - sum(1 for d in dets if min(np.hypot(d.u - pts[:, 0], d.v - pts[:, 1])) < 10.0)
    assert total == 1000
    assert wrong == 0


def _cap_scene(stone_offset_px=None, occluder=None):
    c = Canvas(BELT_BGR)
    cu, cv = 320.3, 200.6
    c.cap(cu, cv)
    if stone_offset_px is not None:
        c.octagon(cu + stone_offset_px, cv, STONE_FLAT_PX, STONE_BGR['yellow'])
    robot = None
    if occluder == 'half':
        robot = np.zeros((H, W), np.uint8)
        robot[:, int(cu):] = 1
    return c.image(), robot


_R_IN = 0.5 * CAP_OUTER_PX * CAP_INNER_RATIO
_R_OUT = 0.5 * CAP_OUTER_PX
_R_S = 0.5 * STONE_FLAT_PX


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-003')
@pytest.mark.tc('TC-VIS-003-01')
@pytest.mark.parametrize('case, offset, occluder, expect', [
    ('center', 0.0, None, True),
    ('inner_rim_touch', _R_IN - _R_S, None, True),
    ('outer_rim_touch', _R_OUT + _R_S + 1.0, None, False),
    ('empty', None, None, False),
    ('gripper_half_mask', 0.0, 'half', None),
])
def test_stone_inside(case, offset, occluder, expect):
    img, robot = _cap_scene(offset, occluder)
    dets = detect(img, BELT, CLASSES, robot_mask=robot)
    caps = [d for d in dets if d.object_class == CLASS_CAP]
    assert len(caps) == 1, case
    stones = [d for d in dets if d.object_class == CLASS_STONE]
    if offset is not None and occluder is None:
        assert len(stones) == 1, case
    assert stone_inside(caps[0], stones, INSIDE) is expect


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-007')
@pytest.mark.tc('TC-VIS-007-01')
def test_robot_mask_overlap_marks_occluded_and_is_not_counted():
    c = Canvas(MAT_BGR)
    c.octagon(200.0, 200.0, STONE_FLAT_PX, STONE_BGR['red'])
    c.octagon(400.0, 200.0, STONE_FLAT_PX, STONE_BGR['red'])
    robot = np.zeros((H, W), np.uint8)
    robot[150:250, 410:520] = 1           # 오른쪽 돌의 오른쪽 절반을 덮는다
    dets = detect(c.image(), MAT, {CLASS_STONE: STONE}, robot_mask=robot)
    by_u = sorted(dets, key=lambda d: d.u)
    assert [d.occluded_partial for d in by_u] == [False, True]
    assert sum(1 for d in dets if not d.occluded_partial) == 1


@pytest.mark.level('U')
def test_degenerate_images_return_empty():
    black = np.zeros((H, W, 3), np.uint8)
    white = np.full((H, W, 3), 255, np.uint8)
    assert detect(black, MAT, CLASSES) == []
    assert detect(white, MAT, CLASSES) == []
    assert detect(black, Roi(REGION_MAT, ((5.0, 5.0), (6.0, 5.0), (6.0, 6.0))), CLASSES) == []


@pytest.mark.level('U')
def test_occluded_cap_has_no_inside_verdict():
    cap = Blob(CLASS_CAP, REGION_BELT, 10.0, 10.0, 40.0, 1256.0, 1.0, 1.0, (0.25, 0, 0, 0.25),
               occluded_partial=True)
    assert stone_inside(cap, [], INSIDE) is None
