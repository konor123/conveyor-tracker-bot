"""pipeline.py·stable_window.py 단위 시험."""
import numpy as np
import pytest

from synth_images import BELT_BGR, CAP_INNER_RATIO, MAT_BGR, STONE_BGR, STONE_FLAT_PX, H, W, Canvas
from vision.detector import (CLASS_CAP, CLASS_STONE, REGION_BELT, REGION_MAT, Blob, ColorModel, InsideParams,
                             Roi)
from vision.localizer import CameraModel, HeightParams, Plane, pixel_to_plane, select_plane
from vision.pipeline import Detection, Geometry, SceneConfig, process_frame
from vision.stable_window import FAILED, OK, DetectionWindow, WindowParams

STONE = ColorModel(diameter_min_px=20.0, diameter_max_px=45.0)
CAP = ColorModel(diameter_min_px=45.0, diameter_max_px=75.0, circularity_min=0.75,
                 hsv_lo=(90, 120, 70), hsv_hi=(118, 255, 255))


def _two_region_image():
    img = Canvas(MAT_BGR)
    img.big[H * 8 // 2:] = BELT_BGR                    # 위 절반 매트, 아래 절반 벨트
    img.octagon(150.0, 100.0, STONE_FLAT_PX, STONE_BGR['red'])
    img.cap(400.0, 300.0)
    img.octagon(402.0, 299.0, STONE_FLAT_PX, STONE_BGR['yellow'])   # 뚜껑 안
    img.octagon(250.0, 300.0, STONE_FLAT_PX, STONE_BGR['pink'])     # 뚜껑 밖 벨트 위
    return img.image()


def _cfg():
    mat = Roi(REGION_MAT, ((0.0, 0.0), (W - 1.0, 0.0), (W - 1.0, H / 2 - 1), (0.0, H / 2 - 1)))
    belt = Roi(REGION_BELT, ((0.0, H / 2), (W - 1.0, H / 2), (W - 1.0, H - 1.0), (0.0, H - 1.0)))
    return SceneConfig(mat_roi=mat, belt_roi=belt, classes={CLASS_STONE: STONE, CLASS_CAP: CAP},
                       inside=InsideParams(CAP_INNER_RATIO))


def _geom():
    K = np.array([[600.0, 0, W / 2], [0, 600.0, H / 2], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])            # 아래를 보는 카메라
    T[:3, 3] = [0.4, 0.0, 0.45]
    return Geometry('test', CameraModel(K, np.zeros(5)), T, Plane(np.array([0, 0, 1.0]), 0.07),
                    Plane(np.array([0, 0, 1.0]), 0.05), HeightParams(0.004, 0.006, 0.0015))


@pytest.mark.level('U')
def test_process_frame_classes_regions_and_inside():
    dets = process_frame(_two_region_image(), _cfg())
    got = sorted((d.region, d.object_class, round(d.blob.u), d.inside_cap) for d in dets)
    assert got == [(REGION_MAT, CLASS_STONE, 150, False), (REGION_BELT, CLASS_STONE, 250, False),
                   (REGION_BELT, CLASS_STONE, 402, True), (REGION_BELT, CLASS_CAP, 400, False)]
    cap = next(d for d in dets if d.object_class == CLASS_CAP)
    assert cap.stone_inside and cap.stone_inside_valid
    assert not any(d.position_valid for d in dets)      # 보정 없으면 위치 없음


@pytest.mark.level('U')
def test_process_frame_uses_class_planes():
    g = _geom()
    dets = process_frame(_two_region_image(), _cfg(), g)
    z = {(d.region, d.object_class, d.inside_cap): d.position[2] for d in dets}
    assert all(d.position_valid for d in dets)
    assert z[(REGION_MAT, CLASS_STONE, False)] == pytest.approx(0.07 + 0.004)
    assert z[(REGION_BELT, CLASS_CAP, False)] == pytest.approx(0.05 + 0.006)
    assert z[(REGION_BELT, CLASS_STONE, True)] == pytest.approx(0.05 + 0.0015 + 0.004)
    on_belt = [d for d in dets if d.region == REGION_BELT and d.object_class == CLASS_STONE and not d.inside_cap]
    assert on_belt[0].position[2] == pytest.approx(0.05 + 0.004)
    d = next(d for d in dets if d.region == REGION_MAT)
    p, _ = pixel_to_plane(d.blob.u, d.blob.v, g.cam, g.T_base_cam,
                          select_plane(CLASS_STONE, g.mat, g.belt, g.heights))
    assert np.allclose(d.position, p)


def _det(x, y, occluded=False, valid=True):
    b = Blob(CLASS_STONE, REGION_MAT, 100.0 + x * 1000, 100.0 + y * 1000, 20.0, 314.0, 1.0, 1.0,
             (0.25, 0, 0, 0.25), occluded_partial=occluded)
    return Detection(blob=b, position=(x, y, 0.074), position_valid=valid)


def _fill(win, jitter_m, n=31, dt=1 / 30, seed=0, **kw):
    rng = np.random.default_rng(seed)
    for k in range(n):
        win.add(k * dt, [_det(0.30 + rng.normal(0, jitter_m), 0.10 + rng.normal(0, jitter_m), **kw),
                         _det(0.40, 0.10, **kw)])


@pytest.mark.level('U')
@pytest.mark.parametrize('jitter_m, expect_ok', [(0.0003, True), (0.0006, False)])
def test_window_jitter(jitter_m, expect_ok):
    win = DetectionWindow()
    _fill(win, jitter_m)
    r = win.evaluate(REGION_MAT, CLASS_STONE, 2, 1.0, WindowParams())
    assert (r.result == OK) is expect_ok
    if not expect_ok:
        assert r.result == FAILED and len(r.detections) == 1


@pytest.mark.level('U')
def test_window_not_filled_returns_none_and_backstep_resets():
    win = DetectionWindow()
    _fill(win, 0.0, n=10)
    assert win.evaluate(REGION_MAT, CLASS_STONE, 1, 1.0) is None
    _fill(win, 0.0, n=31)                               # stamp가 0으로 돌아가 창을 다시 채운다
    assert win.evaluate(REGION_MAT, CLASS_STONE, 2, 1.0).result == OK


@pytest.mark.level('U')
def test_window_ignores_occluded():
    win = DetectionWindow()
    _fill(win, 0.0, occluded=True)
    r = win.evaluate(REGION_MAT, CLASS_STONE, 1, 1.0)
    assert r.result == FAILED and r.detections == ()


@pytest.mark.level('U')
def test_window_pixel_fallback_without_calibration():
    win = DetectionWindow()
    _fill(win, 0.0, valid=False)
    r = win.evaluate(REGION_MAT, CLASS_STONE, 2, 1.0)
    assert r.result == OK and all(np.isnan(s) for s in r.position_std)
