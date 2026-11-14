"""순수 기하 및 픽셀 불확실성(SI), 장비 연결 없음."""
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from fakes.camera import SyntheticCamera
from vision.localizer import CameraModel, Plane, HeightParams, pixel_to_plane, select_plane



def camera(tilted=False, distorted=True):
    fake = SyntheticCamera()
    if tilted:
        rotation, _ = cv2.Rodrigues(np.array([.12, -.08, .03]))
        fake.T_base_cam[:3, :3] = rotation @ fake.T_base_cam[:3, :3]
    if not distorted:
        fake.dist = np.zeros(5)
    return fake, CameraModel(K=fake.K, dist=fake.dist)


@pytest.mark.parametrize('seed', range(20))
@pytest.mark.parametrize('tilted', [False, True])
@pytest.mark.parametrize('distorted', [False, True])
def test_round_trip(seed, tilted, distorted):
    fake, model = camera(tilted, distorted)
    rng = np.random.default_rng(seed)
    plane = Plane(np.array([.04, -.03, 1.]), .05)
    xy = rng.uniform([.28, -.13], [.62, .17], size=(12, 2))
    points = np.column_stack([xy, [plane.height_at(x, y) for x, y in xy]])
    for point, uv in zip(points, fake.project(points)):
        actual, cov = pixel_to_plane(*uv, model, fake.T_base_cam, plane)
        assert np.linalg.norm(actual - point) <= 1e-6
        assert cov.shape == (3, 3)


def test_factory_defaults_match_shared_config():
    config = Path(__file__).resolve().parents[2] / 'calibration/config/d455_color_1280x800.yaml'
    values = yaml.safe_load(config.read_text())
    model = CameraModel()
    np.testing.assert_allclose(model.K, [[values['fx'], 0, values['cx']],
                                        [0, values['fy'], values['cy']], [0, 0, 1]], rtol=0, atol=0)
    np.testing.assert_allclose(model.dist, values['distortion'], rtol=0, atol=0)


@pytest.mark.parametrize('kind,inside,base,height', [
    ('STONE', False, 'mat', .004), ('CAP', False, 'belt', .006),
    ('STONE', True, 'belt', .0055), (1, False, 'mat', .004), (2, False, 'belt', .006)])
def test_object_planes(kind, inside, base, height):
    # calibration.Plane과 같은 속성만 가진 입력도 받는다.
    mat = SimpleNamespace(normal=np.array([.03, -.02, 1.]), offset=.07)
    belt = SimpleNamespace(normal=np.array([-.02, .01, 1.]), offset=.05)
    params = HeightParams(h_stone_offset=.004, h_rim=.006, floor_thickness=.0015)
    selected = select_plane(kind, mat, belt, params, inside_cap=inside)
    reference = {'mat': mat, 'belt': belt}[base]
    fake, model = camera(tilted=True)
    uv = (790., 290.)
    point, _ = pixel_to_plane(*uv, model, fake.T_base_cam, selected)
    expected_z = (reference.offset - reference.normal[:2] @ point[:2]) / reference.normal[2] + height
    assert point[2] == pytest.approx(expected_z, abs=1e-12)
    assert selected.height_at(*point[:2]) == pytest.approx(expected_z, abs=1e-12)


@pytest.mark.parametrize('distorted', [False, True])
@pytest.mark.parametrize('pixel_cov', [np.diag([.25, .25]), np.array([[.36, .09], [.09, .16]]), np.zeros((2, 2))])
def test_covariance_matches_independent_finite_difference(distorted, pixel_cov):
    fake, model = camera(tilted=True, distorted=distorted)
    plane = Plane(np.array([.08, -.06, 1.]), .05)
    uv = np.array([850., 250.])
    point, covariance = pixel_to_plane(*uv, model, fake.T_base_cam, plane, pixel_cov=pixel_cov)

    def reference(pixel):
        xy = cv2.undistortPoints(np.array(pixel).reshape(1, 1, 2), model.K, model.dist).ravel()
        direction = fake.T_base_cam[:3, :3] @ np.r_[xy, 1.]
        origin = fake.T_base_cam[:3, 3]
        return origin + direction * ((plane.offset - plane.normal @ origin) / (plane.normal @ direction))

    step = .05
    J = np.column_stack([(reference(uv + axis * step) - reference(uv - axis * step)) / (2 * step)
                         for axis in np.eye(2)])
    expected = J @ pixel_cov @ J.T
    np.testing.assert_allclose(point, reference(uv), atol=1e-12)
    np.testing.assert_allclose(covariance, expected, rtol=.05, atol=1e-18)
    np.testing.assert_allclose(covariance, covariance.T, atol=1e-20)
    assert np.linalg.eigvalsh(covariance).min() >= -1e-20
    np.testing.assert_allclose(plane.normal @ covariance, 0, atol=1e-18)


def test_default_covariance():
    fake, model = camera()
    args = (740., 320., model, fake.T_base_cam, Plane([0, 0, 1], .05))
    _, default = pixel_to_plane(*args)
    _, explicit = pixel_to_plane(*args, pixel_cov=np.diag([.25, .25]))
    np.testing.assert_array_equal(default, explicit)


@pytest.mark.parametrize('plane', [Plane([1, 0, 0], 1), Plane([0, 0, 1], .5), Plane([0, 0, 1], .4)])
def test_parallel_or_nonforward_ray_rejected(plane):
    fake, model = camera(distorted=False)
    with pytest.raises(ValueError):
        pixel_to_plane(model.K[0, 2], model.K[1, 2], model, fake.T_base_cam, plane)


@pytest.mark.parametrize('cov', [np.eye(3), [[1, 2], [0, 1]], [[-1, 0], [0, 1]], [[np.nan, 0], [0, 1]]])
def test_invalid_pixel_covariance_rejected(cov):
    fake, model = camera()
    with pytest.raises(ValueError):
        pixel_to_plane(750, 300, model, fake.T_base_cam, Plane([0, 0, 1], .05), pixel_cov=cov)


def test_unknown_class_and_cap_inside_context_rejected():
    plane = Plane([0, 0, 1], 0)
    p = HeightParams(.004, .006, .0015)
    with pytest.raises(ValueError):
        select_plane('UNKNOWN', plane, plane, p)
    with pytest.raises(ValueError):
        select_plane('CAP', plane, plane, p, inside_cap=True)
