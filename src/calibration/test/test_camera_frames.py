import numpy as np
import pytest

import synth
from calibration.camera_intrinsics_check import compare_camera_info
from calibration.geometry import base_to_camera_link

PROFILE = {'width': 1280, 'height': 800, 'fx': 644.14013671875, 'fy': 643.2152709960938,
           'cx': 640.61962890625, 'cy': 405.6260681152344, 'distortion_model': 'plumb_bob',
           'distortion': synth.DIST.tolist()}


def _info(**kw):
    info = {'width': 1280, 'height': 800, 'distortion_model': 'plumb_bob',
            'k': synth.K.ravel().tolist(), 'd': synth.DIST.tolist()}
    info.update(kw)
    return info


def test_base_to_camera_link_composes_back():
    rng = np.random.default_rng(0)
    T_base_opt = synth.camera_pose()
    # RealSense: camera_link(x 앞, y 왼쪽, z 위) → 광학(x 오른쪽, y 아래, z 앞) + 컬러 렌즈 위치
    T_link_opt = np.eye(4)
    T_link_opt[:3, :3] = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    T_link_opt[:3, 3] = [0.0, 0.059, 0.0] + rng.normal(0, 1e-4, 3)
    T_base_link = base_to_camera_link(T_base_opt, T_link_opt)
    assert np.abs(T_base_link @ T_link_opt - T_base_opt).max() <= 1e-12
    assert np.linalg.det(T_base_link[:3, :3]) == pytest.approx(1.0, abs=1e-12)


def test_camera_info_matches_profile():
    assert compare_camera_info(_info(), PROFILE) == []
    k = synth.K.copy()
    k[0, 0] += 0.4                                    # 0.4 px 차이는 허용
    assert compare_camera_info(_info(k=k.ravel().tolist()), PROFILE) == []


@pytest.mark.parametrize("kw,word", [
    ({'width': 1280, 'height': 720}, 'height'),
    ({'distortion_model': 'rational_polynomial'}, 'distortion_model'),
    ({'k': (synth.K + np.array([[0, 0, 0.6], [0, 0, 0], [0, 0, 0]])).ravel().tolist()}, 'cx'),
    ({'d': (synth.DIST + [0, 0, 0, 0, 0.01]).tolist()}, 'distortion'),
    ({'d': [0.0] * 5}, 'distortion'),
])
def test_camera_info_mismatch(kw, word):
    out = compare_camera_info(_info(**kw), PROFILE)
    assert out and any(word in m for m in out)


def test_camera_info_focal_drift_can_be_ignored():
    k = synth.K.copy()
    k[0, 0] += 2.2                                    # 2026-11-07 실측 보고값 변동
    k[1, 1] += 2.2
    info = _info(k=k.ravel().tolist())
    assert {m.split(':')[0] for m in compare_camera_info(info, PROFILE)} == {'fx', 'fy'}
    assert compare_camera_info(info, PROFILE, check_focal=False) == []
