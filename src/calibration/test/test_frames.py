import numpy as np
import pytest

from calibration.frames import plane_frame


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


@pytest.mark.req('REQ-CAL-011')
@pytest.mark.tc('TC-CAL-011-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('normal, offset, x_hint, origin_xy', [
    ([0.0138, -0.0062, 0.9999], -0.02329, [1, 0, 0], [0.6, 0.0]),     # 11-09 매트 평면
    ([0.1, 0.2, 0.97], 0.07, [0, 1, 0], [0.4, -0.1]),                # 많이 기운 평면
    ([-0.0138, 0.0062, -0.9999], 0.02329, [1, 1, 0], [0.5, 0.2]),    # 아래쪽 법선(같은 평면)
])
def test_plane_frame(normal, offset, x_hint, origin_xy):
    T = plane_frame(normal, offset, x_hint, origin_xy)
    R, p = T[:3, :3], T[:3, 3]
    n = _unit(normal)
    d = offset / np.linalg.norm(normal)
    if n[2] < 0:                                       # 같은 평면을 위쪽 법선으로
        n, d = -n, -d
    assert abs(n @ p - d) <= 1e-12                    # 원점은 평면 위
    np.testing.assert_allclose(p[:2], origin_xy, atol=1e-12)   # origin_xy를 지나는 수직선 위
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_allclose(R[:, 2], n, atol=1e-12)
    x_proj = _unit(np.asarray(x_hint, float) - (np.asarray(x_hint, float) @ n) * n)
    # arccos는 1 근처에서 반올림에 약하다(1 − 1e-16 → 1.5e-8 rad). 각도는 atan2(|a×b|, a·b)로 잰다.
    angle = np.arctan2(np.linalg.norm(np.cross(R[:, 0], x_proj)), R[:, 0] @ x_proj)
    assert angle <= 1e-9
    np.testing.assert_array_equal(T[3], [0, 0, 0, 1])


@pytest.mark.req('REQ-CAL-011')
@pytest.mark.tc('TC-CAL-011-01')
@pytest.mark.level('U')
def test_plane_frame_rejects_hint_along_normal():
    with pytest.raises(ValueError):
        plane_frame([0, 0, 1], 0.0, [0, 0, 2], [0, 0])
