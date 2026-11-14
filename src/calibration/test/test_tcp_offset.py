import numpy as np
import pytest

import synth
from calibration.errors import CalibrationError
from calibration.tcp_offset import fit_tcp_offset

SEEDS = range(20)


def _poses(rng, n, tilt_deg):
    R, t = [], []
    for _ in range(n):
        Ri = synth.rot([0.0, 0.0, 1.0], rng.uniform(-np.pi, np.pi))
        if tilt_deg:
            Ri = synth.rot(rng.normal(size=3), np.deg2rad(rng.uniform(-tilt_deg, tilt_deg))) @ Ri
        R.append(Ri @ synth.rot([1.0, 0.0, 0.0], np.pi))   # TCP z축이 아래를 본다
        t.append([rng.uniform(0.3, 0.6), rng.uniform(-0.2, 0.2), rng.uniform(0.10, 0.15)])
    return np.array(R), np.array(t)


@pytest.mark.parametrize("seed", SEEDS)
def test_tcp_offset_full_3d(seed):
    rng = np.random.default_rng(seed)
    o = np.array([0.0015, -0.0008, 0.012])
    R, t = _poses(rng, 30, tilt_deg=30.0)
    p = np.einsum('nij,j->ni', R, o) + t + rng.normal(0.0, 0.0002, size=(30, 3))
    est, rms = fit_tcp_offset(R, t, p)
    assert np.linalg.norm(est - o) <= 0.0002
    assert rms < 0.0005


@pytest.mark.parametrize("seed", SEEDS)
def test_tcp_offset_xy_only_vertical_approach(seed):
    """위에서 수직 접근(yaw만 변함) + 카메라 XY 관측: z 오프셋은 주어진 값으로 고정하고 XY만 푼다."""
    rng = np.random.default_rng(seed)
    o = np.array([0.0015, -0.0008, 0.012])
    R, t = _poses(rng, 30, tilt_deg=0.0)
    p = np.einsum('nij,j->ni', R, o) + t + rng.normal(0.0, 0.0002, size=(30, 3))
    p[:, 2] = np.nan                                     # z는 관측하지 않는다(평면 교차값)
    est, _ = fit_tcp_offset(R, t, p, fixed={2: 0.012})
    assert np.linalg.norm(est[:2] - o[:2]) <= 0.0002
    assert est[2] == 0.012
    with pytest.raises(CalibrationError) as e:          # z를 고정하지 않으면 풀 수 없다
        fit_tcp_offset(R, t, p)
    assert e.value.code == 'DEGENERATE'
