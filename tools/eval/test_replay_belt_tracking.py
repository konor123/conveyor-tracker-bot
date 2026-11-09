"""replay_belt_tracking.py 단위 시험: 합성 검출 CSV(알려진 방향·속도·주기)로 축·통과·주기·예측 오차를 확인한다."""
import csv
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_belt_tracking as rbt  # noqa: E402

FIELDS = ['frame', 'stamp_ns', 'u_px', 'v_px', 'diameter_px', 'x_m', 'y_m', 'z_m',
          'cov_xx_m2', 'cov_xy_m2', 'cov_yy_m2', 'position_valid', 'stone_inside', 'stone_inside_valid']


def synth_csv(path: Path, *, speed=0.03, lap=20.0, visible=0.26, laps=3, sigma=0.0003,
              direction=(0.0, 1.0), seed=0):
    """뚜껑 1개가 lap마다 같은 구간(visible m)을 지난다. robot_base 방향 direction."""
    rng = np.random.default_rng(seed)
    d = np.array(direction, float) / np.linalg.norm(direction)
    start = np.array([0.80, -0.13])
    rows = []
    k = 0
    for n in range(laps):
        for i in range(int(visible / speed * 30)):
            t = n * lap + i / 30
            p = start + d * speed * (i / 30) + rng.normal(0, sigma, 2)
            rows.append([k, int(1_000_000_000_000 + t * 1e9), 0, 0, 40, p[0], p[1], 0.08,
                         sigma ** 2, 0.0, sigma ** 2, 1, 0, 1])
            k += 1
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(FIELDS)
        w.writerows(rows)
    return path


@pytest.mark.req('REQ-TRK-013')
@pytest.mark.level('U')
@pytest.mark.parametrize('direction', [(0.0, 1.0), (0.0, -1.0), (1.0, 0.2)])
def test_axis_follows_motion_direction(tmp_path, direction):
    fr = rbt.load(synth_csv(tmp_path / 'a.caps.csv', direction=direction))
    _, ex, _ = rbt.conveyor_axes(fr)
    d = np.array(direction) / np.linalg.norm(direction)
    assert ex @ d == pytest.approx(1.0, abs=1e-3)


@pytest.mark.req('REQ-TRK-013')
@pytest.mark.level('U')
def test_passes_lap_and_prediction(tmp_path):
    r = rbt.analyse(synth_csv(tmp_path / 'b.caps.csv'), {})
    assert len(r['passes']) == 3 and r['tracks_total'] == 3
    assert r['lap_s'] == pytest.approx([20.0, 20.0], abs=0.02)
    for p in r['passes']:
        assert p['speed_fit_mps'] == pytest.approx(0.03, abs=2e-4)
    assert r['speed_change_alarms_s'] == []
    # 등속 합성: 예측 오차는 측정 잡음 수준(σ 0.3 mm, 2D p95 약 0.75 mm)
    e = r['prediction_error']['1']
    assert e['n'] > 100 and e['xy_p95_mm'] < 1.2
