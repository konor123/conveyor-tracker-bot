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
    # 등속 합성: 1 s 앞 오차 = 측정 잡음(σ 0.3 mm) + 속도 추정 오차(sigma_accel 0.01에서 p95 약 1.2 mm/s) × 1 s
    e = r['prediction_error']['1']
    assert e['n'] > 100 and e['xy_p95_mm'] < 2.0


@pytest.mark.req('REQ-TRK-013')
@pytest.mark.level('U')
def test_reported_std_is_opt_in_and_consistent(tmp_path):
    """보고 표준편차 통계는 요청할 때만 나오고(기본 출력은 그대로), 등속 합성에서는 실제 오차와 같은 크기다."""
    path = synth_csv(tmp_path / 'c.caps.csv')
    base = rbt.analyse(path, {})
    assert 'std_p95_mm' not in base['prediction_error']['1']
    r = rbt.analyse(path, {}, horizons=(1.0, 3.0), std_limit_m=0.00325)
    assert set(r['prediction_error']) == {'1', '3'}
    e1, e3 = r['prediction_error']['1'], r['prediction_error']['3']
    assert e1['xy_p95_mm'] == pytest.approx(base['prediction_error']['1']['xy_p95_mm'])
    assert 0.3 < e1['std_p50_mm'] < 3.0 and e3['std_p50_mm'] > e1['std_p50_mm']      # 예측 구간이 길수록 커진다
    assert 0.0 <= e3['std_le_limit_ratio'] <= e1['std_le_limit_ratio'] <= 1.0
    assert e1['norm_err_p95'] < 4.0            # 모델이 맞는 자료: 오차 / 보고 표준편차 p95가 2.45(χ²₂ 95%) 근처
    # WAIT_CAP 조건(steady, speed_std)을 지난 상태만의 통계. 등속 합성이라 steady는 전부 참이다
    w = r['wait_cap']
    assert 'wait_cap' not in base
    assert w['steady_ratio'] == 1.0 and 0.0 <= w['both_ok_ratio'] <= w['speed_std_ok_ratio'] <= 1.0
    assert e1['n_wait_ok'] <= e1['n']
    assert w['passes'] == 3 and 0 <= w['passes_any_ok'] <= 3
    if w['reported_std']:
        assert w['reported_std']['3']['std_p50_mm'] > w['reported_std']['1']['std_p50_mm']
    loose = rbt.analyse(path, {}, horizons=(1.0,), std_limit_m=0.00325, speed_std_limit_mps=1.0)
    assert loose['wait_cap']['both_ok_ratio'] == 1.0 and loose['prediction_error']['1']['n_wait_ok'] == e1['n']
    assert loose['wait_cap']['passes_any_ok'] == 3 and loose['wait_cap']['reported_std']['1']['le_limit_ratio'] == 1.0


OFF1 = sorted((Path(__file__).resolve().parents[2] / 'records/bench/belt-video-2026-11-07').glob('*.caps.csv'))


@pytest.mark.req('REQ-TRK-006')
@pytest.mark.tc('TC-TRK-006-03')
@pytest.mark.level('U')
@pytest.mark.parametrize('path', OFF1, ids=[p.name.split('analog-')[1].replace('.caps.csv', '') for p in OFF1])
def test_off1_belt_pulsation_does_not_block_wait_cap(path):
    """다이얼을 고정한 실제 벨트(맥동 있음)에서 기본 설정의 tracker가 WAIT_CAP 조건을 대부분 지난다."""
    w = rbt.analyse(path, {}, horizons=(3.0,), std_limit_m=0.00325, speed_std_limit_mps=0.001)['wait_cap']
    assert w['alarms_per_60s_observed'] <= 25.0
    assert w['both_ok_ratio'] >= 0.85
    assert w['passes_any_ok'] == w['passes'] >= 4
    assert w['first_ok_s_max'] <= 1.6              # 첫 관측부터 조건 통과까지(관측 30회 = 1 s 포함)
    assert w['reported_std']['3']['le_limit_ratio'] == 1.0     # 계획 시점(3 s 앞) 보고 표준편차 ≤ 3.25 mm
