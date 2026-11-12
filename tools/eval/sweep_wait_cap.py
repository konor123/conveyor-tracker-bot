#!/usr/bin/env python3
"""tracker 설정 후보를 실제 벨트 영상과 합성 궤적으로 비교한다(WAIT_CAP 기준 묶음, SPEC-300 3절).

  python3 tools/eval/sweep_wait_cap.py --json /tmp/sweep.json --md /tmp/sweep.md
  python3 tools/eval/sweep_wait_cap.py --sigma 0.01 --kh 2:16 --json /tmp/one.json     # 후보 하나

후보 (sigma_accel, cusum_k, cusum_h)마다 다음을 낸다. numpy만 쓴다.
  실영상(검출 CSV, 조건마다 다이얼 고정 → 경보는 모두 맥동에 의한 것):
    관측 시간 60 s당 속도 변화 경보 수, WAIT_CAP 조건(steady, speed_std)을 지난 상태 비율(recover_s별),
    조건을 한 번이라도 지난 통과 수, 예측 오차 p95, 조건을 지난 상태의 3 s 앞 보고 표준편차
  합성(등속 20 mm/s, 측정 잡음 0.3 mm): 10·15·20·30 mm/s 계단을 1 s 안에 검출한 seed 수, 60 s 등속 오검출,
    위치·속도 오차 p95(REQ-TRK-001)
speed_std 기준은 후보마다 그 설정의 중앙값 × 1.15로 둔다(설정이 바뀌면 필터가 수렴하는 값도 바뀐다).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in ('tools/eval', 'src/tracking', 'src/intercept_planner', 'src/tracking/test'):
    sys.path.insert(0, str(ROOT / _p))

import replay_belt_tracking as rbt  # noqa: E402
from fixtures import TrajectoryFixture  # noqa: E402
from intercept_planner.prediction import TrackState, predict  # noqa: E402
from tracking.track_manager import OBSERVED, TrackManager  # noqa: E402

OFF1 = sorted((ROOT / 'records/bench/belt-video-2026-11-07').glob('*.caps.csv'))
RECOVER_S = (2.0, 1.0, 0.5)
SPEED_STD_MARGIN = 1.15
HORIZONS_S = (0.1, 1.0)
STEPS_MMPS = (10, 15, 20, 30)


def _pct(a, q):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if len(a) else None


def real(path: Path, params: dict) -> dict:
    """검출 CSV 하나를 후보 설정으로 재생한 지표."""
    fr = rbt.load(path)
    origin, ex, ey = rbt.conveyor_axes(fr)
    states, alarms, belt = rbt.run(fr, origin, ex, ey, params)
    cd = rbt.conv_dets(fr, origin, ex, ey)
    by: dict[int, list] = {}
    for s in states:
        if s.status == OBSERVED:
            by.setdefault(s.track_id, []).append(s)
    passes = {tid: ss for tid, ss in by.items() if len(ss) >= rbt.MIN_OBS}
    obs_time = sum(ss[-1].t_state - ss[0].t_state for ss in passes.values())
    elig = [s for ss in passes.values() for s in ss if s.observation_count >= rbt.MIN_OBS]
    t = np.array([s.t_state for s in elig])
    tid = np.array([s.track_id for s in elig])
    sp = np.array([belt.get(float(s.t_state), (False, np.inf))[1] for s in elig])
    sp50 = float(np.median(sp[np.isfinite(sp)])) if np.isfinite(sp).any() else float('nan')
    limit = SPEED_STD_MARGIN * sp50
    errs = {h: [] for h in HORIZONS_S}
    std3 = []
    for s in elig:
        ts = TrackState(s.track_id, s.epoch, s.status, s.t_state, s.position[:2], s.velocity[:2],
                        s.state_cov, s.valid_until, s.last_observed)
        std3.append(float(np.sqrt(max(0., np.linalg.eigvalsh(predict(ts, s.t_state + 3.0)[1])[-1]))))
        for h in HORIZONS_S:
            i = np.searchsorted(fr.t, s.t_state + h)
            cand = [j for j in (i - 1, i) if 0 <= j < len(fr.t) and abs(fr.t[j] - s.t_state - h) <= rbt.MATCH_DT_S]
            err = np.nan
            if cand:
                j = min(cand, key=lambda j: abs(fr.t[j] - s.t_state - h))
                pos, _ = predict(ts, float(fr.t[j]))
                near = [np.linalg.norm(z - pos) for z in cd[fr.t[j]] if np.linalg.norm(z - pos) <= rbt.MATCH_RADIUS_M]
                err = min(near) if near else np.nan
            errs[h].append(err)
    std3 = np.array(std3)
    al = np.array(sorted(alarms))
    out = dict(file=path.name, passes=len(passes), eligible=len(elig), alarms=len(alarms),
               alarms_per_60s=60.0 * len(alarms) / obs_time if obs_time else None,
               speed_std_p50_mmps=sp50 * 1e3, speed_std_limit_mmps=limit * 1e3,
               err_p95_mm={f'{h:g}': (_pct(errs[h], 95) or 0.0) * 1e3 if _pct(errs[h], 95) is not None else None
                           for h in HORIZONS_S})
    for r in RECOVER_S:
        steady = (np.array([not np.any((al <= x) & (x < al + r)) for x in t]) if len(al) else np.ones(len(t), bool))
        ok = steady & (sp <= limit)
        out[f'recover_{r:g}'] = dict(
            steady_ratio=float(steady.mean()) if len(t) else None, ok_ratio=float(ok.mean()) if len(t) else None,
            passes_any_ok=int(sum(1 for k in passes if ok[tid == k].any())),
            std3_ok_p50_mm=(_pct(std3[ok], 50) or 0.0) * 1e3 if ok.any() else None)
    return out


def synth(params: dict, seeds: int = 20) -> dict:
    """합성 궤적 지표: 계단 검출, 등속 오검출, 정확도."""
    def manager():
        return TrackManager(x_obs_min_m=-10., x_obs_max_m=10., **params)
    out = {}
    for step in STEPS_MMPS:
        delays = []
        for seed in range(seeds):
            m, first = manager(), None
            for f in TrajectoryFixture(seed, duration=13., step=(10., .02 + step * 1e-3)):
                m.process_frame(f.t, f.dets)
                stamp = m.detector.last_change_stamp
                if first is None and stamp is not None and stamp >= 10.:
                    first = stamp - 10.
            delays.append(first)
        hit = [d for d in delays if d is not None and d <= 1.0]
        out[f'step{step}'] = dict(detected_1s=len(hit), of=seeds, delay_max_s=max(hit) if hit else None)
    false, pos, vel = 0, [], []
    for seed in range(seeds):
        m, e = manager(), []
        for f in TrajectoryFixture(seed=seed):
            tr = m.process_frame(f.t, f.dets)
            if tr and tr[0].observation_count >= rbt.MIN_OBS:
                e.append(np.r_[tr[0].position[:2], tr[0].velocity[:2]] - f.truth)
        false += m.detector.last_change_stamp is not None
        e = np.array(e)
        pos.append(np.percentile(np.linalg.norm(e[:, :2], axis=1), 95))
        vel.append(np.percentile(np.linalg.norm(e[:, 2:], axis=1), 95))
    out.update(false_alarm_seeds=int(false), pos_p95_max_mm=max(pos) * 1e3, vel_p95_max_mmps=max(vel) * 1e3)
    return out


def evaluate(sigma: float, k: float, h: float, files, seeds: int = 20) -> dict:
    params = dict(sigma_accel_mps2=sigma, cusum_k=k, cusum_h=h)
    return dict(sigma_accel=sigma, cusum_k=k, cusum_h=h, real=[real(p, params) for p in files],
                synth=synth(params, seeds))


def markdown(rows) -> str:
    """후보마다 한 줄. 실영상 값은 파일들의 최소~최대(또는 합)다."""
    def span(vals, nd):
        vals = [v for v in vals if v is not None]
        if not vals:
            return '—'
        lo, hi = min(vals), max(vals)
        return f'{lo:.{nd}f}' if round(lo, nd) == round(hi, nd) else f'{lo:.{nd}f}~{hi:.{nd}f}'
    head = ['sigma_accel', 'k', 'h'] + [f'계단 {s} mm/s' for s in STEPS_MMPS] + [
        '등속 오검출', '위치 p95 [mm]', '속도 p95 [mm/s]', '경보/60 s', 'speed_std p50 [mm/s]']
    for r in RECOVER_S:
        head += [f'통과 비율(recover {r:g} s)', f'통과(recover {r:g} s)']
    head += ['오차 0.1 s p95 [mm]', '오차 1 s p95 [mm]', '3 s 앞 std [mm]']
    lines = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    for row in sorted(rows, key=lambda r: (r['sigma_accel'], r['cusum_k'], r['cusum_h'])):
        sy, re_ = row['synth'], row['real']
        cells = [f"{row['sigma_accel']:g}", f"{row['cusum_k']:g}", '끔' if row['cusum_h'] >= 1e8 else f"{row['cusum_h']:g}"]
        cells += [f"{sy[f'step{s}']['detected_1s']}/{sy[f'step{s}']['of']}" for s in STEPS_MMPS]
        cells += [str(sy['false_alarm_seeds']), f"{sy['pos_p95_max_mm']:.2f}", f"{sy['vel_p95_max_mmps']:.2f}",
                  span([f['alarms_per_60s'] for f in re_], 0), span([f['speed_std_p50_mmps'] for f in re_], 2)]
        for r in RECOVER_S:
            key = f'recover_{r:g}'
            cells += [span([f[key]['ok_ratio'] for f in re_], 2),
                      f"{sum(f[key]['passes_any_ok'] for f in re_)}/{sum(f['passes'] for f in re_)}"]
        cells += [span([f['err_p95_mm']['0.1'] for f in re_], 1), span([f['err_p95_mm']['1'] for f in re_], 1),
                  span([f[f'recover_{RECOVER_S[-1]:g}']['std3_ok_p50_mm'] for f in re_], 2)]
        lines.append('| ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', nargs='*', type=Path, default=OFF1, help='검출 CSV(기본: OFF-1 벨트 영상 5조건)')
    ap.add_argument('--sigma', default='0.01,0.02,0.03,0.05,0.1', help='sigma_accel_mps2 후보, 쉼표로 구분')
    ap.add_argument('--kh', default='1:8,1:16,2:8,2:16,2:20,3:16,1:1e9', help='cusum k:h 후보(h 1e9 = 검출 끔)')
    ap.add_argument('--seeds', type=int, default=20, help='합성 궤적 seed 수')
    ap.add_argument('--part', default='0/1', help='i/n: 후보를 n등분한 i번째만 계산(병렬 실행용)')
    ap.add_argument('--json', type=Path, required=True)
    ap.add_argument('--md', type=Path, help='후보마다 한 줄인 markdown 표를 쓴다')
    ap.add_argument('--merge', nargs='*', type=Path, default=[], help='이미 계산한 JSON을 합쳐 표만 만든다')
    a = ap.parse_args(argv)
    rows = [r for p in a.merge for r in json.loads(p.read_text())]
    if not a.merge:
        grid = [(float(s), float(k), float(h)) for s in a.sigma.split(',')
                for k, h in (kh.split(':') for kh in a.kh.split(','))]
        i, n = (int(v) for v in a.part.split('/'))
        for idx, (s, k, h) in enumerate(grid):
            if idx % n != i:
                continue
            t0 = time.time()
            rows.append(evaluate(s, k, h, a.csv, a.seeds))
            print(f'{idx + 1}/{len(grid)} sigma={s:g} k={k:g} h={h:g} {time.time() - t0:.0f} s', flush=True)
            a.json.write_text(json.dumps(rows, indent=1))
    a.json.write_text(json.dumps(rows, indent=1))
    if a.md:
        a.md.write_text(markdown(rows))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
