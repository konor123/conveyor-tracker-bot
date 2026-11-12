#!/usr/bin/env python3
"""병뚜껑 검출 CSV(`extract_cap_detections.py`) → 실제 TrackManager·planner 예측 재생과 통계.

  python3 tools/eval/replay_belt_tracking.py records/bench/belt-video-2026-11-07/*.caps.csv \
      --json records/bench/belt-video-2026-11-07/replay-summary.json

numpy만 쓴다. conveyor_frame은 보정 전이므로 검출 XY(robot_base)의 주성분 방향을 x축(진행 방향,
시간에 따라 증가하도록 부호 결정), 검출 평균을 원점으로 둔 임시 좌표계다(SPEC-300 2절의 T_base_conveyor 대용).
독립 GT가 없으므로 예측 오차는 "나중에 실제로 검출된 위치" 대비이며 검출 잡음을 포함한다.

`--std-limit-mm 3.25`를 주면 planner가 보고하는 위치 표준편차(예측 공분산의 최대 고유값 제곱근,
SPEC-400 `position_std`)를 같은 표본에서 함께 집계한다: std_p50/p95, 기준 이하 비율, 오차 / 표준편차 p95.
`--horizons 1,3,4`로 예측 구간을 바꿀 수 있다. 두 인자를 주지 않으면 출력은 이전과 같다.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for pkg in ('tracking', 'intercept_planner'):
    sys.path.insert(0, str(ROOT / 'src' / pkg))

from tracking.track_manager import COASTING, OBSERVED, TrackManager  # noqa: E402
from intercept_planner.prediction import TrackState, predict  # noqa: E402

HORIZONS_S = (0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 2.8)
MIN_OBS = 30                     # WAIT_CAP observation_count 하한(SPEC-100)
MATCH_RADIUS_M = 0.04            # 같은 뚜껑 판정 반경(관측 구간에 뚜껑 1개, 다음 통과까지 수 초 간격)
MATCH_DT_S = 0.02


@dataclass
class Frames:
    t: np.ndarray                # 검출이 있는 프레임 시각 [s], 오름차순 고유값
    dets: dict                   # 시각 → [(xy_base, cov2)]
    all_t: np.ndarray            # 검출 행 시각
    xy: np.ndarray               # 검출 행 XY (robot_base)


def load(path: Path) -> Frames:
    rows = list(csv.DictReader(open(path)))
    rows = [r for r in rows if r['position_valid'] == '1']
    t0 = None
    dets: dict[float, list] = {}
    ts, xy = [], []
    for r in rows:
        ns = int(r['stamp_ns'])
        t0 = ns if t0 is None else min(t0, ns)
    for r in rows:
        t = (int(r['stamp_ns']) - t0) * 1e-9
        p = np.array([float(r['x_m']), float(r['y_m'])])
        c = np.array([[float(r['cov_xx_m2']), float(r['cov_xy_m2'])],
                      [float(r['cov_xy_m2']), float(r['cov_yy_m2'])]])
        dets.setdefault(t, []).append((p, c))
        ts.append(t)
        xy.append(p)
    return Frames(np.array(sorted(dets)), dets, np.array(ts), np.array(xy))


def conveyor_axes(fr: Frames) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(원점, x축, y축) in robot_base XY. x축은 주성분, 시간과 양의 상관이 되도록 부호를 맞춘다."""
    origin = fr.xy.mean(axis=0)
    d = fr.xy - origin
    _, _, vt = np.linalg.svd(d, full_matrices=False)
    ex = vt[0]
    # 연속한 검출 사이 이동이 +x가 되도록(같은 통과 안에서는 시간에 따라 증가)
    order = np.argsort(fr.all_t)
    step = np.diff(d[order] @ ex)
    dt = np.diff(fr.all_t[order])
    near = (dt > 0) & (dt < 0.2) & (np.abs(step) < 0.02)
    if np.median(step[near]) < 0:
        ex = -ex
    ey = np.array([-ex[1], ex[0]])
    return origin, ex, ey


def to_conv(p, c, origin, ex, ey):
    R = np.vstack([ex, ey])
    return R @ (p - origin), R @ c @ R.T


def run(fr: Frames, origin, ex, ey, params: dict):
    tm = TrackManager(x_obs_min_m=-10., x_obs_max_m=10., **params)
    states, alarms = [], []
    prev_change = None
    for t in fr.t:
        dets = []
        for p, c in fr.dets[t]:
            pc, cc = to_conv(p, c, origin, ex, ey)
            dets.append(SimpleNamespace(object_class=2, position_valid=True, position=pc, position_cov=cc))
        out = tm.process_frame(float(t), dets)
        if tm.detector.last_change_stamp != prev_change:
            prev_change = tm.detector.last_change_stamp
            alarms.append(float(prev_change))
        states.extend(out)
    return states, alarms


def conv_dets(fr: Frames, origin, ex, ey):
    return {t: [to_conv(p, c, origin, ex, ey)[0] for p, c in v] for t, v in fr.dets.items()}


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float('nan')


def analyse(path: Path, params: dict, horizons=HORIZONS_S, std_limit_m: float | None = None) -> dict:
    fr = load(path)
    origin, ex, ey = conveyor_axes(fr)
    states, alarms = run(fr, origin, ex, ey, params)
    cd = conv_dets(fr, origin, ex, ey)
    xy_c = (fr.xy - origin) @ np.vstack([ex, ey]).T

    by_track: dict[int, list] = {}
    for s in states:
        by_track.setdefault(s.track_id, []).append(s)
    passes = []
    for tid, ss in sorted(by_track.items()):
        obs = [s for s in ss if s.status == OBSERVED]
        if len(obs) < MIN_OBS:
            continue
        t_obs = np.array([s.t_state for s in obs])
        x_obs = np.array([s.position[0] for s in obs])
        y_obs = np.array([s.position[1] for s in obs])
        v_fit = np.polyfit(t_obs, x_obs, 1)[0]
        # 1 s 창 속도(검출 기반 선형 적합) → 맥동
        win = []
        for t_a in np.arange(t_obs[0], t_obs[-1] - 1.0, 0.5):
            m = (t_obs >= t_a) & (t_obs < t_a + 1.0)
            if m.sum() >= 20:
                win.append(np.polyfit(t_obs[m], x_obs[m], 1)[0])
        v_kf = np.array([s.velocity[0] for s in obs if s.observation_count >= MIN_OBS])
        coast = sum(1 for s in ss if s.status == COASTING)
        passes.append(dict(track_id=tid, t_start_s=float(t_obs[0]), t_end_s=float(t_obs[-1]),
                           n_observed=len(obs), n_coasting=coast,
                           x_start_m=float(x_obs[0]), x_end_m=float(x_obs[-1]),
                           speed_fit_mps=float(v_fit), y_std_m=float(np.std(y_obs)),
                           speed_1s_min_mps=float(min(win)) if win else None,
                           speed_1s_max_mps=float(max(win)) if win else None,
                           speed_kf_p05_mps=pct(v_kf, 5), speed_kf_p95_mps=pct(v_kf, 95)))
    # 같은 x(구간 중앙) 통과 시각 차 → 한 바퀴 주기
    x_ref = 0.0
    crossings = []
    for p in passes:
        ss = [s for s in by_track[p['track_id']] if s.status == OBSERVED]
        for a, b in zip(ss, ss[1:]):
            if a.position[0] < x_ref <= b.position[0]:
                f = (x_ref - a.position[0]) / (b.position[0] - a.position[0])
                crossings.append(a.t_state + f * (b.t_state - a.t_state))
                break
    laps = np.diff(crossings).tolist()

    # 예측 오차: 관측 30회 이상 OBSERVED 상태에서 planner predict → t+H 실제 검출(같은 뚜껑) 대비
    t_arr = fr.t
    errs = {h: [] for h in horizons}
    stds = {h: [] for h in horizons}
    for s in states:
        if s.status != OBSERVED or s.observation_count < MIN_OBS:
            continue
        ts = TrackState(s.track_id, s.epoch, s.status, s.t_state, s.position[:2], s.velocity[:2],
                        s.state_cov, s.valid_until, s.last_observed)
        for h in horizons:
            i = np.searchsorted(t_arr, s.t_state + h)
            cand = [j for j in (i - 1, i) if 0 <= j < len(t_arr) and abs(t_arr[j] - s.t_state - h) <= MATCH_DT_S]
            if not cand:
                continue
            j = min(cand, key=lambda j: abs(t_arr[j] - s.t_state - h))
            pos, cov = predict(ts, float(t_arr[j]))
            near = [z for z in cd[t_arr[j]] if np.linalg.norm(z - pos) <= MATCH_RADIUS_M]
            if near:
                z = min(near, key=lambda z: np.linalg.norm(z - pos))
                errs[h].append(z - pos)
                stds[h].append(float(np.sqrt(max(0., np.linalg.eigvalsh(cov)[-1]))))   # planner position_std
    pred = {}
    for h, e in errs.items():
        e = np.array(e)
        if len(e):
            pred[f'{h:g}'] = dict(n=len(e), along_p50_mm=pct(np.abs(e[:, 0]), 50) * 1e3,
                                  along_p95_mm=pct(np.abs(e[:, 0]), 95) * 1e3,
                                  cross_p95_mm=pct(np.abs(e[:, 1]), 95) * 1e3,
                                  xy_p95_mm=pct(np.linalg.norm(e, axis=1), 95) * 1e3,
                                  xy_max_mm=float(np.linalg.norm(e, axis=1).max()) * 1e3)
            if std_limit_m is not None:
                sd = np.array(stds[h])
                pred[f'{h:g}'].update(
                    std_p50_mm=pct(sd, 50) * 1e3, std_p95_mm=pct(sd, 95) * 1e3,
                    std_le_limit_ratio=float(np.mean(sd <= std_limit_m)),
                    norm_err_p95=pct(np.linalg.norm(e, axis=1) / sd, 95))
    return dict(file=path.name, frames_with_detection=len(fr.t), detections=len(fr.xy),
                span_s=float(fr.t[-1] - fr.t[0]), direction_base=ex.tolist(), origin_base_m=origin.tolist(),
                lateral_std_mm=float(np.std(xy_c[:, 1]) * 1e3),
                passes=passes, lap_s=laps, speed_change_alarms_s=alarms,
                tracks_total=len(by_track), prediction_error=pred, params=params)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='+', type=Path)
    ap.add_argument('--json', type=Path)
    ap.add_argument('--sigma-accel', type=float)
    ap.add_argument('--gate-chi2', type=float)
    ap.add_argument('--cusum-k', type=float)
    ap.add_argument('--cusum-h', type=float)
    ap.add_argument('--horizons', type=lambda s: tuple(float(v) for v in s.split(',')), default=HORIZONS_S,
                    help='예측 구간 [s], 쉼표로 구분')
    ap.add_argument('--std-limit-mm', type=float, help='보고 표준편차 통계를 추가하고 이 기준 이하 비율을 낸다')
    a = ap.parse_args(argv)
    params = {k: v for k, v in dict(sigma_accel_mps2=a.sigma_accel, gate_chi2=a.gate_chi2,
                                    cusum_k=a.cusum_k, cusum_h=a.cusum_h).items() if v is not None}
    limit = None if a.std_limit_mm is None else a.std_limit_mm * 1e-3
    results = [analyse(p, params, a.horizons, limit) for p in a.csv]
    for r in results:
        sp = [p['speed_fit_mps'] * 1e3 for p in r['passes']]
        print(f"{r['file']}: passes {len(r['passes'])}/{r['tracks_total']} tracks, "
              f"speed {np.mean(sp) if sp else float('nan'):.2f} mm/s, lap {np.round(r['lap_s'], 2).tolist()} s, "
              f"alarms {len(r['speed_change_alarms_s'])}, lateral std {r['lateral_std_mm']:.2f} mm")
        for h, e in r['prediction_error'].items():
            extra = (f", std p50 {e['std_p50_mm']:.2f} p95 {e['std_p95_mm']:.2f} mm, "
                     f"std<=limit {e['std_le_limit_ratio']:.2f}, err/std p95 {e['norm_err_p95']:.1f}"
                     if 'std_p50_mm' in e else '')
            print(f"  H={h:>4} s n={e['n']:5d} along p95 {e['along_p95_mm']:.2f} mm, "
                  f"xy p95 {e['xy_p95_mm']:.2f} mm, max {e['xy_max_mm']:.2f} mm{extra}")
    if a.json:
        a.json.parent.mkdir(parents=True, exist_ok=True)
        a.json.write_text(json.dumps(results, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
