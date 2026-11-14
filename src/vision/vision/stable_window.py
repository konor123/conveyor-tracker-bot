"""DetectObjects 안정 창 판정. 최근 프레임 검출을 모아 창 안에서 계속 보이고 흔들림이 작은 것만 낸다.

ROS와 무관한 순수 모듈. 시각은 영상 노출 시각 [s]이다. 결과 코드는 DetectObjects.srv와 같다.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .pipeline import Detection

OK, FAILED = 0, 1


@dataclass(frozen=True)
class WindowParams:
    std_max_m: float = 0.0005        # 창 안 위치 표준편차 상한(축별 최대). orchestrator detect_stone.window_std_max_m
    std_max_px: float = 1.0          # 보정이 없을 때 픽셀 중심 표준편차 상한(제안값)
    presence_min_ratio: float = 0.8  # 창 안 프레임 중 보여야 하는 비율
    gate_m: float = 0.005            # 프레임 간 같은 대상물로 보는 거리
    gate_px: float = 8.0
    span_tol_s: float = 0.05         # 창 길이를 채웠다고 보는 여유(프레임 주기 이상)


@dataclass(frozen=True)
class WindowResult:
    result: int
    reason_code: str
    detections: tuple[Detection, ...]
    position_std: tuple[float, ...]   # [m], 위치가 없으면 nan


class DetectionWindow:
    def __init__(self, horizon_s: float = 3.0):
        self.horizon_s = horizon_s
        self._frames: deque[tuple[float, tuple[Detection, ...]]] = deque()

    def add(self, t: float, detections: Sequence[Detection]) -> None:
        if self._frames and t <= self._frames[-1][0]:
            self._frames.clear()              # stamp 역행·반복은 창을 다시 채운다
        self._frames.append((t, tuple(detections)))
        while self._frames and self._frames[0][0] < t - self.horizon_s:
            self._frames.popleft()

    def clear(self) -> None:
        self._frames.clear()

    def evaluate(self, region: int, object_class: int, min_count: int, window_s: float,
                 p: WindowParams | None = None) -> WindowResult | None:
        """창이 아직 차지 않았으면 None(호출자가 다음 프레임까지 기다린다)."""
        p = p or WindowParams()
        if not self._frames:
            return None
        t_last = self._frames[-1][0]
        frames = [(t, ds) for t, ds in self._frames if t >= t_last - window_s - 1e-9]
        if t_last - self._frames[0][0] < window_s - p.span_tol_s:
            return None

        def pick(ds):
            return [d for d in ds if d.region == region and d.object_class == object_class
                    and not d.blob.occluded_partial]

        seeds = pick(frames[-1][1])
        stable, stds = [], []
        for seed in seeds:
            use_m = seed.position_valid
            ref = np.array(seed.position[:2]) if use_m else np.array([seed.blob.u, seed.blob.v])
            gate = p.gate_m if use_m else p.gate_px
            track = []
            for _, ds in frames:
                cands = [d for d in pick(ds) if d.position_valid == use_m]
                if not cands:
                    continue
                xy = np.array([d.position[:2] if use_m else (d.blob.u, d.blob.v) for d in cands])
                dist = np.hypot(*(xy - ref).T)
                k = int(np.argmin(dist))
                if dist[k] <= gate:
                    track.append(xy[k])
            if len(track) < p.presence_min_ratio * len(frames):
                continue
            std = float(np.max(np.std(np.array(track), axis=0)))
            if std > (p.std_max_m if use_m else p.std_max_px):
                continue
            stable.append(seed)
            stds.append(std if use_m else float('nan'))
        if len(stable) < min_count:
            return WindowResult(FAILED, 'NOT_ENOUGH_STABLE', tuple(stable), tuple(stds))
        return WindowResult(OK, '', tuple(stable), tuple(stds))
