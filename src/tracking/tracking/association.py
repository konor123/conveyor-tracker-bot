"""Detections and deterministic greedy Mahalanobis association."""
from dataclasses import dataclass
import numpy as np
from .kalman import measurement_covariance, inverse2


@dataclass
class Det:
    position: np.ndarray
    position_cov: np.ndarray | None = None
    object_class: int = 2  # CLASS_CAP
    position_valid: bool = True

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float).copy()


@dataclass(frozen=True)
class Assignment:
    matches: list[tuple[int, int]]  # indices into the original input sequences
    unmatched_tracks: list[int]
    unmatched_dets: list[int]


def associate(tracks, dets, gate_chi2):
    pairs = []
    for i, track in enumerate(tracks):
        if track.status == 2:
            continue
        for j, det in enumerate(dets):
            if not det.position_valid or det.object_class != 2:
                continue
            delta = det.position[:2] - track.position[:2]
            s = track.state_cov[:2, :2] + measurement_covariance(det.position_cov)
            d2 = float(delta @ inverse2(s) @ delta)
            if d2 <= gate_chi2:
                pairs.append((d2, i, j))
    used_tracks, used_dets, matches = set(), set(), []
    for _, i, j in sorted(pairs):
        if i not in used_tracks and j not in used_dets:
            used_tracks.add(i)
            used_dets.add(j)
            matches.append((i, j))
    return Assignment(matches, [i for i in range(len(tracks)) if i not in used_tracks],
                      [j for j in range(len(dets)) if j not in used_dets])
