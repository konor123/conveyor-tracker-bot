"""시도 기록 분석 통계."""

import math

import numpy as np

_COUNTED = ("SUCCESS", "FAILURE", "UNKNOWN")
_IGNORED = ("CANCELED", "SAFE_ABORT", "PROTECTIVE_STOP", "NOT_ATTEMPTED")


def wilson_interval(k: int, n: int, z: float = 1.959963984540054) -> tuple[float, float]:
    """Wilson score 구간(기본 95%). n == 0이면 (0.0, 1.0)."""
    if n < 0:
        raise ValueError(f"n must be >= 0: {n}")
    if k < 0 or k > n:
        raise ValueError(f"k must satisfy 0 <= k <= n: k={k}, n={n}")
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    z2 = z * z
    denom = 1.0 + z2 / n
    center = (p + z2 / (2.0 * n)) / denom
    margin = (z / denom) * math.sqrt(p * (1.0 - p) / n + z2 / (4.0 * n * n))
    lo = max(0.0, center - margin)
    hi = min(1.0, center + margin)
    return (lo, hi)


def success_rate(results: list[str]) -> tuple[int, int]:
    """(성공 수, 시도 수). UNKNOWN은 분모에만 넣는다."""
    num = 0
    den = 0
    for r in results:
        if r in _IGNORED:
            continue
        if r not in _COUNTED:
            raise ValueError(f"unknown result: {r!r}")
        den += 1
        if r == "SUCCESS":
            num += 1
    return (num, den)


def quantiles(samples, qs=(50, 95, 99)) -> dict[str, float]:
    """p50/p95/p99/최대값. numpy percentile(method="linear")와 같다."""
    if len(samples) == 0:
        raise ValueError("samples must not be empty")
    out = {}
    for q in qs:
        out[f"p{q}"] = float(np.percentile(samples, q, method="linear"))
    out["max"] = float(np.max(samples))
    return out
