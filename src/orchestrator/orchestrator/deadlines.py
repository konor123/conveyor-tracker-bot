"""단계·동적·사이클 deadline(SPEC-110 5절)."""
from __future__ import annotations

import math

from .common import EPS


def effective_deadline(now: float, step_limit: float, dynamic: float | None, cycle: float) -> float:
    """실제 deadline = min(now + 단계 제한시간, 동적 deadline, 사이클 deadline). 동적이 None이면 뺀다."""
    candidates = [now + step_limit, cycle]
    if dynamic is not None:
        candidates.append(dynamic)
    return min(candidates)


def derive_step_timeout(p95: float, p99: float) -> float:
    """측정 기반 단계 제한시간 = max(ceil(1.5 × p99), p95 + 0.5) [s]."""
    return max(float(math.ceil(1.5 * p99 - EPS)), p95 + 0.5)
