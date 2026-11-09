"""Analytical maximum prediction age, in seconds (SPEC-400 §4.6)."""
import math


def tau_max(e_allow: float, e_pos: float, dv: float, a: float, v: float, dt: float) -> float:
    if a < 0:
        raise ValueError('a must be a nonnegative acceleration bound')
    remaining = e_allow - e_pos - abs(v) * dt
    if remaining < 0:
        return 0.
    speed_error = abs(dv)
    if a > 0:
        if remaining == 0:
            return 0.
        # Rationalized positive root avoids cancellation for small acceleration.
        return 2 * remaining / (speed_error + math.sqrt(speed_error ** 2 + 2 * a * remaining))
    if speed_error:
        return remaining / speed_error
    return math.inf
