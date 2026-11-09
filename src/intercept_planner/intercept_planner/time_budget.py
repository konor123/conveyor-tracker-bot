"""Single source of the conveyor time-budget calculation (SPEC-400 §4.2)."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Budget:
    t_available: float
    t_required: float
    ok: bool
    components: dict[str, float]
    reason: str


def evaluate_budget(x_cap: float, v_cap: float, x_exit: float,
                    components: dict[str, float], margin: float, *, v_min: float = 0.0) -> Budget:
    required = sum(components.values())
    # SPEC-400 4.2: reversed if v < 0 and v <= -v_min; otherwise stopped if v <= v_min
    # (zero is stopped even when v_min = 0).
    if v_cap < 0 and v_cap <= -v_min:
        return Budget(0., required, False, dict(components), 'BELT_REVERSED')
    if v_cap <= v_min:
        return Budget(0., required, False, dict(components), 'BELT_STOPPED')
    available = (x_exit - x_cap) / v_cap
    ok = available - required >= margin
    return Budget(available, required, ok, dict(components), '' if ok else 'TIME_MARGIN')
