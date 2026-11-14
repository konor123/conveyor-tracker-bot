import pytest

from intercept_planner.time_budget import evaluate_budget
from orchestrator.deadlines import derive_step_timeout, dynamic_deadline, effective_deadline


@pytest.mark.parametrize("now, step, dynamic, cycle, expected", [
    (100.0, 2.0, 105.0, 110.0, 102.0),     # 단계가 최소
    (100.0, 8.0, 105.0, 110.0, 105.0),     # 동적이 최소
    (100.0, 8.0, 112.0, 104.0, 104.0),     # 사이클이 최소
    (100.0, 8.0, None, 110.0, 108.0),      # 동적 없음
    (100.0, 15.0, None, 110.0, 110.0),     # 동적 없음, 사이클이 최소
])
def test_effective_deadline_is_minimum(now, step, dynamic, cycle, expected):
    assert effective_deadline(now, step, dynamic, cycle) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("x_cap, v, expected", [
    (0.17, 0.02, 100.0 + 11.5 - 1.0),      # T_available = (0.40 − 0.17) / 0.02 = 11.5 s
    (0.0, 0.0, 100.0 - 1.0),               # BELT_STOPPED: T_available 0, 이미 지남
    (0.0, -0.02, 100.0 - 1.0),             # BELT_REVERSED
])
def test_dynamic_deadline_uses_time_budget(x_cap, v, expected):
    b = evaluate_budget(x_cap, v, 0.40, {'plan': 0.5, 'approach': 1.5}, 1.0)
    assert dynamic_deadline(100.0, b.t_available, 1.0) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("p95, p99, expected", [
    (1.0, 1.2, 2.0),
    (2.0, 2.1, 4.0),
])
def test_derive_step_timeout(p95, p99, expected):
    assert derive_step_timeout(p95, p99) == pytest.approx(expected, abs=1e-9)


def test_derive_step_timeout_floor_is_p95_plus_half():
    # ceil(1.5 × 0.2) = 1 < 1.8 + 0.5
    assert derive_step_timeout(1.8, 0.2) == pytest.approx(2.3, abs=1e-9)
