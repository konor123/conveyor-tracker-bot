"""tools/eval/stats.py 단위 테스트(TEST_SPEC.md 3.12절)."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import stats  # noqa: E402


@pytest.mark.req("REQ-REC-004")
@pytest.mark.tc("TC-REC-004-01")
@pytest.mark.level("U")
@pytest.mark.parametrize(
    "k, n, expected",
    [
        (45, 50, (0.7864, 0.9565)),
        (50, 50, (0.9287, 1.0000)),
        (0, 20, (0.0000, 0.1611)),
    ],
)
def test_wilson_interval(k, n, expected):
    lo, hi = stats.wilson_interval(k, n)
    assert lo == pytest.approx(expected[0], abs=1e-4)
    assert hi == pytest.approx(expected[1], abs=1e-4)


@pytest.mark.req("REQ-REC-004")
@pytest.mark.tc("TC-REC-004-01")
@pytest.mark.level("U")
def test_wilson_interval_empty():
    assert stats.wilson_interval(0, 0) == (0.0, 1.0)


@pytest.mark.req("REQ-REC-004")
@pytest.mark.tc("TC-REC-004-01")
@pytest.mark.level("U")
def test_wilson_interval_invalid():
    with pytest.raises(ValueError):
        stats.wilson_interval(-1, 10)
    with pytest.raises(ValueError):
        stats.wilson_interval(11, 10)


@pytest.mark.req("REQ-REC-004")
@pytest.mark.tc("TC-REC-004-02")
@pytest.mark.level("U")
def test_success_rate_unknown_in_denominator():
    results = (
        ["SUCCESS"] * 40
        + ["FAILURE"] * 5
        + ["UNKNOWN"] * 5
        + ["NOT_ATTEMPTED"] * 3
    )
    assert stats.success_rate(results) == (40, 50)


@pytest.mark.req("REQ-REC-004")
@pytest.mark.tc("TC-REC-004-02")
@pytest.mark.level("U")
def test_success_rate_invalid():
    with pytest.raises(ValueError):
        stats.success_rate(["SUCCESS", "BOGUS"])


@pytest.mark.req("REQ-REC-005")
@pytest.mark.tc("TC-REC-005-01")
@pytest.mark.level("U")
def test_quantiles_random_sample():
    samples = np.random.default_rng(0).exponential(1.0, 1000)
    got = stats.quantiles(samples)
    for q, key in ((50, "p50"), (95, "p95"), (99, "p99")):
        assert got[key] == pytest.approx(
            float(np.percentile(samples, q, method="linear")), abs=1e-12
        )
    assert got["max"] == pytest.approx(float(np.max(samples)), abs=1e-12)


@pytest.mark.req("REQ-REC-005")
@pytest.mark.tc("TC-REC-005-01")
@pytest.mark.level("U")
def test_quantiles_small_sample():
    got = stats.quantiles([1, 2, 3, 4])
    assert got["p50"] == pytest.approx(2.5, abs=1e-12)
    for q, key in ((95, "p95"), (99, "p99")):
        assert got[key] == pytest.approx(
            float(np.percentile([1, 2, 3, 4], q, method="linear")), abs=1e-12
        )
    assert got["max"] == pytest.approx(4.0, abs=1e-12)


@pytest.mark.req("REQ-REC-005")
@pytest.mark.tc("TC-REC-005-01")
@pytest.mark.level("U")
def test_quantiles_empty():
    with pytest.raises(ValueError):
        stats.quantiles([])
