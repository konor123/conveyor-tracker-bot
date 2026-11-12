"""tools/bench/check_clock_steps.py 단위 시험(가짜 시계)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
import check_clock_steps as ccs  # noqa: E402


class FakeClocks:
    """단조 시계는 sleep만큼 흐르고, 벽시계는 지정한 단조 시각에 계단만큼 어긋난다."""

    def __init__(self, steps):
        self.m, self.steps = 100.0, dict(steps)

    def mono(self):
        return self.m

    def wall(self):
        return 1_000_000.0 + self.m + sum(d for t, d in self.steps.items() if self.m >= t)

    def sleep(self, dt):
        self.m += dt


@pytest.mark.req('REQ-TRK-008')
@pytest.mark.level('U')
def test_backward_step_is_found():
    c = FakeClocks({100.5: -1.0, 101.25: 0.002})
    rows = ccs.sample(2.0, 0.005, wall=c.wall, mono=c.mono, sleep=c.sleep)
    steps = ccs.find_steps([off for _, off in rows])
    assert [round(d, 6) for _, d in steps] == [-1.0, 0.002]
    assert rows[steps[0][0]][0] == pytest.approx(100.5, abs=0.006)


@pytest.mark.req('REQ-TRK-008')
@pytest.mark.level('U')
def test_slew_below_threshold_is_not_a_step():
    offsets = [i * 1e-6 for i in range(1000)]            # 표본마다 1 µs(느린 보정)
    assert ccs.find_steps(offsets) == []
    assert ccs.find_steps([0.0, 0.0006]) == [(1, 0.0006)]
