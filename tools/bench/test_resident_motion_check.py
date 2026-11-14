"""resident_motion_check.py 순수 함수 단위 시험."""
import pytest

from tools.bench.resident_motion_check import DZ_MAX_MM, offset_z


def test_offset_z_moves_only_z():
    assert offset_z([373.0, 0.0, 405.0, 0.7, -180.0, 0.7, 2.0], 10.0) == [373.0, 0.0, 415.0, 0.7, -180.0, 0.7]


@pytest.mark.parametrize('dz', [DZ_MAX_MM + 0.1, -(DZ_MAX_MM + 0.1), float('nan')])
def test_offset_z_rejects_large_or_bad_steps(dz):
    with pytest.raises(ValueError):
        offset_z([0.0] * 6, dz)
