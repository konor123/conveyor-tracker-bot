"""TC-ROB-001~006: SI↔Doosan 변환·속도·신선도·범위·추종 지령."""
from __future__ import annotations

import math

import numpy as np
import pytest

from robot_control.motion_adapter import (
    CommandAuthority,
    FreshnessWatchdog,
    MotionError,
    Workspace,
    matrix_from_euler_zyz,
    pose_doosan_to_si,
    pose_si_to_doosan,
    quat_to_matrix,
    rotation_error,
    speed_doosan_to_si,
    speed_si_to_doosan,
    tracking_command,
)
from fakes.clock import FakeClock


@pytest.mark.req('REQ-ROB-001')
@pytest.mark.tc('TC-ROB-001-01')
@pytest.mark.level('U')
def test_pose_reference_values():
    assert pose_si_to_doosan([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) == pytest.approx(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], abs=1e-9)

    x100 = pose_si_to_doosan([0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    assert x100[:3] == pytest.approx([100.0, 0.0, 0.0], abs=1e-9)

    half = math.sqrt(0.5)
    z90 = pose_si_to_doosan([0.0, 0.0, 0.0], [0.0, 0.0, half, half])
    assert z90[3:] == pytest.approx([90.0, 0.0, 0.0], abs=1e-9)

    # 손으로 만든 ZYZ 기준값과 일치
    rot = matrix_from_euler_zyz(math.radians(20.0), math.radians(35.0), math.radians(-50.0))
    from robot_control.motion_adapter import matrix_to_quat
    got = pose_si_to_doosan([0.0, 0.0, 0.0], matrix_to_quat(rot))
    assert got[3:] == pytest.approx([20.0, 35.0, -50.0], abs=1e-9)


@pytest.mark.req('REQ-ROB-001')
@pytest.mark.tc('TC-ROB-001-02')
@pytest.mark.level('U')
def test_pose_roundtrip_random_and_gimbal_singular():
    rng = np.random.default_rng(0)
    for _ in range(1000):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        pos = rng.normal(size=3) * 0.2
        native = pose_si_to_doosan(pos, q)
        pos2, q2 = pose_doosan_to_si(native)
        assert np.allclose(pos, pos2, atol=1e-12)
        assert rotation_error(quat_to_matrix(q), quat_to_matrix(q2)) <= 1e-9

    for beta in (0.0, math.pi):
        rot = matrix_from_euler_zyz(0.7, beta, -0.4)
        from robot_control.motion_adapter import matrix_to_quat
        q = matrix_to_quat(rot)
        pos2, q2 = pose_doosan_to_si(pose_si_to_doosan([0.0, 0.0, 0.0], q))
        assert rotation_error(rot, quat_to_matrix(q2)) <= 1e-9


@pytest.mark.req('REQ-ROB-002')
@pytest.mark.tc('TC-ROB-002-01')
@pytest.mark.level('U')
def test_speed_unit_conversion():
    native = speed_si_to_doosan([0.02, 0.0, 0.0], [0.1, 0.0, 0.0])
    assert native[:3] == pytest.approx([20.0, 0.0, 0.0], abs=1e-9)
    assert native[3] == pytest.approx(5.729577951308232, abs=1e-9)
    lin, ang = speed_doosan_to_si(native)
    assert np.allclose(lin, [0.02, 0.0, 0.0], atol=1e-12)
    assert np.allclose(ang, [0.1, 0.0, 0.0], atol=1e-12)

    with pytest.raises(MotionError):
        speed_si_to_doosan([math.nan, 0.0, 0.0], [0.0, 0.0, 0.0])


@pytest.mark.req('REQ-ROB-003')
@pytest.mark.tc('TC-ROB-003-01')
@pytest.mark.level('U')
def test_freshness_boundary_and_explicit_rearm():
    clock = FakeClock(steady=10.0)
    watchdog = FreshnessWatchdog(0.01, clock)
    assert watchdog.on_command() is True

    clock.advance(0.020)
    assert watchdog.check() is True
    assert watchdog.on_command() is True

    clock.advance(0.020001)
    assert watchdog.check() is False
    assert watchdog.latched is True
    assert watchdog.on_command() is False

    watchdog.rearm()
    assert watchdog.latched is False
    assert watchdog.on_command() is True


@pytest.mark.req('REQ-ROB-004')
@pytest.mark.tc('TC-ROB-004-01')
@pytest.mark.level('U')
def test_command_authority_rejects_range_and_nonfinite():
    ws = Workspace(-0.1, 0.1, -0.1, 0.1, 0.0, 0.2)
    auth = CommandAuthority(workspace=ws, v_max_mps=0.05)
    assert auth.validate_pose([0.0, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0]) is None
    assert auth.validate_pose([0.2, 0.0, 0.1]) == 'RANGE'
    assert auth.validate_pose([math.inf, 0.0, 0.1]) == 'NONFINITE'
    assert auth.validate_pose([0.0, 0.0, 0.1], [0.0, 0.0, 0.0, 0.0]) == 'NONFINITE'
    assert auth.validate_speed([0.02, 0.0, 0.0]) is None
    assert auth.validate_speed([0.06, 0.0, 0.0]) == 'RANGE'
    assert auth.validate_speed([math.nan, 0.0, 0.0]) == 'NONFINITE'


@pytest.mark.req('REQ-ROB-005')
@pytest.mark.tc('TC-ROB-005-01')
@pytest.mark.level('U')
def test_single_command_authority_owner():
    auth = CommandAuthority()
    assert auth.acquire('place_stone') is True
    assert auth.acquire('move_to_pose') is False
    assert auth.owner == 'place_stone'
    auth.release('place_stone')
    assert auth.acquire('move_to_pose') is True


@pytest.mark.req('REQ-ROB-006')
@pytest.mark.tc('TC-ROB-006-01')
@pytest.mark.level('U')
def test_tracking_command_law_and_limits():
    v = tracking_command([0.0, 0.0, 0.0], [0.02, 0.0, 0.0], [0.0, 0.0, 0.0],
                         1.0, 0.05, 0.1, [0.02, 0.0, 0.0], 0.01)
    assert np.allclose(v, [0.02, 0.0, 0.0])

    v = tracking_command([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                         1.0, 0.05, 100.0, [0.0, 0.0, 0.0], 0.01)
    assert np.linalg.norm(v) == pytest.approx(0.05)

    v = tracking_command([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                         1.0, 1.0, 0.1, [0.0, 0.0, 0.0], 0.01)
    assert np.linalg.norm(v) <= 0.001 + 1e-12

    # v_ff는 유지되고 오차가 0이면 그대로
    v = tracking_command([0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.0, 0.0, 0.0],
                         2.0, 0.05, 1.0, [0.01, 0.0, 0.0], 0.01)
    assert np.allclose(v, [0.01, 0.0, 0.0])


@pytest.mark.req('REQ-ROB-006')
@pytest.mark.tc('TC-ROB-006-01')
@pytest.mark.level('U')
def test_tracking_command_rejects_bad_inputs():
    with pytest.raises(MotionError):
        tracking_command([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                         1.0, 0.05, 0.1, [0.0, 0.0, 0.0], 0.0)
    with pytest.raises(MotionError):
        tracking_command([math.nan, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                         1.0, 0.05, 0.1, [0.0, 0.0, 0.0], 0.01)
