"""e0509_ctl.py 단위 테스트."""

import pytest
from tools.robot.e0509_ctl import (
    MECHANICAL_POSJ,
    STANDBY_POSX,
    check_network,
    main,
    validate_gripper,
    validate_posj,
    validate_posx,
)


def test_validate_posx_valid():
    validate_posx(STANDBY_POSX)
    validate_posx([500.0, 0.0, 300.0, 0.0, -180.0, 0.0])


def test_validate_posx_out_of_bounds():
    with pytest.raises(ValueError, match="안전 범위"):
        validate_posx([50.0, 0.0, 300.0, 0.0, 0.0, 0.0])  # x < 100

    with pytest.raises(ValueError, match="안전 범위"):
        validate_posx([500.0, 600.0, 300.0, 0.0, 0.0, 0.0])  # y > 500

    with pytest.raises(ValueError, match="안전 범위"):
        validate_posx([500.0, 0.0, 20.0, 0.0, 0.0, 0.0])  # z < 50


def test_validate_posj_valid():
    validate_posj(MECHANICAL_POSJ)
    validate_posj([0.0, 10.0, -90.0, 0.0, 80.0, 0.0])


def test_validate_posj_invalid_length():
    with pytest.raises(ValueError, match="길이는 6"):
        validate_posj([0.0, 0.0, 0.0])


def test_validate_gripper_valid():
    validate_gripper(0, 100)
    validate_gripper(750, 50)
    validate_gripper(1150, 100, allow_hardware_limit=True)


def test_validate_gripper_invalid():
    with pytest.raises(ValueError, match="pulse"):
        validate_gripper(-1, 100)

    with pytest.raises(ValueError, match="pulse"):
        validate_gripper(800, 100, allow_hardware_limit=False)

    with pytest.raises(ValueError, match="current_ma"):
        validate_gripper(0, 0)  # current < 1

    with pytest.raises(ValueError, match="current_ma"):
        validate_gripper(0, 150)  # current > 100


def test_check_network_mocked(monkeypatch):
    import socket

    class MockSocket:
        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

    monkeypatch.setattr(socket, "create_connection", lambda *args, **kwargs: MockSocket())
    assert check_network("127.0.0.1", 12345) is True

    monkeypatch.setattr(socket, "create_connection", lambda *args, **kwargs: (_ for _ in ()).throw(OSError))
    assert check_network("127.0.0.1", 12345) is False


def test_cli_dry_run_home(capsys):
    ret = main(["home", "--type", "standby", "--dry-run"])
    assert ret == 0
    captured = capsys.readouterr()
    assert "[dry-run]" in captured.out
    assert "standby" in captured.out


def test_cli_dry_run_gripper(capsys):
    ret = main(["gripper", "open", "--dry-run"])
    assert ret == 0
    captured = capsys.readouterr()
    assert "[dry-run]" in captured.out
    assert "open" in captured.out


def test_cli_dry_run_move_posx(capsys):
    ret = main(["move-posx", "372.96", "0.0", "404.80", "0.71", "-179.97", "0.71", "--dry-run"])
    assert ret == 0
    captured = capsys.readouterr()
    assert "[dry-run]" in captured.out
