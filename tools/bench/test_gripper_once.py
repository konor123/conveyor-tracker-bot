"""Offline checks for the on-site one-shot gripper command."""
import importlib.util
from pathlib import Path
import pytest

_spec = importlib.util.spec_from_file_location('gripper_once', Path(__file__).with_name('gripper_once.py'))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)

def test_validated_wire_frames():
    code = m.render(500, 100)
    ns = {}
    exec(code[:code.index('def read_words')], ns)
    assert ns['crc16']([1, 3, 0, 0, 0, 1]) == 0x0a84
    assert ns['crc16']([1, 6, 1, 0, 0, 1]) == 0xf649
    assert ns['crc16']([1, 16, 1, 26, 0, 2, 4, 1, 44, 0, 0]) == 0x79bf
    compile(code, '<DRL>', 'exec')
    assert 'move_checked(500, 1)' in code
    assert 'move_checked(0, 2)' not in code  # one requested goal only
    assert 'position() > 750' in code
    assert 'write_checked([1, 6, 1, 19, 0, 100])' in code

@pytest.mark.parametrize('pulse,current', [(-1,100),(751,100),(0,0),(0,151),(0,65536)])
def test_reject_outside_approved_envelope(pulse, current):
    with pytest.raises(ValueError):
        m.render(pulse, current)

def test_site_endpoint_and_close_default(capsys):
    assert 'move_checked(750, 1)' in m.render(750, 100)
    with pytest.raises(ValueError):
        m.render(751, 100)
    assert m.main(['close', '--dry-run']) == 0
    assert 'move_checked(750, 1)' in capsys.readouterr().out

@pytest.mark.parametrize('action', ['open', 'close', 'set'])
def test_all_commands_default_100_and_reject_151(action, capsys):
    args = [action, '--dry-run']
    if action == 'set':
        args += ['--pulse', '300']
    assert m.main(args) == 0
    assert 'write_checked([1, 6, 1, 19, 0, 100])' in capsys.readouterr().out
    with pytest.raises(SystemExit) as error:
        m.main(args + ['--current', '151'])
    assert error.value.code == 2

def test_legacy_transports_reject_101_before_io(monkeypatch):
    import gripper_direct
    import gripper_cmd
    import gripper_pulse_table
    def forbidden_socket(*args, **kwargs):
        raise AssertionError('network access forbidden in unit test')
    monkeypatch.setattr(gripper_direct.socket, 'socket', forbidden_socket)
    with pytest.raises(ValueError):
        gripper_direct.send_gripper_command(0, 101)
    with pytest.raises(ValueError):
        gripper_cmd.send_register_command(None, None, None, None, 0, 101, {})
    writes = []
    writer = gripper_pulse_table.RegisterWriter(lambda a, v: writes.append((a, v)))
    with pytest.raises(ValueError):
        writer.send(gripper_pulse_table.Command(0, 101))
    assert not writes


def test_enable_torque_writes_ram_256_only_when_requested():
    assert 'if False and read_words(256, 1)[0] != 1:' in m.render(750, 150)
    code = m.render(750, 150, enable_torque=True)
    assert 'if True and read_words(256, 1)[0] != 1:' in code
    assert 'write_checked([1, 6, 1, 0, 0, 1])' in code
    assert code.index('fail(247)') < code.index('write_checked([1, 6, 1, 0, 0, 1])')   # 모드 확인이 먼저
