"""노드 진입점 종료 처리 C 시험(SPEC-000 4.5절 REQ-SYS-007, TEST_SPEC.md 3.1절 TC-SYS-007-01).

진입점과 같은 모양(`ShutdownGuard` → `rclpy.init` → spin → `is_shutdown` → 정리)의 탐침을 새 프로세스로 띄워
종료 신호를 넣는다. 사본 5개가 같은지는 TC-SYS-007-02(`tools/spec/test_node_shutdown.py`)가 본다.
"""
import os
import signal
import subprocess
import sys
import time

import pytest

pytest.importorskip('rclpy')

DOMAIN = 100 + os.getpid() % 100

PROBE = r'''
import signal, sys
from drop_mocks.node_shutdown import ShutdownGuard
case = sys.argv[1]
guard = ShutdownGuard()
import rclpy
from rclpy.node import Node
from drop_interfaces.msg import Heartbeat
rclpy.init()
node = None
try:
    node = Node('shutdown_probe')
    if case == 'take_race':
        # 받은 메시지를 Python 객체로 바꾸는 중(executor `_take_subscription` 안)에 SIGINT
        armed = []
        orig = Heartbeat.__init__
        def patched(self, *a, **k):
            orig(self, *a, **k)
            if armed and sys._getframe(1).f_code.co_name == '_take_subscription':
                armed.clear()
                print('RAISED_IN_TAKE', flush=True)
                signal.raise_signal(signal.SIGINT)
        Heartbeat.__init__ = patched
        pub = node.create_publisher(Heartbeat, 'probe_hb', 10)
        node.create_subscription(Heartbeat, 'probe_hb', lambda m: None, 10)
        msg = Heartbeat()
        def tick():
            pub.publish(msg)
            armed.append(1)
        node.create_timer(0.05, tick)
    elif case == 'second_sigint':
        node.create_timer(0.05, lambda: signal.raise_signal(signal.SIGINT))
    print('READY', flush=True)
    rclpy.spin(node)
except BaseException as exc:
    if not guard.is_shutdown(exc):
        raise
finally:
    if case == 'second_sigint':
        signal.raise_signal(signal.SIGINT)         # 정리 도중 두 번째 SIGINT
    if node is not None:
        node.destroy_node()
    rclpy.try_shutdown()
    guard.close()
print('CLEAN', guard.sigint_count, flush=True)
'''


def _run(case, send=None):
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN), PYTHONPATH=os.pathsep.join(p for p in sys.path if p))
    proc = subprocess.Popen([sys.executable, '-c', PROBE, case], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)
    t_sent = None
    try:
        if send is not None:
            assert proc.stdout.readline().strip() == 'READY'
            time.sleep(0.3)                        # spin 대기에 들어간 뒤
            t_sent = time.monotonic()
            proc.send_signal(send)
        out, err = proc.communicate(timeout=20)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
    exit_s = None if t_sent is None else time.monotonic() - t_sent
    return proc.returncode, out, err, exit_s


@pytest.mark.req('REQ-SYS-007')
@pytest.mark.tc('TC-SYS-007-01')
@pytest.mark.level('C')
def test_sigint_while_taking_a_message_exits_quietly():
    """rclpy가 받은 메시지를 바꾸는 중의 SIGINT는 `RuntimeError`가 된다. 이것도 traceback 없이 끝난다."""
    code, out, err, _ = _run('take_race')
    assert 'RAISED_IN_TAKE' in out, (out, err[-2000:])
    assert (code, out.split()[-2:]) == (0, ['CLEAN', '1']) and 'Traceback' not in err, (out, err[-2000:])


@pytest.mark.req('REQ-SYS-007')
@pytest.mark.tc('TC-SYS-007-01')
@pytest.mark.level('C')
def test_second_sigint_during_cleanup_exits_quietly():
    code, out, err, _ = _run('second_sigint')
    assert (code, out.split()[-2:]) == (0, ['CLEAN', '2']) and 'Traceback' not in err, (out, err[-2000:])


@pytest.mark.req('REQ-SYS-007')
@pytest.mark.tc('TC-SYS-007-01')
@pytest.mark.level('C')
@pytest.mark.parametrize('sig, count', [(signal.SIGINT, '1'), (signal.SIGTERM, '0')], ids=['SIGINT', 'SIGTERM'])
def test_idle_node_exits_promptly_on_signal(sig, count):
    """타이머·구독이 없어 깨어날 일이 없는 노드도 신호 뒤 곧 끝난다(rclpy 처리기가 빠지지 않음)."""
    code, out, err, exit_s = _run('idle', send=sig)
    assert (code, out.split()[-2:]) == (0, ['CLEAN', count]) and 'Traceback' not in err, (out, err[-2000:])
    assert exit_s < 3.0


@pytest.mark.req('REQ-SYS-007')
@pytest.mark.tc('TC-SYS-007-01')
@pytest.mark.level('C')
def test_sigint_after_cleanup_does_not_kill_with_minus_two():
    """정리 뒤 Python 종료 단계에 도착한 SIGINT(launch가 늦게 보낸 것)에도 종료 코드 0이다.

    Python은 종료 단계에서 Python 처리기가 걸린 SIGINT를 SIG_DFL로 되돌리므로, `guard.close()`가 없으면 그 뒤 SIGINT에
    traceback 없이 −2로 죽는다(2026-11-13 사무실 PC WSL I0 launch 로그). `CLEAN` 뒤 프로세스가 끝날 때까지 SIGINT를 계속 보낸다.
    """
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN), PYTHONPATH=os.pathsep.join(p for p in sys.path if p))
    proc = subprocess.Popen([sys.executable, '-c', PROBE, 'idle'], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)
    sent = 0
    try:
        assert proc.stdout.readline().strip() == 'READY'
        time.sleep(0.3)
        proc.send_signal(signal.SIGINT)
        clean = proc.stdout.readline().split()
        while proc.poll() is None and sent < 20000:
            proc.send_signal(signal.SIGINT)
            sent += 1
            time.sleep(0.0005)
        out, err = proc.communicate(timeout=20)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
    assert clean == ['CLEAN', '1'] and sent > 0
    assert proc.returncode == 0 and 'Traceback' not in err, (proc.returncode, sent, err[-2000:])
