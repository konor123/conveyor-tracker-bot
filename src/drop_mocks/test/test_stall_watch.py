"""heartbeat 경로 정지 진단(SPEC-700 4.8절, REQ-MCK-012). faulthandler 상태는 프로세스 전역이라 하위 프로세스에서 본다."""
import os
import subprocess
import sys
import textwrap

import pytest

from drop_mocks.stall_watch import STALL_DUMP_ENV, stall_dump_s

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def _run(body: str) -> subprocess.CompletedProcess:
    code = textwrap.dedent('''
        import time
        from drop_mocks.stall_watch import StallWatch

        class Log:
            def warning(self, msg):
                print('WARN ' + msg, flush=True)
    ''') + textwrap.dedent(body)
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([PKG, os.environ.get('PYTHONPATH', '')]))
    return subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=30, env=env)


@pytest.mark.req('REQ-MCK-012')
@pytest.mark.tc('TC-MCK-012-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('value,expected', [(None, 0.0), ('', 0.0), ('0', 0.0), ('-1', 0.0), ('abc', 0.0),
                                            ('nan', 0.0), ('inf', 0.0), ('0.25', 0.25)])
def test_env_value(monkeypatch, value, expected):
    """미설정·0·음수·숫자 아님·비유한은 끔(0), 유한한 양수만 켠다."""
    if value is None:
        monkeypatch.delenv(STALL_DUMP_ENV, raising=False)
    else:
        monkeypatch.setenv(STALL_DUMP_ENV, value)
    assert stall_dump_s() == expected


@pytest.mark.req('REQ-MCK-012')
@pytest.mark.tc('TC-MCK-012-01')
@pytest.mark.level('U')
def test_gap_dumps_stacks_while_stuck_and_reports_on_resume():
    """기준보다 긴 공백: 막힌 동안 모든 스레드 스택(막힌 함수 이름 포함)이 stderr에 나오고, 재개한 beat가
    공백·CPU 시간·문맥 전환을 한 줄로 남긴다. 기준보다 짧은 간격은 아무것도 남기지 않는다."""
    r = _run('''
        w = StallWatch(Log(), 0.2)
        for _ in range(5):
            w.beat()
            time.sleep(0.05)

        def stuck_in_callback():
            time.sleep(0.5)

        stuck_in_callback()
        w.beat()
        w.stop()
        print('stalls', w.stalls, flush=True)
    ''')
    assert r.returncode == 0, r.stderr
    assert r.stderr.count('Timeout (') == 1
    assert 'stuck_in_callback' in r.stderr
    warn = [ln for ln in r.stdout.splitlines() if ln.startswith('WARN STALL')]
    assert len(warn) == 1 and '공백 0.' in warn[0] and 'CPU' in warn[0] and '비자발' in warn[0]
    assert 'stalls 1' in r.stdout


@pytest.mark.req('REQ-MCK-012')
@pytest.mark.tc('TC-MCK-012-01')
@pytest.mark.level('U')
def test_stop_cancels_pending_dump():
    """종료 때 `stop()`은 걸린 감시를 풀고, 그 뒤의 `beat()`도 다시 걸지 않아 종료 과정에서 스택이 나오지 않는다."""
    r = _run('''
        w = StallWatch(Log(), 0.1)
        w.beat()
        w.stop()
        w.beat()                       # 종료 뒤 다른 스레드의 진입: 다시 걸지 않는다
        time.sleep(0.4)
        print('done', flush=True)
    ''')
    assert r.returncode == 0, r.stderr
    assert 'Timeout (' not in r.stderr and 'done' in r.stdout
