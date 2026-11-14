"""heartbeat 경로 정지 진단. ROS와 무관한 순수 모듈.

시험 환경에서 `DROP_MOCK_STALL_DUMP_S` > 0으로 켠다. 기본(미설정·0)은 아무것도 하지 않는다.
"""
from __future__ import annotations

import faulthandler
import math
import os
import resource
import sys
import threading
import time

STALL_DUMP_ENV = 'DROP_MOCK_STALL_DUMP_S'


def stall_dump_s() -> float:
    """`DROP_MOCK_STALL_DUMP_S` 값 [s]. 없거나 숫자가 아니거나 유한한 양수가 아니면 0(끔)."""
    try:
        v = float(os.environ.get(STALL_DUMP_ENV, '') or 0.0)
    except ValueError:
        return 0.0
    return v if math.isfinite(v) and v > 0.0 else 0.0


class StallWatch:
    """heartbeat 경로 정지 진단. 시험 환경에서만 켠다.

    `beat()`를 heartbeat 콜백에 들어올 때마다 부른다. 부를 때마다 faulthandler 감시를 `limit_s`로 다시 건다.
    그 시간 안에 다음 `beat()`가 없으면 faulthandler의 C 감시 스레드가 모든 Python 스레드의 스택을 stderr에 쓴다.
    이 스레드는 GIL 없이 돌므로 다른 스레드가 GIL을 쥔 채 C 호출에서 막혀 있어도 그 순간의 스택이 남는다.
    다음 `beat()`는 공백 길이, 그 사이 프로세스 CPU 시간(사용자 + 커널)과 자발·비자발 문맥 전환 증가량을 경고로 남긴다.
    이 값으로 세 경우를 가른다: 스레드가 C 호출(그래프·발행)에서 막힘, 같은 callback group의 다른 콜백이 실행 중,
    모든 스레드가 쉬는데 프로세스가 CPU를 받지 못함.
    """

    def __init__(self, logger, limit_s: float):
        self.limit_s = float(limit_s)
        self._logger = logger
        self._lock = threading.Lock()
        self._last = None                  # (monotonic, cpu_s, nvcsw, nivcsw)
        self._stopped = False
        self.stalls = 0

    def beat(self) -> None:
        now = time.monotonic()
        ru = resource.getrusage(resource.RUSAGE_SELF)
        cur = (now, ru.ru_utime + ru.ru_stime, ru.ru_nvcsw, ru.ru_nivcsw)
        with self._lock:
            if self._stopped:              # 종료 뒤 다른 스레드의 진입이 감시를 다시 걸지 않게
                return
            prev, self._last = self._last, cur
            faulthandler.dump_traceback_later(self.limit_s, repeat=False, file=sys.stderr, exit=False)
        if prev is not None and now - prev[0] > self.limit_s:
            self.stalls += 1
            self._logger.warning(
                f'STALL heartbeat 경로 공백 {now - prev[0]:.3f} s(기준 {self.limit_s:.3f} s): '
                f'그동안 프로세스 CPU {cur[1] - prev[1]:.3f} s, 자발 문맥 전환 {cur[2] - prev[2]}, '
                f'비자발 {cur[3] - prev[3]}. 스택은 바로 앞 faulthandler "Timeout" 출력')

    def stop(self) -> None:
        with self._lock:
            self._stopped = True
            self._last = None
            faulthandler.cancel_dump_traceback_later()
