"""Stop 정지 유지와 작업자 해제. ROS와 무관한 순수 판정.

`Stop`을 받으면 요청자와 무관하게 정지를 유지한다. 유지 중에는 지령 준비가 아니고(`ready=false`),
heartbeat `error_code`는 101(보호정지가 함께면 100)이다. 해제는 작업자가 비어 있지 않은 사유를
줄 때만 하며, 실행 중 goal이나 보호정지가 있으면 거부한다. 자동 해제는 없다.
유지 중이 아닐 때의 해제 요청(파라미터 선언 시 기본값 포함)은 아무것도 하지 않고 성공이다.
"""
from __future__ import annotations

import threading

ERROR_NONE = 0
ERROR_PROTECTIVE_STOP = 100
ERROR_STOP_LATCHED = 101


class StopLatch:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latched = False
        self._reason = ''
        self.count = 0                      # 받은 Stop 수(진단)
        self.releases: list[str] = []       # 받아들인 해제 사유(진단)

    @property
    def latched(self) -> bool:
        with self._lock:
            return self._latched

    @property
    def reason(self) -> str:
        with self._lock:
            return self._reason

    def latch(self, reason: str) -> None:
        """Stop 수신. 이미 유지 중이면 처음 사유를 남긴다."""
        with self._lock:
            self.count += 1
            if not self._latched:
                self._latched = True
                self._reason = str(reason)

    def release(self, reason: str, goal_active: bool, protective_stop: bool) -> str:
        """작업자 해제. 성공이면 '', 거부면 이유 코드(유지). 유지 중이 아니면 아무것도 하지 않는다."""
        reason = (reason or '').strip()
        if not self.latched:
            return ''
        if not reason:
            return 'REASON_REQUIRED'
        if goal_active:
            return 'GOAL_ACTIVE'
        if protective_stop:
            return 'PROTECTIVE_STOP'
        with self._lock:
            self._latched = False
            self._reason = ''
            self.releases.append(reason)
        return ''

    def ready(self, active: bool, protective_stop: bool) -> bool:
        return bool(active) and not protective_stop and not self.latched

    def heartbeat_code(self, protective_stop: bool) -> int:
        if protective_stop:
            return ERROR_PROTECTIVE_STOP
        return ERROR_STOP_LATCHED if self.latched else ERROR_NONE
