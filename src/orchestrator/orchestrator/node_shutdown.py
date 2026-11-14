"""노드 진입점의 종료 신호 처리.

같은 내용의 사본을 `tracking`·`intercept_planner`·`orchestrator`·`robot_control`·`drop_mocks`에 둔다(패키지 사이
의존을 늘리지 않는다). 사본이 같은지와 진입점이 이 처리를 쓰는지는 `tools/checks/test_node_shutdown.py`가 본다.
"""
from __future__ import annotations

import signal


class ShutdownGuard:
    """SIGINT는 첫 번째만 `KeyboardInterrupt`로 올리고, 종료 신호로 생긴 예외를 가려낸다.

    - `rclpy.init()` 전에 만든다. rclpy는 init 때 기존 SIGINT 처리기를 저장해 자기 처리기(context 종료) 안에서
      이어 부른다. init 뒤에 `signal.signal`을 부르면 rclpy 처리기가 빠져 대기 중인 executor가 깨지 않는다.
    - 두 번째 이후 SIGINT(launch와 셸이 함께 보낸 것 등)는 세기만 한다. 정리(`destroy_node`) 도중의 SIGINT가
      traceback을 남기지 않는다.
    - rclpy가 받은 메시지를 Python 객체로 바꾸는 중에 `KeyboardInterrupt`가 나면 pybind11이 이를
      `RuntimeError('Unable to convert call argument ...')`로 바꾸고 원래 예외를 남기지 않는다(rclpy jazzy
      `subscription.cpp` `Subscription::take_message`). 그래서 SIGINT를 받은 뒤의 `RuntimeError`는 종료로 본다.
    """

    def __init__(self) -> None:
        self.sigint_count = 0
        signal.signal(signal.SIGINT, self._on_sigint)

    def _on_sigint(self, signum, frame) -> None:
        self.sigint_count += 1
        if self.sigint_count == 1:
            raise KeyboardInterrupt

    def close(self) -> None:
        """정리(`destroy_node`·`try_shutdown`)를 마친 뒤 부른다. 이후 SIGINT는 무시한다.

        Python은 종료 단계(`Py_FinalizeEx` → `_PySignal_Fini`)에서 Python 처리기가 걸린 신호를 기본 처리(SIG_DFL)로
        되돌린다. 그 뒤 도착한 SIGINT(launch가 늦게 보낸 것)는 traceback 없이 종료 코드 −2를 낸다. SIG_IGN은 되돌리지
        않는다.
        """
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    def is_shutdown(self, exc: BaseException) -> bool:
        """종료 신호로 생긴 예외면 True(조용히 끝낸다). 그 밖의 예외(`SystemExit` 포함)는 False(다시 올린다)."""
        from rclpy.executors import ExternalShutdownException
        if isinstance(exc, (KeyboardInterrupt, ExternalShutdownException)):
            return True
        return isinstance(exc, RuntimeError) and self.sigint_count > 0
