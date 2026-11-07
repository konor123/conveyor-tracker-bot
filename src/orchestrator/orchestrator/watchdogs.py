"""감시 타이머(SPEC-110 4.3절). 시계는 생성자로 받는다(steady / ROS)."""
from __future__ import annotations

from .common import Clock, SteadyClock, ge


class HeartbeatMonitor:
    """마지막 수신 후 timeout_s 이상이면 손실(steady 시계). 한 번도 받지 못한 노드는 손실이다."""

    def __init__(self, timeout_s: float, clock: SteadyClock):
        self._timeout = timeout_s
        self._clock = clock
        self._last: dict[str, float] = {}

    def beat(self, node: str) -> None:
        self._last[node] = self._clock.steady_now()

    def lost(self, node: str) -> bool:
        t = self._last.get(node)
        return t is None or ge(self._clock.steady_now() - t, self._timeout)

    def age(self, node: str) -> float:
        t = self._last.get(node)
        return float('inf') if t is None else self._clock.steady_now() - t


class FreshnessMonitor:
    """동적 구간 관측 신선도(ROS 시각): t_now − t_last_obs > limit_s 이면 stale."""

    def __init__(self, limit_s: float):
        self._limit = limit_s

    def stale(self, t_now: float, t_last_obs: float) -> bool:
        return t_now - t_last_obs > self._limit + 1e-9


class ClockMonitor:
    """ROS 시각 역행 검출. 멈춤(같은 값)은 역행이 아니다."""

    def __init__(self):
        self._last: float | None = None

    def update(self, t_ros: float) -> bool:
        back = self._last is not None and t_ros < self._last
        self._last = t_ros
        return back


class GripperMonitor:
    """오류 비트·응답 시간·상주 루프 생존 카운터(steady 시계)."""

    def __init__(self, response_max_s: float, alive_stall_s: float, clock: SteadyClock):
        self._response_max = response_max_s
        self._stall = alive_stall_s
        self._clock = clock
        self._error_bits = 0
        self._response_s = 0.0
        self._counter: int | None = None
        self._counter_t: float | None = None
        self.reason = ''

    def update(self, error_bits: int, response_s: float, alive_counter: int | None) -> None:
        self._error_bits = error_bits
        self._response_s = response_s
        if alive_counter is not None and alive_counter != self._counter:
            self._counter = alive_counter
            self._counter_t = self._clock.steady_now()

    def fault(self) -> bool:
        if self._error_bits != 0:
            self.reason = 'GRIPPER_ERROR'
        elif self._response_s > self._response_max + 1e-9:
            self.reason = 'GRIPPER_SLOW'
        elif (self._counter_t is not None
              and self._clock.steady_now() - self._counter_t > self._stall + 1e-9):
            self.reason = 'GRIPPER_DEAD'
        else:
            self.reason = ''
        return bool(self.reason)


class CycleTimer:
    """사이클 제한시간: ROS deadline 또는 steady 상한(timeout × 2) 중 먼저 오는 쪽."""

    def __init__(self, clock: Clock, ros_deadline: float, steady_limit_s: float):
        self._clock = clock
        self._ros_deadline = ros_deadline
        self._steady_end = clock.steady_now() + steady_limit_s

    def expired(self) -> bool:
        return (ge(self._clock.ros_now(), self._ros_deadline)
                or ge(self._clock.steady_now(), self._steady_end))
