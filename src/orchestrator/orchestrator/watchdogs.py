"""감시 타이머(SPEC-110 4.3절). 시계는 생성자로 받는다(steady / ROS)."""
from __future__ import annotations

from .abort_policy import Event
from .common import DYNAMIC_STATES, Clock, State, SteadyClock, ge

S = State
# 사이클 감시를 latch하는 단계: PRECHECK 다음부터 VERIFY까지의 정상 열. PRECHECK는 자기 판정으로 거부하고(로봇 미동작),
# 중단 경로·WAIT_OPERATOR에서는 새 비하드 장애를 latch하지 않는다(중단 경로 대기를 끊어 L4로 올리지 않는다).
WATCH_STATES = frozenset({S.DETECT_STONE, S.PICK_STONE, S.MOVE_STANDBY, S.WAIT_CAP, S.PLAN_INTERCEPT,
                          S.APPROACH, S.SYNCHRONIZE, S.DESCEND, S.RELEASE, S.RETREAT, S.VERIFY})


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


class BeltSteadyMonitor:
    """`BeltMotion.steady == false` 지속 시간(메시지 stamp, ROS 시각, REQ-ABT-013).

    false가 처음 온 stamp부터 마지막 stamp까지가 hold_s를 넘으면 불안정이다. true를 받으면 다시 센다.
    stamp가 거꾸로 가면(시각 역행) 그 stamp부터 다시 센다. 쓰는 스레드는 하나이고, 읽는 쪽(감시 타이머)이
    다른 메시지의 값을 섞어 읽지 않도록 (시작, 마지막) 쌍을 한 번에 바꾼다.
    """

    def __init__(self, hold_s: float):
        self._hold = hold_s
        self._run: tuple[float | None, float | None] = (None, None)    # (false 시작 stamp, 마지막 stamp)

    def update(self, steady: bool, t: float) -> bool:
        since, last = self._run
        if steady:
            since = None
        elif since is None or (last is not None and t < last):
            since = t
        self._run = (since, t)
        return self.unsteady()

    def unsteady(self) -> bool:
        since, last = self._run
        return since is not None and last - since > self._hold + 1e-9


class ClockMonitor:
    """ROS 시각 역행 검출. 멈춤(같은 값)은 역행이 아니다."""

    def __init__(self):
        self._last: float | None = None

    def update(self, t_ros: float) -> bool:
        back = self._last is not None and t_ros < self._last
        self._last = t_ros
        return back


class GripperMonitor:
    """오류 비트·응답 시간·상주 루프 생존(steady 시계).

    생존은 카운터(`alive_counter`, 멈춘 시간 > alive_stall_s) 또는 controller가 판정한 `alive`(RobotStatus.gripper_alive)로 받는다.
    """

    def __init__(self, response_max_s: float, alive_stall_s: float, clock: SteadyClock):
        self._response_max = response_max_s
        self._stall = alive_stall_s
        self._clock = clock
        self._error_bits = 0
        self._response_s = 0.0
        self._counter: int | None = None
        self._counter_t: float | None = None
        self._alive = True
        self.reason = ''

    def update(self, error_bits: int, response_s: float, alive_counter: int | None = None,
               alive: bool = True) -> None:
        self._error_bits = error_bits
        self._response_s = response_s
        self._alive = alive
        if alive_counter is not None and alive_counter != self._counter:
            self._counter = alive_counter
            self._counter_t = self._clock.steady_now()

    def fault(self) -> bool:
        if self._error_bits != 0:
            self.reason = 'GRIPPER_ERROR'
        elif self._response_s > self._response_max + 1e-9:
            self.reason = 'GRIPPER_SLOW'
        elif not self._alive or (self._counter_t is not None
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


def cycle_watch_events(state: State, released: bool, *, clock_back: bool = False, gripper_fault: bool = False,
                       belt_unsteady: bool = False, obs_stale: bool = False) -> list[Event]:
    """감시 결과 중 지금 단계에서 latch할 이벤트(SPEC-110 4.3절 적용 구간).

    - 시각 역행·그리퍼: `WATCH_STATES`(PRECHECK 다음 ~ VERIFY).
    - 벨트 불안정(지속 시간은 `BeltSteadyMonitor`): 동적 단계, 해제 뒤 RETREAT·VERIFY. WAIT_CAP은 판정이 보류한다.
    - 관측 신선도: 동적 단계, 해제 전.
    """
    if state not in WATCH_STATES:
        return []
    events = []
    if clock_back:
        events.append(Event.CLOCK_JUMP)
    if gripper_fault:
        events.append(Event.GRIPPER_FAULT)
    if belt_unsteady and (state in DYNAMIC_STATES or released):
        events.append(Event.BELT_UNSTEADY)
    if obs_stale and state in DYNAMIC_STATES and not released:
        events.append(Event.OBS_STALE)
    return events
