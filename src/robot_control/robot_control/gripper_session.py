"""static·dynamic 공통 그리퍼 순서(REQ-GRP-014). 순수 Python, 지령은 `GripperAdapter`로만 보낸다.

통신 왕복을 줄이는 규칙 네 가지를 한곳에 둔다.
  1. 시작 확인과 상주 루프 기동은 세션당 1회(`start`).
  2. 이미 목표 폭이면 지령을 보내지 않는다(`ensure_open`). 판단은 기억한 값이 아니라 방금 읽은 폭으로 한다.
  3. 목표를 미리 써 두고(`arm_open`) 정한 순간에 트리거만 보낸다(`fire_open`).
  4. 지령마다 시각과 생략 여부를 `events`에 남긴다.
static은 팔이 도착했을 때, dynamic은 해제 게이트를 통과했을 때 `fire_open`을 부른다.
"""
from __future__ import annotations

from dataclasses import dataclass

from robot_control.gripper_adapter import GripperAdapter, GripResult, OpenResult, PreconditionError


@dataclass(frozen=True)
class GripperEvent:
    name: str                          # ensure_open / fire_open / grasp
    skipped: bool
    width_m: float | None
    t_cmd: float | None = None
    t_done: float | None = None


class GripperSession:
    def __init__(self, adapter: GripperAdapter, width_tolerance_m: float):
        if width_tolerance_m < 0.0:
            raise ValueError('width_tolerance_m은 음수일 수 없다')
        self.adapter = adapter
        self.width_tolerance_m = float(width_tolerance_m)
        self.events: list[GripperEvent] = []
        self._started = False
        self._armed: tuple[float, int | None] | None = None

    @property
    def started(self) -> bool:
        return self._started

    @property
    def armed(self) -> bool:
        return self._armed is not None

    def start(self, robot_standby: bool) -> None:
        """상주 루프 기동 → 시작 확인. 이미 시작했으면 아무것도 보내지 않는다."""
        if self._started:
            return
        self.adapter.start_resident_loop(robot_standby)
        self.adapter.startup_check(robot_standby)
        self._started = True

    def ensure_open(self, width_m: float, current_limit_ma: int | None = None) -> OpenResult | None:
        """방금 읽은 폭이 목표 ± 허용오차면 지령 없이 None, 아니면 연다."""
        self._require_started()
        state = self.adapter.read_state()
        if abs(state.width_m - width_m) <= self.width_tolerance_m:
            self.events.append(GripperEvent('ensure_open', True, state.width_m, state.t_read, state.t_read))
            return None
        self._armed = None
        result = self.adapter.open(width_m, current_limit_ma)
        self.events.append(GripperEvent('ensure_open', False, result.width_m, result.t_cmd, result.t_done))
        return result

    def arm_open(self, width_m: float, current_limit_ma: int | None = None) -> None:
        """목표값만 미리 쓴다. 그리퍼는 움직이지 않는다. 팔이 이동하기 전에 부른다."""
        self._require_started()
        self.adapter.prewrite_open(width_m, current_limit_ma)
        self._armed = (float(width_m), current_limit_ma)

    def fire_open(self) -> OpenResult:
        """예약한 열기를 트리거 1회로 실행한다. 예약이 없으면 보내지 않는다."""
        self._require_started()
        if self._armed is None:
            raise PreconditionError('NOT_ARMED')
        width_m, current = self._armed
        self._armed = None
        result = self.adapter.open(width_m, current, prewritten=True)
        self.events.append(GripperEvent('fire_open', False, result.width_m, result.t_cmd, result.t_done))
        return result

    def grasp(self, current_limit_ma: int) -> GripResult:
        """닫아 쥔다. 목표 레지스터를 덮어쓰므로 앞선 예약은 무효가 된다."""
        self._require_started()
        self._armed = None
        result = self.adapter.close(current_limit_ma)
        self.events.append(GripperEvent('grasp', False, result.width_m, result.t_cmd, result.t_done))
        return result

    def summary(self) -> dict:
        sent = [e for e in self.events if not e.skipped]
        busy = sum(e.t_done - e.t_cmd for e in sent if e.t_cmd is not None and e.t_done is not None)
        return {'commands': len(sent), 'skipped': len(self.events) - len(sent), 'busy_s': busy}

    def _require_started(self) -> None:
        if not self._started:
            raise PreconditionError('SESSION_NOT_STARTED')
