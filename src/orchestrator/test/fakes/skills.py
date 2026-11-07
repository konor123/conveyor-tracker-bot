"""FakeSkillPort(TEST_SPEC.md 1절): skill 호출을 대본대로 응답한다.

대본 키(호출 기록 이름과 같다):
  status, DetectObjects.MAT, PickStone, MoveToPose.standby, MoveToPose.safe, SelectTarget,
  DetectObjects.BELT, observe_cap, ComputeIntercept, DynamicPlace, PlaceStone,
  feedback.SYNCHRONIZE, feedback.DESCEND, feedback.RELEASE, result.RETREAT, VerifyDrop,
  SafeRetreat, Stop, cancel
키마다 Reply 열을 두고 차례로 쓴다. 열을 다 쓰면 마지막 Reply를 반복한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from orchestrator.cycle_runner import CapObservation, DetectResult

from . import measures as M


@dataclass
class Reply:
    value: Any = None              # 응답 값(측정값). callable이면 port를 받아 값을 만든다
    duration: float = 0.1          # 소요시간 [s]
    hang: bool = False             # 응답 없음(deadline까지 시간만 흐름)


def default_script(clock) -> dict[str, list[Reply]]:
    return {
        'status': [Reply(M.OK_PRE, 0.0)],
        'DetectObjects.MAT': [Reply(lambda port: DetectResult(M.OK_DET, port.clock.ros_now(),
                                                              (0.3, 0.0, 0.0)), 0.3)],
        'PickStone': [Reply(M.OK_PICK, 4.0)],
        'MoveToPose.standby': [Reply(M.OK_STB, 2.0)],
        'MoveToPose.safe': [Reply(True, 2.0)],
        'SelectTarget': [Reply(lambda port: CapObservation(M.OK_WAIT, port.clock.ros_now() + 8.0), 0.05)],
        'DetectObjects.BELT': [Reply(lambda port: CapObservation(M.OK_WAIT, None), 0.3)],
        'observe_cap': [Reply(lambda port: CapObservation(M.OK_WAIT, port.clock.ros_now() + 8.0), 0.1)],
        'ComputeIntercept': [Reply(lambda port: _plan_ok(port), 0.05)],
        'DynamicPlace': [Reply(M.OK_APP, 1.0)],
        'PlaceStone': [Reply(M.OK_APP, 1.0)],
        'feedback.SYNCHRONIZE': [Reply(lambda port: M.sync_samples(t0=port.clock.ros_now()), 0.3)],
        'feedback.DESCEND': [Reply(lambda port: M.OK_STATIC if port.mode == 'STATIC' else M.OK_DESC, 0.8)],
        'feedback.RELEASE': [Reply(M.OK_REL, 0.3)],
        'result.RETREAT': [Reply(M.OK_RET, 1.0)],
        'VerifyDrop': [Reply(M.OK_VER, 1.0)],
        'SafeRetreat': [Reply(True, 3.0)],
        'Stop': [Reply(True, 0.0)],
        'cancel': [Reply(True, 0.05)],
    }


def _plan_ok(port):
    from dataclasses import replace
    now = port.clock.ros_now()
    return replace(M.OK_PLAN, now=now, valid_until=now + 5.0)


class FakeSkillPort:
    def __init__(self, clock, script: dict[str, list[Reply]] | None = None, mode_entry=None):
        self.clock = clock
        self.script = default_script(clock)
        if mode_entry is not None:
            self.script['feedback.SYNCHRONIZE'] = [Reply(mode_entry, 0.3)]
        self.script.update(script or {})
        self._idx: dict[str, int] = {}
        self.calls: list[tuple[str, dict]] = []
        self.cancel_flag = False
        self.mode = ''                 # place_start에서 받은 모드

    # ------------------------------------------------------------ 공통
    def names(self) -> list[str]:
        return [c[0] for c in self.calls]

    def _reply(self, key: str) -> Reply:
        seq = self.script[key]
        i = self._idx.get(key, 0)
        self._idx[key] = i + 1
        return seq[min(i, len(seq) - 1)]

    def _call(self, key: str, deadline: float | None, **kw):
        self.calls.append((key, kw))
        r = self._reply(key)
        now = self.clock.ros_now()
        if r.hang or (deadline is not None and now + r.duration > deadline):
            if deadline is not None:
                self.clock.advance(max(0.0, deadline - now))
            return None
        self.clock.advance(r.duration)
        return r.value(self) if callable(r.value) else r.value

    # ------------------------------------------------------------ SkillPort
    def status(self):
        return self._call('status', None)

    def detect_stones(self, deadline):
        return self._call('DetectObjects.MAT', deadline, region='MAT', object_class='STONE')

    def pick_stone(self, target, deadline):
        return self._call('PickStone', deadline, target=target)

    def move_to_pose(self, name, deadline):
        return self._call(f'MoveToPose.{name}', deadline, name=name)

    def select_target(self, pass_index, deadline):
        return self._call('SelectTarget', deadline, pass_index=pass_index)

    def detect_cap(self, deadline):
        return self._call('DetectObjects.BELT', deadline, region='BELT', object_class='CAP')

    def observe_cap(self, deadline):
        return self._call('observe_cap', deadline)

    def compute_intercept(self, deadline):
        return self._call('ComputeIntercept', deadline)

    def place_start(self, mode, deadline):
        self.mode = mode.name
        key = 'PlaceStone' if mode.name == 'STATIC' else 'DynamicPlace'
        return self._call(key, deadline, mode=mode.name)

    def place_feedback(self, phase, deadline):
        key = 'result.RETREAT' if phase == 'RETREAT' else f'feedback.{phase}'
        return self._call(key, deadline)

    def verify(self, deadline):
        return self._call('VerifyDrop', deadline)

    def safe_retreat(self, keep_sync, place_back, deadline):
        return self._call('SafeRetreat', deadline, keep_sync=keep_sync, place_back=place_back,
                          place_back_pose='mat')

    def stop(self):
        return self._call('Stop', None)

    def cancel_active(self, wait_s):
        r = self._reply('cancel')
        self.calls.append(('cancel', {}))
        if r.hang:
            self.clock.advance(wait_s)
            return False
        self.clock.advance(min(r.duration, wait_s))
        return True

    def cancel_requested(self):
        return self.cancel_flag
