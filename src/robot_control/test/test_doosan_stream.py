"""Doosan speedl 스트림 해제 transport(실기 미검증, SPEC-500 REQ-ROB-015, TEST_SPEC.md TC-ROB-015-01).

가짜 발행 함수·메시지 형식·TCP·그리퍼 세션만 쓴다. `dsr_msgs2`와 `/dsr01`에 연결하지 않는다.
"""
import math
from dataclasses import dataclass, field

import pytest

from robot_control.doosan_stream import DoosanSpeedlTransport
from robot_control.dynamic_place import MotionRef, ReleasePermit

from fakes.clock import FakeClock

pytestmark = [pytest.mark.level('U'), pytest.mark.req('REQ-ROB-015'), pytest.mark.tc('TC-ROB-015-01')]


@dataclass
class SpeedlStream:                       # dsr_msgs2/msg/SpeedlStream과 같은 필드
    vel: list = field(default_factory=list)
    acc: list = field(default_factory=list)
    time: float = 0.0


class Session:
    def __init__(self):
        self.armed = False
        self.calls = []

    def arm_open(self, width_m, current_limit_ma=None):
        self.calls.append(('arm', width_m))
        self.armed = True

    def fire_open(self):
        self.calls.append(('fire', None))
        self.armed = False
        return type('R', (), {'width_m': 0.0219})()


class Deferred:
    def __init__(self):
        self.jobs = []

    def __call__(self, fn):
        self.jobs.append(fn)

    def run(self):
        for fn in self.jobs:
            fn()


def rig(**kw):
    clock = FakeClock()
    sent = []
    session = Session()
    runner = Deferred()
    tcp = {'pos': (0.5, 0.0, 0.2), 'vel': (0.02, 0.0, 0.0)}
    t = DoosanSpeedlTransport(sent.append, SpeedlStream, clock,
                              lambda: (clock.ros_now(), tcp['pos'], tcp['vel']), session,
                              runner=runner, **kw)
    return t, clock, sent, session, runner, tcp


def ref(clock, cmd=(0.02, 0.0, -0.005), ff=(0.02, 0.0, 0.0)):
    return MotionRef(clock.ros_now(), (0.5, 0.0, 0.2), ff, cmd)


def permit(clock, lead=0.15):
    now = clock.ros_now()
    return ReleasePermit(now, now + lead, (0.5, 0.0, 0.2), (0.02, 0.0, 0.0), 0.022, 0.3, 0.252, 0.05, {})


def test_message_format_uses_motion_adapter_units():
    t, clock, sent, *_ = rig()
    assert t.update(ref(clock))
    m = sent[-1]
    assert m.vel == pytest.approx([20.0, 0.0, -5.0, 0.0, 0.0, 0.0])
    assert m.acc == pytest.approx([300.0, 60.0])                 # a_max 0.3 m/s², 회전 60 deg/s²
    assert m.time == pytest.approx(0.02)                         # 지령이 끊기면 2주기 뒤 스스로 멈춤
    assert t.capabilities.mid_course_update and t.capabilities.keep_sync_retreat


@pytest.mark.parametrize('gap_s,kept', [(0.019999, True), (0.020, True), (0.020001, False)])
def test_freshness_latches_l0_with_single_zero(gap_s, kept):
    t, clock, sent, *_ = rig()
    assert t.update(ref(clock))
    clock.advance(gap_s)
    assert t.update(ref(clock)) is kept
    if not kept:
        assert sent[-1].vel == [0.0] * 6 and len(sent) == 2   # 속도 0 1회
        clock.advance(0.005)
        assert not t.update(ref(clock)) and len(sent) == 2    # 재무장 전까지 거부
        assert t.poll().fault == 'STREAM_STALE'
        t.rearm()
        assert t.update(ref(clock)) and len(sent) == 3


def test_tick_detects_silence_and_sends_zero_once():
    t, clock, sent, *_ = rig()
    t.update(ref(clock))
    clock.advance(0.025)
    t.tick()
    t.tick()
    assert [m.vel for m in sent[1:]] == [[0.0] * 6]


def test_range_violation_not_sent():
    t, clock, sent, *_ = rig(v_max_mps=0.05)
    assert not t.update(ref(clock, cmd=(0.06, 0.0, 0.0)))
    assert [m.vel for m in sent] == [[0.0] * 6]
    assert t.poll().fault == 'RANGE'
    assert not t.update(ref(clock))
    assert not t.update(ref(clock, cmd=(math.nan, 0.0, 0.0)))


def test_commit_fires_open_once_off_the_control_loop():
    t, clock, sent, session, runner, _ = rig()
    t.prepare(0.022)
    assert session.calls == [('arm', 0.022)]
    assert t.commit_release(permit(clock))
    assert not t.commit_release(permit(clock))                   # goal당 1회
    assert session.calls == [('arm', 0.022)] and len(runner.jobs) == 1
    s = t.poll()
    assert s.committed and not s.released                         # 트리거 결과 전
    runner.run()
    assert session.calls[-1] == ('fire', None)
    s = t.poll()
    assert s.released and s.gripper_width_m == pytest.approx(0.0219)
    assert not t.late_arm


def test_commit_without_prepare_arms_late():
    t, clock, _, session, runner, _ = rig()
    assert t.commit_release(permit(clock))
    runner.run()
    assert [c[0] for c in session.calls] == ['arm', 'fire'] and t.late_arm


def test_abort_keep_sync_streams_plan_velocity_then_stops():
    t, clock, sent, *_ = rig(abort_sync_hold_s=0.05)
    t.update(ref(clock, cmd=(0.021, 0.001, -0.01)))
    assert t.abort(True)
    vels = []
    for _ in range(8):
        clock.advance(0.01)
        t.tick()
        vels.append(sent[-1].vel)
    assert vels[0] == pytest.approx([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # XY 계획 속도, z 0
    assert vels[-1] == [0.0] * 6
    assert sum(1 for m in sent if m.vel == [0.0] * 6) == 1


def test_stop_and_feedback():
    t, clock, sent, _, _, tcp = rig()
    t.update(ref(clock))
    assert t.stop() and sent[-1].vel == [0.0] * 6
    assert not t.is_stopped()                                     # 피드백 속도가 아직 0이 아니다
    tcp['vel'] = (0.0, 0.0, 0.0)
    assert t.is_stopped()
    assert not t.update(ref(clock))


def test_module_does_not_import_ros():
    import sys
    assert 'dsr_msgs2' not in sys.modules
