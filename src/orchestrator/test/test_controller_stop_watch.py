"""controller 정지 유지·보호정지 감시(SPEC-100 REQ-ORC-012, SPEC-010 4.5절, TEST_SPEC TC-ORC-012-01·02).

감시 노드(safety_supervisor)가 skill이 없는 구간에 `Stop`을 보내면 로봇은 정지를 유지하고 heartbeat
`error_code` 101을 낸다. orchestrator는 이 코드를 감시 주기마다 latch해 어느 상태에서든 L4로 끝낸다.
"""
import math
import threading
from types import SimpleNamespace as NS

import pytest
from fakes.skills import FakeSkillPort
from orchestrator.abort_policy import Event
from orchestrator.common import AbortLevel, Mode, State
from orchestrator.cycle_runner import CycleRunner, DropGoal, OrchestratorCfg
from orchestrator.watchdogs import (BeltSteadyMonitor, ClockMonitor, FreshnessMonitor, GripperMonitor,
                                    cycle_watch_events)
from test_node_failure_regressions import load, port

pytestmark = [pytest.mark.level('U'), pytest.mark.req('REQ-ORC-012')]

CTRL = 'robot_controller_node'
ROBOT_SKILLS = ('PickStone', 'MoveToPose', 'PlaceStone', 'DynamicPlace', 'SafeRetreat', 'feedback.', 'result.')
ACCEPT, REJECT = 'ACCEPT', 'REJECT'
NODE_ENV = dict(cycle_watch_events=cycle_watch_events, _t=lambda st: st, math=math, State=State,
                AbortLevel=AbortLevel, DropGoal=lambda **kw: NS(**kw), LcState=NS(PRIMARY_STATE_ACTIVE=3),
                FRESH_CONTROLLER_HEARTBEATS=2, RESULT_NAMES=('SUCCESS', 'FAILURE', 'UNKNOWN', 'CANCELED',
                                                            'SAFE_ABORT', 'PROTECTIVE_STOP', 'NOT_ATTEMPTED'),
                GoalResponse=NS(ACCEPT=ACCEPT, REJECT=REJECT), DropTask=NS(Result=lambda **kw: NS(**kw)),
                Parameter=lambda *a, **kw: None)


def node(p, cls=None):
    cls = cls or load('orchestrator_node.py', ['OrchestratorNode'], **NODE_ENV)['OrchestratorNode']
    clock = NS(ros_now=lambda: 0.0, steady_now=lambda: 0.0)
    n = NS(hb=NS(beat=lambda name: None, lost=lambda name: False), lifecycle={}, hb_error_code={},
           robot_status=NS(protective_stop=False, connected=True, ready=False),
           node_cfg={'required_nodes': [CTRL], 'startup_wait_s': 3.0}, port=p, CONTROLLER_NODE=cls.CONTROLLER_NODE,
           ERROR_PROTECTIVE_STOP=cls.ERROR_PROTECTIVE_STOP, ERROR_STOP_LATCHED=cls.ERROR_STOP_LATCHED,
           state=State.IDLE, clock=clock, tracked=None, ctrl_hb_count=0, status_count=0, _accepted_counts=(0, 0),
           fresh=FreshnessMonitor(0.3), belt_watch=BeltSteadyMonitor(1.0), clock_watch=ClockMonitor(),
           gripper_watch=GripperMonitor(0.5, 0.1, clock), _obs=(None, None))
    n._obs_stale = lambda: cls._obs_stale(n)
    return cls, n


def beat(cls, n, code, name=CTRL):
    cls._on_hb(n, NS(node_name=name, lifecycle_state=3, error_code=code, detail=''))
    cls._watch(n)


@pytest.mark.tc('TC-ORC-012-01')
@pytest.mark.parametrize('code,want', [(100, Event.PROTECTIVE_STOP), (101, Event.ROBOT_STOPPED), (0, None)])
def test_controller_heartbeat_code_is_latched(code, want):
    p = port()
    cls, n = node(p)
    beat(cls, n, code)
    assert p._fault is want


@pytest.mark.tc('TC-ORC-012-01')
def test_other_node_codes_are_ignored():
    p = port()
    cls, n = node(p)
    beat(cls, n, 101, name='vision_node')
    assert p._fault is None


@pytest.mark.tc('TC-ORC-012-01')
def test_protective_stop_is_kept_when_latch_code_follows():
    p = port()
    cls, n = node(p)
    beat(cls, n, 100)
    beat(cls, n, 101)
    assert p._fault is Event.PROTECTIVE_STOP


@pytest.mark.tc('TC-ORC-012-01')
def test_latch_code_upgrades_to_protective_stop():
    p = port()
    cls, n = node(p)
    beat(cls, n, 101)
    beat(cls, n, 100)
    assert p._fault is Event.PROTECTIVE_STOP


class StopAfter(FakeSkillPort):
    """`trigger` 호출 뒤부터 감시가 ROBOT_STOPPED를 계속 보고한다(정지 유지는 풀리지 않는다)."""

    def __init__(self, clock, trigger):
        super().__init__(clock)
        self.trigger, self.fault, self.t_index = trigger, None, None

    def _call(self, key, deadline, **kw):
        r = super()._call(key, deadline, **kw)
        if key == self.trigger and self.fault is None:
            self.fault, self.t_index = Event.ROBOT_STOPPED, len(self.calls)
        return r

    def pending_fault(self):
        return self.fault


@pytest.fixture
def cfg(orch_cfg):
    return OrchestratorCfg.from_dict(orch_cfg)


@pytest.mark.tc('TC-ORC-012-01')
@pytest.mark.parametrize('mode', [Mode.STATIC, Mode.SYNCHRONIZED])
@pytest.mark.parametrize('trigger,where', [
    ('status', 'PRECHECK'),                 # 해제하지 않은 채 시작한 사이클
    ('DetectObjects.MAT', 'DETECT_STONE'),  # 검출 서비스 대기 중
    ('SelectTarget', 'WAIT_CAP'),           # 병뚜껑 선택·관측 대기 중
    ('PickStone', 'PICK_STONE'),            # 단계 사이(파지 성공 뒤, 다음 skill 전)
])
def test_robot_stopped_without_running_skill_ends_in_l4(fake_clock, cfg, mode, trigger, where):
    if mode is Mode.STATIC and trigger == 'SelectTarget':
        trigger = 'DetectObjects.BELT'
    p = StopAfter(fake_clock, trigger)
    out = CycleRunner(p, fake_clock, cfg).run(DropGoal(mode=mode, max_belt_speed_mps=0.06, timeout_s=90.0,
                                                        max_passes=2))
    assert p.t_index is not None, p.names()
    assert out.abort_level is AbortLevel.L4
    assert out.states[-2:] == [State.PROTECTIVE_STOP, State.WAIT_OPERATOR]
    assert out.records[-1].step_name == (where if where != 'WAIT_CAP' else 'WAIT_CAP')
    assert out.records[-1].reason_code == 'ROBOT_STOPPED'
    after = [k for k, _ in p.calls[p.t_index:]]
    assert not [k for k in after if k.startswith(ROBOT_SKILLS)], after
    assert after[-1] == 'Stop'


# ---------------------------------------------------------------- Stop 해제 직후 goal(TC-ORC-012-02)
class _StartTime:
    """`_run_cycle` 시작 대기용 가짜 time 모듈. sleep마다 감시 타이머(_watch)가 돌고 대본의 다음 수신이 들어온다."""

    def __init__(self, on_sleep):
        self.t = 0.0
        self.on_sleep = on_sleep

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += dt
        self.on_sleep()

    def strftime(self, fmt):
        return '20261113-000000'


def _start_cycle(fresh_codes):
    """정지 유지를 막 푼 순간: 캐시의 마지막 controller heartbeat는 101이다. goal을 수락한 뒤 받는 heartbeat
    `error_code` 열이 fresh_codes일 때, 실행기가 PRECHECK에서 처음 묻는 비동기 장애를 돌려준다."""
    env = load('orchestrator_node.py', ['OrchestratorNode', 'RosSkillPort'], **NODE_ENV)
    cls = env['OrchestratorNode']
    cls_, n = node(None, cls)
    n.port = env['RosSkillPort'](n)
    n.cfg_dict = {'watchdog': {'cycle_steady_factor_ratio': 2.0}}
    n.get_logger = lambda: NS(warning=lambda m: None, info=lambda m: None)
    n.robot_status = NS(protective_stop=False, connected=True, ready=True, gripper_error_bits=0,
                        gripper_response_s=0.1, gripper_alive=True, holding=False)
    n.belt = n.detections = object()
    n.lifecycle, n.hb_error_code = {CTRL: 3}, {CTRL: 101}
    n.cycle_count, n._busy = 0, threading.Lock()
    n.set_parameters = n._publish_status = lambda *a: None
    n._watch = lambda: cls._watch(n)
    seen = []
    n.runner = NS(run=lambda goal: seen.append((n.port.pending_fault(), env['time'].t)) or NS(
        abort_level=AbortLevel.NONE, states=[], records=[], result='NOT_ATTEMPTED', reason_code='',
        passes_skipped=0, cycle_time_s=0.0))
    feed = iter(fresh_codes)

    def on_sleep():
        cls._watch(n)                                        # 20 Hz 감시 타이머
        code = next(feed, None)
        if code is not None:                                 # 10 Hz controller heartbeat·robot/status
            cls._on_hb(n, NS(node_name=CTRL, lifecycle_state=3, error_code=code))
            cls._on_status(n, n.robot_status)
    env['time'] = _StartTime(on_sleep)
    assert cls._on_goal(n, None) == ACCEPT
    cls._run_cycle(n, NS(request=NS(mode=int(Mode.SYNCHRONIZED), max_belt_speed=0.05, timeout=30.0, max_passes=1),
                         is_cancel_requested=False, succeed=lambda: None, canceled=lambda: None))
    return seen[0]                                           # (PRECHECK 앞 장애, 실행기 시작 시각)


@pytest.mark.tc('TC-ORC-012-02')
def test_stale_stop_latched_heartbeat_does_not_end_released_cycle():
    """해제 직후 goal: 캐시의 101은 낡은 값이다. 수락 뒤 새 heartbeat(0)를 받은 다음 장애 latch를 무장한다."""
    fault, waited = _start_cycle([0, 0, 0])
    assert fault is None
    assert waited <= 0.2 + 1e-9                              # 10 Hz heartbeat 2개, startup_wait_s(3 s)보다 짧다


@pytest.mark.tc('TC-ORC-012-02')
def test_fresh_stop_latched_heartbeat_still_ends_in_l4():
    """새 heartbeat도 101이면 정지 유지가 풀리지 않은 것이다. PRECHECK에서 ROBOT_STOPPED(L4)."""
    fault, _ = _start_cycle([101, 101, 101])
    assert fault is Event.ROBOT_STOPPED


@pytest.mark.tc('TC-ORC-012-02')
def test_start_wait_is_bounded_without_fresh_heartbeat():
    """controller heartbeat가 오지 않으면 startup_wait_s까지만 기다린다. 판정은 PRECHECK(heartbeat 나이)가 한다."""
    fault, waited = _start_cycle([])
    assert 3.0 - 1e-9 <= waited <= 3.0 + 0.05 + 1e-9         # 대기 주기 0.05 s
