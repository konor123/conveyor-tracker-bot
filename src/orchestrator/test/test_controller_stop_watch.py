"""controller 정지 유지·보호정지 감시(SPEC-100 REQ-ORC-012, SPEC-010 4.5절, TEST_SPEC TC-ORC-012-01).

감시 노드(safety_supervisor)가 skill이 없는 구간에 `Stop`을 보내면 로봇은 정지를 유지하고 heartbeat
`error_code` 101을 낸다. orchestrator는 이 코드를 감시 주기마다 latch해 어느 상태에서든 L4로 끝낸다.
"""
from types import SimpleNamespace as NS

import pytest
from fakes.skills import FakeSkillPort
from orchestrator.abort_policy import Event
from orchestrator.common import AbortLevel, Mode, State
from orchestrator.cycle_runner import CycleRunner, DropGoal, OrchestratorCfg
from test_node_failure_regressions import load, port

pytestmark = [pytest.mark.level('U'), pytest.mark.req('REQ-ORC-012'), pytest.mark.tc('TC-ORC-012-01')]

CTRL = 'robot_controller_node'
ROBOT_SKILLS = ('PickStone', 'MoveToPose', 'PlaceStone', 'DynamicPlace', 'SafeRetreat', 'feedback.', 'result.')


def node(p):
    cls = load('orchestrator_node.py', ['OrchestratorNode'])['OrchestratorNode']
    n = NS(hb=NS(beat=lambda name: None, lost=lambda name: False), lifecycle={}, hb_error_code={},
           robot_status=NS(protective_stop=False, connected=True, ready=False),
           node_cfg={'required_nodes': [CTRL]}, port=p, CONTROLLER_NODE=cls.CONTROLLER_NODE,
           ERROR_PROTECTIVE_STOP=cls.ERROR_PROTECTIVE_STOP, ERROR_STOP_LATCHED=cls.ERROR_STOP_LATCHED)
    return cls, n


def beat(cls, n, code, name=CTRL):
    cls._on_hb(n, NS(node_name=name, lifecycle_state=3, error_code=code, detail=''))
    cls._watch(n)


@pytest.mark.parametrize('code,want', [(100, Event.PROTECTIVE_STOP), (101, Event.ROBOT_STOPPED), (0, None)])
def test_controller_heartbeat_code_is_latched(code, want):
    p = port()
    cls, n = node(p)
    beat(cls, n, code)
    assert p._fault is want


def test_other_node_codes_are_ignored():
    p = port()
    cls, n = node(p)
    beat(cls, n, 101, name='vision_node')
    assert p._fault is None


def test_protective_stop_is_kept_when_latch_code_follows():
    p = port()
    cls, n = node(p)
    beat(cls, n, 100)
    beat(cls, n, 101)
    assert p._fault is Event.PROTECTIVE_STOP


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
