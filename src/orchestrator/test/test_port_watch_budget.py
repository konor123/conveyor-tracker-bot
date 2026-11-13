"""RosSkillPort·노드 구독의 감시·시간 예산 연결(ROS 없이 노드 코드를 실행한다).

- PRECHECK의 벨트 불안정은 지속 시간 규칙(SPEC-110 REQ-ABT-013, TC-ABT-013-01).
- ROS 시각 역행 뒤 이전 대상·계획 재사용 거부(REQ-ABT-012, TC-ABT-012-03).
- WAIT_CAP 시간 예산 = `intercept_planner.time_budget.evaluate_budget`, 동적 deadline = `deadlines.dynamic_deadline`
  (SPEC-100 REQ-ORC-019, TC-ORC-019-01).
"""
import math
from types import SimpleNamespace as NS

import pytest
from intercept_planner.time_budget import evaluate_budget
from orchestrator import step_judges as sj
from orchestrator.common import Mode, Result
from orchestrator.cycle_runner import CapObservation, OrchestratorCfg
from orchestrator.deadlines import dynamic_deadline
from orchestrator.watchdogs import BeltSteadyMonitor
from test_node_failure_regressions import load

ENV = dict(LcState=NS(PRIMARY_STATE_ACTIVE=3), sj=sj, math=math, TrackedObject=NS(OBSERVED=0, COASTING=1, LOST=2),
           ObjectDetection=NS(CLASS_CAP=2, CLASS_STONE=1, REGION_BELT=2, REGION_MAT=1),
           evaluate_budget=evaluate_budget, dynamic_deadline=dynamic_deadline, CapObservation=CapObservation,
           SelectTarget='SelectTarget', ComputeIntercept='ComputeIntercept', _t=lambda x: x,
           DynamicPlace=NS(Goal=lambda **kw: NS(**kw)))
REQUIRED = ['vision_node', 'tracker_node', 'robot_controller_node']


class _Clock:
    def __init__(self):
        self.t = 100.0

    def ros_now(self):
        return self.t

    def steady_now(self):
        return self.t


def _node(orch_cfg):
    env = load('orchestrator_node.py', ['RosSkillPort', 'OrchestratorNode'], **ENV)
    clock = _Clock()
    n = NS(clock=clock, cycle_id='c', step_id=1, cfg=OrchestratorCfg.from_dict(orch_cfg), cfg_dict=orch_cfg,
           budget=dict(orch_cfg['budget']), node_cfg=orch_cfg['node'], tracked=None, tracked_seq=0,
           belt=None, detections=None, robot_status=None, lifecycle={r: 3 for r in REQUIRED},
           hb=NS(age=lambda name: 0.05), get_parameter=lambda name: NS(value=''),
           get_logger=lambda: NS(warning=lambda m: None), _watch=lambda: None,
           belt_watch=BeltSteadyMonitor(float(orch_cfg['watchdog']['belt_unsteady_hold_s'])))
    port = env['RosSkillPort'](n)
    port.reset(NS(is_cancel_requested=False))
    port._mode_goal = Mode.SYNCHRONIZED
    return env['OrchestratorNode'], n, port


def _track(x=0.17, v=0.02, track_id=7, epoch=1):
    return NS(track_id=track_id, epoch=epoch, status=0, observation_count=40, position=NS(x=x),
              velocity=NS(x=v), last_observed_stamp=0.0)


def _tracking(port, calls):
    """SelectTarget·ComputeIntercept 서비스 대역. 호출 이름을 calls에 남긴다."""
    n = port.n

    def call(typ, name, deadline, **kw):
        calls.append(name)
        if typ == 'SelectTarget':
            return NS(result=0, track_id=7, epoch=1)
        plan = NS(plan_id=11, status=0, OK=0, target_epoch=kw['target_epoch'], time_margin=5.0, position_std=0.001,
                  valid_until=n.clock.t + 5.0)
        return NS(result=0, plan=plan)
    port._call = call
    n.tracked, n.tracked_seq = NS(tracks=[_track()]), 1
    n.belt = NS(steady=True, speed=0.02, speed_std=0.0002)
    n.detections = NS(detections=[NS(object_class=2, stone_inside=False, stone_inside_valid=True)])


# ---------------------------------------------------------------- PRECHECK 벨트 지속 규칙
@pytest.mark.req('REQ-ABT-013')
@pytest.mark.tc('TC-ABT-013-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('pulse_s, reason', [(0.5, ''), (1.000, ''), (1.001, 'BELT_UNSTEADY')])
def test_precheck_rejects_only_persistent_belt_unsteady(orch_cfg, pulse_s, reason):
    """PRECHECK도 1.0 s 지속 규칙을 쓴다. 맥동 경보 한 번(0.5 s)으로 시도를 포기(NOT_ATTEMPTED)하지 않는다."""
    cls, n, port = _node(orch_cfg)
    n.robot_status = NS(ready=True, protective_stop=False, gripper_alive=True, gripper_response_s=0.1,
                        gripper_error_bits=0, gripper_open=True)
    n.detections = NS(calibration_id='mock')
    t0 = 1000.0
    for k in range(int(round(pulse_s * 1000)) + 1):        # 1 ms 간격 BeltMotion(steady=false)
        cls._on_belt(n, NS(steady=False, speed=0.02, header=NS(stamp=t0 + k * 1e-3)))
    j = sj.judge_precheck(port.status(), sj.PrecheckParams.from_dict(orch_cfg['judge']['precheck'],
                                                                      max_belt_speed_mps=0.05))
    assert j.reason_code == reason
    assert (j.result is Result.OK) is (reason == '')


# ---------------------------------------------------------------- 시각 역행 뒤 재사용 거부
@pytest.mark.req('REQ-ABT-012')
@pytest.mark.tc('TC-ABT-012-03')
@pytest.mark.level('U')
def test_clock_regression_refuses_old_target_and_plan(orch_cfg):
    cls, n, port = _node(orch_cfg)
    calls, sent = [], []
    _tracking(port, calls)
    port._send = lambda name, goal, deadline, goal_deadline=None: sent.append(name) and False
    assert port.select_target(1, deadline=110.0).target_lost is False
    assert port.compute_intercept(deadline=110.0).plan_ok
    port.place_start(Mode.SYNCHRONIZED, deadline=110.0)
    assert sent == ['robot/dynamic_place']                  # 역행 전: 계획을 보낸다(대조군)

    port.invalidate_clock()                                 # ROS 시각 역행 1회
    assert port.clock_epoch == 1
    assert port._cap_obs().target_lost                      # 이전 대상은 소실로 본다(같은 통과에서 다시 고른다)
    assert port.compute_intercept(deadline=110.0) is None   # 이전 대상으로 계획하지 않는다
    assert port.place_start(Mode.SYNCHRONIZED, deadline=110.0) is None
    assert calls == ['tracking/select_target', 'planner/compute_intercept']
    assert sent == ['robot/dynamic_place']                  # 이전 계획을 보내지 않는다

    assert port.select_target(1, deadline=110.0).target_lost is False   # 역행 뒤 새로 고른 대상은 쓴다
    assert port.compute_intercept(deadline=110.0).plan_ok
    port.place_start(Mode.SYNCHRONIZED, deadline=110.0)
    assert sent == ['robot/dynamic_place'] * 2


# ---------------------------------------------------------------- 시간 예산 단일 출처
@pytest.mark.req('REQ-ORC-019')
@pytest.mark.tc('TC-ORC-019-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('x, v, reason', [(0.17, 0.02, ''), (0.0, 0.0, 'BELT_STOPPED'), (0.1, -0.02, 'BELT_REVERSED'),
                                          (0.39, 0.02, 'TIME_MARGIN')])
def test_wait_cap_budget_comes_from_time_budget(orch_cfg, x, v, reason):
    cls, n, port = _node(orch_cfg)
    _tracking(port, [])
    assert port.select_target(1, deadline=110.0) is not None
    n.tracked, n.tracked_seq = NS(tracks=[_track(x=x, v=v)]), 2
    obs = port._cap_obs()
    b = orch_cfg['budget']
    comps = {k[:-2]: v_ for k, v_ in b.items() if k.endswith('_s') and k != 'retreat_margin_s'}
    want = evaluate_budget(x, v, b['x_exit_m'], comps, orch_cfg['judge']['wait_cap']['margin_s'])
    assert want.reason == reason
    m = obs.meas
    assert (m.t_available_s, m.t_required_s, m.budget_components) == (want.t_available, want.t_required,
                                                                       want.components)
    assert m.t_required_s == pytest.approx(3.6)             # plan 0.5 + approach 1.5 + synchronize 0.5 + descend 0.8 + release 0.3
    assert obs.dynamic_deadline == pytest.approx(dynamic_deadline(100.0, want.t_available, b['retreat_margin_s']))
    j = sj.judge_wait_cap(m, n.cfg.wait_cap)
    if reason in ('BELT_STOPPED', 'BELT_REVERSED'):
        assert m.t_to_window_s == math.inf and j.detail == 'TOO_EARLY'    # 정지·역방향: 계획 시점이 오지 않는다
    elif reason == 'TIME_MARGIN':
        assert j.reason_code == sj.PASS_SKIP and j.detail == 'TIME_MARGIN'
    else:
        assert j.result is Result.OK
        assert obs.dynamic_deadline == pytest.approx(100.0 + 11.5 - 1.0)


# ---------------------------------------------------------------- 대상 관측 나이(노드 캐시)
@pytest.mark.req('REQ-ABT-011')
@pytest.mark.tc('TC-ABT-011-01')
@pytest.mark.level('U')
def test_target_observation_age_from_tracked_cache(orch_cfg):
    """대상 track의 last_observed_stamp로 나이를 잰다. 목록에서 빠지면 마지막 값, 대상이 바뀌면 새로 잰다."""
    from orchestrator.watchdogs import FreshnessMonitor
    cls, n, port = _node(orch_cfg)
    n.fresh, n._obs, n.port = FreshnessMonitor(0.3), (None, None), port
    assert not cls._obs_stale(n)                            # 대상 없음
    port.target = (7, 1)
    tr = _track()
    tr.last_observed_stamp = 100.0
    n.tracked = NS(tracks=[tr])
    n.clock.t = 100.300
    assert not cls._obs_stale(n)
    n.clock.t = 100.301
    assert cls._obs_stale(n)
    n.tracked = NS(tracks=[])                               # LOST 뒤 목록에서 빠짐: 마지막 관측으로 계속 잰다
    assert cls._obs_stale(n)
    other = _track(track_id=8)
    other.last_observed_stamp = 100.2
    port.target, n.tracked = (8, 1), NS(tracks=[other])     # 다시 고른 대상
    assert not cls._obs_stale(n)
    stale_epoch = _track(epoch=2)
    stale_epoch.last_observed_stamp = 100.3
    port.target, n.tracked = (7, 1), NS(tracks=[stale_epoch])   # 같은 id라도 epoch가 다르면 대상이 아니다
    assert not cls._obs_stale(n)                            # 새 대상의 관측을 아직 받지 못함
