"""I0 전체 대역 통합 시험(PLAN.md 7.5절, TEST_SPEC.md 3.11·3.12·3.13절).

TC-INT-001-01, TC-INT-002-01, TC-INT-003-01, TC-INT-004-01, TC-INT-005-01, TC-INT-006-01,
TC-MCK-006-02, TC-REC-002-01, TC-REC-003-01을 격리된 ROS_DOMAIN_ID에서 검증한다.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from drop_interfaces.action import DropTask  # noqa: E402
from drop_interfaces.msg import Heartbeat, RobotStatus, StepRecord, TaskStatus  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_mocks.ros_common import scenario_file  # noqa: E402
from drop_mocks.scenario_engine import ScenarioEngine  # noqa: E402
from drop_ros import DOMAIN, Harness, wait_future  # noqa: E402
from spec_metrics import EXPECTED_METRIC_KEYS
from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState  # noqa: E402
from orchestrator.record_logger_node import (  # noqa: E402
    steps_path,
    trials_path,
    validate_trial_record,
)
from rcl_interfaces.msg import Parameter as RosParam  # noqa: E402
from rcl_interfaces.msg import ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import GetParameters, SetParameters  # noqa: E402

DATE_STR = '2026-11-07'
_DOMAIN_SEQ = 0
_ACTIVE_DOMAIN = DOMAIN


@pytest.fixture
def ros():
    global _DOMAIN_SEQ, _ACTIVE_DOMAIN
    _DOMAIN_SEQ += 1
    _ACTIVE_DOMAIN = 100 + (os.getpid() + _DOMAIN_SEQ * 13) % 100
    rclpy.init(domain_id=_ACTIVE_DOMAIN)
    yield
    rclpy.shutdown()


def _launch(*args):
    env = dict(
        os.environ,
        ROS_DOMAIN_ID=str(_ACTIVE_DOMAIN),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
    )
    log = tempfile.NamedTemporaryFile(mode='w+', encoding='utf-8', prefix='drop-i0-', suffix='.log',
                                      delete=False)
    proc = subprocess.Popen(
        ['ros2', 'launch', 'drop_bringup', 'mock.launch.py', *args],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    proc.i0_log = log
    return proc


def _stop(proc) -> str:
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
    proc.i0_log.seek(0)
    out = proc.i0_log.read()
    print(f'I0 launch log: {proc.i0_log.name}\n{out[-12000:]}')
    proc.i0_log.close()
    return out


def _wait_ready(h: Harness, timeout_s: float = 15.0) -> None:
    hbs = h.record(Heartbeat, 'system/heartbeat')
    statuses = h.record(TaskStatus, 'task/status')
    end = time.monotonic() + timeout_s
    want = {'vision_node', 'tracker_node', 'robot_controller_node'}
    while time.monotonic() < end:
        seen = {m.node_name for _, m in hbs if m.lifecycle_state == 3}
        if want <= seen and statuses:
            if not getattr(h, 'settled', False):
                _settle(h)
            return
        time.sleep(0.05)
    raise AssertionError(f'mock bringup 준비 시간 초과: seen={ {m.node_name for _, m in hbs} }, task={len(statuses)}')


BRINGUP_SETTLE_S = 1.5
BRINGUP_RELEASE_REASON = 'i0: bring-up supervisor heartbeat transient'


def _settle(h: Harness) -> None:
    """기동 직후 안정 대기(TEST_SPEC 0.4절 I0 기동 절차).

    감시 노드는 기동 직후 controller heartbeat를 0.3 s 넘게 늦게 받아 `CONTROLLER_HEARTBEAT_LOST` Stop을
    보낼 수 있다(대역 발행은 연속). 로봇은 Stop을 정지 유지로 남기므로(SPEC-010 4.5절) 첫 goal 전에 시험이
    작업자 역할로 사유를 남기고 해제한다. 그 밖의 사유거나 이 구간 뒤의 감시 노드 Stop은 해제하지 않는다.
    """
    time.sleep(BRINGUP_SETTLE_S)
    stops = _get_cancel_stop_history(h)[2]
    sup = [s['reason_code'] for s in stops if s['requester'] == Stop.Request.SUPERVISOR]
    assert set(sup) <= {'CONTROLLER_HEARTBEAT_LOST'}, f'기동 중 예기치 않은 Stop: {stops}'
    if sup:
        print(f'I0 기동 과도 감시 노드 Stop {len(sup)}회 → 해제', flush=True)
        _set_params(h, 'mock_robot_controller_node', [_param_str('stop_release_reason', BRINGUP_RELEASE_REASON)])
    h.settled = True
    h.bringup_stops = len(stops)


def _supervisor_stops_after_settle(h: Harness) -> list[str]:
    stops = _get_cancel_stop_history(h)[2][getattr(h, 'bringup_stops', 0):]
    return [s['reason_code'] for s in stops if s['requester'] == Stop.Request.SUPERVISOR]


def _set_params(h: Harness, node_name: str, params: list[RosParam]) -> None:
    res = h.call(SetParameters, f'/{node_name}/set_parameters', parameters=params, timeout=5.0)
    assert all(r.successful for r in res.results), f'{node_name} set_parameters 실패: {res.results}'


def _param_int(name: str, val: int) -> RosParam:
    return RosParam(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(val)),
    )


def _param_double(name: str, val: float) -> RosParam:
    return RosParam(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val)),
    )


def _param_str(name: str, val: str) -> RosParam:
    return RosParam(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(val)),
    )


def _reset_cycle_clock(h: Harness, seed: int | None = None, t0_offset_s: float = 0.0) -> None:
    t0 = time.time() + t0_offset_s
    world_params = [_param_double('t0_s', t0)]
    robot_params = [_param_double('t0_s', t0)]
    if seed is not None:
        world_params.append(_param_int('seed', seed))
        robot_params.append(_param_int('seed', seed))
    _set_params(h, 'mock_world_node', world_params)
    _set_params(h, 'mock_robot_controller_node', robot_params)
    time.sleep(0.08)


def _get_stop_info(h: Harness) -> tuple[int, int, str]:
    res = h.call(
        GetParameters,
        '/mock_robot_controller_node/get_parameters',
        names=['stop_count', 'last_stop_requester', 'last_stop_reason'],
        timeout=5.0,
    )
    vals = res.values
    return int(vals[0].integer_value), int(vals[1].integer_value), str(vals[2].string_value)


def _get_cancel_stop_history(h):
    res = h.call(GetParameters, '/mock_robot_controller_node/get_parameters',
                 names=['cancel_count', 'cancel_received_steady_s', 'stop_history_json'], timeout=5.0)
    return (res.values[0].integer_value, res.values[1].double_value,
            json.loads(res.values[2].string_value))


def _read_jsonl(path: str | Path) -> list[dict]:
    p = Path(path)
    if not p.exists():
        return []
    return [json.loads(line) for line in p.read_text(encoding='utf-8').splitlines() if line.strip()]


def _send_goal(h: Harness, goal: DropTask.Goal, timeout: float = 20.0):
    ac = h.action(DropTask, 'drop_task', timeout=5.0)
    gh = wait_future(ac.send_goal_async(goal), timeout=5.0)
    assert gh.accepted
    out = wait_future(gh.get_result_async(), timeout=timeout)
    return out.status, out.result


@pytest.mark.req('REQ-ORC-001', 'REQ-MCK-006', 'REQ-REC-001', 'REQ-REC-002', 'REQ-REC-003')
@pytest.mark.tc('TC-INT-001-01', 'TC-MCK-006-02', 'TC-REC-003-01')
@pytest.mark.level('I0')
def test_i0_nominal_20_cycles_and_records(ros, tmp_path):
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _launch(
        'scenario:=nominal_20mms',
        'duration_scale_ratio:=0.12',
        f'records_dir:={tmp_path}',
        f'date:={DATE_STR}',
    )
    expected_states = [
        'PRECHECK',
        'DETECT_STONE',
        'PICK_STONE',
        'MOVE_STANDBY',
        'WAIT_CAP',
        'PLAN_INTERCEPT',
        'APPROACH',
        'SYNCHRONIZE',
        'DESCEND',
        'RELEASE',
        'RETREAT',
        'VERIFY',
        'DONE',
    ]
    try:
        _wait_ready(h)
        for seed in range(20):
            _reset_cycle_clock(h, seed=seed, t0_offset_s=-2.5)
            before_idx = len(task_msgs)
            goal = DropTask.Goal(
                mode=DropTask.Goal.MODE_SYNCHRONIZED,
                max_belt_speed=0.05,
                timeout=15.0,
                max_passes=3,
            )
            _, res = _send_goal(h, goal, timeout=20.0)
            cyc = [m for _, m in step_msgs if task_msgs and m.cycle_id == task_msgs[-1][1].cycle_id]
            last = [(m.step_name, m.reason_code, round((m.t_end.sec - m.t_start.sec)
                                                       + (m.t_end.nanosec - m.t_start.nanosec) * 1e-9, 3))
                    for m in cyc[-2:]]
            assert res.result == DropTask.Result.SUCCESS, (
                f'seed={seed} 실패: {res.result} ({res.reason_code}); last={last}; '
                f'steps={[(m.step_name, m.reason_code, dict(zip(m.metric_keys, m.metric_values))) for _, m in step_msgs if m.cycle_id == task_msgs[-1][1].cycle_id]}'
            )
            observed = []
            for _, m in task_msgs[before_idx:]:
                if not observed or observed[-1] != m.state:
                    observed.append(m.state)
            while observed and observed[0] in ('IDLE', 'DONE'):
                observed = observed[1:]
            assert observed == expected_states, f'seed={seed} 상태 열 불일치: {observed}'

        assert _supervisor_stops_after_settle(h) == []
        t_file = trials_path(str(tmp_path), DATE_STR)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(_read_jsonl(t_file)) < 20:
            time.sleep(0.05)
        trials = _read_jsonl(t_file)
        assert len(trials) == 20
        expected_assumed = ScenarioEngine.from_file(scenario_file('nominal_20mms'), 0).assumed_params()
        for idx, rec in enumerate(trials):
            assert validate_trial_record(rec) == []
            assert rec['scenario']['name'] == 'nominal_20mms'
            assert rec['scenario']['seed'] == idx
            assert rec['result'] == 'SUCCESS'
            assert rec['abort_level'] == 0
            # TC-MCK-006-02: assumed_params 복사 확인
            assert rec['assumed_params'] == expected_assumed
            assert len(rec['assumed_params']) > 0
            # TC-REC-003-01: 단계 기록 파일 및 필수 메트릭 검증
            s_file = steps_path(str(tmp_path), rec['trial_id'], DATE_STR)
            step_rows = _read_jsonl(s_file)
            assert [r['step_name'] for r in step_rows] == expected_states[:-1]
            live = [m for _, m in step_msgs if m.cycle_id == rec['trial_id']]
            assert len(live) == 12
            budget_names = {'plan', 'approach', 'synchronize', 'descend', 'release'}
            for srec, msg in zip(step_rows, live):
                # These are raw ROS stamps, which may go backward during NTP
                # correction. The logger must preserve both endpoints verbatim.
                assert srec['t_start_s'] == msg.t_start.sec + msg.t_start.nanosec * 1e-9
                assert srec['t_end_s'] == msg.t_end.sec + msg.t_end.nanosec * 1e-9
                assert srec['attempt'] == msg.attempt == 1
                assert srec['step_seq'] == msg.step_seq
                keys = set(srec['metric_keys'])
                expected_keys = EXPECTED_METRIC_KEYS[srec['step_name']]
                if srec['step_name'] == 'WAIT_CAP':
                    expected_keys = expected_keys | {f'budget_{n}_s' for n in budget_names}
                assert keys == expected_keys
                assert len(srec['metric_keys']) == len(keys) == len(srec['metric_values'])
                assert srec['metric_keys'] == list(msg.metric_keys)
                assert srec['metric_values'] == list(msg.metric_values)
                assert srec['metrics'] == dict(zip(msg.metric_keys, msg.metric_values))
    finally:
        _stop(proc)
        h.close()


@pytest.mark.req('REQ-ABT-003', 'REQ-ABT-004')
@pytest.mark.tc('TC-INT-002-01')
@pytest.mark.level('I0')
def test_i0_robot_hang_escalates_and_stops(ros, tmp_path):
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status', steady=True)
    proc = _launch(
        'scenario:=robot_hang',
        'duration_scale_ratio:=0.2',
        f'records_dir:={tmp_path}',
        f'date:={DATE_STR}',
    )
    try:
        _wait_ready(h)
        goal = DropTask.Goal(
            mode=DropTask.Goal.MODE_SYNCHRONIZED,
            max_belt_speed=0.05,
            timeout=1.2,
            max_passes=3,
        )
        _, res = _send_goal(h, goal, timeout=15.0)
        assert res.result == DropTask.Result.PROTECTIVE_STOP
        stop_count, requester, _ = _get_stop_info(h)
        assert stop_count >= 1
        assert requester == Stop.Request.ORCHESTRATOR
        cancel_count, cancel_received, stops = _get_cancel_stop_history(h)
        assert cancel_count == 1, _get_cancel_stop_history(h)
        ours = [s for s in stops if s['requester'] == Stop.Request.ORCHESTRATOR]
        assert ours
        # Both endpoints are observed in one mock process. 20 ms covers cancel transport offset.
        delay = ours[0]['received_steady_s'] - cancel_received
        assert 0.5 - 0.02 <= delay <= 0.7, delay
        before = [m.abort_level for t, m in task_msgs if t < cancel_received]
        assert before and max(before) < 4
        assert any(m.state == 'WAIT_OPERATOR' and m.abort_level == 4 for _, m in task_msgs)
    finally:
        _stop(proc)
        h.close()


@pytest.mark.req('REQ-ABT-001', 'REQ-ABT-004', 'REQ-ABT-005')
@pytest.mark.tc('TC-INT-003-01')
@pytest.mark.level('I0')
def test_i0_protective_stop_waits_for_operator_resume(ros, tmp_path):
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    robot_msgs = h.record(RobotStatus, 'robot/status')
    proc = _launch(
        'scenario:=protective_stop',
        'duration_scale_ratio:=0.25',
        f'records_dir:={tmp_path}',
        f'date:={DATE_STR}',
    )
    try:
        _wait_ready(h)
        _reset_cycle_clock(h, t0_offset_s=-2.5)
        goal = DropTask.Goal(
            mode=DropTask.Goal.MODE_SYNCHRONIZED,
            max_belt_speed=0.05,
            timeout=15.0,
            max_passes=3,
        )
        _, res = _send_goal(h, goal, timeout=15.0)
        assert res.result == DropTask.Result.PROTECTIVE_STOP, (
            f'res={res} stop_info={_get_stop_info(h)}'
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not any(m.protective_stop for _, m in robot_msgs):
            time.sleep(0.05)
        assert any(m.protective_stop for _, m in robot_msgs)

        # safety_supervisor_node의 Stop 호출 대기 및 확인
        deadline = time.monotonic() + 2.0
        stop_count, requester, _ = _get_stop_info(h)
        _, _, stops = _get_cancel_stop_history(h)
        while time.monotonic() < deadline and not any(s['requester'] == Stop.Request.SUPERVISOR for s in stops):
            time.sleep(0.05)
            _, _, stops = _get_cancel_stop_history(h)
        assert any(s['requester'] == Stop.Request.SUPERVISOR for s in stops), stops

        # WAIT_OPERATOR 상태 유지(자동 복귀 0회) 및 재시도 거부 확인
        time.sleep(0.4)
        assert task_msgs[-1][1].state == 'WAIT_OPERATOR'
        assert task_msgs[-1][1].abort_level == 4
        _, blocked_res = _send_goal(h, goal, timeout=5.0)
        assert blocked_res.result == DropTask.Result.NOT_ATTEMPTED
        assert blocked_res.reason_code == 'WAIT_OPERATOR'

        # 운영자 사유 입력 후 IDLE 복귀 확인
        _set_params(h, 'orchestrator_node', [_param_str('resume_reason', 'pstop_cleared_by_operator')])
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and task_msgs[-1][1].state != 'IDLE':
            time.sleep(0.05)
        assert task_msgs[-1][1].state == 'IDLE'
    finally:
        _stop(proc)
        h.close()


@pytest.mark.req('REQ-ABT-001', 'REQ-ABT-010')
@pytest.mark.tc('TC-INT-004-01')
@pytest.mark.level('I0')
def test_i0_heartbeat_loss_all_roles(ros, tmp_path):
    h = Harness()
    proc = _launch(
        'scenario:=heartbeat_loss',
        'duration_scale_ratio:=0.2',
        f'records_dir:={tmp_path}',
        f'date:={DATE_STR}',
    )
    try:
        _wait_ready(h)
        goal = DropTask.Goal(
            mode=DropTask.Goal.MODE_SYNCHRONIZED,
            max_belt_speed=0.05,
            timeout=15.0,
            max_passes=3,
        )
        def inject(role, node, state):
            # Recover all heartbeats before PRECHECK, then inject in an observed
            # state. Absolute scenario offsets can fire during DDS discovery or
            # the preceding trial and turn this into a PRECHECK rejection test.
            _set_params(h, 'mock_world_node', [_param_str('drop_role', 'none'),
                                             _param_double('drop_at_s', 999.0)])
            _set_params(h, 'mock_robot_controller_node', [_param_str('drop_role', 'none'),
                                                        _param_double('drop_at_s', 999.0)])
            _reset_cycle_clock(h)
            _wait_ready(h)
            states = h.record(TaskStatus, 'task/status')
            ac = h.action(DropTask, 'drop_task')
            gh = wait_future(ac.send_goal_async(goal), 5.0)
            assert gh.accepted
            h.wait_for(lambda: any(m.state == state for _, m in states), timeout=5.0,
                       what=f'{role} injection state {state}')
            _set_params(h, node, [_param_str('drop_role', role), _param_double('drop_at_s', 0.0)])
            out = wait_future(gh.get_result_async(), 15.0).result
            return out

        # 1) PickStone still runs with holding=False -> L2 (FAILURE).
        res_vis = inject('vision', 'mock_world_node', 'PICK_STONE')
        assert res_vis.result == DropTask.Result.FAILURE
        assert res_vis.reason_code == 'HEARTBEAT_LOST'

        # 2) PickStone completed, holding=True -> L3 (SAFE_ABORT).
        _set_params(h, 'orchestrator_node', [_param_str('resume_reason', 'resume_after_vision_loss')])
        res_trk = inject('tracking', 'mock_world_node', 'MOVE_STANDBY')
        assert res_trk.result == DropTask.Result.SAFE_ABORT
        assert res_trk.reason_code == 'HEARTBEAT_LOST'

        # 3) Controller heartbeat loss -> L4 and Stop.
        _set_params(h, 'orchestrator_node', [_param_str('resume_reason', 'resume_after_tracking_loss')])
        res_ctrl = inject('robot_controller', 'mock_robot_controller_node', 'PICK_STONE')
        assert res_ctrl.result == DropTask.Result.PROTECTIVE_STOP
        assert res_ctrl.reason_code == 'CONTROLLER_HEARTBEAT_LOST'
        stop_count, _, _ = _get_stop_info(h)
        assert stop_count >= 1
    finally:
        _stop(proc)
        h.close()


@pytest.mark.req('REQ-ORC-006', 'REQ-ORC-007', 'REQ-REC-002')
@pytest.mark.tc('TC-INT-005-01', 'TC-REC-002-01')
@pytest.mark.level('I0')
def test_i0_time_insufficient_pass_skip_and_not_attempted_record(ros, tmp_path):
    h = Harness()
    proc = _launch(
        'scenario:=time_insufficient',
        'duration_scale_ratio:=0.2',
        f'records_dir:={tmp_path}',
        f'date:={DATE_STR}',
    )
    try:
        _wait_ready(h)
        goal = DropTask.Goal(
            mode=DropTask.Goal.MODE_SYNCHRONIZED,
            max_belt_speed=0.05,
            timeout=25.0,
            max_passes=2,
        )
        # 1) t0_offset_s=-7.8 (WAIT_CAP 진입 시 t≈9.0s): 1차 통과(x0=0.15m) 스킵 후 2차 통과(x0=-0.05m) SUCCESS
        _reset_cycle_clock(h, t0_offset_s=-7.8)
        _, res_skip_then_ok = _send_goal(h, goal, timeout=20.0)
        assert res_skip_then_ok.result == DropTask.Result.SUCCESS, (
            f'res={res_skip_then_ok} stop_info={_get_stop_info(h)}'
        )
        assert res_skip_then_ok.passes_skipped == 1

        # 2) t0_offset_s=-4.8 (WAIT_CAP 진입 시 t≈6.0s): 앞선 3개 통과 모두 스킵되어 NOT_ATTEMPTED 기록
        _reset_cycle_clock(h, t0_offset_s=-4.8)
        _, res_exhausted = _send_goal(h, goal, timeout=20.0)
        assert res_exhausted.result == DropTask.Result.NOT_ATTEMPTED
        assert res_exhausted.passes_skipped == 3

        t_file = trials_path(str(tmp_path), DATE_STR)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(_read_jsonl(t_file)) < 2:
            time.sleep(0.05)
        trials = _read_jsonl(t_file)
        assert len(trials) == 2
        assert trials[0]['result'] == 'SUCCESS'
        assert trials[0]['passes_skipped'] == 1
        assert trials[1]['result'] == 'NOT_ATTEMPTED'
        assert trials[1]['passes_skipped'] == 3
        assert validate_trial_record(trials[1]) == []
    finally:
        _stop(proc)
        h.close()


@pytest.mark.req('REQ-ORC-002', 'REQ-SYS-003')
@pytest.mark.tc('TC-INT-006-01')
@pytest.mark.level('I0')
def test_i0_inactive_node_rejected_at_precheck(ros, tmp_path):
    h = Harness()
    proc = _launch(
        'scenario:=nominal_20mms',
        f'records_dir:={tmp_path}',
        f'date:={DATE_STR}',
    )
    try:
        _wait_ready(h)
        # mock_world_node를 INACTIVE 상태로 전이
        res_deact = h.call(
            ChangeState,
            '/mock_world_node/change_state',
            transition=Transition(id=Transition.TRANSITION_DEACTIVATE),
            timeout=5.0,
        )
        assert res_deact.success
        time.sleep(0.25)

        goal = DropTask.Goal(
            mode=DropTask.Goal.MODE_SYNCHRONIZED,
            max_belt_speed=0.05,
            timeout=10.0,
            max_passes=3,
        )
        _, res = _send_goal(h, goal, timeout=10.0)
        assert (res.result, res.reason_code) == (DropTask.Result.NOT_ATTEMPTED, 'NODE_INACTIVE'), (
            res, _get_cancel_stop_history(h)[2])
        assert _supervisor_stops_after_settle(h) == []

        t_file = trials_path(str(tmp_path), DATE_STR)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(_read_jsonl(t_file)) < 1:
            time.sleep(0.05)
        trials = _read_jsonl(t_file)
        assert len(trials) == 1
        assert trials[0]['result'] == 'NOT_ATTEMPTED'
        assert trials[0]['reason_code'] == 'NODE_INACTIVE'
        # PRECHECK 외 스킬이 0회 호출되었음을 검증
        assert list(trials[0]['step_durations_s'].keys()) == ['PRECHECK']
    finally:
        _stop(proc)
        h.close()


def _skill_runs(h) -> int:
    res = h.call(GetParameters, '/mock_robot_controller_node/get_parameters', names=['skill_run_count'],
                 timeout=5.0)
    return int(res.values[0].integer_value)


def _drop_goal(timeout: float = 15.0) -> DropTask.Goal:
    return DropTask.Goal(mode=DropTask.Goal.MODE_SYNCHRONIZED, max_belt_speed=0.05, timeout=timeout,
                         max_passes=3)


@pytest.mark.req('REQ-ORC-012', 'REQ-MCK-008')
@pytest.mark.tc('TC-INT-008-01')
@pytest.mark.level('I0')
def test_i0_supervisor_stop_without_running_skill(ros, tmp_path):
    """감시 노드 Stop이 실행 중 skill이 없는 WAIT_CAP에 와도 L4로 끝나고, 해제 전에는 다시 시작하지 않는다."""
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    proc = _launch('scenario:=nominal_20mms', 'duration_scale_ratio:=0.12', f'records_dir:={tmp_path}',
                   f'date:={DATE_STR}')
    try:
        _wait_ready(h)
        ac = h.action(DropTask, 'drop_task', timeout=5.0)

        # 1) WAIT_CAP(병뚜껑 선택·관측 대기, 실행 중 skill 없음)에서 감시 노드 Stop
        _reset_cycle_clock(h, seed=0)
        n0 = len(task_msgs)
        gh = wait_future(ac.send_goal_async(_drop_goal()), 5.0)
        assert gh.accepted
        try:
            h.wait_for(lambda: any(m.state == 'WAIT_CAP' for _, m in task_msgs[n0:]), timeout=10.0,
                       what='WAIT_CAP')
        except TimeoutError as exc:
            raise AssertionError(f'WAIT_CAP 미도달, stops={_get_cancel_stop_history(h)[2]}') from exc
        r = h.call(Stop, 'robot/stop', requester=Stop.Request.SUPERVISOR, reason_code='I0_WAIT_CAP')
        assert r.stopped
        res = wait_future(gh.get_result_async(), 15.0).result
        stops = _get_cancel_stop_history(h)[2]
        assert (res.result, res.reason_code) == (DropTask.Result.PROTECTIVE_STOP, 'ROBOT_STOPPED'), (res, stops)
        # 이 시험이 보낸 Stop 외 감시 노드 Stop은 없어야 한다(대역 heartbeat 공백 검출)
        assert _supervisor_stops_after_settle(h) == ['I0_WAIT_CAP'], stops
        h.wait_for(lambda: task_msgs[-1][1].state == 'WAIT_OPERATOR', timeout=2.0, what='WAIT_OPERATOR')
        assert task_msgs[-1][1].abort_level == 4

        # 2) resume만 하고 로봇을 해제하지 않으면 다음 사이클은 PRECHECK에서 다시 L4
        _set_params(h, 'orchestrator_node', [_param_str('resume_reason', 'resume_without_robot_release')])
        h.wait_for(lambda: task_msgs[-1][1].state == 'IDLE', timeout=2.0, what='IDLE')
        _reset_cycle_clock(h, seed=1)
        runs = _skill_runs(h)
        _, res = _send_goal(h, _drop_goal(), timeout=15.0)
        assert res.reason_code == 'ROBOT_STOPPED', (res.result, res.reason_code)
        h.wait_for(lambda: task_msgs[-1][1].state == 'WAIT_OPERATOR', timeout=2.0, what='WAIT_OPERATOR')
        assert task_msgs[-1][1].abort_level == 4
        assert _skill_runs(h) == runs

        # 3) 작업자가 로봇을 해제하고 재개하면 정상 사이클
        _set_params(h, 'mock_robot_controller_node', [_param_str('stop_release_reason', 'operator checked')])
        _set_params(h, 'orchestrator_node', [_param_str('resume_reason', 'robot_released')])
        h.wait_for(lambda: task_msgs[-1][1].state == 'IDLE', timeout=2.0, what='IDLE')
        _reset_cycle_clock(h, seed=2, t0_offset_s=-2.5)
        _, res = _send_goal(h, _drop_goal(), timeout=20.0)
        assert res.result == DropTask.Result.SUCCESS, (res.result, res.reason_code, _get_cancel_stop_history(h)[2])
    finally:
        _stop(proc)
        h.close()
