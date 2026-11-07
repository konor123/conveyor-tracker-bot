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
from drop_interfaces.msg import Heartbeat, RobotStatus, TaskStatus  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_mocks.ros_common import scenario_file  # noqa: E402
from drop_mocks.scenario_engine import ScenarioEngine  # noqa: E402
from drop_ros import DOMAIN, Harness, wait_future  # noqa: E402
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
            return
        time.sleep(0.05)
    raise AssertionError(f'mock bringup 준비 시간 초과: seen={ {m.node_name for _, m in hbs} }, task={len(statuses)}')


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
            assert res.result == DropTask.Result.SUCCESS, f'seed={seed} 실패: {res.result} ({res.reason_code})'
            observed = []
            for _, m in task_msgs[before_idx:]:
                if not observed or observed[-1] != m.state:
                    observed.append(m.state)
            while observed and observed[0] in ('IDLE', 'DONE'):
                observed = observed[1:]
            assert observed == expected_states, f'seed={seed} 상태 열 불일치: {observed}'

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
            assert len(step_rows) == 12
            for srec in step_rows:
                assert srec['t_end_s'] >= srec['t_start_s']
                assert len(srec['metric_keys']) == len(srec['metric_values'])
            by_step = {s['step_name']: s for s in step_rows}
            assert 'stone_count' in by_step['DETECT_STONE']['metric_keys']
            assert 'grip_width_m' in by_step['PICK_STONE']['metric_keys']
            assert 't_available_s' in by_step['WAIT_CAP']['metric_keys']
            assert 'time_margin_s' in by_step['PLAN_INTERCEPT']['metric_keys']
            assert 'e_xy_m' in by_step['SYNCHRONIZE']['metric_keys']
            assert 'verdict' in by_step['VERIFY']['metric_keys']
    finally:
        _stop(proc)
        h.close()


@pytest.mark.req('REQ-ABT-003', 'REQ-ABT-004')
@pytest.mark.tc('TC-INT-002-01')
@pytest.mark.level('I0')
def test_i0_robot_hang_escalates_and_stops(ros, tmp_path):
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
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
        while time.monotonic() < deadline and stop_count < 1:
            time.sleep(0.05)
            stop_count, requester, _ = _get_stop_info(h)
        assert stop_count >= 1

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
        # 1) vision heartbeat 손실 (파지 전 holding=False) -> L2 (FAILURE)
        _set_params(h, 'mock_world_node', [_param_str('drop_role', 'vision'), _param_double('drop_at_s', 0.2)])
        _reset_cycle_clock(h)
        _, res_vis = _send_goal(h, goal, timeout=15.0)
        assert res_vis.result == DropTask.Result.FAILURE
        assert res_vis.reason_code == 'HEARTBEAT_LOST'

        # 2) tracking heartbeat 손실 (파지 후 holding=True) -> L3 (SAFE_ABORT)
        _set_params(h, 'orchestrator_node', [_param_str('resume_reason', 'resume_after_vision_loss')])
        _set_params(h, 'mock_world_node', [_param_str('drop_role', 'tracking'), _param_double('drop_at_s', 1.05)])
        _reset_cycle_clock(h)
        _, res_trk = _send_goal(h, goal, timeout=15.0)
        assert res_trk.result == DropTask.Result.SAFE_ABORT
        assert res_trk.reason_code == 'HEARTBEAT_LOST'

        # 3) robot_controller heartbeat 손실 -> L4 (PROTECTIVE_STOP + Stop 호출)
        _set_params(h, 'orchestrator_node', [_param_str('resume_reason', 'resume_after_tracking_loss')])
        _set_params(h, 'mock_world_node', [_param_str('drop_role', 'none'), _param_double('drop_at_s', 999.0)])
        _set_params(
            h,
            'mock_robot_controller_node',
            [_param_str('drop_role', 'robot_controller'), _param_double('drop_at_s', 0.25)],
        )
        _reset_cycle_clock(h)
        _, res_ctrl = _send_goal(h, goal, timeout=15.0)
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
        assert res.result == DropTask.Result.NOT_ATTEMPTED
        assert res.reason_code == 'NODE_INACTIVE'

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
