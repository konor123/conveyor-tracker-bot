"""I0 전체 대역 통합 시험.

대역 노드로 전체 사이클, 장애 경로, 단계 기록, 감시 연결을 격리된 ROS_DOMAIN_ID에서 검증한다.
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from drop_interfaces.action import DropTask  # noqa: E402
from drop_interfaces.msg import (BeltMotion, Heartbeat, InterceptPlan, RobotStatus, StepRecord,  # noqa: E402
                                 TaskStatus, TrackedObjectArray)
from drop_interfaces.srv import ResetTracking, Stop  # noqa: E402
from drop_mocks.ros_common import scenario_file  # noqa: E402
from drop_mocks.scenario_engine import ScenarioEngine  # noqa: E402
from drop_ros import DOMAIN, Harness, SteadyRosClock, wait_future  # noqa: E402
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


I0_STALL_DUMP_S = '0.25'          # 감시 노드의 controller heartbeat 손실 판정 0.3 s보다 짧게


def _launch(*args, launch_file='mock.launch.py'):
    env = dict(
        os.environ,
        ROS_DOMAIN_ID=str(_ACTIVE_DOMAIN),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
    )
    # 대역 프로세스의 heartbeat 경로 정지 진단. 값을 주면 그 값(0이면 끔)을 따른다
    env.setdefault('DROP_MOCK_STALL_DUMP_S', I0_STALL_DUMP_S)
    log = tempfile.NamedTemporaryFile(mode='w+', encoding='utf-8', prefix='drop-i0-', suffix='.log',
                                      delete=False)
    proc = subprocess.Popen(
        ['ros2', 'launch', 'drop_bringup', launch_file, *args],
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
    hbs = h.record(Heartbeat, 'system/heartbeat', steady=True)
    robot = h.record(RobotStatus, 'robot/status', steady=True)
    statuses = h.record(TaskStatus, 'task/status')
    end = time.monotonic() + timeout_s
    want = {'vision_node', 'tracker_node', 'robot_controller_node'}
    while time.monotonic() < end:
        seen = {m.node_name for _, m in hbs if m.lifecycle_state == 3}
        if want <= seen and statuses:
            if not getattr(h, 'settled', False):
                _settle(h, hbs, robot)
            return
        time.sleep(0.05)
    raise AssertionError(f'mock bringup 준비 시간 초과: seen={ {m.node_name for _, m in hbs} }, task={len(statuses)}')


BRINGUP_SETTLE_S = 5.0     # 기동 직후 발행 정지 관찰 2.5~3.9 s 뒤
BRINGUP_RELEASE_REASON = 'i0: bring-up supervisor heartbeat transient'


def _settle(h: Harness, hbs, robot) -> None:
    """기동 직후 안정 대기.

    감시 노드는 기동 직후 controller heartbeat를 0.3 s 넘게 늦게 받아 `CONTROLLER_HEARTBEAT_LOST` Stop을
    보낼 수 있다(대역 발행은 연속). 로봇은 Stop을 정지 유지로 남기므로 첫 goal 전에 시험이
    작업자 역할로 사유를 남기고 해제한다. 그 밖의 사유거나 이 구간 뒤의 감시 노드 Stop은 해제하지 않는다.
    해제 뒤에는 controller heartbeat `error_code` 0과 `RobotStatus.ready`를 받은 다음 돌아간다(운용 절차와 같은 순서).
    hbs·robot은 steady 수신 시각 기록이다.
    """
    time.sleep(BRINGUP_SETTLE_S)
    stops = _get_cancel_stop_history(h)[2]
    sup = [s['reason_code'] for s in stops if s['requester'] == Stop.Request.SUPERVISOR]
    assert set(sup) <= {'CONTROLLER_HEARTBEAT_LOST'}, f'기동 중 예기치 않은 Stop: {stops}'
    if sup:
        print(f'I0 기동 과도 감시 노드 Stop {len(sup)}회 → 해제', flush=True)
        _set_params(h, 'mock_robot_controller_node', [_param_str('stop_release_reason', BRINGUP_RELEASE_REASON)])
    t_rel = time.monotonic()
    h.wait_for(lambda: any(t > t_rel and m.node_name == 'robot_controller_node' and m.error_code == 0
                           for t, m in list(hbs))
               and any(t > t_rel and m.ready for t, m in list(robot)),
               timeout=5.0, what='해제 뒤 controller heartbeat error_code 0·RobotStatus.ready')
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


def _reset_cycle_clock(h: Harness, seed: int | None = None, t0_offset_s: float = 0.0, now=None) -> None:
    """시나리오 시각 0을 지금 + t0_offset_s로 옮긴다. now는 ROS 시각 원천(sim time이면 그 시계의 now)."""
    t0 = (time.time() if now is None else now()) + t0_offset_s
    world_params = [_param_double('t0_s', t0)]
    robot_params = [_param_double('t0_s', t0)]
    if seed is not None:
        world_params.append(_param_int('seed', seed))
        robot_params.append(_param_int('seed', seed))
    _set_params(h, 'mock_world_node', world_params)
    _set_params(h, 'mock_robot_controller_node', robot_params)
    time.sleep(0.08)
    return t0


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
            # assumed_params 복사 확인
            assert rec['assumed_params'] == expected_assumed
            assert len(rec['assumed_params']) > 0
            # 단계 기록 파일 및 필수 메트릭 검증
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


@pytest.mark.parametrize('mode', [DropTask.Goal.MODE_SYNCHRONIZED, DropTask.Goal.MODE_PREDICTED],
                         ids=['synchronized', 'predicted'])
def test_i0_real_tracker_planner_cycle(ros, tmp_path, mode):
    """mock vision·robot + 실제 tracker·planner로 계획 시점·계획 단계 기준을 거쳐 한 사이클을 끝낸다."""
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _launch('use_mock_vision:=true', 'use_mock_robot:=true', 'use_mock_tracking:=false',
                   'use_mock_planner:=false', 'scenario:=nominal_20mms', 'duration_scale_ratio:=0.12',
                   f'records_dir:={tmp_path}', f'date:={DATE_STR}', launch_file='drop.launch.py')
    expected_states = ['PRECHECK', 'DETECT_STONE', 'PICK_STONE', 'MOVE_STANDBY', 'WAIT_CAP', 'PLAN_INTERCEPT',
                       'APPROACH', 'SYNCHRONIZE', 'DESCEND', 'RELEASE', 'RETREAT', 'VERIFY', 'DONE']
    try:
        _wait_ready(h, timeout_s=30.0)
        assert sorted(i.node_name for i in h.client.get_publishers_info_by_topic('/drop/intercept_plan')) \
            == ['intercept_planner_node']
        for seed in range(4):
            _reset_cycle_clock(h, seed=seed, t0_offset_s=-2.5)
            before_idx = len(task_msgs)
            goal = _drop_goal(timeout=30.0)
            goal.mode = mode
            _, res = _send_goal(h, goal, timeout=40.0)
            cyc_id = task_msgs[-1][1].cycle_id
            steps = {m.step_name: dict(zip(m.metric_keys, m.metric_values))
                     for _, m in step_msgs if m.cycle_id == cyc_id and m.result == 0}
            assert res.result == DropTask.Result.SUCCESS, (seed, res.result, res.reason_code, steps)
            observed = []
            for _, m in task_msgs[before_idx:]:
                if not observed or observed[-1] != m.state:
                    observed.append(m.state)
            while observed and observed[0] in ('IDLE', 'DONE'):
                observed = observed[1:]
            assert observed == expected_states, f'seed={seed} 상태 열 불일치: {observed}'
            assert steps['WAIT_CAP']['t_to_window_s'] <= 3.0, (seed, steps['WAIT_CAP'])
            assert steps['PLAN_INTERCEPT']['position_std_m'] <= 0.00325, (seed, steps['PLAN_INTERCEPT'])
        assert _supervisor_stops_after_settle(h) == []
    finally:
        _stop(proc)
        h.close()


def test_i0_realtime_scale_cycle(ros, tmp_path):
    """대역 소요시간을 줄이지 않아도(축척 1.0, DynamicPlace 3.0 s) goal 하나가 APPROACH~RETREAT를 끝낸다.

    goal.deadline이 APPROACH 단계 제한시간(3.0 s)이면 서버가 RETREAT 도중 스스로 TIMEOUT으로 끝난다.
    """
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _launch('scenario:=nominal_20mms', 'duration_scale_ratio:=1.0', f'records_dir:={tmp_path}',
                   f'date:={DATE_STR}')
    try:
        _wait_ready(h, timeout_s=30.0)
        _reset_cycle_clock(h, seed=0)
        _, res = _send_goal(h, _drop_goal(timeout=60.0), timeout=75.0)
        cyc_id = task_msgs[-1][1].cycle_id
        steps = [(m.step_name, m.result, m.reason_code) for _, m in step_msgs if m.cycle_id == cyc_id]
        assert res.result == DropTask.Result.SUCCESS, (res.result, res.reason_code, steps)
        assert ('RETREAT', 0, '') in steps
    finally:
        _stop(proc)
        h.close()


def test_i0_lost_target_is_reselected(ros, tmp_path):
    """WAIT_CAP 도중 tracker가 초기화되어 고른 track이 사라져도 같은 통과에서 다시 골라 사이클을 끝낸다."""
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    tracked = h.record(TrackedObjectArray, 'objects/tracked')
    proc = _launch('use_mock_vision:=true', 'use_mock_robot:=true', 'use_mock_tracking:=false',
                   'use_mock_planner:=false', 'scenario:=nominal_20mms', 'duration_scale_ratio:=0.12',
                   f'records_dir:={tmp_path}', f'date:={DATE_STR}', launch_file='drop.launch.py')
    try:
        _wait_ready(h, timeout_s=30.0)
        _reset_cycle_clock(h, seed=0, t0_offset_s=-2.5)
        ac = h.action(DropTask, 'drop_task', timeout=5.0)
        gh = wait_future(ac.send_goal_async(_drop_goal(timeout=30.0)), timeout=5.0)
        assert gh.accepted
        result = gh.get_result_async()

        def target_chosen():
            return (task_msgs[-1][1].state == 'WAIT_CAP' and bool(tracked)
                    and any(t.observation_count >= 40 for t in tracked[-1][1].tracks))
        h.wait_for(target_chosen, 15.0, 'WAIT_CAP에서 대상 선택')
        epoch = tracked[-1][1].tracks[0].epoch
        reset = h.call(ResetTracking, 'tracking/reset')
        assert (reset.result, reset.new_epoch) == (0, epoch + 1)
        res = wait_future(result, timeout=40.0).result
        cyc_id = task_msgs[-1][1].cycle_id
        steps = [(m.step_name, m.result, m.reason_code) for _, m in step_msgs if m.cycle_id == cyc_id]
        assert res.result == DropTask.Result.SUCCESS, (res.result, res.reason_code, steps)
        assert res.passes_skipped == 0
    finally:
        _stop(proc)
        h.close()


def _sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def test_i0_predicted_marker_matches_observed_tf(ros, tmp_path):
    """예측 Marker(해제 시각의 TCP 목표)와 관측 TF(`cap_<id>`)가 같은 좌표계에서 맞는다(RViz 화면 대신 수치).

    `frames.launch.py scenario:=…`가 낸 robot_base → conveyor_frame으로 관측 TF를 옮기면, 해제 시각의 뚜껑 위치가
    그 시각을 예측한 계획·Marker의 XY와 같아야 한다.
    """
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker
    h = Harness()
    plans = h.record(InterceptPlan, 'drop/intercept_plan')
    tf, tf_static, markers = [], [], []
    h.client.create_subscription(TFMessage, '/tf', lambda m: tf.extend(m.transforms), 100)
    h.client.create_subscription(TFMessage, '/tf_static', lambda m: tf_static.extend(m.transforms),
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
    h.client.create_subscription(Marker, 'drop/predicted_marker', markers.append,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    proc = _launch('use_mock_vision:=true', 'use_mock_robot:=true', 'use_mock_tracking:=false',
                   'use_mock_planner:=false', 'scenario:=nominal_20mms', 'duration_scale_ratio:=0.12',
                   f'records_dir:={tmp_path}', f'date:={DATE_STR}', launch_file='drop.launch.py')
    frames = _launch('publish_world:=true', 'scenario:=nominal_20mms', launch_file='frames.launch.py')
    try:
        _wait_ready(h, timeout_s=30.0)
        _reset_cycle_clock(h, seed=0, t0_offset_s=-2.5)
        _, res = _send_goal(h, _drop_goal(timeout=30.0), timeout=40.0)
        assert res.result == DropTask.Result.SUCCESS, (res.result, res.reason_code)
        plan = [m for _, m in plans if m.status == InterceptPlan.OK][-1]
        t_rel = _sec(plan.release_time)
        h.wait_for(lambda: h.now() >= t_rel + 0.3, 15.0, '해제 시각 경과')

        # 고정 TF: frames.launch.py가 mock 세계와 같은 컨베이어 변환을 낸다(yaw 0)
        conv = next(t.transform for t in tf_static
                    if (t.header.frame_id, t.child_frame_id) == ('robot_base', 'conveyor_frame'))
        assert (conv.translation.x, conv.translation.y) == pytest.approx((0.30, -0.25))
        assert abs(conv.rotation.w) == pytest.approx(1.0, abs=1e-9)

        # 관측 TF: 해제 시각 앞뒤 표본을 보간한 뚜껑 위치(robot_base)
        cap = sorted(((_sec(t.header.stamp), t.transform.translation) for t in tf
                      if t.header.frame_id == 'conveyor_frame' and t.child_frame_id == f'cap_{plan.target_track_id}'),
                     key=lambda r: r[0])
        before = [r for r in cap if r[0] <= t_rel][-1]
        after = [r for r in cap if r[0] > t_rel][0]
        assert after[0] - before[0] <= 0.1, '해제 시각 주변 관측 TF 공백'
        f = (t_rel - before[0]) / (after[0] - before[0])
        x = conv.translation.x + before[1].x + f * (after[1].x - before[1].x)
        y = conv.translation.y + before[1].y + f * (after[1].y - before[1].y)
        target = plan.target_tcp_pose.position
        assert math.hypot(x - target.x, y - target.y) <= 0.001, (x, y, target.x, target.y)

        # Marker: robot_base에서 계획의 TCP 목표를 그대로 보여 준다(미래 위치는 TF가 아니라 Marker)
        added = [m for m in markers if m.action == Marker.ADD]
        assert added and {m.header.frame_id for m in added} == {'robot_base'}
        assert any(math.hypot(m.pose.position.x - target.x, m.pose.position.y - target.y) <= 1e-6 for m in added)
    finally:
        _stop(frames)
        _stop(proc)
        h.close()


# ------------------------------------------------------------------ MODE_STATIC
STATIC_STATES = ['PRECHECK', 'DETECT_STONE', 'PICK_STONE', 'MOVE_STANDBY', 'WAIT_CAP', 'APPROACH', 'DESCEND', 'RELEASE',
                 'RETREAT', 'VERIFY', 'DONE']


def _observed_states(task_msgs, start: int) -> list[str]:
    observed = []
    for _, m in task_msgs[start:]:
        if not observed or observed[-1] != m.state:
            observed.append(m.state)
    while observed and observed[0] in ('IDLE', 'DONE', 'WAIT_OPERATOR'):
        observed = observed[1:]
    return observed


def test_i0_static_mode_full_sequence(ros, tmp_path):
    """MODE_STATIC 한 사이클: WAIT_CAP은 병뚜껑 검출, APPROACH~RETREAT는 PlaceStone, PLAN_INTERCEPT·SYNCHRONIZE 없음.

    축척 0.4(PlaceStone 1.0 s, phase 0.25 s)로 phase feedback 간격을 대역 소요시간 축척 0.12보다 넓힌다.
    """
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _launch('scenario:=nominal_20mms', 'duration_scale_ratio:=0.4', f'records_dir:={tmp_path}',
                   f'date:={DATE_STR}')
    try:
        _wait_ready(h)
        _reset_cycle_clock(h, seed=0, t0_offset_s=-2.5)
        runs = _skill_runs(h)
        start = len(task_msgs)
        goal = DropTask.Goal(mode=DropTask.Goal.MODE_STATIC, max_belt_speed=0.05, timeout=30.0, max_passes=1)
        _, res = _send_goal(h, goal, timeout=40.0)
        cyc_id = task_msgs[-1][1].cycle_id
        h.wait_for(lambda: any(m.cycle_id == cyc_id and m.step_name == 'VERIFY' for _, m in step_msgs), 2.0,
                   'VERIFY 기록')
        steps = [m for _, m in step_msgs if m.cycle_id == cyc_id]
        assert res.result == DropTask.Result.SUCCESS, (res.result, res.reason_code,
                                                       [(m.step_name, m.result, m.reason_code) for m in steps])
        assert _observed_states(task_msgs, start) == STATIC_STATES
        assert [m.step_name for m in steps] == STATIC_STATES[:-1]
        assert all(m.result == 0 for m in steps), [(m.step_name, m.reason_code) for m in steps]
        by_name = {m.step_name: m for m in steps}
        assert set(by_name['DESCEND'].metric_keys) == EXPECTED_METRIC_KEYS['DESCEND_STATIC']
        assert set(by_name['WAIT_CAP'].metric_keys) == EXPECTED_METRIC_KEYS['WAIT_CAP']
        assert _skill_runs(h) - runs == 3                    # PickStone, MoveToPose(standby), PlaceStone
        assert _supervisor_stops_after_settle(h) == []
        t_file = trials_path(str(tmp_path), DATE_STR)
        h.wait_for(lambda: _read_jsonl(t_file), 5.0, 'trial 기록')
        trial = _read_jsonl(t_file)[-1]
        assert (trial['mode'], trial['result']) == ('MODE_STATIC', 'SUCCESS')
        assert validate_trial_record(trial) == []
    finally:
        _stop(proc)
        h.close()


# ------------------------------------------------------------------ 감시 연결
WATCH_SKILLS = {'PickStone': 1.0, 'MoveToPose': 0.5, 'PlaceStone': 2.5, 'DynamicPlace': 4.0, 'SafeRetreat': 1.0}
DYNAMIC = ('APPROACH', 'SYNCHRONIZE', 'DESCEND', 'RELEASE')
SAFE_ABORT, FAILURE, SUCCESS = DropTask.Result.SAFE_ABORT, DropTask.Result.FAILURE, DropTask.Result.SUCCESS


def _scenario(tmp_path, name: str, events=(), belt=None) -> str:
    """nominal_20mms 세계에 이벤트·벨트 설정과 감시 시험용 skill 소요시간을 넣은 시나리오 파일(축척 1.0으로 띄운다).

    t0_offset −2.5 s면 대상 뚜껑(x0 −0.05 m, 20 mm/s)이 시나리오 시각 9.5 s에 계획 시점(진입 3.0 s 전)이 되어
    DynamicPlace가 약 9.55 s에 시작한다. DynamicPlace 4.0 s는 phase 5개가 0.8 s씩이라(RELEASE 단계 제한 1.0 s 안)
    해제 응답이 약 12.75 s, RETREAT 끝이 약 13.55 s다. t0_offset −30 s면 두 번째 뚜껑(x0 0.55 m)으로 같은 일정이
    30 s 늦다(DynamicPlace 약 39.55 s 시작).
    """
    import yaml
    with open(scenario_file('nominal_20mms'), encoding='utf-8') as fh:
        sc = yaml.safe_load(fh)
    sc['name'] = name
    sc['world']['belt'].update(belt or {})
    sc['robot']['skills'] = {k: [{'duration_s': v}] for k, v in WATCH_SKILLS.items()}
    sc['events'] = [dict(e) for e in events]
    path = tmp_path / f'{name}.yaml'
    path.write_text(yaml.safe_dump(sc, allow_unicode=True, sort_keys=False), encoding='utf-8')
    return str(path)


def _watch_launch(tmp_path, scenario: str, *args):
    return _launch(f'scenario:={scenario}', 'duration_scale_ratio:=1.0', f'records_dir:={tmp_path}',
                   f'date:={DATE_STR}', *args)


def _cycle(h, task_msgs, step_msgs, timeout_s: float = 30.0):
    """MODE_SYNCHRONIZED 한 사이클. (결과, 그 사이클 StepRecord 열, 상태 열)."""
    start = len(task_msgs)
    _, res = _send_goal(h, _drop_goal(timeout=timeout_s), timeout=timeout_s + 15.0)
    cyc_id = task_msgs[-1][1].cycle_id
    time.sleep(0.2)                                        # 마지막 StepRecord 수신
    return res, [m for _, m in step_msgs if m.cycle_id == cyc_id], _observed_states(task_msgs, start)


def _resume(h, task_msgs, reason: str) -> None:
    _set_params(h, 'orchestrator_node', [_param_str('resume_reason', reason)])
    h.wait_for(lambda: task_msgs[-1][1].state == 'IDLE', timeout=3.0, what='IDLE')


def _fault_record(steps, reason: str):
    rec = [m for m in steps if m.reason_code == reason]
    assert rec, [(m.step_name, m.result, m.reason_code) for m in steps]
    return rec[0]


def _state_at(task_steady, t: float) -> str:
    """steady 수신 시각 t에 orchestrator가 알린 마지막 상태."""
    before = [m.state for ts, m in task_steady if ts <= t]
    return before[-1] if before else ''


def test_i0_stale_target_observation_aborts_before_release(ros, tmp_path):
    """동적 구간에서 대상 track이 가려져 관측 나이가 0.3 s를 넘으면 해제 전 L3(`OBS_STALE`)."""
    occ_t = 10.0                                           # APPROACH 중(약 9.55 s 시작) 가림 시작
    path = _scenario(tmp_path, 'watch_occlusion', events=[{'type': 'occlusion', 't_s': occ_t, 'duration_s': 1.5}])
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _watch_launch(tmp_path, path)
    try:
        _wait_ready(h)
        t0 = _reset_cycle_clock(h, seed=0, t0_offset_s=-2.5)
        res, steps, states = _cycle(h, task_msgs, step_msgs)
        assert (res.result, res.reason_code) == (SAFE_ABORT, 'OBS_STALE'), (res.result, res.reason_code, states)
        rec = _fault_record(steps, 'OBS_STALE')
        assert rec.step_name in DYNAMIC and rec.abort_level == 3, (rec.step_name, rec.abort_level)
        assert 'RELEASE' not in [m.step_name for m in steps if m.result == 0]
        last_obs = t0 + occ_t - 1.0 / 30.0                  # 가림 직전 프레임
        assert _sec(rec.t_end) - last_obs > 0.3, _sec(rec.t_end) - last_obs
        assert states[-2:] == ['SAFE_ABORT', 'WAIT_OPERATOR'], states
    finally:
        _stop(proc)
        h.close()


def test_i0_ros_clock_regression(ros, tmp_path):
    """`use_sim_time`에서 /clock을 1 s 되돌린다: 들고 있으면(WAIT_CAP) L3, 아니면(PICK_STONE) L2, `CLOCK_JUMP`.

    역행 전에 고른 대상·계획은 쓰지 않는다(WAIT_CAP 뒤 PLAN_INTERCEPT 기록 없음).
    """
    clock = SteadyRosClock(domain_id=_ACTIVE_DOMAIN)
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _watch_launch(tmp_path, _scenario(tmp_path, 'watch_clock'), 'use_sim_time:=true')
    ac = None
    try:
        _wait_ready(h, timeout_s=30.0)
        ac = h.action(DropTask, 'drop_task', timeout=5.0)
        out = {}
        for where, want, level in (('WAIT_CAP', SAFE_ABORT, 3), ('PICK_STONE', FAILURE, 2)):
            _reset_cycle_clock(h, seed=0, t0_offset_s=-2.5, now=clock.now)
            start = len(task_msgs)
            gh = wait_future(ac.send_goal_async(_drop_goal(timeout=30.0)), 5.0)
            assert gh.accepted
            h.wait_for(lambda: any(m.state == where for _, m in task_msgs[start:]), 15.0, where)
            time.sleep(0.3)
            clock.jump(-1.0)
            res = wait_future(gh.get_result_async(), 30.0).result
            cyc_id = task_msgs[-1][1].cycle_id
            time.sleep(0.2)
            steps = [m for _, m in step_msgs if m.cycle_id == cyc_id]
            out[where] = (res.result, res.reason_code)
            assert (res.result, res.reason_code) == (want, 'CLOCK_JUMP'), (where, res.result, res.reason_code,
                                                                           [(m.step_name, m.reason_code) for m in steps])
            rec = _fault_record(steps, 'CLOCK_JUMP')
            assert (rec.step_name, rec.abort_level) == (where, level)
            assert 'PLAN_INTERCEPT' not in [m.step_name for m in steps]
            _resume(h, task_msgs, f'clock regression at {where} checked')
    finally:
        log = _stop(proc)
        h.close()
        clock.close()
    assert log.count('ROS 시각 역행') >= 2, log[-3000:]


def test_i0_short_belt_pulse_does_not_abort(ros, tmp_path):
    """동적 구간의 0.5 s `steady=false`(맥동 경보 한 번)는 중단하지 않는다."""
    path = _scenario(tmp_path, 'watch_belt_pulse', belt={'unsteady_hold_s': 0.5},
                     events=[{'type': 'speed_change', 't_s': 10.0, 'speed_mps': 0.02}])
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    task_steady = h.record(TaskStatus, 'task/status', steady=True)
    belt = h.record(BeltMotion, 'belt/measured_motion', steady=True)
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _watch_launch(tmp_path, path)
    try:
        _wait_ready(h)
        _reset_cycle_clock(h, seed=0, t0_offset_s=-2.5)
        n_belt = len(belt)
        res, steps, states = _cycle(h, task_msgs, step_msgs)
        assert res.result == SUCCESS, (res.result, res.reason_code, [(m.step_name, m.reason_code) for m in steps])
        unsteady = [t for t, m in belt[n_belt:] if not m.steady]
        assert unsteady and all(_state_at(task_steady, t) in DYNAMIC for t in unsteady), \
            [(round(t, 3), _state_at(task_steady, t)) for t in unsteady]
        assert 0.4 <= unsteady[-1] - unsteady[0] <= 0.6
    finally:
        _stop(proc)
        h.close()


def test_i0_persistent_belt_unsteady_before_and_after_release(ros, tmp_path):
    """2.0 s `steady=false`: 해제 전이면 1.0 s를 넘는 순간 L3(`BELT_UNSTEADY`), 해제 뒤면 RETREAT→VERIFY로 계속한다."""
    pre_t, post_t = 10.0, 42.1                             # 첫 사이클 APPROACH 중 / 둘째 사이클 해제 0.65 s 전
    path = _scenario(tmp_path, 'watch_belt_unsteady', belt={'unsteady_hold_s': 2.0},
                     events=[{'type': 'speed_change', 't_s': pre_t, 'speed_mps': 0.02},
                             {'type': 'speed_change', 't_s': post_t, 'speed_mps': 0.02}])
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _watch_launch(tmp_path, path)
    try:
        _wait_ready(h)
        # 1) 해제 전: L3
        t0 = _reset_cycle_clock(h, seed=0, t0_offset_s=-2.5)
        res, steps, states = _cycle(h, task_msgs, step_msgs)
        assert (res.result, res.reason_code) == (SAFE_ABORT, 'BELT_UNSTEADY'), (res.result, res.reason_code, states)
        rec = _fault_record(steps, 'BELT_UNSTEADY')
        assert rec.step_name in DYNAMIC and rec.abort_level == 3, (rec.step_name, rec.abort_level)
        assert _sec(rec.t_end) - (t0 + pre_t) >= 1.0 - 1.0 / 30.0       # 1.0 s 지속 뒤에야 중단
        _resume(h, task_msgs, 'belt unsteady before release checked')

        # 2) 해제 뒤: 이벤트는 기록하고 RETREAT → VERIFY → DONE
        t0 = _reset_cycle_clock(h, seed=0, t0_offset_s=-30.0)
        res, steps, states = _cycle(h, task_msgs, step_msgs, timeout_s=40.0)
        names = [(m.step_name, m.result, m.reason_code, m.abort_level) for m in steps]
        assert res.result == SUCCESS, (res.result, res.reason_code, names)
        rec = _fault_record(steps, 'BELT_UNSTEADY')
        assert rec.step_name in ('RETREAT', 'VERIFY') and rec.abort_level == 0, names
        assert [m.step_name for m in steps][-3:] == ['RELEASE', 'RETREAT', 'VERIFY'], names
        assert states[-3:] == ['RETREAT', 'VERIFY', 'DONE'], states
    finally:
        _stop(proc)
        h.close()


def test_i0_gripper_status_watch(ros, tmp_path):
    """`robot/status` 그리퍼 응답 0.500 s는 유지, 0.501 s는 들고 있으면 L3, 아니면 L2(`GRIPPER_FAULT`).

    대역 응답 시간 = 0.1 s + `gripper_delay`. 첫 사이클(t0 −2.5 s)은 WAIT_CAP(들고 있음)에서 5~6 s 0.500 s, 7~8 s
    0.501 s, 둘째 사이클(t0 −20 s)은 PICK_STONE(들지 않음) 중 20.7 s부터 0.501 s다.
    """
    path = _scenario(tmp_path, 'watch_gripper', events=[
        {'type': 'gripper_delay', 't_s': 5.0, 'duration_s': 1.0, 'delay_s': 0.400},
        {'type': 'gripper_delay', 't_s': 7.0, 'duration_s': 1.0, 'delay_s': 0.401},
        {'type': 'gripper_delay', 't_s': 20.7, 'duration_s': 1.0, 'delay_s': 0.401}])
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    task_steady = h.record(TaskStatus, 'task/status', steady=True)
    robot = h.record(RobotStatus, 'robot/status', steady=True)
    step_msgs = h.record(StepRecord, 'task/step_record')
    proc = _watch_launch(tmp_path, path)
    try:
        _wait_ready(h)
        # 1) 들고 있음(WAIT_CAP): 0.500 s 동안 유지, 0.501 s에서 L3
        t0 = _reset_cycle_clock(h, seed=0, t0_offset_s=-2.5)
        n_robot = len(robot)
        res, steps, states = _cycle(h, task_msgs, step_msgs)
        assert (res.result, res.reason_code) == (SAFE_ABORT, 'GRIPPER_FAULT'), (res.result, res.reason_code, states)
        rec = _fault_record(steps, 'GRIPPER_FAULT')
        assert (rec.step_name, rec.abort_level) == ('WAIT_CAP', 3)
        assert _sec(rec.t_end) >= t0 + 7.0
        held = [t for t, m in robot[n_robot:] if m.gripper_response_s == pytest.approx(0.5, abs=1e-9)]
        assert held and all(_state_at(task_steady, t) == 'WAIT_CAP' for t in held)   # 0.500 s를 WAIT_CAP에서 받았다
        _resume(h, task_msgs, 'gripper fault while holding checked')

        # 2) 들지 않음(PICK_STONE): L2
        _reset_cycle_clock(h, seed=0, t0_offset_s=-20.0)
        res, steps, states = _cycle(h, task_msgs, step_msgs)
        assert (res.result, res.reason_code) == (FAILURE, 'GRIPPER_FAULT'), (res.result, res.reason_code, states)
        rec = _fault_record(steps, 'GRIPPER_FAULT')
        assert (rec.step_name, rec.abort_level) == ('PICK_STONE', 2)
    finally:
        _stop(proc)
        h.close()
