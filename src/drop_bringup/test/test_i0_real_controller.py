"""I0: 오케스트레이터 + 실제 robot_controller_node(backend:=fake) + 실제 tracker·planner + mock 세계(vision 역할).

TEST_SPEC.md 3.13절 TC-INT-014-01~018-01. 실제 controller는 시뮬레이션 backend만 쓰며 `/dsr01`에
연결하지 않는다. mock 세계의 VerifyDrop은 실제 controller가 떨어뜨린 공깃돌을 모르므로 UNKNOWN을 낸다
(SPEC-700 REQ-MCK-010): 정상 사이클의 결과는 SPEC-100 4.7절대로 `UNKNOWN`이다. skill 결과(`released`,
`stone_placed_back`)는 오케스트레이터만 받으므로 action 서버의 get_result 서비스로 다시 읽는다.
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import tempfile
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatusArray  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from drop_interfaces.action import DropTask, DynamicPlace, PlaceStone, SafeRetreat  # noqa: E402
from drop_interfaces.msg import Heartbeat, RobotStatus, StepRecord, TaskStatus  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_mocks.ros_common import scenario_file  # noqa: E402
from drop_ros import Harness, wait_future  # noqa: E402
from orchestrator.record_logger_node import trials_path, validate_trial_record  # noqa: E402
from rcl_interfaces.msg import Parameter as RosParam  # noqa: E402
from rcl_interfaces.msg import ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import GetParameters, SetParameters  # noqa: E402
from spec_metrics import EXPECTED_METRIC_KEYS  # noqa: E402

DATE_STR = '2026-11-13'
ROBOT = 'robot_controller_node'
BRINGUP_SETTLE_S = 5.0     # 기동 직후 발행 정지 관찰 2.5~3.9 s 뒤(records/reviews/decisions-2026-11-14.md D3)
BRINGUP_RELEASE_REASON = 'i0: bring-up supervisor heartbeat transient'
DYNAMIC_STATES = ['PRECHECK', 'DETECT_STONE', 'PICK_STONE', 'MOVE_STANDBY', 'WAIT_CAP', 'PLAN_INTERCEPT', 'APPROACH',
                  'SYNCHRONIZE', 'DESCEND', 'RELEASE', 'RETREAT', 'VERIFY', 'DONE']
STATIC_STATES = ['PRECHECK', 'DETECT_STONE', 'PICK_STONE', 'MOVE_STANDBY', 'WAIT_CAP', 'APPROACH', 'DESCEND', 'RELEASE',
                 'RETREAT', 'VERIFY', 'DONE']
UNKNOWN = DropTask.Result.UNKNOWN
VERDICT_UNKNOWN = 2.0
_DOMAIN_SEQ = 0
_ACTIVE_DOMAIN = 0


@pytest.fixture
def ros():
    global _DOMAIN_SEQ, _ACTIVE_DOMAIN
    _DOMAIN_SEQ += 1
    _ACTIVE_DOMAIN = 100 + (os.getpid() + _DOMAIN_SEQ * 17 + 7) % 100
    rclpy.init(domain_id=_ACTIVE_DOMAIN)
    yield
    rclpy.shutdown()


# ------------------------------------------------------------------ 기동
def _launch(tmp_path, *args):
    """mock vision + 실제 tracker·planner·robot_controller(backend:=fake) + 오케스트레이터."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(_ACTIVE_DOMAIN), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    log = tempfile.NamedTemporaryFile(mode='w+', encoding='utf-8', prefix='drop-i0-real-', suffix='.log',
                                      delete=False)
    proc = subprocess.Popen(
        ['ros2', 'launch', 'drop_bringup', 'drop.launch.py', 'use_mock_vision:=true', 'use_mock_tracking:=false',
         'use_mock_planner:=false', 'use_mock_robot:=false', 'robot_backend:=fake', f'records_dir:={tmp_path}',
         f'date:={DATE_STR}', *args],
        env=env, stdout=log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
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
    print(f'I0 launch log: {proc.i0_log.name}\n{out[-15000:]}')
    proc.i0_log.close()
    return out


def _params_file(tmp_path, **overrides) -> str:
    """robot_params_file: robot_controller_node 파라미터 덮어쓰기(None = 미설정)."""
    path = tmp_path / 'robot_params.yaml'
    path.write_text(yaml.safe_dump({ROBOT: {'ros__parameters': overrides}}), encoding='utf-8')
    return str(path)


def _set_params(h, node, params):
    res = h.call(SetParameters, f'/{node}/set_parameters', parameters=params, timeout=5.0)
    assert all(r.successful for r in res.results), f'{node} set_parameters 실패: {res.results}'


def _param_str(name, val):
    return RosParam(name=name, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(val)))


def _param_double(name, val):
    return RosParam(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val)))


def _param_int(name, val):
    return RosParam(name=name, value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(val)))


def _stop_history(h) -> list[dict]:
    res = h.call(GetParameters, f'/{ROBOT}/get_parameters', names=['stop_history_json'], timeout=5.0)
    return json.loads(res.values[0].string_value)


def _wait_ready(h, timeout_s: float = 40.0) -> None:
    """TEST_SPEC 0.3절 I0 기동 절차를 실제 controller에 적용한다(Stop 이력은 controller 진단 파라미터)."""
    hbs = h.record(Heartbeat, 'system/heartbeat', steady=True)
    robot = h.record(RobotStatus, 'robot/status', steady=True)
    statuses = h.record(TaskStatus, 'task/status')
    want = {'vision_node', 'tracker_node', ROBOT}
    h.wait_for(lambda: want <= {m.node_name for _, m in list(hbs) if m.lifecycle_state == 3} and statuses,
               timeout_s, 'I0 기동(heartbeat ACTIVE·task/status)')
    time.sleep(BRINGUP_SETTLE_S)
    stops = _stop_history(h)
    sup = [s['reason_code'] for s in stops if s['requester'] == Stop.Request.SUPERVISOR]
    assert set(sup) <= {'CONTROLLER_HEARTBEAT_LOST'} and len(sup) == len(stops), f'기동 중 예기치 않은 Stop: {stops}'
    if sup:
        print(f'I0 기동 과도 감시 노드 Stop {len(sup)}회 → 해제', flush=True)
        _set_params(h, ROBOT, [_param_str('stop_release_reason', BRINGUP_RELEASE_REASON)])
    t_rel = time.monotonic()
    h.wait_for(lambda: any(t > t_rel and m.node_name == ROBOT and m.error_code == 0 for t, m in list(hbs))
               and any(t > t_rel and m.ready for t, m in list(robot)),
               timeout=5.0, what='해제 뒤 controller heartbeat error_code 0·RobotStatus.ready')
    h.bringup_stops = len(stops)


def _stops_after_settle(h) -> list[dict]:
    return _stop_history(h)[getattr(h, 'bringup_stops', 0):]


def _reset_world_clock(h, seed: int, t0_offset_s: float) -> None:
    """mock 세계의 시나리오 시각 0을 지금 + t0_offset_s로 옮긴다(실제 controller는 시나리오를 쓰지 않는다)."""
    _set_params(h, 'mock_world_node', [_param_double('t0_s', time.time() + t0_offset_s), _param_int('seed', seed)])
    time.sleep(0.08)


# ------------------------------------------------------------------ 관찰
def _servers(h, service):
    names = []
    for name, ns in h.client.get_node_names_and_namespaces():
        try:
            served = h.client.get_service_names_and_types_by_node(name, ns)
        except Exception:                      # 노드가 그 사이 사라짐
            continue
        if any(s == service for s, _ in served):
            names.append(name)
    return sorted(names)


def _goal_ids(status_box) -> list:
    """action status 기록에서 goal id를 처음 나온 순서로."""
    seen, out = set(), []
    for _, msg in list(status_box):
        for st in msg.status_list:
            key = bytes(st.goal_info.goal_id.uuid)
            if key not in seen:
                seen.add(key)
                out.append(st.goal_info.goal_id)
    return out


def _results(h, action_type, name, status_box) -> list:
    """`name` action 서버에 들어온 goal마다 (status, result)를 get_result 서비스로 읽는다."""
    out = []
    for gid in _goal_ids(status_box):
        res = h.call(action_type.Impl.GetResultService, f'{name}/_action/get_result', timeout=5.0, goal_id=gid)
        out.append((res.status, res.result))
    return out


def _record_action(h, name):
    return h.record(GoalStatusArray, f'{name}/_action/status')


def _send_goal(h, goal, timeout):
    ac = h.action(DropTask, 'drop_task', timeout=5.0)
    gh = wait_future(ac.send_goal_async(goal), 5.0)
    assert gh.accepted
    return wait_future(gh.get_result_async(), timeout).result


def _observed_states(task_msgs, start):
    observed = []
    for _, m in task_msgs[start:]:
        if not observed or observed[-1] != m.state:
            observed.append(m.state)
    while observed and observed[0] in ('IDLE', 'DONE', 'WAIT_OPERATOR'):
        observed = observed[1:]
    return observed


def _cycle_steps(task_msgs, step_msgs):
    cyc = task_msgs[-1][1].cycle_id
    time.sleep(0.3)                                      # 마지막 StepRecord 수신
    return [m for _, m in step_msgs if m.cycle_id == cyc]


def _brief(steps):
    return [(m.step_name, m.result, m.reason_code, m.abort_level,
             {k: round(v, 5) for k, v in zip(m.metric_keys, m.metric_values)}) for m in steps]


def _read_jsonl(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding='utf-8') as fh:
        return [json.loads(line) for line in fh if line.strip()]


def _drop_goal(mode, timeout=30.0, max_passes=3):
    return DropTask.Goal(mode=mode, max_belt_speed=0.05, timeout=timeout, max_passes=max_passes)


# ------------------------------------------------------------------ TC-INT-014-01
@pytest.mark.req('REQ-ORC-001', 'REQ-ROB-008', 'REQ-MCK-010')
@pytest.mark.tc('TC-INT-014-01')
@pytest.mark.level('I0')
@pytest.mark.realtime_pc       # 실제 controller 100 Hz 동적 루프(간격 한도 30 ms, LOOP_GAP): PC 판정, CI는 기록만
@pytest.mark.parametrize('mode', [DropTask.Goal.MODE_SYNCHRONIZED, DropTask.Goal.MODE_PREDICTED],
                         ids=['synchronized', 'predicted'])
def test_i0_real_controller_dynamic_cycle(ros, tmp_path, mode):
    """실제 controller의 DynamicPlace로 한 사이클: 상태 열은 대역과 같고 DynamicPlace OK·`released`, VERIFY는 UNKNOWN."""
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    dyn_status = _record_action(h, 'robot/dynamic_place')
    proc = _launch(tmp_path, 'scenario:=nominal_20mms')
    try:
        _wait_ready(h)
        # 단일 제공자: robot 역할 서버는 실제 controller 하나뿐(대역 없음)
        assert _servers(h, '/robot/dynamic_place/_action/send_goal') == [ROBOT]
        assert _servers(h, '/robot/stop') == [ROBOT]
        assert 'mock_robot_controller_node' not in h.client.get_node_names()
        _reset_world_clock(h, seed=0, t0_offset_s=-2.5)
        start = len(task_msgs)
        res = _send_goal(h, _drop_goal(mode), timeout=45.0)
        steps = _cycle_steps(task_msgs, step_msgs)
        assert (res.result, res.reason_code) == (UNKNOWN, ''), (res.result, res.reason_code, _brief(steps))
        assert _observed_states(task_msgs, start) == DYNAMIC_STATES
        assert [m.step_name for m in steps] == DYNAMIC_STATES[:-1]
        assert all(m.result == 0 and m.abort_level == 0 for m in steps), _brief(steps)
        by_name = {m.step_name: dict(zip(m.metric_keys, m.metric_values)) for m in steps}
        assert by_name['VERIFY'] == {'responded': 1.0, 'verdict': VERDICT_UNKNOWN}
        assert by_name['RETREAT']['tcp_z_m'] >= 0.10 and by_name['RETREAT']['outside_workspace'] == 1.0
        sync_keys = 'SYNCHRONIZE' if mode == DropTask.Goal.MODE_SYNCHRONIZED else 'SYNCHRONIZE_PREDICTED'
        assert set(by_name['SYNCHRONIZE']) == EXPECTED_METRIC_KEYS[sync_keys]
        results = _results(h, DynamicPlace, 'robot/dynamic_place', dyn_status)
        assert len(results) == 1, results
        status, r = results[0]
        assert (status, r.result, r.reason_code, r.released) == (4, 0, '', True), (status, r)
        assert all(math.isfinite(v) for v in (r.gate_e_xy, r.gate_v_rel, r.gate_t_since_obs, r.gate_position_std))
        assert _stops_after_settle(h) == []
        t_file = trials_path(str(tmp_path), DATE_STR)
        h.wait_for(lambda: _read_jsonl(t_file), 5.0, 'trial 기록')
        trial = _read_jsonl(t_file)[-1]
        assert (trial['result'], trial['abort_level']) == ('UNKNOWN', 0)
        assert validate_trial_record(trial) == []
    finally:
        log = _stop(proc)
        h.close()
    assert 'Traceback' not in log, log[-3000:]


# ------------------------------------------------------------------ TC-INT-015-01
@pytest.mark.req('REQ-ORC-001', 'REQ-ROB-007')
@pytest.mark.tc('TC-INT-015-01')
@pytest.mark.level('I0')
def test_i0_real_controller_static_cycle(ros, tmp_path):
    """MODE_STATIC: 실제 controller의 PlaceStone으로 APPROACH~RETREAT. 단계 기록은 모두 OK, VERIFY는 UNKNOWN."""
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    place_status = _record_action(h, 'robot/place_stone')
    proc = _launch(tmp_path, 'scenario:=nominal_20mms')
    try:
        _wait_ready(h)
        _reset_world_clock(h, seed=0, t0_offset_s=-2.5)
        start = len(task_msgs)
        res = _send_goal(h, _drop_goal(DropTask.Goal.MODE_STATIC, max_passes=1), timeout=45.0)
        steps = _cycle_steps(task_msgs, step_msgs)
        assert (res.result, res.reason_code) == (UNKNOWN, ''), (res.result, res.reason_code, _brief(steps))
        assert _observed_states(task_msgs, start) == STATIC_STATES
        assert [m.step_name for m in steps] == STATIC_STATES[:-1]
        assert all(m.result == 0 and m.abort_level == 0 for m in steps), _brief(steps)
        by_name = {m.step_name: dict(zip(m.metric_keys, m.metric_values)) for m in steps}
        assert set(by_name['DESCEND']) == EXPECTED_METRIC_KEYS['DESCEND_STATIC']
        assert by_name['RELEASE']['sync_held'] == 1.0
        assert by_name['RETREAT']['tcp_z_m'] >= 0.10 and by_name['RETREAT']['outside_workspace'] == 1.0
        assert by_name['VERIFY']['verdict'] == VERDICT_UNKNOWN
        results = _results(h, PlaceStone, 'robot/place_stone', place_status)
        assert [(s, r.result, r.released) for s, r in results] == [(4, 0, True)], results
        assert _stops_after_settle(h) == []
    finally:
        log = _stop(proc)
        h.close()
    assert 'Traceback' not in log, log[-3000:]


# ------------------------------------------------------------------ TC-INT-016-01
def _scenario(tmp_path, base: str, name: str, zones=None) -> str:
    with open(scenario_file(base), encoding='utf-8') as fh:
        sc = yaml.safe_load(fh)
    sc['name'] = name
    sc['world']['zones'].update(zones or {})
    path = tmp_path / f'{name}.yaml'
    path.write_text(yaml.safe_dump(sc, allow_unicode=True, sort_keys=False), encoding='utf-8')
    return str(path)


def _launch_module():
    import importlib.util
    path = os.path.join(get_package_share_directory('drop_bringup'), 'launch', 'drop.launch.py')
    spec = importlib.util.spec_from_file_location('drop_launch_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _last_status(robot):
    return robot[-1][1]


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


@pytest.mark.req('REQ-ORC-007', 'REQ-ROB-016', 'REQ-ABT-002')
@pytest.mark.tc('TC-INT-016-01')
@pytest.mark.level('I0')
@pytest.mark.parametrize('place_back', ['set', 'unset'])
def test_i0_real_controller_passes_exhausted_place_back(ros, tmp_path, place_back):
    """통과 보내기 소진(L3): 실제 controller가 SafeRetreat(keep_sync=false, place_back=true)로 공깃돌을 매트에 내려놓고
    `NOT_ATTEMPTED`로 끝난다(L4·Stop 없음). 반납 위치가 미설정이면 SafeRetreat는 지령 없이 PRECONDITION
    (`PLACE_BACK_POSE_UNSET`)이고 중단 경로 실패로 L4(Stop, `PROTECTIVE_STOP`)다(SPEC-110 4.2절).

    도달 구간을 0.35~0.36 m로 줄인 time_insufficient 세계: 계획 시점(진입 3.0 s 전, x ≥ 0.29 m)의 T_available ≤ 3.5 s가
    T_required 3.6 s + margin 1.0 s보다 짧아 어느 통과든 PASS_SKIP이다. max_passes 0이면 첫 통과에서 소진한다.
    """
    path = _scenario(tmp_path, 'time_insufficient', 'real_passes_exhausted', zones={'reach_end_m': 0.36})
    geo = _launch_module()._robot_params('fake', path, True, False)
    standby = geo['standby_pose'][:3]
    extra = [f'robot_params_file:={_params_file(tmp_path, place_back_xyz_m=None)}'] if place_back == 'unset' else []
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    robot = h.record(RobotStatus, 'robot/status')
    retreat_status = _record_action(h, 'robot/safe_retreat')
    dyn_status = _record_action(h, 'robot/dynamic_place')
    proc = _launch(tmp_path, f'scenario:={path}', *extra)
    try:
        _wait_ready(h)
        _reset_world_clock(h, seed=0, t0_offset_s=-2.0)
        start = len(task_msgs)
        res = _send_goal(h, _drop_goal(DropTask.Goal.MODE_SYNCHRONIZED, timeout=40.0, max_passes=0), timeout=55.0)
        steps = _cycle_steps(task_msgs, step_msgs)
        wait_cap = [m for m in steps if m.step_name == 'WAIT_CAP']
        assert [(m.result, m.reason_code, m.abort_level) for m in wait_cap] == [(1, 'PASS_SKIP', 3)], _brief(steps)
        assert res.passes_skipped == 1
        assert _results(h, DynamicPlace, 'robot/dynamic_place', dyn_status) == []
        retreats = _results(h, SafeRetreat, 'robot/safe_retreat', retreat_status)
        assert len(retreats) == 1, retreats
        status, r = retreats[0]
        h.wait_for(lambda: task_msgs[-1][1].state == 'WAIT_OPERATOR', 3.0, 'WAIT_OPERATOR')
        time.sleep(0.3)                                  # 반납 뒤 robot/status
        last = _last_status(robot)
        tcp = (last.tcp_pose.position.x, last.tcp_pose.position.y, last.tcp_pose.position.z)
        if place_back == 'set':
            assert (res.result, res.reason_code) == (DropTask.Result.NOT_ATTEMPTED, 'PASS_SKIP'), _brief(steps)
            assert (status, r.result, r.reason_code, r.stone_placed_back) == (4, 0, '', True), (status, r)
            assert _observed_states(task_msgs, start)[-3:] == ['WAIT_CAP', 'SAFE_ABORT', 'WAIT_OPERATOR']
            assert task_msgs[-1][1].abort_level == 3
            assert _stops_after_settle(h) == []                              # L4·Stop 없음
            want = geo['place_back_xyz_m'][:2] + [geo['safe_z_m']]           # 반납 위치 위 안전 높이로 올림
            assert tcp == pytest.approx(want, abs=1e-6)
            assert (last.holding, last.gripper_open, last.ready) == (False, True, True)
        else:
            assert (res.result, res.reason_code) == (DropTask.Result.PROTECTIVE_STOP, 'PASS_SKIP'), _brief(steps)
            assert (status, r.result, r.reason_code, r.stone_placed_back) == (6, 3, 'PLACE_BACK_POSE_UNSET', False)
            assert task_msgs[-1][1].abort_level == 4
            stops = _stops_after_settle(h)
            assert [(s['requester'], s['reason_code']) for s in stops] == [
                (Stop.Request.ORCHESTRATOR, 'ORCHESTRATOR_L4')], stops
            assert tcp == pytest.approx(standby, abs=1e-6)                   # 지령 없음: 대기 자세 그대로
            assert (last.holding, last.ready) == (True, False)               # 공깃돌을 쥔 채 정지 유지
    finally:
        log = _stop(proc)
        h.close()
    assert 'Traceback' not in log, log[-3000:]


# ------------------------------------------------------------------ TC-INT-017-01
@pytest.mark.req('REQ-ROB-010', 'REQ-ROB-016', 'REQ-ABT-010')
@pytest.mark.tc('TC-INT-017-01')
@pytest.mark.level('I0')
@pytest.mark.realtime_pc       # 실제 controller 100 Hz 동적 루프(간격 한도 30 ms, LOOP_GAP): PC 판정, CI는 기록만
def test_i0_real_controller_dynamic_failure_keep_sync_place_back(ros, tmp_path):
    """해제 전 동적 구간 실패(L3): SYNCHRONIZE에서 vision heartbeat를 끊으면 orchestrator가 DynamicPlace를 cancel하고
    실제 controller가 SafeRetreat(keep_sync=true, place_back=true)로 벨트 속도를 유지하며 올린 뒤 매트에 내려놓는다.
    결과 `SAFE_ABORT`(`HEARTBEAT_LOST`), L4·Stop 없음. 안전 높이를 0.25 m로 올려 동기 유지 상승 구간(접근 높이
    0.2056 m → 0.252 m, 약 1 s)을 10 Hz `robot/status`로 볼 수 있게 한다.
    """
    params = _params_file(tmp_path, safe_z_m=0.25)
    geo = _launch_module()._robot_params('fake', 'nominal_20mms', True, False, params)
    assert geo['standby_pose'][2] == 0.25
    h = Harness()
    task_msgs = h.record(TaskStatus, 'task/status')
    step_msgs = h.record(StepRecord, 'task/step_record')
    robot = h.record(RobotStatus, 'robot/status')
    retreat_status = _record_action(h, 'robot/safe_retreat')
    dyn_status = _record_action(h, 'robot/dynamic_place')
    proc = _launch(tmp_path, 'scenario:=nominal_20mms', f'robot_params_file:={params}')
    try:
        _wait_ready(h)
        _reset_world_clock(h, seed=0, t0_offset_s=-2.5)
        start = len(task_msgs)
        ac = h.action(DropTask, 'drop_task', timeout=5.0)
        gh = wait_future(ac.send_goal_async(_drop_goal(DropTask.Goal.MODE_SYNCHRONIZED)), 5.0)
        assert gh.accepted
        h.wait_for(lambda: any(m.state == 'SYNCHRONIZE' for _, m in task_msgs[start:]), 30.0, 'SYNCHRONIZE')
        _set_params(h, 'mock_world_node', [_param_str('drop_role', 'vision'), _param_double('drop_at_s', 0.0)])
        res = wait_future(gh.get_result_async(), 30.0).result
        steps = _cycle_steps(task_msgs, step_msgs)
        dyn = [(s, r.result, r.reason_code, r.released)
               for s, r in _results(h, DynamicPlace, 'robot/dynamic_place', dyn_status)]
        retreats = [(s, r.result, r.reason_code, r.stone_placed_back)
                    for s, r in _results(h, SafeRetreat, 'robot/safe_retreat', retreat_status)]
        assert (res.result, res.reason_code) == (DropTask.Result.SAFE_ABORT, 'HEARTBEAT_LOST'), \
            (res.result, res.reason_code, dyn, retreats, _brief(steps))
        fault = [m for m in steps if m.reason_code == 'HEARTBEAT_LOST']
        assert fault and fault[0].step_name in ('SYNCHRONIZE', 'DESCEND') and fault[0].abort_level == 3, _brief(steps)
        assert 'RELEASE' not in [m.step_name for m in steps]
        assert _observed_states(task_msgs, start)[-2:] == ['SAFE_ABORT', 'WAIT_OPERATOR']
        assert dyn == [(5, 4, 'CANCELED', False)], dyn
        assert retreats == [(4, 0, '', True)], retreats
        assert _stops_after_settle(h) == []
        h.wait_for(lambda: task_msgs[-1][1].state == 'WAIT_OPERATOR', 3.0, 'WAIT_OPERATOR')
        time.sleep(0.3)
        # 동기 유지 상승: 벨트 쪽(y < −0.1 m)에서 z가 오르는 robot/status 표본 쌍(발행 stamp)의 x 속도 ≈ 계획 속도.
        # 앞 표본도 상승 시작 높이보다 위인 쌍만 센다. DynamicPlace 끝 → SafeRetreat 시작(약 25 ms) 사이에는 가짜
        # transport가 진행되지 않아 그때 발행된 위치가 stamp보다 오래될 수 있다(2026-11-13 사무실 PC WSL: 첫 쌍만 0.023~0.025)
        pts = [(_stamp(m), m.tcp_pose.position) for _, m in robot]
        z_start = min((p.z for _, p in pts if p.y < -0.1), default=0.0)
        rising = [((b.x - a.x) / (tb - ta)) for (ta, a), (tb, b) in zip(pts, pts[1:])
                  if tb > ta and a.y < -0.1 and b.y < -0.1 and a.z - z_start > 0.0005 and b.z - a.z > 0.0005
                  and b.z < geo['safe_z_m']]
        assert len(rising) >= 3 and all(abs(v - 0.02) <= 0.003 for v in rising), rising
        last = _last_status(robot)
        tcp = (last.tcp_pose.position.x, last.tcp_pose.position.y, last.tcp_pose.position.z)
        assert tcp == pytest.approx(geo['place_back_xyz_m'][:2] + [geo['safe_z_m']], abs=1e-6)
        assert (last.holding, last.gripper_open, last.ready) == (False, True, True)
    finally:
        log = _stop(proc)
        h.close()
    assert 'Traceback' not in log, log[-3000:]


# ------------------------------------------------------------------ TC-INT-018-01
@pytest.mark.req('REQ-MCK-007', 'REQ-ROB-005')
@pytest.mark.tc('TC-INT-018-01')
@pytest.mark.level('I0')
def test_real_controller_bringup_defaults_and_single_provider(ros, tmp_path):
    """use_mock_robot:=false의 기본 backend는 disabled(연결·준비 없음), robot 역할 서버·발행자는 실제 controller 하나."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(_ACTIVE_DOMAIN), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    log = tempfile.NamedTemporaryFile(mode='w+', encoding='utf-8', suffix='.log', delete=False)
    proc = subprocess.Popen(['ros2', 'launch', 'drop_bringup', 'drop.launch.py', 'use_mock_vision:=true',
                             'use_mock_tracking:=true', 'use_mock_planner:=true', 'launch_orchestrator:=false'],
                            env=env, stdout=log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    proc.i0_log = log
    h = Harness()
    robot = h.record(RobotStatus, 'robot/status')
    try:
        h.wait_for(lambda: len(robot) >= 2, 30.0, 'robot/status')
        time.sleep(1.0)                                  # 그래프 정착
        backend = h.call(GetParameters, f'/{ROBOT}/get_parameters', names=['backend']).values[0].string_value
        assert backend == 'disabled'
        assert (robot[-1][1].connected, robot[-1][1].ready) == (False, False)
        for service in ('/robot/stop', '/robot/dynamic_place/_action/send_goal', '/robot/safe_retreat/_action/send_goal',
                        '/robot/pick_stone/_action/send_goal'):
            assert _servers(h, service) == [ROBOT], service
        assert sorted(i.node_name for i in h.client.get_publishers_info_by_topic('/robot/status')) == [ROBOT]
        assert proc.poll() is None
    finally:
        h.close()
        out = _stop(proc)
    assert 'Traceback' not in out, out[-2000:]


@pytest.mark.req('REQ-ROB-016')
@pytest.mark.tc('TC-INT-018-01')
@pytest.mark.level('I0')
def test_robot_params_follow_mock_world_geometry(tmp_path):
    """backend:=fake + mock 세계: 대기 자세 = 도달 구간 시작 위 안전 높이, 반납 위치 = mock 세계의 매트 공깃돌(PickStone
    목표), 시작 TCP = 대기 자세. robot_params_file이 마지막에 덮고(null = 미설정), backend는 launch 인자만 정한다."""
    mod = _launch_module()
    p = mod._robot_params('fake', 'nominal_20mms', True, False)
    assert p['standby_pose'] == pytest.approx([0.50, -0.25, p['safe_z_m'], 0.0, 0.0, 0.0, 1.0])
    assert p['place_back_xyz_m'] == pytest.approx([0.35, 0.15, 0.008])
    assert (p['fake_tcp_x_m'], p['fake_tcp_y_m'], p['fake_tcp_z_m']) == pytest.approx(tuple(p['standby_pose'][:3]))
    assert (p['backend'], p['use_sim_time'], p['dynamic.open_width_m']) == ('fake', False, 0.022)
    from drop_mocks.scenario_engine import CLASS_STONE, REGION_MAT, ScenarioEngine
    stone = ScenarioEngine.from_file(scenario_file('nominal_20mms')).detect(5.0, REGION_MAT, CLASS_STONE, 1)
    assert p['place_back_xyz_m'] == pytest.approx(list(stone.detections[0].position))
    over = _params_file(tmp_path, place_back_xyz_m=None, safe_z_m=0.25, backend='fake')
    q = mod._robot_params('disabled', 'nominal_20mms', True, False, over)
    assert 'place_back_xyz_m' not in q and q['safe_z_m'] == 0.25 and q['backend'] == 'disabled'
    q = mod._robot_params('fake', 'nominal_20mms', True, False, over)       # 대기 자세는 덮어쓴 안전 높이
    assert 'place_back_xyz_m' not in q and q['standby_pose'][:3] == pytest.approx([0.50, -0.25, 0.25])
    assert q['fake_tcp_z_m'] == 0.25
    assert mod._robot_params('disabled', 'nominal_20mms', True, True) == {'backend': 'disabled', 'use_sim_time': True}
