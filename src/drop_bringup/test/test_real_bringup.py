"""실제 planner·tracker bringup."""
import importlib.util
import os
import signal
import subprocess
import tempfile
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from lifecycle_msgs.msg import State  # noqa: E402
from rcl_interfaces.srv import GetParameters  # noqa: E402
from drop_ros import DOMAIN, Harness  # noqa: E402

REACH_PARAMS = {                                 # 노드 → (도달 구간 시작, 끝) 파라미터 이름(끝이 없으면 None)
    'tracker_node': ('select.x_reach_start_m', None),
    'intercept_planner_node': ('reach.x_reach_start_m', 'budget.x_exit_m'),
    'orchestrator_node': ('budget.x_reach_start_m', 'budget.x_exit_m'),
}


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.shutdown()


def _launch(*args, orchestrator=False, records_dir=''):
    """출력은 임시 파일로 받는다(노드가 많으면 읽지 않는 PIPE가 차서 launch가 멈춘다)."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    log = tempfile.TemporaryFile(mode='w+', encoding='utf-8')
    extra = [f'records_dir:={records_dir}'] if records_dir else []
    proc = subprocess.Popen(['ros2', 'launch', 'drop_bringup', 'drop.launch.py',
                             f'launch_orchestrator:={"true" if orchestrator else "false"}',
                             'use_mock_vision:=true', 'use_mock_robot:=true', *extra, *args], env=env,
                            stdout=log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    proc.log = log
    return proc


def _output(proc):
    proc.log.seek(0)
    return proc.log.read()


def _stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
    return _output(proc)


def _reach_window(h, node):
    """노드의 (도달 구간 시작, 끝). 끝 파라미터가 없는 노드(tracker)는 None."""
    start, end = REACH_PARAMS[node]
    names = [start] + ([end] if end else [])
    vals = [v.double_value for v in h.call(GetParameters, f'/{node}/get_parameters', names=names).values]
    return vals[0], (vals[1] if end else None)


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


def _publishers(h, topic):
    return sorted(i.node_name for i in h.client.get_publishers_info_by_topic(topic))


def _active(h, node):
    try:
        return h.lifecycle_state(node, timeout=1.0) == State.PRIMARY_STATE_ACTIVE
    except TimeoutError:
        return False


@pytest.mark.parametrize('real_tracker', [False, True], ids=['mock_tracking', 'real_tracker'])
def test_real_planner_replaces_mock_plan(ros, tmp_path, real_tracker):
    proc = _launch('use_mock_planner:=false', f'use_mock_tracking:={"false" if real_tracker else "true"}',
                   orchestrator=True, records_dir=str(tmp_path))
    h = Harness()
    try:
        h.wait_for(lambda: _active(h, 'intercept_planner_node'), 20.0, 'planner ACTIVE')
        if real_tracker:
            h.wait_for(lambda: _active(h, 'tracker_node'), 20.0, 'tracker ACTIVE')
        time.sleep(1.0)                        # 그래프 정착
        assert _servers(h, '/planner/compute_intercept') == ['intercept_planner_node']
        assert _publishers(h, '/drop/intercept_plan') == ['intercept_planner_node']
        if real_tracker:
            assert _publishers(h, '/objects/tracked') == ['tracker_node']
        # mock 세계의 컨베이어 위치·구간(nominal_20mms: (0.30, -0.25, 0), 관측 0~0.45, 도달 0.20~0.40)을 받는다
        p = h.call(GetParameters, '/intercept_planner_node/get_parameters',
                   names=['T_base_conveyor', 'reach.x_reach_start_m', 'budget.x_exit_m']).values
        assert (p[0].double_array_value[3], p[0].double_array_value[7]) == (0.30, -0.25)
        assert (p[1].double_value, p[2].double_value) == (0.20, 0.40)
        if real_tracker:
            p = h.call(GetParameters, '/tracker_node/get_parameters',
                       names=['T_base_conveyor', 'region.x_obs_max_m', 'select.x_reach_start_m']).values
            assert (p[0].double_array_value[3], p[1].double_value, p[2].double_value) == (0.30, 0.45, 0.20)
        # orchestrator도 같은 도달 구간을 쓴다(WAIT_CAP t_to_window·시간 예산)
        h.wait_for(lambda: 'orchestrator_node' in h.client.get_node_names(), 20.0, 'orchestrator 기동')
        assert _reach_window(h, 'orchestrator_node') == (0.20, 0.40)
        # 자동 활성화는 기동 때 한 번뿐이다. 사람이 끈 노드를 launch가 다시 켜면 안 된다
        assert h.deactivate('intercept_planner_node')
        time.sleep(1.5)
        assert h.lifecycle_state('intercept_planner_node') == State.PRIMARY_STATE_INACTIVE
        assert proc.poll() is None
    finally:
        h.close()
        out = _stop(proc)
    assert 'Traceback' not in out, out[-2000:]


@pytest.mark.parametrize('scenario', ['time_insufficient'])
def test_reach_window_is_the_same_in_three_nodes(ros, tmp_path, scenario):
    """도달 구간이 다른 시나리오(time_insufficient: 0.35~0.40 m)에서도 tracker·planner·orchestrator가 같은 값을 쓴다."""
    with open(os.path.join(get_package_share_directory('drop_mocks'), 'scenarios', f'{scenario}.yaml')) as fh:
        zones = yaml.safe_load(fh)['world']['zones']
    want = (zones['reach_start_m'], zones['reach_end_m'])
    assert want[0] != 0.20                         # orchestrator.yaml 기본값과 다른 시나리오여야 주입을 확인한다
    proc = _launch('use_mock_planner:=false', 'use_mock_tracking:=false', f'scenario:={scenario}',
                   orchestrator=True, records_dir=str(tmp_path))
    h = Harness()
    try:
        for node in REACH_PARAMS:
            h.wait_for(lambda: node in h.client.get_node_names(), 20.0, f'{node} 기동')
            got = _reach_window(h, node)
            assert got == (want[0], want[1] if REACH_PARAMS[node][1] else None), (node, got)
    finally:
        h.close()
        out = _stop(proc)
    assert 'Traceback' not in out, out[-2000:]


def _launch_module():
    path = os.path.join(get_package_share_directory('drop_bringup'), 'launch', 'drop.launch.py')
    spec = importlib.util.spec_from_file_location('drop_launch_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_real_layout_goes_to_three_nodes():
    """실제 bringup(검출·track 모두 실제)은 system.yaml layout 하나를 세 노드에 넣는다. 그 값은 각 패키지 기본값과 같다."""
    mod = _launch_module()
    with open(os.path.join(get_package_share_directory('drop_bringup'), 'config', 'system.yaml')) as fh:
        layout = yaml.safe_load(fh)['layout']
    start, end = layout['x_reach_start_m'], layout['x_exit_m']
    tracker, planner, orch = mod._geometry_params('nominal_20mms', mock_world=False)
    assert tracker == {'select.x_reach_start_m': start}
    assert planner == {'reach.x_reach_start_m': start, 'budget.x_exit_m': end}
    assert orch == {'budget.x_reach_start_m': start, 'budget.x_exit_m': end}
    with open(os.path.join(get_package_share_directory('tracking'), 'config', 'tracking.yaml')) as fh:
        assert yaml.safe_load(fh)['select']['x_reach_start_m'] == start
    with open(os.path.join(get_package_share_directory('intercept_planner'), 'config', 'planner.yaml')) as fh:
        p = yaml.safe_load(fh)
        assert (p['reach']['x_reach_start_m'], p['budget']['x_exit_m']) == (start, end)
    # mock 세계면 시나리오 world.zones
    tracker, planner, orch = mod._geometry_params('time_insufficient', mock_world=True)
    assert (tracker['select.x_reach_start_m'], planner['reach.x_reach_start_m'], planner['budget.x_exit_m']) \
        == (0.35, 0.35, 0.40)
    assert orch == {'budget.x_reach_start_m': 0.35, 'budget.x_exit_m': 0.40}


def test_mock_planner_without_mock_tracking_is_refused():
    """임시 plan은 tracking 대역이 낸다. 실제 tracker + mock planner는 계획을 내는 노드가 없어 기동을 거부한다."""
    proc = _launch('use_mock_planner:=true', 'use_mock_tracking:=false')
    try:
        code = proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        _stop(proc)
        pytest.fail('launch가 끝나지 않았다')
    out = _output(proc)
    assert code != 0
    assert 'use_mock_planner:=true는 use_mock_tracking:=true가 필요하다' in out
    assert 'tracker_node' not in out


def test_planner_default_is_real(ros):
    """use_mock_planner를 주지 않으면 실제 planner다(다른 역할과 같은 기본값)."""
    proc = _launch('use_mock_tracking:=true')
    h = Harness()
    try:
        h.wait_for(lambda: _active(h, 'intercept_planner_node'), 20.0, 'planner ACTIVE')
        time.sleep(1.0)
        assert _publishers(h, '/drop/intercept_plan') == ['intercept_planner_node']
    finally:
        h.close()
        _stop(proc)
