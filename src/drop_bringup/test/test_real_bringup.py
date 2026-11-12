"""실제 planner·tracker bringup(SPEC-400 REQ-PLN-012, TEST_SPEC.md TC-PLN-012-01)."""
import os
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from lifecycle_msgs.msg import State  # noqa: E402
from rcl_interfaces.srv import GetParameters  # noqa: E402
from drop_ros import DOMAIN, Harness  # noqa: E402

pytestmark = [pytest.mark.req('REQ-PLN-012'), pytest.mark.tc('TC-PLN-012-01'), pytest.mark.level('I2')]


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.shutdown()


def _launch(*args):
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    return subprocess.Popen(['ros2', 'launch', 'drop_bringup', 'drop.launch.py', 'launch_orchestrator:=false',
                             'use_mock_vision:=true', 'use_mock_robot:=true', *args], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def _stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
    return proc.stdout.read()


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
def test_real_planner_replaces_mock_plan(ros, real_tracker):
    proc = _launch('use_mock_planner:=false', f'use_mock_tracking:={"false" if real_tracker else "true"}')
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
        # 자동 활성화는 기동 때 한 번뿐이다. 사람이 끈 노드를 launch가 다시 켜면 안 된다(TC-INT-006 전제)
        assert h.deactivate('intercept_planner_node')
        time.sleep(1.5)
        assert h.lifecycle_state('intercept_planner_node') == State.PRIMARY_STATE_INACTIVE
        assert proc.poll() is None
    finally:
        h.close()
        out = _stop(proc)
    assert 'Traceback' not in out, out[-2000:]


def test_mock_planner_without_mock_tracking_is_refused():
    """임시 plan은 tracking 대역이 낸다. 실제 tracker + mock planner는 계획을 내는 노드가 없어 기동을 거부한다."""
    proc = _launch('use_mock_planner:=true', 'use_mock_tracking:=false')
    try:
        code = proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        _stop(proc)
        pytest.fail('launch가 끝나지 않았다')
    out = proc.stdout.read()
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
