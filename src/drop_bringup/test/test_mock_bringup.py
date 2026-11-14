"""mock bringup 시험."""
import os
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from drop_interfaces.msg import Heartbeat  # noqa: E402
from drop_interfaces.srv import DetectObjects  # noqa: E402
from drop_ros import DOMAIN, Harness  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.shutdown()


def _launch(*args):
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    return subprocess.Popen(['ros2', 'launch', 'drop_bringup', 'mock.launch.py', *args], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)


def _stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
    return proc.stdout.read()


def test_mock_refuses_when_real_provider_is_running(ros):
    real = rclpy.create_node('vision_node')                 # 실제 vision 역할을 흉내 내는 서버
    real.create_service(DetectObjects, 'vision/detect_objects', lambda req, res: res)
    h = Harness(real)
    proc = _launch()
    try:
        proc.wait(timeout=20)                               # 대역이 거부 → bringup 전체 종료
    except subprocess.TimeoutExpired:
        pass
    finally:
        out = _stop(proc)
        h.close()
    assert proc.returncode is not None
    assert 'REAL_NODE_PRESENT' in out, out[-2000:]
    assert 'bringup 중단' in out


def test_mock_bringup_starts_all_roles(ros):
    h = Harness()
    hbs = h.record(Heartbeat, 'system/heartbeat')
    proc = _launch()
    try:
        end = time.monotonic() + 15.0
        want = {'vision_node', 'tracker_node', 'robot_controller_node'}
        while time.monotonic() < end and not want <= {m.node_name for _, m in hbs}:
            time.sleep(0.1)
        assert proc.poll() is None
        assert want <= {m.node_name for _, m in hbs}
    finally:
        _stop(proc)
        h.close()
