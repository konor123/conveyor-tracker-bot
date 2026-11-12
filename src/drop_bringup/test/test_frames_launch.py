"""frames.launch.py의 conveyor_frame 출처(SPEC-600 REQ-CAL-012, TEST_SPEC.md TC-CAL-012-03)."""
import os
import signal
import subprocess

import pytest

rclpy = pytest.importorskip('rclpy')

from rcl_interfaces.srv import GetParameters  # noqa: E402
from drop_ros import DOMAIN, Harness  # noqa: E402

pytestmark = [pytest.mark.req('REQ-CAL-012'), pytest.mark.tc('TC-CAL-012-03'), pytest.mark.level('I2')]


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.shutdown()


@pytest.mark.parametrize('args, origin_xy', [
    ((), (0.0, 0.0)),                                  # tracking.yaml(항등)
    (('scenario:=nominal_20mms',), (0.30, -0.25)),     # mock 세계와 같은 변환
], ids=['tracking_yaml', 'scenario'])
def test_conveyor_frame_source(ros, args, origin_xy):
    proc = subprocess.Popen(['ros2', 'launch', 'drop_bringup', 'frames.launch.py', 'publish_world:=true', *args],
                            env=dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN)), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True)
    h = Harness()
    try:
        p = h.call(GetParameters, '/static_frames_publisher/get_parameters', timeout=20.0,
                   names=['T_base_conveyor']).values[0].double_array_value
        assert (p[3], p[7]) == pytest.approx(origin_xy)
        assert proc.poll() is None
    finally:
        h.close()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
