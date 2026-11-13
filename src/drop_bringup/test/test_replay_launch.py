"""replay bringup 시험(PLAN.md 7.4절 5항·7.5절 I1, TEST_SPEC.md TC-INT-007-01).

replay.launch.py는 bag 영상 → 실제 vision → tracker → planner만 띄우고 로봇 지령 경로를 만들지 않는다.
시험 안에서 합성 bag을 쓴다(외부 자료 불필요). bag에는 영상과 함께 미끼 토픽을 넣는다:
Doosan 스트림 지령(`/dsr01/servol_stream`)과 이전 파이프라인 출력(`/objects/detections`). 재생기는 영상 토픽만 내야 한다.

판정은 ROS 그래프로 한다. 대상 노드의 heartbeat(ACTIVE)와 검출·track 흐름을 먼저 확인한 뒤, 같은 그래프 조회로
금지 항목(스트림 토픽 발행자, `robot/*` action 클라이언트·서버, `/dsr01/*`·`/robot/*` 서비스 클라이언트)을 센다.
조회가 금지 항목을 실제로 찾는지는 시험 프로세스의 탐침 노드(같은 금지 항목을 일부러 만든다)로 확인한다.
"""
import collections
import os
import signal
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from drop_interfaces.action import PickStone  # noqa: E402
from drop_interfaces.msg import Heartbeat, ObjectDetectionArray, TrackedObjectArray  # noqa: E402
from drop_interfaces.srv import Stop  # noqa: E402
from drop_ros import DOMAIN, Harness  # noqa: E402
from rclpy.action import (ActionClient, get_action_client_names_and_types_by_node,  # noqa: E402
                          get_action_server_names_and_types_by_node)
from std_msgs.msg import Float64MultiArray  # noqa: E402

pytestmark = [pytest.mark.tc('TC-INT-007-01'), pytest.mark.level('I1')]

IMAGE = '/camera/camera/color/image_raw'
INFO = '/camera/camera/color/camera_info'
STREAM_DECOY = '/dsr01/servol_stream'
NODES = ('vision_node', 'tracker_node', 'intercept_planner_node')
FORBIDDEN_NODES = {'robot_controller_node', 'mock_robot_controller_node', 'orchestrator_node',
                   'safety_supervisor_node', 'record_logger_node'}
PROBE = 'replay_probe'


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.try_shutdown()


def _write_bag(path, seconds=3.0, fps=30.0):
    """합성 bag(mcap): 회색 320×200 bgr8 영상·camera_info + 미끼 토픽 2개."""
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from builtin_interfaces.msg import Time
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, Image
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = {IMAGE: 'sensor_msgs/msg/Image', INFO: 'sensor_msgs/msg/CameraInfo',
              STREAM_DECOY: 'std_msgs/msg/Float64MultiArray',
              '/objects/detections': 'drop_interfaces/msg/ObjectDetectionArray'}
    for i, (name, typ) in enumerate(topics.items()):
        try:
            meta = rosbag2_py.TopicMetadata(id=i, name=name, type=typ, serialization_format='cdr')
        except TypeError:                       # id 인자가 없는 rosbag2_py
            meta = rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr')
        writer.create_topic(meta)
    t0 = 1_763_000_000 * 10**9
    for k in range(int(seconds * fps)):
        ns = t0 + round(k * 1e9 / fps)
        stamp = Time(sec=ns // 10**9, nanosec=ns % 10**9)
        img = Image(height=200, width=320, encoding='bgr8', step=960, data=bytes([128]) * (200 * 960))
        img.header.stamp, img.header.frame_id = stamp, 'camera_color_optical_frame'
        info = CameraInfo(height=200, width=320)
        info.header = img.header
        writer.write(IMAGE, serialize_message(img), ns)
        writer.write(INFO, serialize_message(info), ns)
        writer.write(STREAM_DECOY, serialize_message(Float64MultiArray(data=[0.0] * 6)), ns)
        writer.write('/objects/detections', serialize_message(ObjectDetectionArray(calibration_id='old')), ns)
    del writer
    return str(path)


def _launch(*args):
    """launch 출력은 스레드가 계속 비운다. 비우지 않으면 파이프(64 KB)가 차서 launch와 노드가 출력에서 멈출 수 있다."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    proc = subprocess.Popen(['ros2', 'launch', 'drop_bringup', 'replay.launch.py', *args], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    proc.lines = collections.deque(maxlen=2000)
    proc.reader = threading.Thread(target=lambda: proc.lines.extend(proc.stdout), daemon=True)
    proc.reader.start()
    return proc


def _tail(proc, n=40):
    return '\n'.join(line.rstrip() for line in list(proc.lines)[-n:])


def _stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
    proc.reader.join(timeout=5)
    return ''.join(proc.lines)


def _violations(h):
    """그래프의 금지 항목 [(노드, 종류, 이름)]. 시험 클라이언트 자신은 뺀다."""
    c = h.client
    out = []
    for name, ns in c.get_node_names_and_namespaces():
        if name == c.get_name():
            continue
        try:
            pubs = [t for t, _ in c.get_publisher_names_and_types_by_node(name, ns)]
            clients = [s for s, _ in c.get_client_names_and_types_by_node(name, ns)]
            act_clients = [a for a, _ in get_action_client_names_and_types_by_node(c, name, ns)]
            act_servers = [a for a, _ in get_action_server_names_and_types_by_node(c, name, ns)]
        except Exception:                       # 조회 사이 노드가 사라짐
            continue
        if name in FORBIDDEN_NODES:
            out.append((name, 'node', name))
        out += [(name, 'stream_publisher', t) for t in pubs if t.startswith('/dsr01/') or t.endswith('_stream')]
        out += [(name, 'service_client', s) for s in clients if s.startswith(('/dsr01/', '/robot/'))]
        out += [(name, 'action_client', a) for a in act_clients if a.startswith('/robot/')]
        out += [(name, 'action_server', a) for a in act_servers if a.startswith('/robot/')]
    return out


def _publisher_nodes(h, topic):
    return sorted(i.node_name for i in h.client.get_publishers_info_by_topic(topic))


@pytest.mark.parametrize('with_bag', [False, True], ids=['no_bag', 'synthetic_bag'])
def test_replay_has_no_robot_command_path(ros, tmp_path, with_bag):
    args = ['loop:=true', 'delay_s:=1.0'] + ([f'bag:={_write_bag(tmp_path / "bag")}'] if with_bag else [])
    h = Harness()
    hbs = h.record(Heartbeat, 'system/heartbeat')
    dets = h.record(ObjectDetectionArray, 'objects/detections')
    tracks = h.record(TrackedObjectArray, 'objects/tracked')
    proc = _launch(*args)
    probe = None
    try:
        def active():
            return {m.node_name for _, m in hbs[-30:] if m.lifecycle_state == 3}
        try:
            # 공유 CI 러너의 첫 기동은 느리다(2026-11-12 run 43249202773에서 30 s 초과 1회)
            h.wait_for(lambda: set(NODES) <= active(), timeout=60.0, what='vision·tracker·planner ACTIVE heartbeat')
        except TimeoutError as e:
            raise TimeoutError(f'{e}; ACTIVE={sorted(active())}, launch 출력 끝:\n{_tail(proc)}') from None
        h.wait_for(lambda: {'/vision/detect_objects', '/tracking/select_target', '/planner/compute_intercept'}
                   <= set(h.service_types()), timeout=10.0, what='대상 노드 서비스 발견')
        if with_bag:
            # 재생 → 실제 vision → tracker가 흐른다(재생기가 영상을 내고 있다)
            h.wait_for(lambda: dets and tracks, timeout=20.0, what='bag 영상에서 나온 검출·track')
            # 그래프 발견은 데이터 흐름보다 늦을 수 있다(2026-11-13 CI run 43350122545)
            h.wait_for(lambda: 'rosbag2_player' in _publisher_nodes(h, IMAGE), timeout=10.0, what='재생기 영상 발행자 발견')
            assert all(m.calibration_id != 'old' for _, m in dets)        # bag의 이전 출력이 아니다
            assert _publisher_nodes(h, '/objects/detections') == ['vision_node']
            assert _publisher_nodes(h, STREAM_DECOY) == []
        time.sleep(1.0)                                                 # 끝점 발견 여유
        assert proc.poll() is None
        # 탐침: 조회가 금지 항목을 찾는지 같은 그래프 조회로 확인한다
        probe = rclpy.create_node(PROBE)
        probe_entities = [ActionClient(probe, PickStone, 'robot/pick_stone'),
                          probe.create_client(Stop, '/dsr01/gripper/cmd'),
                          probe.create_publisher(Float64MultiArray, STREAM_DECOY, 1)]
        want = {(PROBE, 'action_client', '/robot/pick_stone'), (PROBE, 'service_client', '/dsr01/gripper/cmd'),
                (PROBE, 'stream_publisher', STREAM_DECOY)}
        h.wait_for(lambda: want <= set(_violations(h)), timeout=10.0, what='탐침 금지 항목 발견')
        found = _violations(h)
        assert [v for v in found if v[0] != PROBE] == []
        names = {n for n, _ in h.client.get_node_names_and_namespaces()}
        assert set(NODES) <= names
        assert len(probe_entities) == 3
    finally:
        if probe is not None:
            probe.destroy_node()
        out = _stop(proc)
        h.close()
    assert 'robot_controller' not in out and 'orchestrator_node' not in out
