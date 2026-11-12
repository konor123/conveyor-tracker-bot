"""Synthetic camera → real lifecycle vision node → DDS contracts (no hardware)."""
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('drop_interfaces.msg')
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from drop_interfaces.msg import ObjectDetectionArray, Heartbeat
from drop_interfaces.srv import DetectObjects

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from synth_images import Canvas, MAT_BGR, STONE_BGR, CAP_BGR
from vision.vision_node import VisionNode

pytestmark = [pytest.mark.req('REQ-VIS-008'), pytest.mark.level('C')]


def wait_for(predicate, timeout=4.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    assert predicate(), 'timed out waiting for DDS observation'


@pytest.fixture
def rig(tmp_path):
    # 0.5 mm/px at the stone observation plane: ±0.6/1.2 px → σ=0.3/0.6 mm.
    extr = dict(calibration_id='contract', date='2026-11-09', residuals={'rms_m': 0.},
                camera_profile=dict(serial='synthetic', width=320, height=200,
                    fx=984., fy=984., cx=160., cy=100., distortion_model='plumb_bob',
                    distortion=[0.] * 5),
                transform=dict(parent_frame='robot_base', child_frame='camera_color_optical_frame',
                    translation_m=[0., 0., .5], rotation_xyzw=[1., 0., 0., 0.]))
    cal = tmp_path / 'camera_extrinsics.yaml'
    cal.write_text(yaml.safe_dump(extr))
    plane = tmp_path / 'mat_plane.yaml'
    plane.write_text(yaml.safe_dump(dict(calibration_id='contract', frame='robot_base',
                                        normal=[0., 0., 1.], offset_m=0.)))
    from ament_index_python.packages import get_package_share_directory
    detector = yaml.safe_load((Path(get_package_share_directory('vision')) / 'config/detector.yaml').read_text())
    detector['mat_roi_px'] = [10., 10., 150., 10., 150., 190., 10., 190.]
    detector['belt_roi_px'] = [170., 10., 310., 10., 310., 190., 170., 190.]
    detector_file = tmp_path / 'detector.yaml'
    detector_file.write_text(yaml.safe_dump(detector))
    context = Context()
    rclpy.init(context=context)
    node = VisionNode(context=context, parameter_overrides=[Parameter(k, value=v) for k, v in {
        'calibration_file': str(cal), 'mat_plane_file': str(plane), 'belt_plane_file': str(plane),
        'detector_config_file': str(detector_file), 'image_topic': 'contract/image'}.items()])
    client_node = Node('vision_contract_client', context=context)
    pub = client_node.create_publisher(Image, 'contract/image', 1)
    detections, beats = [], []
    client_node.create_subscription(ObjectDetectionArray, 'objects/detections', detections.append, 5)
    client_node.create_subscription(Heartbeat, 'system/heartbeat', beats.append, 1)
    client = client_node.create_client(DetectObjects, 'vision/detect_objects')
    executor = MultiThreadedExecutor(num_threads=5, context=context)
    executor.add_node(node)
    executor.add_node(client_node)
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=.02)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    assert client.wait_for_service(timeout_sec=4.)
    wait_for(lambda: pub.get_subscription_count() == 1)
    yield node, client_node, pub, client, detections, beats
    node.trigger_deactivate() if node.active else None
    # Stop scheduling before draining worker callbacks and destroying ROS handles.
    stop.set()
    thread.join(3.)
    assert not thread.is_alive()
    # Jazzy Executor.shutdown destroys the guard before joining its pool. Drain
    # first, since a finishing callback still triggers that guard (executors.py).
    executor._executor.shutdown(wait=True)
    assert executor.shutdown(timeout_sec=3.)
    node.destroy_node()
    client_node.destroy_node()
    context.shutdown()


def request(client_node, seconds=2.):
    req = DetectObjects.Request(region=1, object_class=1, min_count=1, stable_window=.5)
    req.deadline = (client_node.get_clock().now() + Duration(seconds=seconds)).to_msg()
    return req


def image(client_node, offset=0., rgb=False, padded=False):
    canvas = Canvas(MAT_BGR, w=320, h=200)
    canvas.disk(80. + offset, 100., 20., STONE_BGR['yellow'])
    canvas.cap(240., 100., outer=38., bgr=CAP_BGR)
    arr = canvas.image(noise_sigma=0.)
    if rgb:
        arr = arr[:, :, ::-1].copy()
    if padded:
        arr = np.pad(arr.reshape(200, -1), ((0, 0), (0, 6)))
    msg = Image(height=200, width=320, encoding='rgb8' if rgb else 'bgr8',
                step=966 if padded else 960, data=arr.tobytes())
    msg.header.stamp = client_node.get_clock().now().to_msg()
    return msg


@pytest.mark.realtime_pc                # 처리 시간·10 Hz 타이머에 의존: CI는 기록만, 판정은 PC
@pytest.mark.tc('TC-VIS-008-01')
@pytest.mark.parametrize('jitter_m,expected', [(.0003, 0), (.0006, 1)])
def test_stable_window(rig, jitter_m, expected):
    node, cn, pub, client, ds, _ = rig
    assert node.trigger_configure().name == 'SUCCESS'
    assert node.trigger_activate().name == 'SUCCESS'
    future = client.call_async(request(cn, 4.))
    for i in range(24):
        pub.publish(image(cn, (-1 if i % 2 else 1) * jitter_m / .0005))
        time.sleep(.035)
        if future.done():
            break
    wait_for(future.done)
    result = future.result()
    assert result.result == expected, result.reason_code
    if expected == 0:
        assert len(result.detections) == len(result.position_std) == 1
        assert result.detections[0].position_valid
        assert .0002 < result.position_std[0] < .0004
    else:
        assert result.reason_code == 'NOT_ENOUGH_STABLE'


@pytest.mark.tc('TC-VIS-008-02')
def test_deadlines(rig):
    node, cn, pub, client, _, beats = rig
    node.trigger_configure()
    node.trigger_activate()
    start = time.monotonic()
    future = client.call_async(request(cn, -.1))
    wait_for(future.done)
    assert future.result().result == 2
    assert future.result().reason_code == 'DEADLINE'
    assert time.monotonic() - start < .3
    pub.publish(image(cn))
    start = time.monotonic()
    future = client.call_async(request(cn, .25))
    wait_for(future.done)
    assert future.result().result == 2
    assert future.result().reason_code == 'DEADLINE'
    assert .20 < time.monotonic() - start < .5
    wait_for(lambda: len(beats) >= 3)


def test_lifecycle_topics_and_heartbeat(rig):
    node, cn, pub, client, ds, beats = rig
    node.trigger_configure()
    future = client.call_async(request(cn))
    wait_for(future.done)
    assert (future.result().result, future.result().reason_code) == (3, 'NODE_INACTIVE')
    node.trigger_activate()
    for rgb, padded in [(False, False), (True, True)]:
        msg = image(cn, rgb=rgb, padded=padded)
        old = len(ds)
        pub.publish(msg)
        wait_for(lambda: len(ds) > old)
        result = ds[-1]
        assert result.header.stamp == msg.header.stamp
        assert result.header.frame_id == 'robot_base'
        assert result.calibration_id == 'contract'
        assert {d.object_class for d in result.detections} == {1, 2}
    assert ds[-1].image_seq > ds[-2].image_seq
    wait_for(lambda: len(beats) >= 4)
    assert all(b.node_name == 'vision_node' for b in beats)
    assert all(b.seq > a.seq for a, b in zip(beats, beats[1:]))
    wait_for(lambda: any(b.detail == 'FRAME_STALE' for b in beats))
    future = client.call_async(request(cn, 2.))
    time.sleep(.05)
    node.trigger_deactivate()
    wait_for(future.done)
    assert future.result().result == 3


@pytest.mark.realtime_pc                # 처리 시간·10 Hz 타이머에 의존: CI는 기록만, 판정은 PC
def test_latest_frame_survives_processing_backlog(rig, monkeypatch):
    import vision.vision_node as module
    node, cn, pub, _, ds, beats = rig
    original = module.process_frame
    entered, release = threading.Event(), threading.Event()

    def delayed(*args):
        entered.set()
        assert release.wait(2.)
        return original(*args)

    monkeypatch.setattr(module, 'process_frame', delayed)
    node.trigger_configure()
    node.trigger_activate()
    first = image(cn)
    pub.publish(first)
    try:
        assert entered.wait(2.)
        before = len(beats)
        for _ in range(8):
            latest = image(cn)
            pub.publish(latest)
            time.sleep(.025)
        wait_for(lambda: len(beats) > before)
        time.sleep(.05)  # Let the receiver retain the final message before processing resumes.
    finally:
        release.set()
    wait_for(lambda: len(ds) >= 2)
    time.sleep(.15)
    assert len(ds) == 2
    assert ds[0].header.stamp == first.header.stamp
    assert ds[1].header.stamp == latest.header.stamp


@pytest.mark.req('REQ-SYS-003')
@pytest.mark.tc('TC-SYS-003-01')
def test_detection_stamp_is_exposure_time_with_processing_delay(rig, monkeypatch):
    """처리에 50 ms가 걸려도 ObjectDetectionArray.header.stamp는 입력 영상 노출 시각 T와 ns까지 같다.

    T는 epoch 규모에 ns 자리까지 있는 값이라 float 초를 거치면 바뀐다(1.76e9 s에서 분해능 약 240 ns).
    """
    import vision.vision_node as module
    from builtin_interfaces.msg import Time
    original = module.process_frame

    def delayed(*args):
        time.sleep(.05)                                # 처리 지연 주입
        return original(*args)

    monkeypatch.setattr(module, 'process_frame', delayed)
    node, cn, pub, _, ds, _ = rig
    assert node.trigger_configure().name == 'SUCCESS'
    assert node.trigger_activate().name == 'SUCCESS'
    for t_ns in (1_763_000_000_123_456_789, 1_763_000_000_156_790_123):
        msg = image(cn)
        msg.header.stamp = Time(sec=t_ns // 10**9, nanosec=t_ns % 10**9)
        old = len(ds)
        sent = time.monotonic()
        pub.publish(msg)
        wait_for(lambda: len(ds) > old)
        assert time.monotonic() - sent >= .05            # 지연이 실제로 들어갔다
        out = ds[-1]
        assert (out.header.stamp.sec, out.header.stamp.nanosec) == (t_ns // 10**9, t_ns % 10**9)
        assert len(out.detections) == 2

def test_without_geometry_keeps_pixel_detections(rig):
    node, cn, pub, _, ds, _ = rig
    node.set_parameters([Parameter('calibration_file', value='')])
    node.trigger_configure()
    node.trigger_activate()
    pub.publish(image(cn))
    wait_for(lambda: bool(ds))
    assert ds[-1].calibration_id == ''
    assert len(ds[-1].detections) == 2
    assert all(not d.position_valid for d in ds[-1].detections)


@pytest.mark.realtime_pc                # 처리 시간·10 Hz 타이머에 의존: CI는 기록만, 판정은 PC
def test_heartbeat_steady_timer_with_frozen_ros_clock(rig):
    node, cn, _, _, _, _ = rig
    received = []
    cn.create_subscription(Heartbeat, 'system/heartbeat',
                           lambda msg: received.append((time.monotonic(), msg)), 1)
    node.set_parameters([Parameter('use_sim_time', value=True)])
    node.trigger_configure()
    node.trigger_activate()
    wait_for(lambda: len(received) >= 10)
    stamps = [t for t, _ in received]
    intervals = np.diff(stamps)
    assert 8. <= (len(stamps) - 1) / (stamps[-1] - stamps[0]) <= 12.
    assert max(intervals) < .3
    assert all(msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
               for _, msg in received)
    assert received[-1][1].lifecycle_state == 3
    assert received[-1][1].detail == 'FRAME_STALE'
    assert received[-1][1].error_code == 0



def test_standard_ros_parameter_file_configures(rig):
    from ament_index_python.packages import get_package_share_directory
    from rclpy.parameter import parameter_dict_from_yaml_file
    node = rig[0]
    path = Path(get_package_share_directory('vision')) / 'config' / 'vision.yaml'
    values = parameter_dict_from_yaml_file(str(path), target_nodes=['vision_node'])
    node.set_parameters([Parameter.from_parameter_msg(value) for value in values.values()])
    assert node.trigger_configure().name == 'SUCCESS'
