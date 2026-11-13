"""Lifecycle/DDS wrapper around the pure frame pipeline and stable window.

Only the most recently received frame is retained while processing is busy.
Calibration and all distances remain in robot_base SI coordinates.
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from drop_interfaces.msg import Heartbeat, ObjectDetection, ObjectDetectionArray
from drop_interfaces.srv import DetectObjects
from geometry_msgs.msg import Point
from lifecycle_msgs.msg import State
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from .detector import Roi, REGION_MAT, REGION_BELT, ColorModel, DetectParams, InsideParams
from calibration.repo_paths import resolve_path

from .geometry_files import load_geometry
from .localizer import HeightParams
from .pipeline import CROP_MARGIN_PX, SceneConfig, process_frame
from .stable_window import DetectionWindow, WindowParams


def qos(depth):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def detection_msg(index, detection):
    b = detection.blob
    return ObjectDetection(detection_id=index, region=b.region, object_class=b.object_class,
        u=b.u, v=b.v, diameter_px=b.diameter_px, confidence=b.confidence,
        center_cov_px=list(b.center_cov_px), position=Point(x=detection.position[0],
            y=detection.position[1], z=detection.position[2]),
        position_cov=list(detection.position_cov), position_valid=detection.position_valid,
        occluded_partial=b.occluded_partial, stone_inside=detection.stone_inside,
        stone_inside_valid=detection.stone_inside_valid)


class VisionNode(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__('vision_node', **kwargs)
        # Installed ROS YAML supplies defaults; CLI and launch overrides retain priority.
        config = Path(get_package_share_directory('vision')) / 'config' / 'vision.yaml'
        defaults = yaml.safe_load(config.read_text())['vision_node']['ros__parameters']
        for key, value in defaults.items():
            if key == 'detector_config_file' and not value:
                value = str(config.with_name('detector.yaml'))
            self.declare_parameter(key, value)
        self.active = False
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        self._condition = threading.Condition()
        self._pending = None
        self._last_frame = None
        self._processing_error = ''
        self._window = DetectionWindow(self.value('window_horizon_s'))
        self._image_seq = 0
        self._heartbeat_seq = 0
        self._geometry = None
        self._generation = 0
        self._scene = SceneConfig()
        self._window_params = WindowParams()
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._receive_group = MutuallyExclusiveCallbackGroup()
        self._process_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._heartbeat_group = MutuallyExclusiveCallbackGroup()
        self._det_pub = self.create_publisher(ObjectDetectionArray, 'objects/detections', qos(5),
            event_callbacks=PublisherEventCallbacks(use_default_callbacks=False))
        self._hb_pub = self.create_publisher(Heartbeat, 'system/heartbeat', qos(1),
            event_callbacks=PublisherEventCallbacks(use_default_callbacks=False))
        self.create_subscription(Image, self.value('image_topic'), self._receive, qos(1),
            callback_group=self._receive_group,
            event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))
        self.create_service(DetectObjects, 'vision/detect_objects', self._detect,
                            callback_group=self._service_group)
        self.create_timer(.005, self._process, callback_group=self._process_group, clock=self._steady)
        self.create_timer(.1, self._heartbeat, callback_group=self._heartbeat_group, clock=self._steady)

    def value(self, key):
        return self.get_parameter(key).value

    def on_configure(self, state):
        try:
            # 상대 경로는 저장소 루트 기준(SPEC-000 REQ-SYS-005)
            detector_path = resolve_path(self.value('detector_config_file')) or str(
                Path(get_package_share_directory('vision')) / 'config' / 'detector.yaml')
            detector = yaml.safe_load(Path(detector_path).read_text())
            def roi(key, region):
                coords = detector.get(key, [])
                if not coords:
                    return None
                if len(coords) < 6 or len(coords) % 2 or not np.isfinite(coords).all():
                    raise ValueError(f'{key}: expected finite polygon coordinate pairs')
                return Roi(region, tuple(zip(coords[::2], coords[1::2])))
            def color(key):
                values = dict(detector[key])
                for old, new in (('circularity_min_ratio', 'circularity_min'),
                                 ('ring_coverage_min_ratio', 'ring_coverage_min')):
                    if old in values:
                        values[new] = values.pop(old)
                for name in ('hsv_lo', 'hsv_hi'):
                    if name in values:
                        values[name] = tuple(values[name])
                return ColorModel(**values)
            self._scene = SceneConfig(mat_roi=roi('mat_roi_px', REGION_MAT),
                belt_roi=roi('belt_roi_px', REGION_BELT), classes={1: color('stone'), 2: color('cap')},
                detect=DetectParams(**detector['detect']),
                inside=InsideParams(inner_diameter_ratio=detector['inner_diameter_ratio']),
                crop_margin_px=CROP_MARGIN_PX)          # 실시간 경로만 ROI를 잘라 계산(REQ-VIS-011)
            self._window_params = WindowParams(std_max_m=self.value('window_std_max_m'),
                std_max_px=detector['window']['std_max_px'],
                presence_min_ratio=self.value('presence_min_ratio'), gate_m=self.value('gate_m'),
                gate_px=detector['window']['gate_px'], span_tol_s=self.value('span_tol_s'))
            self._geometry = self._load_geometry()
            if self.value('frame_timeout_s') <= 0 or self.value('window_horizon_s') <= 0:
                raise ValueError('timeouts and window horizon must be positive')
            self._window = DetectionWindow(self.value('window_horizon_s'))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self.get_logger().error(f'vision configuration: {exc}')
            return TransitionCallbackReturn.FAILURE
        self.state_id = State.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def _load_geometry(self):
        return load_geometry(self.value('calibration_file'), self.value('mat_plane_file'),
                             self.value('belt_plane_file'),
                             HeightParams(self.value('h_stone_offset_m'), self.value('h_rim_m'),
                                          self.value('floor_thickness_m')))

    def on_activate(self, state):
        with self._condition:
            self._generation += 1
            self._window.clear()
            self._pending = None
            self._last_frame = time.monotonic()
            self._processing_error = ''
            self.active = True
            self.state_id = State.PRIMARY_STATE_ACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        with self._condition:
            self._generation += 1
            self.active = False
            self.state_id = State.PRIMARY_STATE_INACTIVE
            self._pending = None
            self._window.clear()
            self._condition.notify_all()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.on_deactivate(state)
        self._geometry = None
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.on_deactivate(state)
        self.state_id = State.PRIMARY_STATE_FINALIZED
        return TransitionCallbackReturn.SUCCESS

    def _receive(self, msg):
        with self._condition:
            if self.active:
                self._pending = msg
                self._last_frame = time.monotonic()

    def _process(self):
        with self._condition:
            if not self.active or self._pending is None:
                return
            msg, self._pending = self._pending, None
            generation = self._generation
        try:
            if (msg.encoding not in ('bgr8', 'rgb8') or msg.height <= 0
                    or msg.width <= 0 or msg.step < msg.width * 3):
                raise ValueError('expected bgr8/rgb8 image with valid row stride')
            if len(msg.data) != msg.height * msg.step:
                raise ValueError('image data size does not match dimensions/stride')
            rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                image = image[:, :, ::-1].copy()
            dets = process_frame(image, self._scene, self._geometry, None)
        except (ValueError, TypeError) as exc:
            with self._condition:
                self._processing_error = f'FRAME_INVALID: {exc}'
            return
        with self._condition:
            if not self.active or generation != self._generation:
                return
            self._processing_error = ''
            self._image_seq += 1
            self._window.add(stamp_s(msg.header.stamp), dets)
            self._det_pub.publish(ObjectDetectionArray(
                header=Header(stamp=msg.header.stamp, frame_id='robot_base'), image_seq=self._image_seq,
                calibration_id=self._geometry.calibration_id if self._geometry else '',
                detections=[detection_msg(i, d) for i, d in enumerate(dets)]))
            self._condition.notify_all()

    def _detect(self, req, res):
        deadline = stamp_s(req.deadline)
        with self._condition:
            while True:
                if not self.active:
                    res.result, res.reason_code = res.PRECONDITION, 'NODE_INACTIVE'
                    return res
                now = self.get_clock().now().nanoseconds * 1e-9
                if deadline > 0 and now >= deadline:
                    res.result, res.reason_code = res.TIMEOUT, 'DEADLINE'
                    return res
                if (not np.isfinite(req.stable_window) or req.stable_window <= 0
                        or req.stable_window > self.value('window_horizon_s') or req.min_count < 1):
                    res.result, res.reason_code = res.PRECONDITION, 'INVALID_REQUEST'
                    return res
                result = self._window.evaluate(req.region, req.object_class, req.min_count,
                                               req.stable_window, self._window_params)
                if result is not None:
                    if deadline > 0 and self.get_clock().now().nanoseconds * 1e-9 >= deadline:
                        res.result, res.reason_code = res.TIMEOUT, 'DEADLINE'
                        return res
                    res.result, res.reason_code = result.result, result.reason_code
                    res.detections = [detection_msg(i, d) for i, d in enumerate(result.detections)]
                    res.position_std = list(result.position_std)
                    return res
                # Condition sleep releases the lock; bounded for ROS clock jumps and shutdown.
                if not self.context.ok():
                    res.result, res.reason_code = res.PRECONDITION, 'NODE_INACTIVE'
                    return res
                self._condition.wait(timeout=min(.1, max(0., deadline - now)) if deadline > 0 else .1)

    def _heartbeat(self):
        with self._condition:
            self._heartbeat_seq += 1
            detail = self._processing_error
            if self.active and (self._last_frame is None or
                    time.monotonic() - self._last_frame > self.value('frame_timeout_s')):
                detail = 'FRAME_STALE'
            # SPEC-010 allocates only 0 (normal) and 100 (robot protective stop).
            # Camera diagnostics use detail until a vision error code is allocated.
            self._hb_pub.publish(Heartbeat(header=Header(stamp=self.get_clock().now().to_msg()),
                node_name=self.get_name(), lifecycle_state=self.state_id, seq=self._heartbeat_seq,
                error_code=0, detail=detail))


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown(None)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
