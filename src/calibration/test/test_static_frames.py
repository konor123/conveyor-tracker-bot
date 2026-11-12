"""static_frames_publisher → /tf_static (no hardware)."""
import json
import math
import os
import threading
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from calibration.geometry import R_from_quat
from calibration.static_frames import StaticFramesPublisher

pytestmark = [pytest.mark.req('REQ-CAL-012'), pytest.mark.tc('TC-CAL-012-01'), pytest.mark.level('C')]
REPO = Path(__file__).resolve().parents[3]
MAT = REPO / 'records/calib/2026-11-09-planes/mat_plane.yaml'


def _yaw_T(yaw, t):
    T = np.eye(4)
    T[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    T[:3, 3] = t
    return T


def _run(T, publish_world):
    """노드를 띄워 /tf_static을 모으고 (노드 ok 여부, {(부모, 자식): 4×4}, 받은 쌍 목록)을 돌려준다."""
    context = Context()
    rclpy.init(context=context, domain_id=100 + os.getpid() % 100)   # 시험용 domain(다른 ROS 그래프와 격리)
    try:
        node = StaticFramesPublisher(context=context, parameter_overrides=[
            Parameter('T_base_conveyor', value=[float(v) for v in np.asarray(T).ravel()]),
            Parameter('mat_plane_file', value=str(MAT)),
            Parameter('publish_world', value=publish_world)])
        listener = Node('tf_static_listener', context=context)
        got = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        listener.create_subscription(TFMessage, '/tf_static', lambda m: got.extend(m.transforms), qos)
        ex = SingleThreadedExecutor(context=context)
        ex.add_node(node)
        ex.add_node(listener)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.05)
            if node.ok and len(got) >= (4 if publish_world else 3):
                time.sleep(0.1)
                ex.spin_once(timeout_sec=0.05)
                break
        pairs = [(t.header.frame_id, t.child_frame_id) for t in got]
        mats = {}
        for t in got:
            M = np.eye(4)
            q = t.transform.rotation
            M[:3, :3] = R_from_quat([q.x, q.y, q.z, q.w])
            M[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            mats[(t.header.frame_id, t.child_frame_id)] = M
        ok = node.ok
        ex.shutdown()
        node.destroy_node()
        listener.destroy_node()
        return ok, mats, pairs
    finally:
        context.shutdown()


@pytest.mark.parametrize('T', [np.eye(4), _yaw_T(math.pi / 2, [0.6, 0.05, 0.07])], ids=['identity', 'yaw90'])
@pytest.mark.parametrize('publish_world', [False, True])
def test_static_tree(T, publish_world):
    ok, mats, pairs = _run(T, publish_world)
    expected = {('base_link', 'robot_base'), ('robot_base', 'conveyor_frame'), ('robot_base', 'mat_frame')}
    if publish_world:
        expected.add(('world', 'base_link'))
    assert ok
    assert set(pairs) == expected
    assert len(pairs) == len(set(pairs))                       # 중복 쌍 0
    np.testing.assert_allclose(mats[('robot_base', 'conveyor_frame')], T, atol=1e-9)
    np.testing.assert_allclose(mats[('base_link', 'robot_base')], np.eye(4), atol=1e-12)


def _tilted_T(digits):
    """기울기 0.02 rad + yaw 1.3 rad 회전을 소수 digits자리로 적은 변환(보정 결과 파일 모양)."""
    cx, sx, cz, sz = math.cos(0.02), math.sin(0.02), math.cos(1.3), math.sin(1.3)
    T = np.eye(4)
    T[:3, :3] = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    T[:3, 3] = [0.6, 0.05, 0.07]
    return np.round(T, digits)


def test_is_rigid_tolerance():
    from calibration.static_frames import is_rigid
    assert is_rigid(_tilted_T(6))           # 직교성 오차 ~1e-6
    assert not is_rigid(_tilted_T(3))       # ~1e-3
    scaled = np.eye(4)
    scaled[0, 0] = 1.001
    assert not is_rigid(scaled)


def test_accepts_six_digit_tilted_transform():
    T = _tilted_T(6)
    ok, mats, _ = _run(T, False)
    assert ok
    np.testing.assert_allclose(mats[('robot_base', 'conveyor_frame')], T, atol=1e-5)


def test_rejects_non_rigid_transform():
    bad = np.eye(4)
    bad[0, 0] = 2.0
    ok, _, pairs = _run(bad, False)
    assert not ok
    assert pairs == []


def test_tracking_and_planner_agree():
    tracking = yaml.safe_load((REPO / 'src/tracking/config/tracking.yaml').read_text())['T_base_conveyor']
    planner = json.loads((REPO / 'src/intercept_planner/config/planner.yaml').read_text())['T_base_conveyor']
    np.testing.assert_allclose(np.asarray(tracking, float).reshape(4, 4), np.asarray(planner, float), atol=0)
