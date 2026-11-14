"""QoS 검사 함수 fixture 및 호환성 시험(실제 orchestrator QoS는 test_watch_timers_ros.py). IF QoS 계약 테스트."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir, "testing", "contracts"))

rclpy = pytest.importorskip("rclpy")

from drop_interfaces.msg import (  # noqa: E402
    BeltMotion,
    Heartbeat,
    InterceptPlan,
    ObjectDetectionArray,
    StepRecord,
    TaskStatus,
    TrackedObjectArray,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

import common  # noqa: E402
from drop_ros import DOMAIN, Harness  # noqa: E402


@pytest.fixture(scope="module")
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.shutdown()


def test_qos_checker_accepts_fixture_publishers(ros):
    node = rclpy.create_node("qos_spec_table_pub")
    data_qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    hb_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    status_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    step_qos = QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    node.create_publisher(ObjectDetectionArray, "objects/detections", data_qos)
    node.create_publisher(TrackedObjectArray, "objects/tracked", data_qos)
    node.create_publisher(BeltMotion, "belt/measured_motion", data_qos)
    node.create_publisher(InterceptPlan, "drop/intercept_plan", data_qos)
    node.create_publisher(Heartbeat, "system/heartbeat", hb_qos)
    node.create_publisher(TaskStatus, "task/status", status_qos)
    node.create_publisher(StepRecord, "task/step_record", step_qos)

    h = Harness(node)
    try:
        for topic in (
            "/objects/detections",
            "/objects/tracked",
            "/belt/measured_motion",
            "/drop/intercept_plan",
        ):
            common.check_data_qos(h, topic, depth=5)

        common.check_data_qos(h, "/system/heartbeat", depth=1)
        common.check_data_qos(h, "/task/step_record", depth=100)

        status_infos = h.publisher_qos("/task/status")
        assert status_infos
        for q in status_infos:
            assert q.reliability == ReliabilityPolicy.RELIABLE
            assert q.durability == DurabilityPolicy.TRANSIENT_LOCAL
            if q.depth > 0:
                assert q.depth == 1

        # depth 불일치 시 check_data_qos가 실패하는지 확인
        with pytest.raises(AssertionError):
            common.check_data_qos(h, "/objects/detections", depth=1)
    finally:
        h.close()


def test_qos_mismatch_best_effort_pub_reliable_sub_detected(ros):
    node = rclpy.create_node("qos_mismatch_pub")
    best_effort_qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    reliable_qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    be_pub = node.create_publisher(ObjectDetectionArray, "qos_test/best_effort", best_effort_qos)
    node.create_publisher(ObjectDetectionArray, "qos_test/reliable", reliable_qos)

    h = Harness(node)
    try:
        # Reliable 구독자 생성(오류 없이 무수신 발생)
        received = h.record(ObjectDetectionArray, "qos_test/best_effort")
        h.wait_for(lambda: bool(h.publisher_qos("/qos_test/best_effort")), 3.0, "best_effort 발행자")
        be_pub.publish(ObjectDetectionArray())
        h.sleep(0.1)
        assert len(received) == 0

        # 계약 검사 함수가 QOS_MISMATCH로 잡아낸다
        with pytest.raises(AssertionError, match="QOS_MISMATCH"):
            common.check_qos_compatible(h, "/qos_test/best_effort", ReliabilityPolicy.RELIABLE)

        # 호환되는 조합(Reliable 발행자 + Reliable 구독자, BestEffort 발행자 + BestEffort 구독자)은 통과
        common.check_qos_compatible(h, "/qos_test/reliable", ReliabilityPolicy.RELIABLE)
        common.check_qos_compatible(h, "/qos_test/best_effort", ReliabilityPolicy.BEST_EFFORT)
    finally:
        h.close()
