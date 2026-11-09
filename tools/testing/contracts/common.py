"""계약 시험 공통 검사(PLAN.md 7.4절, SPEC-010 v0.1): heartbeat·QoS·deadline·lifecycle 비활성."""
from __future__ import annotations

import time

import pytest
from drop_interfaces.msg import Heartbeat
from lifecycle_msgs.msg import State
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

import drop_ros

HEARTBEAT_HZ = 10.0
RESULT_OK, RESULT_TIMEOUT, RESULT_PRECONDITION, RESULT_CANCELED = 0, 2, 3, 4


def started(target: drop_ros.Target, ready_service: str, timeout=15.0):
    """대상 프로세스와 시험 harness. `ready_service`가 보이면 준비된 것으로 본다."""
    rclpy = pytest.importorskip('rclpy')
    rclpy.init(domain_id=drop_ros.DOMAIN)
    proc = drop_ros.spawn(target)
    h = drop_ros.Harness()
    h.heartbeats = h.record(Heartbeat, 'system/heartbeat', steady=True)
    try:
        h.wait_for(lambda: any(n == ready_service for n in h.service_types()), timeout, ready_service)
        h.wait_for(lambda: _state(h, target.node) == State.PRIMARY_STATE_ACTIVE, timeout, 'ACTIVE')
    except Exception:
        h.close()
        drop_ros.stop(proc)
        rclpy.shutdown()
        raise
    return proc, h


def _state(h, node):
    """기동 직후 첫 서비스 응답은 discovery 경쟁으로 사라질 수 있어 짧게 끊고 다시 묻는다(get_state는 멱등)."""
    try:
        return h.lifecycle_state(node, timeout=1.0)
    except TimeoutError:
        return None


def finished(proc, h) -> None:
    import rclpy
    h.close()
    drop_ros.stop(proc)
    rclpy.shutdown()


def check_heartbeat(h, node_name: str, window=1.5) -> None:
    """주기 10 Hz ± 20%, 간격 최대 < 0.3 s(SPEC-110 손실 기준), ACTIVE, seq 증가.

    주기·간격은 수신 steady 시각으로 잰다. heartbeat는 steady 타이머로 발행되고(REQ-ABT-009) stamp는 ROS 시각이라,
    시스템 시계가 점프하면 stamp 간격은 실제 발행 간격과 달라진다.
    """
    t0 = time.monotonic()
    time.sleep(window + 0.05)
    rows = [(t, m) for t, m in h.heartbeats if t0 <= t < t0 + window and m.node_name == node_name]
    got = [m for _, m in rows]
    assert len(got) >= 3, len(got)
    recv = [t for t, _ in rows]
    gaps = [b - a for a, b in zip(recv, recv[1:])]
    rate = (len(recv) - 1) / (recv[-1] - recv[0])
    assert 0.8 * HEARTBEAT_HZ <= rate <= 1.2 * HEARTBEAT_HZ, rate
    assert max(gaps) < 0.3, max(gaps)
    assert all(m.lifecycle_state == State.PRIMARY_STATE_ACTIVE for m in got)
    seqs = [m.seq for m in got]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)


def check_data_qos(h, topic: str, depth: int = 5) -> None:
    qos = h.publisher_qos(topic)
    assert qos, f'발행자 없음: {topic}'
    norm = topic if topic.startswith('/') else f'/{topic}'
    local_depths = [
        p.qos_profile.depth
        for n in getattr(h, 'nodes', ())
        for p in getattr(n, 'publishers', ())
        if p.topic_name == norm and getattr(p.qos_profile, 'depth', 0) > 0
    ]
    for q in qos:
        assert q.reliability == ReliabilityPolicy.RELIABLE, topic
        assert q.durability == DurabilityPolicy.VOLATILE, topic
        if getattr(q, 'depth', 0) > 0:
            assert q.depth == depth, f'{topic}: depth={q.depth} != {depth}'
    for d in local_depths:
        assert d == depth, f'{topic}: depth={d} != {depth}'


def check_qos_compatible(h, topic: str, reliability=ReliabilityPolicy.RELIABLE) -> None:
    qos = h.publisher_qos(topic)
    assert qos, f'발행자 없음: {topic}'
    sub_rel = getattr(reliability, 'reliability', reliability)
    for q in qos:
        assert not (
            sub_rel == ReliabilityPolicy.RELIABLE and q.reliability != ReliabilityPolicy.RELIABLE
        ), f'QOS_MISMATCH: {topic} (publisher={q.reliability}, subscriber={sub_rel})'


def check_services(h, expected: dict[str, str]) -> None:
    types = h.service_types()
    for name, typ in expected.items():
        assert types.get(name) == [typ], (name, types.get(name))


def past_deadline(h):
    return drop_ros.to_time(h.now() - 0.5)
