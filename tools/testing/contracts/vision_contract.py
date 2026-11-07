"""vision_node 외부 계약(PLAN.md 7.4절 최소 범위, SPEC-010 v0.1).

사용: 패키지의 test/contract/test_<대상>_vision_contract.py에서
    from vision_contract import *          # noqa: F401,F403
    @pytest.fixture(scope='module')
    def vision_target(): return Target(...)
"""
import pytest
from drop_interfaces.msg import ObjectDetectionArray
from drop_interfaces.srv import DetectObjects, VerifyDrop
from lifecycle_msgs.msg import State

import common
import drop_ros

__all__ = ['vision_contract', 'TestVisionContract']


@pytest.fixture(scope='module')
def vision_contract(vision_target):
    proc, h = common.started(vision_target, '/vision/detect_objects')
    h.detections = h.record(ObjectDetectionArray, 'objects/detections')
    h.target = vision_target
    yield h
    common.finished(proc, h)


class TestVisionContract:
    def test_vision_heartbeat(self, vision_contract):
        common.check_heartbeat(vision_contract, 'vision_node')

    def test_vision_names_types_qos(self, vision_contract):
        h = vision_contract
        common.check_services(h, {'/vision/detect_objects': 'drop_interfaces/srv/DetectObjects',
                                  '/vision/verify_drop': 'drop_interfaces/srv/VerifyDrop'})
        common.check_data_qos(h, '/objects/detections')
        h.wait_for(lambda: len(h.detections) >= 5, 3.0, 'detections 5건')
        msgs = [m for _, m in h.detections]
        assert all(m.header.frame_id == 'robot_base' for m in msgs)
        stamps = [drop_ros.from_time(m.header.stamp) for m in msgs]
        assert stamps == sorted(stamps)

    def test_vision_deadline_passed_is_timeout(self, vision_contract):
        h = vision_contract
        r = h.call(DetectObjects, 'vision/detect_objects', region=1, object_class=1,
                   deadline=common.past_deadline(h))
        assert r.result == common.RESULT_TIMEOUT
        r = h.call(VerifyDrop, 'vision/verify_drop', deadline=common.past_deadline(h))
        assert r.result == common.RESULT_TIMEOUT

    def test_vision_inactive_refuses(self, vision_contract):
        h = vision_contract
        node = h.target.node
        assert h.deactivate(node)
        try:
            h.sleep(0.2)
            n = len(h.detections)
            r = h.call(DetectObjects, 'vision/detect_objects', region=1, object_class=1)
            assert (r.result, r.reason_code) == (common.RESULT_PRECONDITION, 'NODE_INACTIVE')
            r = h.call(VerifyDrop, 'vision/verify_drop')
            assert (r.result, r.reason_code) == (common.RESULT_PRECONDITION, 'NODE_INACTIVE')
            h.sleep(0.3)
            assert len(h.detections) == n                      # 비활성 중 데이터 발행 없음
            assert [m for t, m in h.heartbeats[-3:] if m.node_name == 'vision_node'
                    and m.lifecycle_state == State.PRIMARY_STATE_INACTIVE]
        finally:
            assert h.activate(node)
