"""tracker_node 외부 계약(PLAN.md 7.4절 최소 범위, SPEC-010 v0.1). 사용법은 vision_contract.py와 같다."""
import pytest
from drop_interfaces.msg import BeltMotion, TrackedObjectArray
from drop_interfaces.srv import ResetTracking, SelectTarget
from lifecycle_msgs.msg import State

import common
import drop_ros

__all__ = ['tracker_contract', 'TestTrackerContract']


@pytest.fixture(scope='module')
def tracker_contract(tracker_target):
    proc, h = common.started(tracker_target, '/tracking/select_target')
    h.tracked = h.record(TrackedObjectArray, 'objects/tracked')
    h.belt = h.record(BeltMotion, 'belt/measured_motion')
    h.target = tracker_target
    yield h
    common.finished(proc, h)


class TestTrackerContract:
    def test_tracker_heartbeat(self, tracker_contract):
        common.check_heartbeat(tracker_contract, 'tracker_node')

    def test_tracker_names_types_qos(self, tracker_contract):
        h = tracker_contract
        common.check_services(h, {'/tracking/reset': 'drop_interfaces/srv/ResetTracking',
                                  '/tracking/select_target': 'drop_interfaces/srv/SelectTarget'})
        for topic in ('/objects/tracked', '/belt/measured_motion'):
            common.check_data_qos(h, topic)
        h.wait_for(lambda: len(h.tracked) >= 5 and len(h.belt) >= 5, 3.0, 'tracked·belt 5건')
        assert all(m.header.frame_id == 'conveyor_frame' for _, m in h.tracked)
        assert all(m.header.frame_id == 'conveyor_frame' for _, m in h.belt)
        for _, arr in h.tracked:
            for tr in arr.tracks:                              # track stamp = 배열 stamp = t_state
                assert tr.header.stamp == arr.header.stamp
                assert drop_ros.from_time(tr.valid_until) >= drop_ros.from_time(tr.header.stamp)

    def test_tracker_deadline_passed_is_timeout(self, tracker_contract):
        h = tracker_contract
        r = h.call(ResetTracking, 'tracking/reset', deadline=common.past_deadline(h))
        assert r.result == common.RESULT_TIMEOUT
        r = h.call(SelectTarget, 'tracking/select_target', object_class=2, deadline=common.past_deadline(h))
        assert r.result == common.RESULT_TIMEOUT

    def test_tracker_reset_increments_epoch(self, tracker_contract):
        h = tracker_contract
        a = h.call(ResetTracking, 'tracking/reset')
        b = h.call(ResetTracking, 'tracking/reset')
        assert a.result == b.result == common.RESULT_OK and b.new_epoch == a.new_epoch + 1
        t = h.now()
        # 응답 직후 0.1 s 안에 나간 메시지는 reset 전 프레임일 수 있어 뺀다
        h.wait_for(lambda: any(rt > t + 0.1 and m.tracks for rt, m in h.tracked), 2.0, 'reset 뒤 track')
        after = [m for rt, m in h.tracked if rt > t + 0.1 and m.tracks]
        assert all(tr.epoch == b.new_epoch for m in after for tr in m.tracks)

    def test_tracker_inactive_refuses(self, tracker_contract):
        h = tracker_contract
        node = h.target.node
        assert h.deactivate(node)
        try:
            h.sleep(0.2)
            n = len(h.tracked)
            r = h.call(SelectTarget, 'tracking/select_target', object_class=2)
            assert (r.result, r.reason_code) == (common.RESULT_PRECONDITION, 'NODE_INACTIVE')
            r = h.call(ResetTracking, 'tracking/reset')
            assert (r.result, r.reason_code) == (common.RESULT_PRECONDITION, 'NODE_INACTIVE')
            h.sleep(0.3)
            assert len(h.tracked) == n
            assert [m for t, m in h.heartbeats[-3:] if m.node_name == 'tracker_node'
                    and m.lifecycle_state == State.PRIMARY_STATE_INACTIVE]
        finally:
            assert h.activate(node)
