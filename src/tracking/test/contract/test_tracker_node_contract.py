"""Synthetic detections → actual lifecycle tracker, services and TF over DDS."""
import sys
from pathlib import Path
import numpy as np
import pytest
pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).parent))
from tracking_rig import Rig, stamp, seconds
from tracking.tracker_node import TrackerNode
from drop_interfaces.msg import ObjectDetection, ObjectDetectionArray, TrackedObjectArray, BeltMotion, Heartbeat
from drop_interfaces.srv import ResetTracking, SelectTarget
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

pytestmark = pytest.mark.level('C')


@pytest.fixture
def rig():
    r = Rig(TrackerNode)
    r.tracks = r.record(TrackedObjectArray, 'objects/tracked')
    r.belt = r.record(BeltMotion, 'belt/measured_motion')
    r.tf = r.record(TFMessage, '/tf')
    r.beats = r.record(Heartbeat, 'system/heartbeat')
    r.pub = r.publisher(ObjectDetectionArray, 'objects/detections')
    r.spin(.2)
    yield r
    r.close()


def frame(r, t, xs=(-.2,), cov=None, transform=None):
    r.set_time(t)
    dets = []
    for x in xs:
        p = np.array([x, .01, .02, 1.])
        if transform is not None:
            p = transform @ p
        dets.append(ObjectDetection(object_class=2, position_valid=True,
            position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
            position_cov=list((np.eye(3)*9e-8 if cov is None else cov).ravel())))
    before = len(r.tracks)
    r.pub.publish(ObjectDetectionArray(header=Header(stamp=stamp(t), frame_id='robot_base'), detections=dets))
    r.wait(lambda: any(a.header.stamp == stamp(t) for a in r.tracks[before:]))
    return next(a for a in reversed(r.tracks) if a.header.stamp == stamp(t))


def moving(r, start=100., x=-.2, n=45):
    for i in range(n):
        result = frame(r, start+i/30., (x+.02*i/30.,))
    return result.tracks[0]


@pytest.mark.req('REQ-TRK-009')
@pytest.mark.tc('TC-TRK-009-01')
def test_reset(rig):
    r = rig
    old = frame(r, 100.).tracks[0]
    result = r.call(ResetTracking, 'tracking/reset')
    assert result.result == 0 and result.new_epoch == old.epoch+1
    assert not frame(r, 100.01, ()).tracks
    new = frame(r, 100.02).tracks[0]
    assert new.epoch == result.new_epoch and new.track_id > old.track_id


@pytest.mark.req('REQ-TRK-010')
@pytest.mark.tc('TC-TRK-010-01')
def test_select(rig):
    r = rig
    assert r.call(SelectTarget, 'tracking/select_target', object_class=2).result == 3
    tr = moving(r)
    result = r.call(SelectTarget, 'tracking/select_target', object_class=2, min_observation_count=30)
    assert result.result == 0 and result.track_id == tr.track_id
    assert seconds(result.t_enter_window) == pytest.approx(107.5, abs=.05)
    r.call(ResetTracking, 'tracking/reset')
    moving(r, r.t+.01, x=0.)
    assert r.call(SelectTarget, 'tracking/select_target', object_class=2).result == 3


@pytest.mark.req('REQ-TRK-011')
@pytest.mark.tc('TC-TRK-011-01')
def test_belt(rig):
    r = rig
    tr = moving(r)
    r.wait(lambda: any(b.header.stamp == tr.header.stamp for b in r.belt))
    b = next(b for b in reversed(r.belt) if b.header.stamp == tr.header.stamp)
    assert b.source == b.SOURCE_CAP
    assert np.linalg.norm([b.direction.x,b.direction.y,b.direction.z]) == pytest.approx(1., abs=1e-9)
    assert b.speed == pytest.approx(.02, abs=.0005)


@pytest.mark.req('REQ-TRK-012')
@pytest.mark.tc('TC-TRK-012-01')
def test_tf_stamp_and_coasting(rig):
    r = rig
    tr = frame(r, 100.).tracks[0]
    frame(r, 100.1, ())
    r.wait(lambda: sum(len(m.transforms) for m in r.tf) >= 2)
    transforms = [t for m in r.tf for t in m.transforms]
    for t in transforms:
        assert t.header.frame_id == 'conveyor_frame'
        assert t.child_frame_id == f'cap_{tr.track_id}'
        assert seconds(t.header.stamp) <= r.t
        assert any(a.header.stamp == t.header.stamp for a in r.tracks)
        assert [t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w] == [0.,0.,0.,1.]
    assert any(seconds(t.header.stamp) == pytest.approx(100.1) for t in transforms)


@pytest.mark.req('REQ-TRK-004')
def test_silence_and_clock_reset(rig):
    r = rig
    tr = frame(r, 100.).tracks[0]
    r.set_time(100.5)
    r.wait(lambda: any(a.tracks and a.tracks[0].status == 1 for a in r.tracks))
    r.set_time(101.1)
    r.wait(lambda: any(a.tracks and a.tracks[0].status == 2 for a in r.tracks))
    lost = next(a.tracks[0] for a in r.tracks if a.tracks and a.tracks[0].status == 2)
    assert seconds(lost.valid_until) < seconds(lost.header.stamp)
    r.set_time(90.)
    new = frame(r, 90.).tracks[0]
    assert new.epoch == tr.epoch+1


@pytest.mark.req('REQ-TRK-009')
def test_deadline_and_lifecycle(rig):
    r = rig
    for typ, name in [(ResetTracking,'tracking/reset'),(SelectTarget,'tracking/select_target')]:
        res = r.call(typ, name, deadline=stamp(r.t))
        assert (res.result,res.reason_code) == (2,'DEADLINE')
    r.node.trigger_deactivate()
    r.spin()
    counts = len(r.tracks),len(r.belt),len(r.tf)
    for typ,name in [(ResetTracking,'tracking/reset'),(SelectTarget,'tracking/select_target')]:
        res = r.call(typ,name)
        assert (res.result,res.reason_code) == (3,'NODE_INACTIVE')
    r.set_time(102.)
    r.spin(.3)
    assert counts == (len(r.tracks),len(r.belt),len(r.tf))
    assert r.beats[-1].lifecycle_state == 2
    assert r.beats[-1].seq > r.beats[0].seq


@pytest.mark.req('REQ-TRK-001')
def test_inverse_transform_and_covariance():
    transform = np.array([[0.,-1.,0.,.4],[1.,0.,0.,-.1],[0.,0.,1.,.03],[0.,0.,0.,1.]])
    r = Rig(TrackerNode, {'T_base_conveyor': list(transform.ravel())})
    try:
        r.tracks = r.record(TrackedObjectArray,'objects/tracked')
        r.pub = r.publisher(ObjectDetectionArray,'objects/detections')
        r.spin()
        covariance = np.diag([4e-8,9e-8,16e-8])
        tr = frame(r,100.,cov=covariance,transform=transform).tracks[0]
        assert [tr.position.x,tr.position.y,tr.position.z] == pytest.approx([-.2,.01,.02])
        assert np.asarray(tr.state_cov).reshape(4,4)[:2,:2] == pytest.approx(np.diag([9e-8,4e-8]))
    finally:
        r.close()
