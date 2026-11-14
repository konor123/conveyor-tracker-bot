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


def test_reset(rig):
    r = rig
    old = frame(r, 100.).tracks[0]
    result = r.call(ResetTracking, 'tracking/reset')
    assert result.result == 0 and result.new_epoch == old.epoch+1
    assert not frame(r, 100.01, ()).tracks
    new = frame(r, 100.02).tracks[0]
    assert new.epoch == result.new_epoch and new.track_id > old.track_id


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


def test_belt(rig):
    r = rig
    tr = moving(r)
    r.wait(lambda: any(b.header.stamp == tr.header.stamp for b in r.belt))
    b = next(b for b in reversed(r.belt) if b.header.stamp == tr.header.stamp)
    assert b.source == b.SOURCE_CAP
    assert np.linalg.norm([b.direction.x,b.direction.y,b.direction.z]) == pytest.approx(1., abs=1e-9)
    assert b.speed == pytest.approx(.02, abs=.0005)


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


def test_clock_reset_is_logged(rig):
    """시각 역행으로 초기화하면 역행 크기와 새 epoch를 WARN으로 남긴다(원인 추적용)."""
    r = rig
    real, warned = r.node.get_logger(), []

    class Spy:
        def warn(self, message, **kwargs):
            warned.append(message)
            return real.warn(message, **kwargs)

        def __getattr__(self, name):
            return getattr(real, name)
    r.node.get_logger = lambda: Spy()
    epoch = frame(r, 100.).tracks[0].epoch
    assert warned == []
    frame(r, 99.)
    assert len(warned) == 1 and '1.000 s' in warned[0] and f'epoch -> {epoch+1}' in warned[0], warned


def test_all_detections_outside_region_is_warned(rig):
    """보정·구간이 맞지 않아 모든 검출이 구간 밖이면 track이 없다는 것을 WARN으로 알린다."""
    r = rig
    real, warned = r.node.get_logger(), []

    class Spy:
        def warn(self, message, **kwargs):
            warned.append(message)

        def __getattr__(self, name):
            return getattr(real, name)
    r.node.get_logger = lambda: Spy()
    for k in range(r.node.IGNORED_WARN_COUNT - 1):
        assert frame(r, 100. + k/30, xs=(.8,)).tracks == []
    assert warned == []
    frame(r, 100. + r.node.IGNORED_WARN_COUNT/30, xs=(.8,))
    assert len(warned) == 1 and 'no track was ever created' in warned[0] and '0.800' in warned[0], warned


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


def _tilted_T(digits):
    """기울기 0.02 rad + yaw 1.3 rad 회전을 소수 digits자리로 적은 변환(보정 결과 파일 모양)."""
    cx, sx, cz, sz = np.cos(.02), np.sin(.02), np.cos(1.3), np.sin(1.3)
    T = np.eye(4)
    T[:3, :3] = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    T[:3, 3] = [.6, .05, .07]
    return np.round(T, digits)


def _configure(cls, T):
    """노드를 만들어 configure 결과 이름만 돌려준다."""
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    context = Context()
    rclpy.init(context=context)
    try:
        node = cls(context=context, parameter_overrides=[
            Parameter('T_base_conveyor', value=[float(v) for v in T.ravel()])])
        try:
            return node.trigger_configure().name
        finally:
            node.destroy_node()
    finally:
        context.shutdown()


@pytest.mark.parametrize('digits, ok', [(6, True), (3, False)])
def test_rigid_tolerance_matches_static_frames(digits, ok):
    assert (_configure(TrackerNode, _tilted_T(digits)) == 'SUCCESS') is ok


def test_track_stamp_is_exposure_time_exactly(rig):
    """노출 시각 T(epoch 규모, ns 자리까지)인 검출을 T+80 ms에 받아도 track stamp = t_state = T(오차 0 ns).

    float 초를 거치면 1.76e9 s에서 분해능이 약 240 ns라 ns 자리가 바뀐다. 시각은 /clock 정수 ns로 맞춘다.
    """
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    r = rig
    det = ObjectDetection(object_class=2, position_valid=True, position=Point(x=-.2, y=.01, z=.02),
                          position_cov=list((np.eye(3)*9e-8).ravel()))
    for t_ns in (1_763_000_000_123_456_789, 1_763_000_000_156_790_123):        # 연속 두 프레임(33 ms 간격)
        T = Time(sec=t_ns // 10**9, nanosec=t_ns % 10**9)
        recv_ns = t_ns + 80_000_000
        r.clock.publish(Clock(clock=Time(sec=recv_ns // 10**9, nanosec=recv_ns % 10**9)))
        r.wait(lambda: r.node.get_clock().now().nanoseconds == recv_ns)
        before = len(r.tracks)
        r.pub.publish(ObjectDetectionArray(header=Header(stamp=T, frame_id='robot_base'), detections=[det]))
        r.wait(lambda: len(r.tracks) > before)
        out = r.tracks[-1]
        assert r.node.get_clock().now().nanoseconds == recv_ns            # 처리 시점은 T + 80 ms
        assert r.node.manager.t_state == seconds(T)
        assert (out.header.stamp.sec, out.header.stamp.nanosec) == (T.sec, T.nanosec)
        assert len(out.tracks) == 1
        tr = out.tracks[0]
        assert (tr.header.stamp.sec, tr.header.stamp.nanosec) == (T.sec, T.nanosec)
        assert (tr.last_observed_stamp.sec, tr.last_observed_stamp.nanosec) == (T.sec, T.nanosec)


def test_late_detections_after_input_gap_are_tracked_again(rig):
    """검출이 노출 뒤 0.3 s(> input_timeout_s 0.2 s)에 오는 입력에서 수신 공백이 한 번 생겨도 회복한다.

    공백 동안 빈 프레임이 t_state를 현재 시각으로 당기면 뒤의 검출(stamp = 수신 − 0.3 s)은 한동안 늦은 프레임이다.
    늦은 프레임도 수신으로 세어야 빈 프레임이 계속 들어가지 않고, stamp가 t_state를 넘는 즉시 다시 추적한다.
    (2026-11-13 현장: vision 6.6 Hz·지연 p95 422 ms에서 track이 영영 생기지 않았다.)
    """
    r = rig
    lag, period = .3, .15

    def send(t):
        r.set_time(t)
        x = -.2 + .02*(t-100.)
        det = ObjectDetection(object_class=2, position_valid=True, position=Point(x=x, y=.01, z=.02),
                              position_cov=list((np.eye(3)*9e-8).ravel()))
        r.pub.publish(ObjectDetectionArray(header=Header(stamp=stamp(t-lag), frame_id='robot_base'),
                                           detections=[det]))
        r.spin(.06)

    t = 100.
    for _ in range(4):
        send(t)
        t += period
    r.set_time(t+.1)                       # 수신 공백 0.25 s: 이 사이에 빈 프레임이 들어간다
    r.spin(.15)
    t += .1
    for _ in range(14):                    # 2 s 동안 같은 지연으로 계속 들어온다
        send(t)
        t += period
    last_stamp = stamp(t-period-lag)
    observed = [tr for msg in r.tracks[-5:] for tr in msg.tracks
                if (tr.last_observed_stamp.sec, tr.last_observed_stamp.nanosec) == (last_stamp.sec, last_stamp.nanosec)]
    assert observed, 'late detections after an input gap were never tracked again'


def _late(r, t_ros, t_stamp, frame_id='robot_base'):
    """ROS 시각 `t_ros`에 stamp `t_stamp`인 검출 하나를 보낸다(늦은 프레임 시험용)."""
    r.set_time(t_ros)
    det = ObjectDetection(object_class=2, position_valid=True, position=Point(x=-.2, y=.01, z=.02),
                          position_cov=list((np.eye(3)*9e-8).ravel()))
    r.pub.publish(ObjectDetectionArray(header=Header(stamp=stamp(t_stamp), frame_id=frame_id), detections=[det]))
    r.spin(.03)


@pytest.mark.parametrize('kind', ['frozen_stamp', 'other_frame'])
def test_late_input_that_does_not_advance_still_ages_tracks(rig, kind):
    """stamp가 앞으로 가지 않는 흐름이나 다른 좌표계의 늦은 흐름은 입력 수신으로 세지 않는다. 그런 흐름이
    `input_timeout_s`보다 자주 와도 빈 프레임이 들어가 track이 LOST까지 늙는다(SelectTarget은 OBSERVED만 고른다).
    """
    r = rig
    frame(r, 100.)
    t = 100.
    while t < 101.4:
        t += .05
        if kind == 'frozen_stamp':                 # 마지막으로 받은 stamp가 되풀이된다
            _late(r, t, 100.)
        else:                                      # 늦은(2 s) 다른 좌표계 검출: 시험 내내 stamp가 t_state 뒤에 있다
            _late(r, t, t - 2., frame_id='camera_color_optical_frame')
    r.wait(lambda: any(a.tracks and a.tracks[0].status == 2 for a in r.tracks))
