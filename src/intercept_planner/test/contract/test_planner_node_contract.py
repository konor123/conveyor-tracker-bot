"""Controlled tracked fixtures → real planner DDS contracts and I2 scenarios."""
import sys
from pathlib import Path
import numpy as np
import pytest
pytest.importorskip('rclpy')
sys.path.insert(0,str(Path(__file__).parent))
from planner_rig import Rig, stamp, seconds
from intercept_planner.intercept_planner_node import InterceptPlannerNode
from drop_interfaces.msg import TrackedObjectArray, TrackedObject, InterceptPlan, Heartbeat
from drop_interfaces.srv import ComputeIntercept
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy



@pytest.fixture
def rig():
    transform = np.eye(4)
    transform[0,3] = .4
    r = Rig(InterceptPlannerNode, {'T_base_conveyor':list(transform.ravel())})
    r.plans = r.record(InterceptPlan,'drop/intercept_plan')
    r.beats = r.record(Heartbeat,'system/heartbeat')
    r.markers = r.record(Marker,'drop/predicted_marker',QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
    r.pub = r.publisher(TrackedObjectArray,'objects/tracked')
    r.spin(.2)
    yield r
    r.close()


def track(r, x=-.1, variance=1e-10, epoch=4, status=0):
    tr = TrackedObject(header=Header(stamp=stamp(r.t),frame_id='conveyor_frame'),
        track_id=7,epoch=epoch,object_class=2,status=status,
        position=Point(x=x,y=.01,z=.02),velocity=Vector3(x=.02),
        state_cov=list((np.eye(4)*variance).ravel()),valid_until=stamp(r.t+1.),
        last_observed_stamp=stamp(r.t),observation_count=60)
    r.pub.publish(TrackedObjectArray(header=tr.header,tracks=[tr]))
    # Subscription completion is observed at the service boundary: FIFO executor
    # draining avoids sleeps tied to the synthetic ROS timeline.
    r.spin(.05)
    return tr


def compute(r, **kw):
    fields = dict(target_track_id=7,target_epoch=4,mode=2)
    fields.update(kw)
    return r.call(ComputeIntercept,'planner/compute_intercept',**fields)


def test_revision_and_expiry(rig):
    r = rig
    track(r)
    result = compute(r)
    assert result.result == 0
    r.wait(lambda: len(r.plans) == 1)
    for i in (1,2):
        r.set_time(100.+i*.1)
        track(r,x=-.1+i*.002)
        r.wait(lambda: len(r.plans) == i+1)
    assert [p.revision for p in r.plans] == [1,2,3]
    assert {p.plan_id for p in r.plans} == {result.plan.plan_id}
    assert all(p.release_time == result.plan.release_time for p in r.plans)
    r.set_time(seconds(result.plan.valid_until)+.01)
    r.wait(lambda: r.plans[-1].status == 4)
    assert r.plans[-1].reason == 'PLAN_EXPIRED'
    count = len(r.plans)
    r.set_time(r.t+1.)
    track(r)
    r.spin(.2)
    assert len(r.plans) == count
    assert compute(r).plan.plan_id > result.plan.plan_id


def test_epoch(rig):
    track(rig)
    result = compute(rig,target_epoch=3)
    assert (result.result,result.reason_code) == (1,'EPOCH_MISMATCH')


@pytest.mark.parametrize('case,expected',[
    ('normal',(0,0,'')), ('time_insufficient',(1,1,'TIME_MARGIN')),
    ('uncertain',(1,3,'UNCERTAIN')), ('lost',(1,4,'TRACK_LOST')),
    ('expired',(1,4,'PLAN_EXPIRED')), ('epoch',(1,4,'EPOCH_MISMATCH')),
    ('clock_reset',(1,4,'CLOCK_RESET')),
])
def test_i2(rig,case,expected):
    r = rig
    track(r, x=.25 if case == 'time_insufficient' else -.1,
          variance=1e-4 if case == 'uncertain' else 1e-10,
          status=2 if case == 'lost' else 0)
    result = compute(r)
    if case in ('expired','epoch','clock_reset'):
        assert result.result == 0
        r.wait(lambda: bool(r.plans))
        if case == 'expired':
            r.set_time(seconds(result.plan.valid_until)+.01)
        elif case == 'epoch':
            r.set_time(100.1)
            track(r,epoch=5)
        else:
            r.set_time(90.)
        r.wait(lambda: r.plans[-1].status == 4)
        p = r.plans[-1]
        assert (1,p.status,p.reason) == expected
        n = len(r.plans)
        r.spin(.25)
        assert len(r.plans) == n
    else:
        assert (result.result,result.plan.status,result.plan.reason) == expected
        if case == 'normal':
            assert result.plan.target_tcp_pose.position.x == pytest.approx(.356)
            assert result.plan.target_velocity.x == pytest.approx(.02)
            assert result.plan.position_std == pytest.approx(np.sqrt(8.84e-10))


def test_lifecycle_deadline_marker_and_qos(rig):
    r = rig
    track(r)
    assert (compute(r,deadline=stamp(r.t)).result) == 2
    assert compute(r,mode=0).reason_code == 'MODE_UNSUPPORTED'
    assert compute(r,target_track_id=99).reason_code == 'TRACK_NOT_FOUND'
    assert compute(r).result == 0
    r.wait(lambda: len(r.markers) >= 3 and len(r.beats) >= 3)
    assert r.markers[-1].header.frame_id == 'robot_base'
    assert r.markers[-1].pose.position.x == pytest.approx(.356)
    for topic,depth,reliable in [('drop/intercept_plan',5,True),('drop/predicted_marker',1,False),('system/heartbeat',1,True)]:
        pub = next(p for p in r.node.publishers if p.topic_name == '/'+topic)
        assert pub.qos_profile.depth == depth
        assert pub.qos_profile.durability == DurabilityPolicy.VOLATILE
        assert pub.qos_profile.reliability == (ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
        assert not pub.event_handlers
    r.node.trigger_deactivate()
    r.spin()
    counts = len(r.plans),len(r.markers)
    res = compute(r)
    assert (res.result,res.reason_code) == (3,'NODE_INACTIVE')
    r.set_time(110.)
    r.spin(.3)
    assert counts == (len(r.plans),len(r.markers))
    assert r.beats[-1].lifecycle_state == 2
    assert r.beats[-1].seq > r.beats[0].seq


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
    assert (_configure(InterceptPlannerNode, _tilted_T(digits)) == 'SUCCESS') is ok
