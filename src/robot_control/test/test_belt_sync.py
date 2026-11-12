from dataclasses import replace
import numpy as np
import pytest
from robot_control.belt_sync import PlanSample, SyncConfig, BeltSynchronizer

pytestmark = [pytest.mark.level('U'), pytest.mark.req('REQ-ROB-013')]

def plan(**kw):
    return replace(PlanSample(1, 7, 2, 1, 1., 1., 2., 3., (0.52, 0.), (0.02, 0.)), **kw)

def controller():
    c = BeltSynchronizer(SyncConfig())
    c.arm(1, 7, 2)
    assert c.accept(plan(), 1., 10.)
    return c

def step(c, now=1., steady=10., pos=(0.5, 0.), stamp=None):
    return c.step(now, steady, pos, now if stamp is None else stamp, 'robot_base')

@pytest.mark.tc('TC-ROB-013-01')
def test_release_time_reference_not_source_stamp():
    out = step(controller())
    assert out.target_xy_m == pytest.approx((.5, 0.))
    assert out.velocity_xy_mps[0] > 0

@pytest.mark.tc('TC-ROB-013-04')
@pytest.mark.req('REQ-ROB-006')
@pytest.mark.tc('TC-ROB-006-01')
def test_speed_and_acceleration_limits():
    c = controller()
    previous = np.zeros(2)
    for k in range(15):
        out = step(c, 1 + k*.01, 10+k*.01, (.40, -.1))
        cmd = np.array(out.velocity_xy_mps)
        assert out.active
        assert np.linalg.norm(cmd) <= c.cfg.max_speed_mps + 1e-10
        assert np.linalg.norm(cmd - previous) <= c.cfg.max_accel_mps2*.01 + 1e-10
        previous = cmd

@pytest.mark.tc('TC-ROB-013-02')
@pytest.mark.parametrize('changes', [dict(epoch=3), dict(track_id=8), dict(plan_id=2), dict(status=4), dict(frame_id='conveyor_frame'), dict(velocity_xy_mps=(float('nan'), 0.)), dict(valid_until_s=.9), dict(source_stamp_s=1.1), dict(position_std_m=.1)])
def test_invalid_plan_latches(changes):
    c = controller()
    assert not c.accept(plan(revision=2, **changes), 1., 10.01)
    assert not step(c).active
    assert not c.accept(plan(revision=3), 1., 10.02)

@pytest.mark.tc('TC-ROB-013-02')
def test_old_revision_ignored_without_refresh():
    c = controller()                                   # revision 1 수신 steady 10.0
    assert not c.accept(plan(), 1., 10.2)              # 같은 revision 재수신: 무시
    # ROS 시계를 멈춰 입력 상태 나이는 0으로 두고, 수신(steady) 감시만 본다
    reasons = [step(c, 1., 10. + k*.01).reason for k in range(27)]
    assert reasons[25] == 'OK'                         # steady 10.25: 수신 뒤 0.25 s
    assert reasons[26] == 'PLAN_STALE'                 # 재수신이 감시를 갱신했다면 10.45까지 OK

@pytest.mark.tc('TC-ROB-013-03')
@pytest.mark.parametrize('now,steady,stamp,reason', [(1.01,10.04,1.01,'LOOP_GAP'),(.9,10.01,.9,'CLOCK_BACKWARD'),(1.01,10.01,.5,'TCP_STALE'),(1.01,10.01,1.1,'TCP_STALE')])
def test_watchdogs(now, steady, stamp, reason):
    c = controller()
    step(c)
    out = step(c,now,steady,stamp=stamp)
    assert not out.active and out.reason == reason
    assert out.velocity_xy_mps == (0.,0.)

@pytest.mark.tc('TC-ROB-013-02')
def test_source_stale_even_with_recent_receipt():
    c = controller()
    assert not c.accept(plan(revision=2, generated_s=1.3), 1.3, 10.3)

@pytest.mark.tc('TC-ROB-013-01')
def test_new_revision_accepted():
    c = controller()
    assert c.accept(plan(revision=2, velocity_xy_mps=(.03,0.)), 1.,10.)
    assert step(c).target_xy_m == pytest.approx((.49,0.))

@pytest.mark.tc('TC-ROB-013-04')
def test_workspace_failure():
    assert not step(controller(),pos=(2.,0.)).active

@pytest.mark.tc('TC-ROB-013-05')
def test_stop_and_explicit_rearm():
    c = controller()
    c.stop('OPERATOR_STOP')
    assert not step(c).active
    c.arm(1,7,2)
    assert c.accept(plan(),1.,10.)
    assert step(c).active

@pytest.mark.tc('TC-ROB-013-03')
def test_freeze_ros_clock_detected_by_steady_time():
    c = controller()
    for k in range(30):
        out = step(c,1.,10.+k*.01)
    assert not out.active and out.reason == 'PLAN_STALE'

@pytest.mark.tc('TC-ROB-013-04')
@pytest.mark.parametrize('config', [dict(period_s=0),dict(kp=float('nan')),dict(min_xy_m=(1,1),max_xy_m=(0,0)),dict(period_s=.1)])
def test_invalid_config(config):
    with pytest.raises(ValueError):
        SyncConfig(**config)

@pytest.mark.tc('TC-ROB-013-03')
def test_expiration_without_loop_gap():
    c = controller()
    assert c.accept(plan(revision=2,valid_until_s=1.005),1.,10.)
    step(c)
    assert step(c,1.01,10.01).reason == 'PLAN_EXPIRED'

@pytest.mark.tc('TC-ROB-013-03')
def test_no_plan_and_nonfinite_tcp():
    c = BeltSynchronizer()
    assert not step(c).active
    c.arm(1,7,2)
    assert step(c).reason == 'WAIT_PLAN'
    assert c.accept(plan(),1.,10.)
    assert step(c,pos=(float('nan'),0)).reason == 'NONFINITE'

@pytest.mark.tc('TC-ROB-013-01')
def test_ros_message_mapping():
    from types import SimpleNamespace as NS
    from robot_control.belt_sync_preview import sample
    stamp = NS(sec=1,nanosec=500000000)
    msg = NS(plan_id=1,target_track_id=7,target_epoch=2,revision=3,
             header=NS(stamp=stamp,frame_id='robot_base'),source_state_stamp=stamp,
             release_time=NS(sec=2,nanosec=0),valid_until=NS(sec=3,nanosec=0),
             target_tcp_pose=NS(position=NS(x=.5,y=.1,z=.2)),
             target_velocity=NS(x=.02,y=.01,z=0),status=0,position_std=.0003)
    p = sample(msg)
    assert p.source_stamp_s == 1.5 and p.release_s == 2.
    assert p.release_xy_m == (.5,.1) and p.velocity_xy_mps == (.02,.01)
    assert p.release_z_m == .2
