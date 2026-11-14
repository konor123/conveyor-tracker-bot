"""Managed lifecycle ROS adapter; all planning decisions live in PlanSession."""
from dataclasses import replace
from pathlib import Path
import math
import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from drop_interfaces.msg import Heartbeat, InterceptPlan, TrackedObjectArray
from drop_interfaces.srv import ComputeIntercept
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from lifecycle_msgs.msg import State
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from .intercept import PlannerParams, PlanRequest
from .node_shutdown import ShutdownGuard
from .plan_session import PlanSession
from .prediction import TrackState


def stamp(t):
    ns = round(float(t)*1e9)
    return Time(sec=ns//10**9,nanosec=ns%10**9)


def seconds(t):
    return t.sec+t.nanosec*1e-9


def qos(depth, reliable=True):
    return QoSProfile(depth=depth, reliability=(ReliabilityPolicy.RELIABLE if reliable
        else ReliabilityPolicy.BEST_EFFORT), durability=DurabilityPolicy.VOLATILE)


class InterceptPlannerNode(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__('intercept_planner_node', **kwargs)
        cfg = yaml.safe_load((Path(get_package_share_directory('intercept_planner'))/'config/planner.yaml').read_text())
        self._keys = {}
        for section,value in cfg.items():
            if section == 'unitless':
                continue
            if isinstance(value,dict):
                for key,default in value.items():
                    if isinstance(default,list):
                        default = list(map(float,default))
                    self.declare_parameter(f'{section}.{key}', default)
                    self._keys[key] = f'{section}.{key}'
            else:
                self.declare_parameter(section,list(np.asarray(value,dtype=float).ravel()))
        self.active = False
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        self.session = PlanSession(PlannerParams())
        self._tracks = {}
        self._epoch = 0
        self._last_clock = None
        self._jumped = False
        self._seq = 0
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),on_clock_change=False),
            post_callback=self._on_jump)
        self._plan_pub = self._publisher(InterceptPlan,'drop/intercept_plan',5)
        self._marker_pub = self._publisher(Marker,'drop/predicted_marker',1,False)
        self._hb_pub = self._publisher(Heartbeat,'system/heartbeat',1)
        self.create_subscription(TrackedObjectArray,'objects/tracked',self._tracked,qos(5),
            event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))
        self.create_service(ComputeIntercept,'planner/compute_intercept',self._compute)
        self.create_timer(.02,self._tick,clock=self._steady)
        self.create_timer(.1,self._marker,clock=self._steady)
        self.create_timer(.1,self._heartbeat,clock=self._steady)

    def _publisher(self,typ,topic,depth,reliable=True):
        return self.create_publisher(typ,topic,qos(depth,reliable),
            event_callbacks=PublisherEventCallbacks(use_default_callbacks=False))

    def value(self,key):
        return self.get_parameter(key).value

    def on_configure(self,state):
        try:
            transform = np.asarray(self.value('T_base_conveyor'),dtype=float).reshape(4,4)
            if (not np.isfinite(transform).all() or not np.allclose(transform[3],[0,0,0,1])
                    # 허용오차는 calibration.static_frames.RIGID_TOL과 같다
                    or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-5,rtol=0)
                    or not np.isclose(np.linalg.det(transform[:3,:3]),1.,atol=1e-5,rtol=0)):
                raise ValueError('T_base_conveyor must be a rigid transform')
            self.session.params = PlannerParams(**{k:self.value(v) for k,v in self._keys.items()},
                T_base_conveyor=transform,current_epoch=self._epoch)
        except (ValueError,TypeError) as exc:
            self.get_logger().error(str(exc))
            return TransitionCallbackReturn.FAILURE
        self.state_id = State.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self,state):
        self.active = True
        self.state_id = State.PRIMARY_STATE_ACTIVE
        self._check_clock()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self,state):
        self.active = False
        self.state_id = State.PRIMARY_STATE_INACTIVE
        self.session.expire('NODE_INACTIVE')
        self._tracks.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self,state):
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self,state):
        self.on_deactivate(state)
        self.state_id = State.PRIMARY_STATE_FINALIZED
        return TransitionCallbackReturn.SUCCESS

    def _on_jump(self,jump):
        self._jumped = True

    def _check_clock(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self._jumped or (self._last_clock is not None and now < self._last_clock):
            self._tracks.clear()
            reply = self.session.expire('CLOCK_RESET')
            if self.active:
                self._publish(reply)
        self._jumped = False
        self._last_clock = now
        return now

    def _tracked(self,msg):
        if not self.active or msg.header.frame_id != 'conveyor_frame':
            return
        now = self._check_clock()
        # Wire schema v0.2 has no epoch on an empty array: retain the last known
        # epoch until the next actual track, and let the plan timer expire silence.
        if msg.tracks:
            self._epoch = msg.tracks[0].epoch
        self.session.params = replace(self.session.params,current_epoch=self._epoch)
        self._tracks = {t.track_id:TrackState(t.track_id,t.epoch,t.status,seconds(t.header.stamp),
            np.array([t.position.x,t.position.y]),np.array([t.velocity.x,t.velocity.y]),
            np.asarray(t.state_cov).reshape(4,4),seconds(t.valid_until),seconds(t.last_observed_stamp),
            float(t.position.z))
            for t in msg.tracks}
        active = self.session.active
        if active is not None:
            self._publish(self.session.update(self._tracks.get(active.target_track_id),now,
                                              current_epoch=self._epoch))

    def _compute(self,req,res):
        if not self.active:
            res.result,res.reason_code = res.PRECONDITION,'NODE_INACTIVE'
            return res
        now = self._check_clock()
        deadline = seconds(req.deadline)
        if deadline > 0 and now >= deadline:
            res.result,res.reason_code = res.TIMEOUT,'DEADLINE'
            return res
        request = PlanRequest(req.target_track_id,req.target_epoch,req.mode,deadline or math.inf,
                              seconds(req.earliest_release))
        reply = self.session.compute(self._tracks.get(req.target_track_id),now,request)
        res.result,res.reason_code = int(reply.result),reply.reason_code
        if reply.plan is not None:
            res.plan = self._message(reply.plan)
            self._plan_pub.publish(res.plan)
        return res

    def _message(self,p):
        return InterceptPlan(header=Header(stamp=self.get_clock().now().to_msg(),frame_id='robot_base'),
            plan_id=p.plan_id,revision=p.revision,target_track_id=p.target_track_id,target_epoch=p.target_epoch,
            source_state_stamp=stamp(p.source_state_stamp),release_time=stamp(p.release_time),
            valid_until=stamp(p.valid_until),target_tcp_pose=Pose(
                position=Point(**dict(zip('xyz',map(float,p.target_tcp_pose.position)))),
                orientation=Quaternion(**dict(zip('xyzw',map(float,p.target_tcp_pose.orientation))))),
            target_velocity=Vector3(**dict(zip('xyz',map(float,p.target_velocity)))),
            position_std=float(p.position_std),time_margin=float(p.time_margin),status=int(p.status),reason=p.reason)

    def _publish(self,reply):
        if reply is not None and reply.plan is not None:
            self._plan_pub.publish(self._message(reply.plan))

    def _tick(self):
        now = self._check_clock()
        if self.active:
            self._publish(self.session.update(None,now,current_epoch=self._epoch))

    def _marker(self):
        if not self.active:
            return
        marker = Marker(header=Header(stamp=self.get_clock().now().to_msg(),frame_id='robot_base'),
                        ns='intercept',id=0,type=Marker.SPHERE)
        if self.session.active is None:
            marker.action = Marker.DELETE
        else:
            marker.action = Marker.ADD
            marker.pose = self._message(self.session.active).target_tcp_pose
            marker.scale = Vector3(x=.02,y=.02,z=.02)
            marker.color.r,marker.color.g,marker.color.a = 1.,.5,1.
        self._marker_pub.publish(marker)

    def _heartbeat(self):
        self._seq += 1
        self._hb_pub.publish(Heartbeat(header=Header(stamp=self.get_clock().now().to_msg()),
            node_name=self.get_name(),lifecycle_state=self.state_id,seq=self._seq))


def main(args=None):
    guard = ShutdownGuard()            # rclpy.init 전
    rclpy.init(args=args)
    node = None
    try:
        node = InterceptPlannerNode()
        rclpy.spin(node)
    except BaseException as exc:
        if not guard.is_shutdown(exc):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        guard.close()


if __name__ == '__main__':
    main()
