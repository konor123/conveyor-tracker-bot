"""Managed lifecycle transport adapter for the pure conveyor TrackManager."""
from collections import OrderedDict
from pathlib import Path
import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from drop_interfaces.msg import BeltMotion, Heartbeat, ObjectDetectionArray, TrackedObject, TrackedObjectArray
from drop_interfaces.srv import ResetTracking, SelectTarget
from geometry_msgs.msg import Point, Vector3, TransformStamped
from lifecycle_msgs.msg import State
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from .association import Det
from .track_manager import TrackManager, LOST
from .select_target import select_target


def stamp(t):
    ns = round(float(t)*1e9)
    return Time(sec=ns//10**9, nanosec=ns%10**9)


def seconds(t):
    return t.sec+t.nanosec*1e-9


def qos(depth):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


class TrackerNode(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__('tracker_node', **kwargs)
        cfg = yaml.safe_load((Path(get_package_share_directory('tracking'))/'config/tracking.yaml').read_text())
        for section, value in cfg.items():
            if section == 'unitless':
                continue
            if isinstance(value, dict):
                for key, default in value.items():
                    self.declare_parameter(f'{section}.{key}', default)
            else:
                self.declare_parameter(section, value)
        self.active = False
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        self.manager = None
        self._last_received = None
        self._jumped = False
        self._seq = 0
        # 관측 프레임 시각(float 초) → 입력 header.stamp. 1.76e9 s에서 float 분해능은 약 240 ns라
        # 발행 stamp를 float에서 되살리면 ns 자리가 바뀐다(SPEC-000 REQ-SYS-003, 오차 0 ns).
        self._input_stamps = OrderedDict()
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=False),
            post_callback=self._on_jump)
        self._tracked_pub = self._publisher(TrackedObjectArray, 'objects/tracked', 5)
        self._belt_pub = self._publisher(BeltMotion, 'belt/measured_motion', 5)
        self._tf_pub = self._publisher(TFMessage, '/tf', 100)
        self._hb_pub = self._publisher(Heartbeat, 'system/heartbeat', 1)
        self.create_subscription(ObjectDetectionArray, 'objects/detections', self._detections, qos(5),
            event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))
        self.create_service(ResetTracking, 'tracking/reset', self._reset)
        self.create_service(SelectTarget, 'tracking/select_target', self._select)
        self.create_timer(1/30., self._tick, clock=self._steady)
        self.create_timer(.1, self._heartbeat, clock=self._steady)

    def _publisher(self, typ, topic, depth):
        return self.create_publisher(typ, topic, qos(depth),
            event_callbacks=PublisherEventCallbacks(use_default_callbacks=False))

    def value(self, key):
        return self.get_parameter(key).value

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_configure(self, state):
        try:
            transform = np.asarray(self.value('T_base_conveyor'), dtype=float).reshape(4,4)
            if (not np.isfinite(transform).all() or not np.allclose(transform[3], [0,0,0,1])
                    # 허용오차는 calibration.static_frames.RIGID_TOL과 같다(SPEC-600 4.9절)
                    or not np.allclose(transform[:3,:3].T@transform[:3,:3], np.eye(3), atol=1e-5, rtol=0)
                    or not np.isclose(np.linalg.det(transform[:3,:3]), 1., atol=1e-5, rtol=0)):
                raise ValueError('T_base_conveyor must be a rigid transform')
            self._inverse = np.linalg.inv(transform)
            if self.value('input_timeout_s') <= 0:
                raise ValueError('input_timeout_s must be positive')
            keys = {'sigma_accel_mps2':'kf.sigma_accel_mps2','meas_std_default_m':'kf.meas_std_default_m',
                'init_vel_std_mps':'kf.init_vel_std_mps','gate_chi2':'assoc.gate_chi2',
                'coast_max_s':'coast.max_s','x_obs_min_m':'region.x_obs_min_m',
                'x_obs_max_m':'region.x_obs_max_m','exit_margin_m':'region.exit_margin_m',
                'cusum_k':'speed_change.cusum_k','cusum_h':'speed_change.cusum_h',
                'recover_s':'speed_change.recover_s','window_max_s':'belt.window_max_s',
                'v_min_mps':'belt.v_min_mps','backstep_tol_s':'clock.backstep_tol_s'}
            previous = self.manager
            self.manager = TrackManager(**{k:self.value(v) for k,v in keys.items()})
            if previous is not None:
                self.manager.epoch = previous.epoch
                self.manager._next_id = previous._next_id
            self._last_received = None
        except (ValueError,TypeError) as exc:
            self.get_logger().error(str(exc))
            return TransitionCallbackReturn.FAILURE
        self.state_id = State.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.active = True
        self.state_id = State.PRIMARY_STATE_ACTIVE
        self._check_clock()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.active = False
        self.state_id = State.PRIMARY_STATE_INACTIVE
        if self.manager is not None:
            self.manager.reset()
        self._last_received = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.state_id = State.PRIMARY_STATE_UNCONFIGURED
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.on_deactivate(state)
        self.state_id = State.PRIMARY_STATE_FINALIZED
        return TransitionCallbackReturn.SUCCESS

    IGNORED_WARN_COUNT = 90        # 30 Hz에서 3 s
    INPUT_STAMPS_KEPT = 300        # 30 Hz에서 10 s(COASTING track의 last_observed_stamp까지 덮는다)

    def _on_jump(self, jump):
        if jump.delta.nanoseconds*1e-9 < -self.value('clock.backstep_tol_s'):
            self._jumped = True

    def _check_clock(self):
        now = self.now()
        if self.manager is not None:
            previous = self.manager.last_clock
            reset = self.manager.on_clock(now)
            if self._jumped and not reset:
                self.manager.reset()
                reset = True
            if reset:
                self._last_received = None
                back = 'unknown' if previous is None else f'{previous-now:.3f} s'
                self.get_logger().warn(
                    f'ROS time went backwards ({back}): tracks cleared, epoch -> {self.manager.epoch}')
        self._jumped = False
        return now

    def _detections(self, msg):
        if not self.active:
            return
        now = self._check_clock()
        t = seconds(msg.header.stamp)
        if self.manager.t_state is not None and t < self.manager.t_state:
            self.manager.process_frame(t, [])  # preserve the pure dropped-frame counter
            return
        if msg.header.frame_id != 'robot_base':
            return
        rotation = self._inverse[:3,:3]
        dets = []
        for d in msg.detections:
            position = (self._inverse @ [d.position.x,d.position.y,d.position.z,1.])[:3]
            covariance = rotation @ np.asarray(d.position_cov).reshape(3,3) @ rotation.T
            dets.append(Det(position, covariance[:2,:2], d.object_class, d.position_valid))
        self._last_received = now
        self._input_stamps[t] = msg.header.stamp
        while len(self._input_stamps) > self.INPUT_STAMPS_KEPT:
            self._input_stamps.popitem(last=False)
        self._publish(t, self.manager.process_frame(t,dets))
        m = self.manager
        if m.created_total == 0 and m.ignored_outside >= self.IGNORED_WARN_COUNT:
            # 보정(T_base_conveyor)·구간 설정이 맞지 않으면 모든 검출이 구간 밖이라 track이 하나도 안 생긴다.
            self.get_logger().warn(
                f'{m.ignored_outside} detections outside the observation region '
                f'[{m.x_obs_min_m:.3f}, {m.x_obs_max_m:.3f}] m were ignored and no track was ever created '
                f'(last x {m.last_ignored_x:.3f} m): check T_base_conveyor and region.*',
                throttle_duration_sec=30.0)

    def _tick(self):
        now = self._check_clock()
        # Input grace avoids projecting to wall time between normally delayed frames.
        # Once the stream stops, synthetic empty frames age the same pure state machine.
        if (self.active and self._last_received is not None
                and now-self._last_received > self.value('input_timeout_s')
                and (self.manager.t_state is None or now > self.manager.t_state)):
            self._publish(now, self.manager.process_frame(now, []))

    def _stamp(self, t):
        """관측 프레임 시각이면 입력 stamp 그대로, 아니면(합성 빈 프레임·예측 시각) float에서 만든다."""
        exact = self._input_stamps.get(t)
        return exact if exact is not None else stamp(t)

    def _publish(self, t, tracks):
        header = Header(stamp=self._stamp(t),frame_id='conveyor_frame')
        messages, transforms = [], []
        for tr in tracks:
            messages.append(TrackedObject(header=header,track_id=tr.track_id,epoch=tr.epoch,
                object_class=tr.object_class,status=tr.status,last_observed_stamp=self._stamp(tr.last_observed),
                observation_count=tr.observation_count,position=Point(**dict(zip('xyz',map(float,tr.position)))),
                velocity=Vector3(**dict(zip('xyz',map(float,tr.velocity)))),
                state_cov=list(tr.state_cov.ravel()),valid_until=stamp(tr.valid_until)))
            if tr.status != LOST:
                tf = TransformStamped(header=header,child_frame_id=f'cap_{tr.track_id}')
                tf.transform.translation = Vector3(**dict(zip('xyz',map(float,tr.position))))
                tf.transform.rotation.w = 1.
                transforms.append(tf)
        self._tracked_pub.publish(TrackedObjectArray(header=header,tracks=messages))
        if transforms:
            self._tf_pub.publish(TFMessage(transforms=transforms))
        belt = self.manager.belt_motion
        if belt is not None:
            self._belt_pub.publish(BeltMotion(header=Header(stamp=self._stamp(belt.t_state),frame_id='conveyor_frame'),
                direction=Vector3(x=float(belt.direction[0]),y=float(belt.direction[1])),
                speed=float(belt.speed),speed_std=float(belt.speed_std),window_sec=float(belt.window_sec),
                source=belt.source,steady=belt.steady,last_change_stamp=stamp(belt.last_change_stamp or 0.)))

    def _guard(self, req, res):
        if not self.active:
            res.result,res.reason_code = res.PRECONDITION,'NODE_INACTIVE'
            return False
        now = self._check_clock()
        deadline = seconds(req.deadline)
        if deadline > 0 and now >= deadline:
            res.result,res.reason_code = res.TIMEOUT,'DEADLINE'
            return False
        return True

    def _reset(self, req, res):
        if self._guard(req,res):
            result = self.manager.reset(req.region)
            self._last_received = None
            self.get_logger().info(f'ResetTracking: tracks cleared, epoch -> {result.new_epoch}')
            res.result,res.new_epoch = result.result,result.new_epoch
        return res

    def _select(self, req, res):
        if self._guard(req,res):
            result = select_target(self.manager.tracks,self.manager.belt_motion,epoch=self.manager.epoch,
                object_class=req.object_class,min_observation_count=req.min_observation_count,
                v_min_mps=self.value('belt.v_min_mps'),x_reach_start_m=self.value('select.x_reach_start_m'))
            res.result,res.reason_code,res.epoch = result.result,result.reason,result.epoch
            if result.track_id is not None:
                res.track_id,res.t_enter_window = result.track_id,stamp(result.t_enter_window)
        return res

    def _heartbeat(self):
        self._seq += 1
        self._hb_pub.publish(Heartbeat(header=Header(stamp=self.get_clock().now().to_msg()),
            node_name=self.get_name(),lifecycle_state=self.state_id,seq=self._seq))


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
