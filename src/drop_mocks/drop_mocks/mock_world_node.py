"""mock_world_node(SPEC-700 2, 4절): vision·tracker 대역. role:=vision|tracking|all."""
from __future__ import annotations

import math

from drop_interfaces.msg import (BeltMotion, Heartbeat, InterceptPlan, ObjectDetection,
                                 ObjectDetectionArray, TrackedObject, TrackedObjectArray)
from drop_interfaces.srv import ComputeIntercept, DetectObjects, ResetTracking, SelectTarget, VerifyDrop
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .ros_common import (DATA_QOS, HEARTBEAT_PERIOD_S, HEARTBEAT_QOS, MockLifecycleNode, no_pub_events,
                         spin_mock, steady_clock, to_time)
from .scenario_engine import CLASS_CAP, RESULT

ROLE_NODE = {'vision': 'vision_node', 'tracking': 'tracker_node'}
ROLE_SERVICES = {'vision': ('vision/detect_objects', 'vision/verify_drop'),
                 'tracking': ('tracking/reset', 'tracking/select_target')}


def detection_msg(i: int, d) -> ObjectDetection:
    return ObjectDetection(
        detection_id=i, region=d.region, object_class=d.object_class, u=d.u, v=d.v,
        diameter_px=d.diameter_px, confidence=1.0, center_cov_px=list(d.center_cov_px),
        position=Point(x=d.position[0], y=d.position[1], z=d.position[2]),
        position_cov=list(d.position_cov), position_valid=True, occluded_partial=False,
        stone_inside=d.stone_inside, stone_inside_valid=d.stone_inside_valid)


class MockWorldNode(MockLifecycleNode):
    def __init__(self, **kwargs):
        super().__init__('mock_world_node', **kwargs)
        self.declare_parameter('role', 'all')
        self.declare_parameter('publish_plan', True)
        # false = VerifyDrop이 대본 대신 UNKNOWN(실제 controller와 함께 띄울 때, REQ-MCK-010)
        self.declare_parameter('verify_scripted', True)
        role = self.get_parameter('role').value
        if role not in ('vision', 'tracking', 'all'):
            raise ValueError(f'role은 vision|tracking|all: {role}')
        self.roles = ('vision', 'tracking') if role == 'all' else (role,)
        self.publish_plan = bool(self.get_parameter('publish_plan').value) and ('tracking' in self.roles)
        self.resets = 0
        self.verify_calls = 0
        self.plan_id = 0
        self._last_k = None
        self._seq = {r: 0 for r in self.roles}

        self.refuse_if_provided([n for r in self.roles for n in ROLE_SERVICES[r]])
        self.hb_pub = self.create_publisher(Heartbeat, 'system/heartbeat', HEARTBEAT_QOS,
                                            event_callbacks=no_pub_events())
        if 'vision' in self.roles:
            self.det_pub = self.create_publisher(ObjectDetectionArray, 'objects/detections', DATA_QOS,
                                                 event_callbacks=no_pub_events())
            self.create_service(DetectObjects, 'vision/detect_objects', self._detect)
            self.create_service(VerifyDrop, 'vision/verify_drop', self._verify)
        if 'tracking' in self.roles:
            self.trk_pub = self.create_publisher(TrackedObjectArray, 'objects/tracked', DATA_QOS,
                                                 event_callbacks=no_pub_events())
            self.belt_pub = self.create_publisher(BeltMotion, 'belt/measured_motion', DATA_QOS,
                                                  event_callbacks=no_pub_events())
            self.create_service(ResetTracking, 'tracking/reset', self._reset)
            self.create_service(SelectTarget, 'tracking/select_target', self._select)
        if self.publish_plan:
            self.plan_pub = self.create_publisher(InterceptPlan, 'drop/intercept_plan', DATA_QOS,
                                                  event_callbacks=no_pub_events())
            self.create_service(ComputeIntercept, 'planner/compute_intercept', self._compute_intercept)
        self.create_timer(1.0 / self.engine.rate, self._on_frame)
        # heartbeat는 프레임·서비스 콜백과 다른 그룹에 둬서 그쪽이 늦어도 끊기지 않게 하고,
        # steady 시계로 돌려 ROS 시각 정지·역행(NTP 보정 포함)에도 끊기지 않게 한다(REQ-ABT-009)
        self.create_timer(HEARTBEAT_PERIOD_S, self._on_heartbeat, callback_group=MutuallyExclusiveCallbackGroup(),
                          clock=steady_clock())
        self.autostart()

    # ------------------------------------------------------------ 발행
    def _stamp(self, t: float, offset: float):
        return to_time(self.t0 + t + offset)

    def _on_frame(self):
        k = self.engine.frame_index(self.scenario_t())
        if k == self._last_k or not self.active:
            return
        self._last_k = k
        fr = self.engine.frame(k, self.resets)
        if fr.dropped or k < 0:
            return
        stamp = self._stamp(fr.t, fr.stamp_offset)
        if 'vision' in self.roles:
            msg = ObjectDetectionArray(image_seq=k, calibration_id='mock')
            msg.header.stamp, msg.header.frame_id = stamp, 'robot_base'
            msg.detections = [detection_msg(i, d) for i, d in enumerate(fr.detections)]
            self.det_pub.publish(msg)
        if 'tracking' in self.roles:
            arr = TrackedObjectArray()
            arr.header.stamp, arr.header.frame_id = stamp, 'conveyor_frame'
            for tr in fr.tracks:
                m = TrackedObject(
                    track_id=tr.track_id, epoch=tr.epoch, object_class=CLASS_CAP, status=tr.status,
                    last_observed_stamp=self._stamp(tr.last_observed_t, fr.stamp_offset),
                    observation_count=tr.observation_count,
                    position=Point(x=tr.position[0], y=tr.position[1], z=tr.position[2]),
                    velocity=Vector3(x=tr.velocity[0], y=tr.velocity[1], z=tr.velocity[2]),
                    state_cov=list(tr.state_cov),
                    valid_until=self._stamp(fr.t + tr.valid_for_s, fr.stamp_offset))
                m.header = arr.header
                arr.tracks.append(m)
            self.trk_pub.publish(arr)
            b = fr.belt
            belt = BeltMotion(direction=Vector3(x=1.0, y=0.0, z=0.0), speed=b.speed,
                              speed_std=b.speed_std, window_sec=b.window_s,
                              source=BeltMotion.SOURCE_CAP, steady=b.steady)
            belt.header = arr.header
            if b.last_change_t is not None:
                belt.last_change_stamp = self._stamp(b.last_change_t, fr.stamp_offset)
            self.belt_pub.publish(belt)

    def _on_heartbeat(self):
        t = self.scenario_t()
        for role in self.roles:
            if not self.engine.heartbeat_alive(role, t):
                continue
            self._seq[role] += 1
            hb = Heartbeat(node_name=ROLE_NODE[role], lifecycle_state=self.state_id,
                           seq=self._seq[role], error_code=0, detail='')
            hb.header.stamp = self.get_clock().now().to_msg()
            self.hb_pub.publish(hb)

    # ------------------------------------------------------------ 서비스
    def _refuse(self, req, res) -> bool:
        """비활성·deadline 경과면 응답을 채우고 True."""
        if not self.active:
            res.result, res.reason_code = RESULT['PRECONDITION'], 'NODE_INACTIVE'
        elif self.deadline_passed(req.deadline):
            res.result, res.reason_code = RESULT['TIMEOUT'], 'DEADLINE'
        else:
            return False
        return True

    def _detect(self, req, res):
        if self._refuse(req, res):
            return res
        r = self.engine.detect(self.scenario_t(), req.region, req.object_class, req.min_count)
        res.result, res.reason_code = r.result, r.reason_code
        res.detections = [detection_msg(i, d) for i, d in enumerate(r.detections)]
        res.position_std = list(r.position_std)
        return res

    def _verify(self, req, res):
        if self._refuse(req, res):
            return res
        r = self.engine.verify(self.verify_calls, scripted=bool(self.get_parameter('verify_scripted').value))
        self.verify_calls += 1
        res.result, res.verdict = RESULT['OK'], r.verdict
        res.stone_inside, res.stone_offset, res.stone_offset_valid = (
            r.stone_inside, r.stone_offset, r.stone_offset_valid)
        return res

    def _reset(self, req, res):
        if self._refuse(req, res):
            return res
        self.resets += 1
        res.result, res.new_epoch = RESULT['OK'], self.engine.epoch(self.scenario_t(), self.resets)
        return res

    def _select(self, req, res):
        if self._refuse(req, res):
            return res
        t = self.scenario_t()
        r = self.engine.select_target(t, req.min_observation_count, self.resets)
        res.result, res.reason_code = r.result, r.reason_code
        if r.result == RESULT['OK']:
            if not math.isfinite(r.t_enter_window):
                res.result, res.reason_code = RESULT['FAILED'], 'BELT_STOPPED'
                return res
            k = self.engine.frame_index(t)
            offset = self.engine.frame(k).stamp_offset
            res.track_id, res.epoch = r.track_id, r.epoch
            res.t_enter_window = self._stamp(r.t_enter_window, offset)
        return res

    def _compute_intercept(self, req, res):
        if self._refuse(req, res):
            return res
        t = self.scenario_t()
        r = self.engine.compute_intercept(t, req.target_track_id, req.target_epoch, self.resets)
        k = max(0, self.engine.frame_index(t))
        offset = self.engine.frame(k).stamp_offset
        self.plan_id += 1
        plan = InterceptPlan(
            plan_id=self.plan_id, revision=1, target_track_id=r.track_id, target_epoch=r.epoch,
            source_state_stamp=self._stamp(r.source_t, offset),
            release_time=self._stamp(r.release_t, offset),
            valid_until=self._stamp(r.valid_until_t, offset),
            target_tcp_pose=Pose(position=Point(x=r.target_xyz[0], y=r.target_xyz[1], z=r.target_xyz[2]),
                                 orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            target_velocity=Vector3(x=r.target_vel[0], y=r.target_vel[1], z=r.target_vel[2]),
            position_std=r.position_std, time_margin=r.time_margin_s,
            status=r.status, reason=r.reason_code)
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.header.frame_id = 'robot_base'
        res.result, res.reason_code, res.plan = r.result, r.reason_code, plan
        if r.result == RESULT['OK'] and self.publish_plan:
            self.plan_pub.publish(plan)
        return res


def main(args=None):
    spin_mock(MockWorldNode, 'mock_world_node', num_threads=2, args=args)


if __name__ == '__main__':
    main()
