"""고정 좌표계 static TF 발행(SPEC-600 4.9절).

  ros2 run calibration static_frames_publisher --ros-args \
      -p T_base_conveyor:="[1.,0.,0.,0., 0.,1.,0.,0., 0.,0.,1.,0., 0.,0.,0.,1.]" \
      -p mat_plane_file:=records/calib/2026-11-09-planes/mat_plane.yaml [-p publish_world:=true]

base_link → robot_base(항등), robot_base → conveyor_frame, robot_base → mat_frame을 한 번 발행한다.
world → base_link는 Doosan robot_state_publisher 몫이라 publish_world:=true(드라이버 없이 볼 때)일 때만 낸다.
카메라는 camera_tf_publisher가 낸다(단일 발행 원칙).
"""
from __future__ import annotations

import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from .frames import plane_frame
from .geometry import quat_from_R
from .repo_paths import resolve_path


RIGID_TOL = 1e-5   # 소수 6자리로 적은 보정 행렬(직교성 오차 ~1e-6)을 받는다(SPEC-600 4.9절)


def is_rigid(T: np.ndarray) -> bool:
    R = T[:3, :3]
    return (T.shape == (4, 4) and bool(np.isfinite(T).all()) and np.allclose(T[3], [0, 0, 0, 1])
            and np.allclose(R.T @ R, np.eye(3), atol=RIGID_TOL, rtol=0) and np.isclose(np.linalg.det(R), 1.0, atol=RIGID_TOL, rtol=0))


class StaticFramesPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('static_frames_publisher', **kwargs)
        self.declare_parameter('T_base_conveyor', [float(v) for v in np.eye(4).ravel()])
        self.declare_parameter('mat_plane_file', '')
        self.declare_parameter('mat_x_hint', [1.0, 0.0, 0.0])
        self.declare_parameter('mat_origin_xy', [0.0, 0.0])
        self.declare_parameter('publish_world', False)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.ok = False
        try:
            frames = self._frames()
        except (OSError, KeyError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(f'고정 TF 발행 안 함: {exc}')
            return
        self.broadcaster.sendTransform([self._msg(p, c, T) for p, c, T in frames])
        self.ok = True
        self.get_logger().info('고정 TF: ' + ', '.join(f'{p}→{c}' for p, c, _ in frames))

    def _frames(self):
        T_conv = np.asarray(self.get_parameter('T_base_conveyor').value, float)
        if T_conv.size != 16 or not is_rigid(T_conv.reshape(4, 4)):
            raise ValueError('T_base_conveyor는 row-major 4×4 강체 변환이어야 한다')
        frames = [('base_link', 'robot_base', np.eye(4)), ('robot_base', 'conveyor_frame', T_conv.reshape(4, 4))]
        path = resolve_path(self.get_parameter('mat_plane_file').value)
        if not path:
            raise ValueError('mat_plane_file이 비었다')
        plane = yaml.safe_load(open(path))
        if plane.get('frame') != 'robot_base':
            raise ValueError(f'{path}: frame이 robot_base가 아니다')
        frames.append(('robot_base', 'mat_frame', plane_frame(
            plane['normal'], plane['offset_m'], self.get_parameter('mat_x_hint').value,
            self.get_parameter('mat_origin_xy').value)))
        if self.get_parameter('publish_world').value:
            frames.insert(0, ('world', 'base_link', np.eye(4)))
        return frames

    def _msg(self, parent, child, T) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = parent, child
        tr = msg.transform.translation
        tr.x, tr.y, tr.z = (float(v) for v in T[:3, 3])
        q = msg.transform.rotation
        q.x, q.y, q.z, q.w = (float(v) for v in quat_from_R(T[:3, :3]))
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = StaticFramesPublisher()
    if not node.ok:
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
