"""보정 결과를 ROS에 연결하는 노드(SPEC-600 4.8절).

  ros2 run calibration camera_tf_publisher --ros-args -p extrinsics_file:=<camera_extrinsics.yaml>
      드라이버 TF(camera_link ← 컬러 광학)를 기다렸다가 base_link ← camera_link를 static TF로 발행한다.
  ros2 run calibration check_camera_info [--topic /camera/camera/color/camera_info] [--profile <yaml>]
      CameraInfo 한 개를 받아 보정에 쓴 프로파일과 비교한다. 불일치면 종료 코드 2.
"""
from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from .calibration_file import load_extrinsics
from .repo_paths import resolve_path
from .camera_intrinsics_check import compare_camera_info
from .cli import DEFAULT_PROFILE
from .geometry import R_from_quat, base_to_camera_link, make_T, quat_from_R


def _to_T(tf: TransformStamped) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    return make_T(R_from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z]))


class CameraTfPublisher(Node):
    def __init__(self):
        super().__init__('camera_tf_publisher')
        self.declare_parameter('extrinsics_file', '')
        self.declare_parameter('camera_link_frame', 'camera_link')
        self.declare_parameter('wait_s', 10.0)
        path = resolve_path(self.get_parameter('extrinsics_file').value)   # 상대 경로는 저장소 루트 기준
        if not path:
            raise RuntimeError('extrinsics_file 파라미터가 비었다')
        self.cal = load_extrinsics(path)
        self.link = self.get_parameter('camera_link_frame').value
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.deadline = self.get_clock().now().nanoseconds * 1e-9 + float(self.get_parameter('wait_s').value)
        self.done = False
        self.timer = self.create_timer(0.2, self._try_publish)
        self.get_logger().info(f'calibration_id {self.cal.calibration_id}: {self.link} ← {self.cal.child_frame} 대기')

    def _try_publish(self):
        if self.done:
            return
        try:
            T_link_opt = _to_T(self.buf.lookup_transform(self.link, self.cal.child_frame, Time()))
        except Exception as e:                                   # tf2 예외 종류가 여럿이다
            if self.get_clock().now().nanoseconds * 1e-9 > self.deadline:
                self.get_logger().error(f'드라이버 TF {self.link} ← {self.cal.child_frame} 없음: {e}')
                raise SystemExit(1)
            return
        T = base_to_camera_link(self.cal.T, T_link_opt)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cal.parent_frame
        msg.child_frame_id = self.link
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = T[:3, 3].tolist()
        q = quat_from_R(T[:3, :3])
        r = msg.transform.rotation
        r.x, r.y, r.z, r.w = q.tolist()
        self.broadcaster.sendTransform(msg)
        self.done = True
        self.timer.cancel()
        self.get_logger().info(f'static TF 발행 {self.cal.parent_frame} → {self.link}: '
                               f't={np.round(T[:3, 3], 4).tolist()} q={np.round(q, 5).tolist()}')


def camera_tf_publisher(args=None):
    rclpy.init(args=args)
    try:
        node = CameraTfPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as e:
        return getattr(e, 'code', 0) or 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def check_camera_info(argv=None):
    ap = argparse.ArgumentParser(description='CameraInfo ↔ 카메라 프로파일 비교')
    ap.add_argument('--topic', default='/camera/camera/color/camera_info')
    ap.add_argument('--profile', default=DEFAULT_PROFILE)
    ap.add_argument('--timeout-s', type=float, default=10.0)
    a, ros_args = ap.parse_known_args(argv)
    with open(a.profile) as fh:
        profile = yaml.safe_load(fh)
    rclpy.init(args=ros_args)
    node = Node('check_camera_info')
    got = []
    node.create_subscription(CameraInfo, a.topic, got.append, qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds * 1e-9 + a.timeout_s
    while not got and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print(f'{a.topic} 수신 없음({a.timeout_s} s)')
        return 1
    m = got[0]
    info = {'width': m.width, 'height': m.height, 'distortion_model': m.distortion_model, 'k': list(m.k),
            'd': list(m.d)}
    print(f'CameraInfo {m.width}x{m.height} {m.distortion_model} frame {m.header.frame_id}')
    print(f'  K fx {m.k[0]:.4f} fy {m.k[4]:.4f} cx {m.k[2]:.4f} cy {m.k[5]:.4f}  D {np.round(m.d, 6).tolist()}')
    mism = compare_camera_info(info, profile, check_focal=False)
    focal = [m for m in compare_camera_info(info, profile) if m.split(':')[0] in ('fx', 'fy')]
    for line in focal:
        print('경고(초점거리 보고값 변동, vision은 보정 파일의 K를 쓴다):', line)
    for line in mism:
        print('불일치:', line)
    print('일치' if not mism else '불일치')
    return 0 if not mism else 2


if __name__ == '__main__':      # pragma: no cover
    sys.exit(check_camera_info())
