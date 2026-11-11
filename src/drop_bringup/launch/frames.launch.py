"""고정 좌표계 static TF와 (선택) RViz2(SPEC-600 4.9절).

  ros2 launch drop_bringup frames.launch.py                       # 로봇 드라이버와 함께(world → base_link는 드라이버 몫)
  ros2 launch drop_bringup frames.launch.py publish_world:=true rviz:=true   # 드라이버 없이 mock·replay로 볼 때

conveyor_frame은 tracking.yaml의 T_base_conveyor를 그대로 쓴다(planner.yaml과 같은 값, TC-CAL-012-01).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('drop_bringup')


def static_frames(context):
    tracking = os.path.join(get_package_share_directory('tracking'), 'config', 'tracking.yaml')
    with open(tracking) as fh:
        T = [float(v) for v in yaml.safe_load(fh)['T_base_conveyor']]
    return [Node(package='calibration', executable='static_frames_publisher', name='static_frames_publisher',
                 output='screen', parameters=[{
                     'T_base_conveyor': T,
                     'mat_plane_file': LaunchConfiguration('mat_plane_file').perform(context),
                     # 매트 좌표계 원점(표시용): 2026-11-09 매트 공깃돌 검출 위치(x 0.643~0.712, y −0.066~0.024 m)의 가운데
                     'mat_origin_xy': [0.68, -0.02],
                     'publish_world': LaunchConfiguration('publish_world').perform(context) == 'true',
                 }])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mat_plane_file', default_value='records/calib/2026-11-09-planes/mat_plane.yaml',
                              description='상대 경로는 저장소 루트 기준(SPEC-000 4.4절)'),
        DeclareLaunchArgument('publish_world', default_value='false', choices=['true', 'false'],
                              description='world → base_link. 로봇 드라이버가 없을 때만 true'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='config/drop.rviz로 RViz2 실행'),
        OpaqueFunction(function=static_frames),
        Node(package='rviz2', executable='rviz2', name='drop_rviz', output='screen',
             arguments=['-d', os.path.join(SHARE, 'config', 'drop.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
