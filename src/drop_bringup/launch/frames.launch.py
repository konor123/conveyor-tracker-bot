"""고정 좌표계 static TF와 (선택) RViz2(SPEC-600 4.9절).

  ros2 launch drop_bringup frames.launch.py                       # 로봇 드라이버와 함께(world → base_link는 드라이버 몫)
  ros2 launch drop_bringup frames.launch.py publish_world:=true rviz:=true   # 드라이버 없이 replay로 볼 때
  ros2 launch drop_bringup frames.launch.py publish_world:=true rviz:=true scenario:=nominal_20mms   # mock 세계와 함께

conveyor_frame은 tracking.yaml의 T_base_conveyor를 그대로 쓴다(planner.yaml과 같은 값, TC-CAL-012-01).
scenario를 주면 그 시나리오의 world.frames 변환을 쓴다. drop.launch.py가 mock 세계일 때 실제 tracker·planner에
넘기는 값과 같아 cap_<id> TF와 예측 Marker가 같은 자리에 보인다(TC-CAL-012-03).
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
    scenario = LaunchConfiguration('scenario').perform(context)
    if scenario:
        from drop_mocks.ros_common import scenario_file
        from drop_mocks.scenario_engine import scenario_geometry
        with open(scenario_file(scenario)) as fh:
            T = scenario_geometry(yaml.safe_load(fh)['world'])['T_base_conveyor']
    else:
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
        DeclareLaunchArgument('scenario', default_value='',
                              description='drop_mocks 시나리오 이름. 주면 그 world.frames를 conveyor_frame으로 쓴다(mock 세계용)'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='config/drop.rviz로 RViz2 실행'),
        OpaqueFunction(function=static_frames),
        Node(package='rviz2', executable='rviz2', name='drop_rviz', output='screen',
             arguments=['-d', os.path.join(SHARE, 'config', 'drop.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
