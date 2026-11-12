"""rosbag replay bringup(PLAN.md 7.4절 5항·7.5절 I1). 로봇 지령 경로를 연결하지 않는다(TC-INT-007-01).

  ros2 launch drop_bringup replay.launch.py bag:=<bag 폴더> vision_params_file:=records/calib/2026-11-09-planes/vision_node.yaml
  ros2 launch drop_bringup replay.launch.py bag:=<bag 폴더> rviz:=true       # static TF + RViz2

띄우는 것: bag 재생기(영상 토픽만), 실제 vision·tracker·planner(lifecycle, configure → activate 자동),
rviz:=true일 때 static TF·RViz2. robot_controller·orchestrator·safety_supervisor·record_logger, 그리퍼 경로,
Doosan 스트림 지령 발행자는 띄우지 않는다. drop.launch.py를 포함하지 않는 것도 그 때문이다(robot 역할이 들어오지 않게).

재생 토픽은 `topics`(공백 구분)만이다. 비우면 `image_topic`과 같은 접두의 camera_info를 더한 두 토픽이다.
녹화에 로봇 토픽(`/dsr01/...`)이나 이전 파이프라인 출력(`objects/...`)이 있어도 재생하지 않는다.
use_sim_time:=true(기본)면 재생기가 bag 시각으로 /clock을 내고 모든 노드가 그 시각을 쓴다.
bag을 비우면 재생 없이 노드만 띄운다(그래프 확인용).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

CLOCK_HZ = '100'


def _repo_path(value):
    """상대 경로는 저장소 루트 기준(SPEC-000 REQ-SYS-005)."""
    try:
        from calibration.repo_paths import resolve_path
    except ImportError:
        return os.path.abspath(os.path.expanduser(value)) if value else value
    return resolve_path(value)


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _autostart(node):
    """lifecycle 노드를 기동 때 한 번 configure → activate까지 올린다(drop.launch.py와 같은 방식)."""
    def change(transition):
        return EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node), transition_id=transition))
    return [RegisterEventHandler(OnStateTransition(target_lifecycle_node=node, start_state='configuring',
                                                   goal_state='inactive', handle_once=True,
                                                   entities=[change(Transition.TRANSITION_ACTIVATE)])),
            node, change(Transition.TRANSITION_CONFIGURE)]


def replay_topics(image_topic, topics):
    """재생할 토픽 목록. 비우면 영상과 같은 접두의 camera_info."""
    listed = topics.split()
    if listed:
        return listed
    return [image_topic, image_topic.rsplit('/', 1)[0] + '/camera_info']


def _actions(context):
    sim_time = _flag(context, 'use_sim_time')
    image_topic = LaunchConfiguration('image_topic').perform(context)
    vision_params = [os.path.join(get_package_share_directory('vision'), 'config', 'vision.yaml')]
    site = LaunchConfiguration('vision_params_file').perform(context)
    if site:
        vision_params.append(_repo_path(site))
    out = []
    out += _autostart(LifecycleNode(package='vision', executable='vision_node', name='vision_node', namespace='',
                                    output='screen', parameters=[*vision_params, {'use_sim_time': sim_time,
                                                                                  'image_topic': image_topic}]))
    out += _autostart(LifecycleNode(package='tracking', executable='tracker_node', name='tracker_node', namespace='',
                                    output='screen', parameters=[{'use_sim_time': sim_time}]))
    out += _autostart(LifecycleNode(package='intercept_planner', executable='intercept_planner_node',
                                    name='intercept_planner_node', namespace='', output='screen',
                                    parameters=[{'use_sim_time': sim_time}]))
    if _flag(context, 'rviz'):
        frames = os.path.join(get_package_share_directory('drop_bringup'), 'launch', 'frames.launch.py')
        out.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(frames),
                                            launch_arguments={'publish_world': 'true', 'rviz': 'true'}.items()))
    bag = LaunchConfiguration('bag').perform(context)
    if bag:
        cmd = ['ros2', 'bag', 'play', _repo_path(bag), '--disable-keyboard-controls',
               '--rate', LaunchConfiguration('rate').perform(context),
               '--delay', LaunchConfiguration('delay_s').perform(context),
               '--topics', *replay_topics(image_topic, LaunchConfiguration('topics').perform(context))]
        if sim_time:
            cmd += ['--clock', CLOCK_HZ]
        if _flag(context, 'loop'):
            cmd.append('--loop')
        out.append(ExecuteProcess(cmd=cmd, name='rosbag_play', output='screen'))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='', description='재생할 bag 폴더. 비우면 재생하지 않는다'),
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw',
                              description='vision_node 입력 영상 토픽(파라미터 파일 값보다 우선)'),
        DeclareLaunchArgument('topics', default_value='',
                              description='재생할 토픽(공백 구분). 비우면 image_topic과 그 camera_info'),
        DeclareLaunchArgument('vision_params_file', default_value='',
                              description='vision_node 현장 파라미터 파일. 상대 경로는 저장소 루트 기준'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'],
                              description='true면 재생기가 bag 시각으로 /clock을 낸다'),
        DeclareLaunchArgument('rate', default_value='1.0', description='재생 속도 배율'),
        DeclareLaunchArgument('delay_s', default_value='2.0', description='재생 전 대기(노드 활성화 시간) [s]'),
        DeclareLaunchArgument('loop', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='static TF(publish_world)와 RViz2'),
        OpaqueFunction(function=_actions),
    ])
