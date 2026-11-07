"""시스템 bringup(PLAN.md 2.4, 7.2절). 역할마다 실제 노드와 대역 중 하나만 띄운다(use_mock_<역할>).

대역은 기동할 때 같은 역할의 서버가 이미 있으면 스스로 종료하고(REQ-MCK-007), 이 launch는 그때 전체를 내린다.
실제 vision·tracker·robot_controller 노드는 아직 없으므로 v0.1에서는 use_mock_<역할>:=false인 역할은 띄우지 않는다.
"""
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROLES = ('vision', 'tracking', 'robot')
REAL_PENDING = {'vision': 'vision vision_node', 'tracking': 'tracking tracker_node',
                'robot': 'robot_control robot_controller_node'}


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _shutdown_on_failure(node):
    """대역이 0이 아닌 코드로 끝나면(동시 기동 거부 등) bringup 전체를 내린다."""
    def on_exit(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f'{node.name} 종료 코드 {event.returncode}: bringup 중단'),
                    EmitEvent(event=Shutdown(reason='mock refused to start'))]
        return []
    return RegisterEventHandler(OnProcessExit(target_action=node, on_exit=on_exit))


def _nodes(context):
    mock = {r: _flag(context, f'use_mock_{r}') for r in ROLES}
    scenario = LaunchConfiguration('scenario').perform(context)
    seed = int(LaunchConfiguration('seed').perform(context))
    sim_time = _flag(context, 'use_sim_time')
    # 역할별 프로세스가 같은 세계를 보도록 시나리오 시각 0을 하나로 정한다(sim time이면 노드 기동 시각)
    t0 = 0.0 if sim_time else time.time()
    common = {'scenario': scenario, 'seed': seed, 't0_s': t0, 'use_sim_time': sim_time}
    out = []
    world_roles = [r for r in ('vision', 'tracking') if mock[r]]
    if world_roles:
        role = 'all' if len(world_roles) == 2 else world_roles[0]
        out.append(Node(package='drop_mocks', executable='mock_world_node', name='mock_world_node',
                        output='screen', parameters=[{**common, 'role': role}]))
    if mock['robot']:
        out.append(Node(package='drop_mocks', executable='mock_robot_controller_node',
                        name='mock_robot_controller_node', output='screen', parameters=[common]))
    for r in ROLES:
        if not mock[r]:
            out.append(LogInfo(msg=f'{r}: 실제 노드({REAL_PENDING[r]})는 아직 없어 띄우지 않는다'))
    return out + [_shutdown_on_failure(n) for n in out if isinstance(n, Node)]


def generate_launch_description():
    args = [DeclareLaunchArgument(f'use_mock_{r}', default_value='false', choices=['true', 'false'],
                                  description=f'true면 {r} 대역을 띄운다') for r in ROLES]
    return LaunchDescription(args + [
        DeclareLaunchArgument('scenario', default_value='nominal_20mms', description='drop_mocks 시나리오'),
        DeclareLaunchArgument('seed', default_value='-1', description='-1이면 시나리오 파일의 seed'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_nodes),
    ])
