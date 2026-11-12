"""시스템 bringup(PLAN.md 2.4, 7.2절). 역할마다 실제 노드와 대역 중 하나만 띄운다(use_mock_<역할>).

대역은 기동할 때 같은 역할의 서버가 이미 있으면 스스로 종료하고(REQ-MCK-007), 이 launch는 그때 전체를 내린다.
vision·tracking·planner는 실제 lifecycle 노드 또는 대역을 선택한다. robot의 실제 경로는 별도 단계다.
실제 tracker·planner는 launch가 configure → activate까지 자동으로 전이한다(SPEC-400 REQ-PLN-012).
use_mock_planner:=false(기본)면 mock_world_node의 임시 plan(publish_plan)을 끈다.
use_mock_planner:=true는 임시 plan을 내는 tracking 대역이 있어야 하므로 use_mock_tracking:=true가 아니면 기동을 거부한다.
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition

ROLES = ('vision', 'tracking', 'planner', 'robot')
REAL_PENDING = {'robot': 'robot_control robot_controller_node'}


def _repo_path(value):
    """상대 경로는 저장소 루트 기준(SPEC-000 REQ-SYS-005). calibration이 없으면 실행 위치 기준으로 둔다."""
    try:
        from calibration.repo_paths import resolve_path
    except ImportError:
        return os.path.abspath(os.path.expanduser(value)) if value else value
    return resolve_path(value)


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


def _scenario_params(scenario):
    """mock 세계와 같은 기하를 실제 tracker·planner에 넘길 파라미터(tracker, planner).

    시나리오 world.frames → robot_base ← conveyor_frame(row-major 16개),
    world.zones(conveyor_frame x) → 관측 구간·도달 구간 시작·끝. 검출이 mock 세계에서 올 때만 쓴다.
    """
    import yaml
    from drop_mocks.ros_common import scenario_file
    from drop_mocks.scenario_engine import scenario_geometry
    with open(scenario_file(scenario)) as fh:
        geo = scenario_geometry(yaml.safe_load(fh)['world'])
    T, zones = geo['T_base_conveyor'], geo['zones']
    tracker = {'T_base_conveyor': T, 'region.x_obs_min_m': zones['obs_start_m'],
               'region.x_obs_max_m': zones['obs_end_m'], 'select.x_reach_start_m': zones['reach_start_m']}
    planner = {'T_base_conveyor': T, 'reach.x_reach_start_m': zones['reach_start_m'],
               'budget.x_exit_m': zones['reach_end_m']}
    return tracker, planner


def _autostart(node):
    """실제 lifecycle 노드를 기동 때 한 번 configure → activate까지 올린다.

    configuring → inactive 전이에만, 한 번만 반응한다. 사람이 나중에 deactivate해서 inactive가 되어도
    다시 켜지 않는다(PRECHECK의 비활성 거부가 실제 노드에서도 성립해야 한다, TC-INT-006).
    """
    def change(transition):
        return EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node), transition_id=transition))
    return [RegisterEventHandler(OnStateTransition(target_lifecycle_node=node, start_state='configuring',
                                                   goal_state='inactive', handle_once=True,
                                                   entities=[change(Transition.TRANSITION_ACTIVATE)])),
            node, change(Transition.TRANSITION_CONFIGURE)]


def _nodes(context):
    mock = {r: _flag(context, f'use_mock_{r}') for r in ROLES}
    if mock['planner'] and not mock['tracking']:
        # 임시 plan은 mock_world_node의 tracking 역할이 낸다. 이 조합은 계획을 내는 노드가 없다.
        raise RuntimeError('use_mock_planner:=true는 use_mock_tracking:=true가 필요하다. '
                           '실제 tracker에는 use_mock_planner:=false(실제 planner)를 쓴다')
    scenario = LaunchConfiguration('scenario').perform(context)
    seed = int(LaunchConfiguration('seed').perform(context))
    sim_time = _flag(context, 'use_sim_time')
    t0_offset = float(LaunchConfiguration('t0_offset_s').perform(context))
    dur_scale = float(LaunchConfiguration('duration_scale_ratio').perform(context))
    drop_role = LaunchConfiguration('drop_role').perform(context)
    drop_at = float(LaunchConfiguration('drop_at_s').perform(context))
    publish_plan = _flag(context, 'publish_plan') and mock['planner']    # 실제 planner가 있으면 mock plan 끔
    # 역할별 프로세스가 같은 세계를 보도록 시나리오 시각 0을 하나로 정한다(sim time이면 노드 기동 시각)
    t0 = (0.0 if sim_time else time.time()) + t0_offset
    common = {'scenario': scenario, 'seed': seed, 't0_s': t0, 'use_sim_time': sim_time,
              'duration_scale_ratio': dur_scale, 'drop_role': drop_role, 'drop_at_s': drop_at}
    # 검출이나 track이 mock 세계에서 오면 실제 tracker·planner도 같은 컨베이어 위치·구간을 써야 한다.
    # 둘 다 실제면 각 패키지 config(보정값·배치)를 그대로 쓴다.
    tracker_geo, planner_geo = _scenario_params(scenario) if mock['vision'] or mock['tracking'] else ({}, {})
    out = []
    world_roles = [r for r in ('vision', 'tracking') if mock[r]]
    if world_roles:
        role = 'all' if len(world_roles) == 2 else world_roles[0]
        out.append(Node(package='drop_mocks', executable='mock_world_node', name='mock_world_node',
                        output='screen', parameters=[{**common, 'role': role, 'publish_plan': publish_plan}]))
    if mock['robot']:
        out.append(Node(package='drop_mocks', executable='mock_robot_controller_node',
                        name='mock_robot_controller_node', output='screen', parameters=[common]))
    for r in ROLES:
        if r == 'vision' and not mock[r]:
            # 기본 vision.yaml 위에 현장 파라미터 파일(보정·평면 경로, 실측 높이)을 덮어쓴다
            params = [os.path.join(get_package_share_directory('vision'), 'config', 'vision.yaml')]
            site = LaunchConfiguration('vision_params_file').perform(context)
            if site:
                params.append(_repo_path(site))
            out.append(Node(package='vision', executable='vision_node', name='vision_node',
                            output='screen', parameters=[*params, {'use_sim_time': sim_time}]))
        elif r == 'tracking' and not mock[r]:
            out.extend(_autostart(LifecycleNode(package='tracking', executable='tracker_node', name='tracker_node',
                                                namespace='', output='screen',
                                                parameters=[{'use_sim_time': sim_time, **tracker_geo}])))
        elif r == 'planner' and not mock[r]:
            out.extend(_autostart(LifecycleNode(package='intercept_planner', executable='intercept_planner_node',
                                                name='intercept_planner_node', namespace='', output='screen',
                                                parameters=[{'use_sim_time': sim_time, **planner_geo}])))
        elif not mock[r]:
            out.append(LogInfo(msg=f'{r}: 실제 노드({REAL_PENDING[r]})는 아직 없어 띄우지 않는다'))
    if _flag(context, 'launch_orchestrator'):
        rec_params = {'use_sim_time': sim_time, 'scenario': scenario, 'seed': max(0, seed)}
        records_dir = LaunchConfiguration('records_dir').perform(context)
        date_val = LaunchConfiguration('date').perform(context)
        if records_dir:
            rec_params['records_dir'] = records_dir
        if date_val:
            rec_params['date'] = date_val
        out.extend([
            Node(package='orchestrator', executable='orchestrator_node', name='orchestrator_node',
                 output='screen', parameters=[{'use_sim_time': sim_time}]),
            Node(package='orchestrator', executable='safety_supervisor_node', name='safety_supervisor_node',
                 output='screen', parameters=[{'use_sim_time': sim_time}]),
            Node(package='orchestrator', executable='record_logger_node', name='record_logger_node',
                 output='screen', parameters=[rec_params]),
        ])
    return out + [_shutdown_on_failure(n) for n in out if isinstance(n, Node)]


def generate_launch_description():
    args = [DeclareLaunchArgument(f'use_mock_{r}', default_value='false', choices=['true', 'false'],
                                  description=(f'true면 {r} 대역을 띄운다' if r != 'planner' else
                                               'true면 mock_world_node의 임시 plan(publish_plan), false면 실제 planner'))
            for r in ROLES]
    return LaunchDescription(args + [
        DeclareLaunchArgument('scenario', default_value='nominal_20mms', description='drop_mocks 시나리오'),
        DeclareLaunchArgument('seed', default_value='-1', description='-1이면 시나리오 파일의 seed'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_orchestrator', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('records_dir', default_value=''),
        DeclareLaunchArgument('vision_params_file', default_value='',
                              description='실제 vision_node의 현장 파라미터 파일. 상대 경로는 저장소 루트 기준'),
        DeclareLaunchArgument('date', default_value=''),
        DeclareLaunchArgument('duration_scale_ratio', default_value='-1.0'),
        DeclareLaunchArgument('t0_offset_s', default_value='0.0'),
        DeclareLaunchArgument('drop_role', default_value=''),
        DeclareLaunchArgument('drop_at_s', default_value='-1.0'),
        DeclareLaunchArgument('publish_plan', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_nodes),
    ])
