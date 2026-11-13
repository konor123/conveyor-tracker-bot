"""시스템 bringup(PLAN.md 2.4, 7.2절). 역할마다 실제 노드와 대역 중 하나만 띄운다(use_mock_<역할>).

대역은 기동할 때 같은 역할의 서버가 이미 있으면 스스로 종료하고(REQ-MCK-007), 이 launch는 그때 전체를 내린다.
vision·tracking·planner·robot은 실제 노드 또는 대역을 선택한다.
실제 tracker·planner·robot_controller는 launch가 configure → activate까지 자동으로 전이한다(SPEC-400 REQ-PLN-012).
use_mock_robot:=false면 실제 robot_controller_node를 robot_backend(기본 disabled = 실기 연결 없음, 시험은 fake)로 띄운다.
backend:=fake이면 config/robot_fake.yaml(시뮬레이션 가정값) 위에, 검출·track이 mock 세계에서 오면 시나리오 기하
(대기 자세·매트 반납 위치·시작 TCP)를 얹고, 마지막에 robot_params_file을 덮는다(값이 null인 키는 미설정).
실제 controller와 mock vision을 함께 띄우면 mock 세계의 VerifyDrop은 대본 대신 UNKNOWN을 낸다(verify_scripted:=false):
mock 세계는 실제 controller가 떨어뜨린 공깃돌을 모른다(SPEC-700 4.4절).
use_mock_planner:=false(기본)면 mock_world_node의 임시 plan(publish_plan)을 끈다.
use_mock_planner:=true는 임시 plan을 내는 tracking 대역이 있어야 하므로 use_mock_tracking:=true가 아니면 기동을 거부한다.
도달 구간(시작·끝)은 tracker·planner·orchestrator에 같은 값을 넣는다: 검출이나 track이 mock 세계에서 오면 시나리오
world.zones, 아니면 config/system.yaml의 layout(SPEC-400 REQ-PLN-012).
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
CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, 'config')
SYSTEM_YAML = os.path.join(CONFIG_DIR, 'system.yaml')
ROBOT_FAKE_YAML = os.path.join(CONFIG_DIR, 'robot_fake.yaml')
ROBOT_NODE = 'robot_controller_node'


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


def _reach_params(x_reach_start, x_exit):
    """도달 구간 시작·끝을 세 노드의 파라미터 이름으로(tracker, planner, orchestrator)."""
    x_reach_start, x_exit = float(x_reach_start), float(x_exit)
    return ({'select.x_reach_start_m': x_reach_start},
            {'reach.x_reach_start_m': x_reach_start, 'budget.x_exit_m': x_exit},
            {'budget.x_reach_start_m': x_reach_start, 'budget.x_exit_m': x_exit})


def _scenario_params(scenario):
    """mock 세계와 같은 기하를 실제 tracker·planner와 orchestrator에 넘길 파라미터(tracker, planner, orchestrator).

    시나리오 world.frames → robot_base ← conveyor_frame(row-major 16개),
    world.zones(conveyor_frame x) → 관측 구간·도달 구간 시작·끝. 검출이 mock 세계에서 올 때만 쓴다.
    """
    import yaml
    from drop_mocks.ros_common import scenario_file
    from drop_mocks.scenario_engine import scenario_geometry
    with open(scenario_file(scenario)) as fh:
        geo = scenario_geometry(yaml.safe_load(fh)['world'])
    T, zones = geo['T_base_conveyor'], geo['zones']
    tracker, planner, orchestrator = _reach_params(zones['reach_start_m'], zones['reach_end_m'])
    tracker.update({'T_base_conveyor': T, 'region.x_obs_min_m': zones['obs_start_m'],
                    'region.x_obs_max_m': zones['obs_end_m']})
    planner['T_base_conveyor'] = T
    return tracker, planner, orchestrator


def _layout_params(path=SYSTEM_YAML):
    """실제 배치(config/system.yaml layout)의 도달 구간(tracker, planner, orchestrator). 보정·관측 구간은 각 config."""
    import yaml
    with open(path, encoding='utf-8') as fh:
        layout = yaml.safe_load(fh)['layout']
    return _reach_params(layout['x_reach_start_m'], layout['x_exit_m'])


def _geometry_params(scenario, mock_world):
    """세 노드가 같은 도달 구간을 쓰도록 넘길 파라미터. 검출·track이 mock 세계에서 오면 시나리오, 아니면 실제 배치."""
    return _scenario_params(scenario) if mock_world else _layout_params()


def _ros_params(path, node=ROBOT_NODE):
    """ROS 파라미터 파일(`<node>: ros__parameters:` 또는 `/**:` 아래, 중첩은 점 이름)을 평탄한 dict로."""
    import yaml
    with open(path, encoding='utf-8') as fh:
        doc = yaml.safe_load(fh) or {}
    params = {}
    for key in ('/**', node, f'/{node}'):
        params.update((doc.get(key) or {}).get('ros__parameters') or {})

    def flat(prefix, value, out):
        if isinstance(value, dict):
            for k, v in value.items():
                flat(f'{prefix}.{k}' if prefix else str(k), v, out)
        else:
            out[prefix] = value
        return out
    return flat('', params, {})


def _robot_geometry(scenario, safe_z_m):
    """mock 세계와 같은 배치의 controller 기하(robot_base [m]).

    대기 자세 = 도달 구간 시작(conveyor_frame x = reach_start, y = 0) 위 안전 높이, 자세는 시뮬레이션 backend가
    보고하는 항등 자세. 매트 반납 위치 = 시나리오 첫 공깃돌 위치(mock 세계가 DetectObjects로 내는 값, PickStone 목표와
    같음). 시작 TCP = 대기 자세.
    """
    import math
    import yaml
    from drop_mocks.ros_common import scenario_file
    with open(scenario_file(scenario)) as fh:
        world = yaml.safe_load(fh)['world']
    fr = world['frames']

    def to_base(xyz, yaw, x, y, z):
        c, s = math.cos(float(yaw)), math.sin(float(yaw))
        return [float(xyz[0]) + c * x - s * y, float(xyz[1]) + s * x + c * y, float(xyz[2]) + z]
    standby = to_base(fr['conveyor_xyz_m'], fr.get('conveyor_yaw_rad', 0.0), float(world['zones']['reach_start_m']),
                      0.0, 0.0)
    standby[2] = float(safe_z_m)
    stone = (world.get('stones') or [{'x_m': 0.0, 'y_m': 0.0}])[0]
    place_back = to_base(fr['mat_xyz_m'], fr.get('mat_yaw_rad', 0.0), float(stone['x_m']), float(stone['y_m']),
                         float(world['objects']['stone_height_m']))
    return {'standby_pose': standby + [0.0, 0.0, 0.0, 1.0], 'place_back_xyz_m': place_back,
            'fake_tcp_x_m': standby[0], 'fake_tcp_y_m': standby[1], 'fake_tcp_z_m': standby[2]}


def _robot_params(backend, scenario, mock_world, sim_time, params_file=''):
    """실제 robot_controller_node 파라미터 하나의 dict. backend는 launch 인자만 정한다(파일로 바꾸지 않는다).

    기하는 덮어쓴 뒤의 안전 높이로 만들고, 파일이 기하 키를 직접 주면 그 값이 이긴다.
    """
    params = {}
    user = _ros_params(_repo_path(params_file)) if params_file else {}
    if backend == 'fake':
        params.update(_ros_params(ROBOT_FAKE_YAML))
        if mock_world:
            safe_z = user['safe_z_m'] if user.get('safe_z_m') is not None else params['safe_z_m']
            params.update(_robot_geometry(scenario, safe_z))
    params.update(user)
    params = {k: v for k, v in params.items() if v is not None}      # null = 미설정(노드 기본값)
    params.update({'backend': backend, 'use_sim_time': sim_time})
    return params


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
    # 둘 다 실제면 보정값·관측 구간은 각 패키지 config, 도달 구간은 system.yaml layout이다.
    mock_world = mock['vision'] or mock['tracking']
    tracker_geo, planner_geo, orch_geo = _geometry_params(scenario, mock_world)
    out = []
    world_roles = [r for r in ('vision', 'tracking') if mock[r]]
    if world_roles:
        role = 'all' if len(world_roles) == 2 else world_roles[0]
        # 실제 controller가 놓은 공깃돌은 mock 세계가 모르므로 VerifyDrop 대본을 쓰지 않는다(UNKNOWN)
        out.append(Node(package='drop_mocks', executable='mock_world_node', name='mock_world_node',
                        output='screen', parameters=[{**common, 'role': role, 'publish_plan': publish_plan,
                                                      'verify_scripted': mock['robot']}]))
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
        elif r == 'robot' and not mock[r]:
            params = _robot_params(LaunchConfiguration('robot_backend').perform(context), scenario, mock_world,
                                   sim_time, LaunchConfiguration('robot_params_file').perform(context))
            out.extend(_autostart(LifecycleNode(package='robot_control', executable=ROBOT_NODE, name=ROBOT_NODE,
                                                namespace='', output='screen', parameters=[params])))
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
                 output='screen', parameters=[{'use_sim_time': sim_time, **orch_geo}]),
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
        DeclareLaunchArgument('robot_backend', default_value='disabled', choices=['disabled', 'fake'],
                              description='실제 robot_controller_node backend. disabled는 실기 연결 없음, fake는 시뮬레이션'),
        DeclareLaunchArgument('robot_params_file', default_value='',
                              description='실제 robot_controller_node에 마지막으로 덮을 파라미터 파일(null 값 = 미설정). '
                                          '상대 경로는 저장소 루트 기준'),
        OpaqueFunction(function=_nodes),
    ])
