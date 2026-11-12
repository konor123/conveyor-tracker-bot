"""전체 대역 bringup(PLAN.md 7.5절 I0): drop.launch.py의 vision·tracking·planner·robot을 모두 대역으로."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drop = os.path.join(get_package_share_directory('drop_bringup'), 'launch', 'drop.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='nominal_20mms'),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_orchestrator', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('records_dir', default_value=''),
        DeclareLaunchArgument('date', default_value=''),
        DeclareLaunchArgument('duration_scale_ratio', default_value='-1.0'),
        DeclareLaunchArgument('t0_offset_s', default_value='0.0'),
        DeclareLaunchArgument('drop_role', default_value=''),
        DeclareLaunchArgument('drop_at_s', default_value='-1.0'),
        DeclareLaunchArgument('publish_plan', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(drop), launch_arguments={
            'use_mock_vision': 'true', 'use_mock_tracking': 'true', 'use_mock_robot': 'true',
            'use_mock_planner': 'true',
            'scenario': LaunchConfiguration('scenario'), 'seed': LaunchConfiguration('seed'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'launch_orchestrator': LaunchConfiguration('launch_orchestrator'),
            'records_dir': LaunchConfiguration('records_dir'),
            'date': LaunchConfiguration('date'),
            'duration_scale_ratio': LaunchConfiguration('duration_scale_ratio'),
            't0_offset_s': LaunchConfiguration('t0_offset_s'),
            'drop_role': LaunchConfiguration('drop_role'),
            'drop_at_s': LaunchConfiguration('drop_at_s'),
            'publish_plan': LaunchConfiguration('publish_plan'),
        }.items()),
    ])
