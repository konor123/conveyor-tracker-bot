"""전체 대역 bringup(PLAN.md 7.5절 I0): drop.launch.py의 vision·tracking·robot을 모두 대역으로."""
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
        IncludeLaunchDescription(PythonLaunchDescriptionSource(drop), launch_arguments={
            'use_mock_vision': 'true', 'use_mock_tracking': 'true', 'use_mock_robot': 'true',
            'scenario': LaunchConfiguration('scenario'), 'seed': LaunchConfiguration('seed'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()),
    ])
