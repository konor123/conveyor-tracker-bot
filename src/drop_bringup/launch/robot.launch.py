"""E0509 드라이버를 띄운다. 기본은 virtual이고, 실기는 mode:=real로 명시한다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

REAL_HOST = '110.120.1.38'
VIRTUAL_HOST = '127.0.0.1'


def include_driver(context):
    mode = LaunchConfiguration('mode').perform(context)
    host = LaunchConfiguration('host').perform(context)
    if not host:
        host = REAL_HOST if mode == 'real' else VIRTUAL_HOST

    driver_launch = os.path.join(
        get_package_share_directory('dsr_bringup2'), 'launch', 'dsr_bringup2_rviz.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'mode': mode,
            'host': host,
            'port': LaunchConfiguration('port'),
            'model': LaunchConfiguration('model'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='dsr01', description='로봇 namespace'),
        DeclareLaunchArgument(
            'mode', default_value='virtual', choices=['real', 'virtual'],
            description='real은 실제 로봇을 연결한다'),
        DeclareLaunchArgument(
            'host', default_value='',
            description=f'비우면 real은 {REAL_HOST}, virtual은 {VIRTUAL_HOST}'),
        DeclareLaunchArgument('port', default_value='12345', description='제어기 포트'),
        DeclareLaunchArgument('model', default_value='e0509', description='로봇 모델'),
        DeclareLaunchArgument('gui', default_value='true', description='RViz2 실행'),
        OpaqueFunction(function=include_driver),
    ])
