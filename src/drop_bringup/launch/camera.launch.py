"""D455를 띄우고, 보정 파일이 있으면 base_link → camera_link static TF를 발행한다.

1280x800 RGB(약 3 MB)를 best effort로 받으려면 같은 PC의 모든 참여 노드가 FastDDS SHM 세그먼트를 키운
프로파일을 써야 한다(config/fastdds_large_shm.xml). 이 launch 안의 노드에는 자동으로 적용되고,
따로 띄우는 노드·ros2 CLI는 셸에서 FASTRTPS_DEFAULT_PROFILES_FILE을 같은 파일로 export한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('drop_bringup')
FASTDDS_PROFILE = os.path.join(SHARE, 'config', 'fastdds_large_shm.xml')


def include_camera(context):
    rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            'rgb_camera.color_profile': '1280,800,30',
            'enable_color': 'true',
            'enable_depth': LaunchConfiguration('enable_depth'),
            'depth_module.depth_profile': '1280,720,30',
            'align_depth.enable': 'false',
            'pointcloud.enable': 'false',
            'rgb_camera.enable_auto_exposure': LaunchConfiguration('auto_exposure'),
        }.items(),
    )]
    extrinsics = LaunchConfiguration('extrinsics_file').perform(context)
    if extrinsics:
        actions.append(Node(package='calibration', executable='camera_tf_publisher', name='camera_tf_publisher',
                            parameters=[{'extrinsics_file': extrinsics}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_depth', default_value='true', description='평면 확인·이상 감시용 depth'),
        DeclareLaunchArgument('auto_exposure', default_value='true',
                              description='OFF-1 조명 확정 후 false + 노출 고정(Progress.md 4절)'),
        DeclareLaunchArgument('extrinsics_file', default_value='',
                              description='camera_extrinsics.yaml. 비우면 static TF를 발행하지 않는다'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', FASTDDS_PROFILE),
        OpaqueFunction(function=include_camera),
    ])
