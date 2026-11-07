from glob import glob

from setuptools import find_packages, setup

package_name = 'calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/calibration/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='konor123',
    maintainer_email='konor123@users.noreply.github.com',
    description='카메라·평면·TCP 보정 도구',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'charuco_view = calibration.cli:charuco_view',
        'calibrate_camera = calibration.cli:calibrate_camera',
        'camera_tf_publisher = calibration.ros_nodes:camera_tf_publisher',
        'check_camera_info = calibration.ros_nodes:check_camera_info',
    ]},
)
