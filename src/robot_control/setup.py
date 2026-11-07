from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/robot_control/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='konor123',
    maintainer_email='konor123@users.noreply.github.com',
    description='Doosan·그리퍼 어댑터, skill 서버, 추종 제어',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'robot_controller_node = robot_control.robot_controller_node:main',
    ]},
)
