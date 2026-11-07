from glob import glob

from setuptools import find_packages, setup

package_name = 'drop_mocks'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/drop_mocks/scenarios', glob('scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='konor123',
    maintainer_email='konor123@users.noreply.github.com',
    description='시나리오 엔진·mock 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mock_world_node = drop_mocks.mock_world_node:main',
        'mock_robot_controller_node = drop_mocks.mock_robot_controller_node:main',
    ]},
)
