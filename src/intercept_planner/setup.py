from glob import glob

from setuptools import find_packages, setup

package_name = 'intercept_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/intercept_planner/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='konor123',
    maintainer_email='konor123@users.noreply.github.com',
    description='예측·시간 예산·해제 계획',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['intercept_planner_node = intercept_planner.intercept_planner_node:main']},
)
