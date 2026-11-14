from glob import glob

from setuptools import find_packages, setup

package_name = 'static_pick_place'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/static_pick_place/config', glob('config/*.yaml')),
    ],
    package_data={'static_pick_place': ['drl/*.drl']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='konor123',
    maintainer_email='konor123@users.noreply.github.com',
    description='정지 벨트 공깃돌 pick-and-place',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'pick_place = static_pick_place.pick_place:main',
    ]},
)
