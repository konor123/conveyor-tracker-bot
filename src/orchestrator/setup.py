from glob import glob

from setuptools import find_packages, setup

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/orchestrator/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='konor123',
    maintainer_email='konor123@users.noreply.github.com',
    description='상태 머신·단계 판정·중단·감시·기록',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'orchestrator_node = orchestrator.orchestrator_node:main',
        'safety_supervisor_node = orchestrator.safety_supervisor_node:main',
        'record_logger_node = orchestrator.record_logger_node:main',
    ]},
)
