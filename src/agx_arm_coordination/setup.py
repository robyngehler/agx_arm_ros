import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'agx_arm_coordination'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'activities'),
            glob('config/activities/*.yaml')),
        (os.path.join('share', package_name, 'config', 'catalogue.d'),
            glob('config/catalogue.d/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Activity-DAG coordinator and performer for the Duo Nero arm+hand system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinator = agx_arm_coordination.coordinator_node:main',
            'run_activity = agx_arm_coordination.run_activity_client:main',
        ],
    },
)
