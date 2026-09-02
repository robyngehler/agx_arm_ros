from setuptools import find_packages, setup
import glob
import sys
import os
from glob import glob

package_name = 'agx_arm_ctrl'

python_version = f'{sys.version_info.major}.{sys.version_info.minor}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='AgileX Robotic Arm ROS Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agx_arm_ctrl_single = agx_arm_ctrl.agx_arm_ctrl_single_node:main',
            'unit_safety = agx_arm_ctrl.unit_safety_node:main',
            'omnihand_bridge = agx_arm_ctrl.omnihand_bridge_node:main',
            'omnihand_skill_controller = agx_arm_ctrl.omnihand_skill_controller_node:main',
            'omnihand_exerciser = agx_arm_ctrl.omnihand_exerciser_node:main',
            'omnihand_follow_joint_trajectory = agx_arm_ctrl.omnihand_follow_joint_trajectory:main',
            'gripper_follow_joint_trajectory = agx_arm_ctrl.gripper_follow_joint_trajectory:main',
        ],
    },
)
