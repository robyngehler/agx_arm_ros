from glob import glob
import os

from setuptools import find_packages, setup


package_name = "agx_arm_mit_controller"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="ROS2 MIT trajectory controller for AgileX Nero arms.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agx_arm_mit_controller = agx_arm_mit_controller.mit_controller_node:main",
            "agx_arm_record_leader_trajectory = agx_arm_mit_controller.leader_trajectory_recorder:main",
            "agx_arm_execute_saved_trajectory = agx_arm_mit_controller.execute_saved_trajectory:main",
            "agx_arm_test_position_hold = agx_arm_mit_controller.test_position_hold:main",
            "agx_arm_validate_urdf_mdh = agx_arm_mit_controller.validate_urdf_mdh:main",
            "agx_arm_compare_gravity = agx_arm_mit_controller.compare_gravity:main",
            "agx_arm_fit_gravity_calibration = agx_arm_mit_controller.fit_gravity_calibration:main",
        ],
    },
)