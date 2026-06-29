from setuptools import find_packages, setup
import os


package_name = "agx_arm_mit_demos"


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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Demo and workflow entry points for the AgileX Nero MIT controller stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agx_arm_record_leader_trajectory = agx_arm_mit_demos.leader_trajectory_recorder:main",
            "agx_arm_execute_saved_trajectory = agx_arm_mit_demos.execute_saved_trajectory:main",
            "agx_arm_capture_anchor_pose = agx_arm_mit_demos.capture_anchor_pose:main",
            "agx_arm_wakeword_motion_manager = agx_arm_mit_demos.wakeword_motion_manager:main",
        ],
    },
)