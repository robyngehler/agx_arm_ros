from setuptools import find_packages, setup
import os


package_name = "agx_arm_mit_tools"


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
    description="Debug, bridge, validation, and calibration tools for the AgileX Nero MIT controller stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agx_arm_mit_joint_state_bridge = agx_arm_mit_tools.joint_state_trajectory_bridge:main",
            "agx_arm_joint_state_name_adapter = agx_arm_mit_tools.joint_state_name_adapter:main",
            "agx_arm_test_position_hold = agx_arm_mit_tools.test_position_hold:main",
            "agx_arm_validate_urdf_mdh = agx_arm_mit_tools.validate_urdf_mdh:main",
            "agx_arm_compare_gravity = agx_arm_mit_tools.compare_gravity:main",
            "agx_arm_fit_gravity_calibration = agx_arm_mit_tools.fit_gravity_calibration:main",
        ],
    },
)