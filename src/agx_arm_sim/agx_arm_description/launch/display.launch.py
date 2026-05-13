"""
agx_arm_description — display.launch.py
========================================
通过唯一入口文件 urdf/agx_arm_description.urdf.xacro 加载机械臂模型。
launch 参数直接作为 xacro mappings 传入，由 xacro 解析后注入 robot_description。

使用示例:
    # 默认 nero，无支架无相机
  ros2 launch agx_arm_description display.launch.py

    # nero 带夹爪
    ros2 launch agx_arm_description display.launch.py arm_type:=nero end_effector:=gripper

  # 带相机支架（无相机）
  ros2 launch agx_arm_description display.launch.py with_camera_stand:=true

  # 带支架 + RealSense D435
  ros2 launch agx_arm_description display.launch.py with_camera_stand:=true with_camera:=true

    # nero + 左手 revo2
    ros2 launch agx_arm_description display.launch.py arm_type:=nero end_effector:=revo2_left

    # nero + 左手 OmniHand
    ros2 launch agx_arm_description display.launch.py arm_type:=nero end_effector:=omnihand_left
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    arm_type          = LaunchConfiguration("arm_type").perform(context)
    end_effector      = LaunchConfiguration("end_effector").perform(context)
    revo2_side        = LaunchConfiguration("revo2_side").perform(context)
    with_camera_stand = LaunchConfiguration("with_camera_stand").perform(context)
    with_camera       = LaunchConfiguration("with_camera").perform(context)
    use_gui           = LaunchConfiguration("use_gui").perform(context)
    rviz_config       = LaunchConfiguration("rviz_config").perform(context)

    pkg_share  = get_package_share_directory("agx_arm_description")
    xacro_file = os.path.join(pkg_share, "urdf", "agx_arm_description.urdf.xacro")

    if not os.path.exists(xacro_file):
        raise FileNotFoundError(
            f"[agx_arm_description] 找不到入口 xacro 文件:\n  {xacro_file}"
        )

    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={
            "arm_type":          arm_type,
            "end_effector":      end_effector,
            "revo2_side":        revo2_side,
            "with_camera_stand": with_camera_stand,
            "with_camera":       with_camera,
        },
    ).toxml()

    if not rviz_config or not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_share, "rviz", "default.rviz")

    return [

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_content}],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(use_gui),
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ]


def generate_launch_description():
    pkg_share    = get_package_share_directory("agx_arm_description")
    default_rviz = os.path.join(pkg_share, "rviz", "default.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "arm_type",
            default_value="nero",
            choices=["nero"],
            description="机械臂型号（当前工作区仅保留 nero）",
        ),
        DeclareLaunchArgument(
            "end_effector",
            default_value="none",
            choices=["none", "gripper", "revo2_left", "revo2_right", "omnihand_left", "omnihand_right"],
            description="末端执行器类型",
        ),
        DeclareLaunchArgument(
            "revo2_side",
            default_value="right",
            choices=["left", "right"],
            description="保留参数；nero-only 资产树下通常不需要单独设置",
        ),
        DeclareLaunchArgument(
            "with_camera_stand",
            default_value="false",
            choices=["true", "false"],
            description="是否加载相机支架（固定挂在 gripper_base 上）",
        ),
        DeclareLaunchArgument(
            "with_camera",
            default_value="false",
            choices=["true", "false"],
            description="是否加载 RealSense D435 相机（须同时设 with_camera_stand:=true）",
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="是否启动 joint_state_publisher_gui 关节滑条",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="RViz2 配置文件路径（.rviz）",
        ),

        OpaqueFunction(function=launch_setup),
    ])