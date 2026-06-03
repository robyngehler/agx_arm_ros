#!/usr/bin/env bash

set -euo pipefail

echo ""
echo "[info] Installing system and ROS dependencies only."
echo "[info] This script intentionally avoids pip so colcon builds stay on system Python."
echo "[info] For the optional conda runtime/dev environment, use scripts/setup_agx_arm_runtime_env.sh after this script."

echo -e "\n[1/3] Installing base Python and CAN tools..."
sudo apt update
sudo apt install -y \
    can-utils \
    ethtool \
    python3-can \
    python3-numpy \
    python3-pytest \
    python3-scipy \
    python3-yaml
echo "  ✓ Base Python and CAN tooling installation completed."

echo -e "\n[2/3] Installing ROS2 control-related dependencies..."
if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "  ✗ Environment variable ROS_DISTRO is not set."
    exit 1
fi

sudo apt install -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro \
    python3-colcon-common-extensions

echo "  ✓ ROS2 control dependencies installation completed."

echo -e "\n[3/3] Installing MoveIt2 and related dependencies..."

sudo apt install -y ros-$ROS_DISTRO-moveit*
moveit_extra_packages=(
    "ros-$ROS_DISTRO-control*"
    "ros-$ROS_DISTRO-joint-trajectory-controller"
    "ros-$ROS_DISTRO-joint-state-*"
    "ros-$ROS_DISTRO-gripper-controllers"
    "ros-$ROS_DISTRO-trajectory-msgs"
)

trac_ik_pkg="ros-$ROS_DISTRO-trac-ik-kinematics-plugin"
if apt-cache show "$trac_ik_pkg" >/dev/null 2>&1; then
    moveit_extra_packages+=("$trac_ik_pkg")
else
    echo "  ! $trac_ik_pkg not found in apt metadata."
    echo "    On ROS 2 Humble / Jetson, use the source-build fallback documented in"
    echo "    docs/development/sprint3/planning/trac_ik_humble_jetson_repro.md"
fi

sudo apt-get install -y "${moveit_extra_packages[@]}"

if [[ "$(locale | grep LC_NUMERIC)" != *"en_US.UTF-8"* ]]; then
    echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
    echo "  ✓ Added LC_NUMERIC=en_US.UTF-8 to ~/.bashrc. Open a new shell or source ~/.bashrc before running MoveIt."
else
    echo "  ✓ LC_NUMERIC locale is already set to en_US.UTF-8."
fi

echo ""
echo "[next] Build with scripts/colcon_build_system_python.sh"
echo "[next] Optional runtime/dev env: scripts/setup_agx_arm_runtime_env.sh"
echo "[next] Run ROS commands inside conda with scripts/run_in_ros_conda.sh -- <command>"