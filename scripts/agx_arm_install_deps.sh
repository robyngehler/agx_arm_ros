#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
system_python="${AGX_ARM_SYSTEM_PYTHON:-/usr/bin/python3.10}"

echo ""
echo "[info] Installing the apt dependency layer plus the pinned pip layer from requirements.txt."
echo "[info] The pip layer targets the SYSTEM interpreter's user site (${system_python}), never conda,"
echo "[info] so colcon builds and the ROS runtime stay on system Python."
echo "[info] Conda is NOT part of the validated runtime path; see docs/control/environment.md."

echo -e "\n[1/4] Installing base Python and CAN tools..."
sudo apt update
# NOTE: python3-can is deliberately NOT installed from apt. Ubuntu 22.04 ships
# python-can 3.3.2, but the arm's CAN error-recovery path uses
# can.CanOperationError / can.CanInitializationError (python-can >= 4.0). A 3.x
# install builds fine and fails at runtime. The pinned version comes from
# requirements.txt in step [4/4].
sudo apt install -y \
    can-utils \
    ethtool \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-pytest \
    python3-scipy \
    python3-vcstool \
    python3-yaml
echo "  ✓ Base Python and CAN tooling installation completed."

echo -e "\n[2/4] Installing ROS2 control-related dependencies..."
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
    ros-$ROS_DISTRO-pinocchio \
    ros-$ROS_DISTRO-eigenpy \
    python3-colcon-common-extensions

# pinocchio/eigenpy back the MIT controller's gravity model
# (agx_arm_mit_controller/gravity_model.py). They must come from apt so they
# match the ROS 2 Humble ABI and import under system python3.10 — do not
# substitute a pip or conda pinocchio.
echo "  ✓ ROS2 control dependencies installation completed."

echo -e "\n[3/4] Installing MoveIt2 and related dependencies..."

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
    echo "  ! $trac_ik_pkg not found in apt metadata (expected on Humble / aarch64)."
    echo "    Build the pinned source overlay instead:"
    echo "      bash ./scripts/setup_trac_ik_overlay.sh"
fi

sudo apt-get install -y "${moveit_extra_packages[@]}"

if [[ "$(locale | grep LC_NUMERIC)" != *"en_US.UTF-8"* ]]; then
    echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
    echo "  ✓ Added LC_NUMERIC=en_US.UTF-8 to ~/.bashrc. Open a new shell or source ~/.bashrc before running MoveIt."
else
    echo "  ✓ LC_NUMERIC locale is already set to en_US.UTF-8."
fi

echo -e "\n[4/4] Installing the pinned pip layer from requirements.txt..."

if [[ ! -x "${system_python}" ]]; then
    echo "  ✗ System interpreter not found: ${system_python}"
    echo "    The ROS runtime is system python3.10. Set AGX_ARM_SYSTEM_PYTHON if it lives elsewhere."
    exit 1
fi

# --user plus the explicit interpreter keep this out of conda. PYTHONNOUSERSITE
# must be fully UNSET, not empty: CPython treats the mere presence of that
# variable as "disable the user site", which is exactly the install target here.
unset PYTHONNOUSERSITE
"${system_python}" -m pip install --user --upgrade -r "${repo_root}/requirements.txt"

# Fail loudly on the one version constraint that only breaks at runtime.
can_version="$("${system_python}" -c 'import can; print(can.__version__)' 2>/dev/null || echo "missing")"
can_major="${can_version%%.*}"
if [[ "${can_version}" == "missing" ]]; then
    echo "  ✗ python-can is not importable by ${system_python}."
    exit 1
elif ! [[ "${can_major}" =~ ^[0-9]+$ ]]; then
    echo "  ! could not parse python-can version '${can_version}'; verify >= 4.0 manually."
elif [[ "${can_major}" -lt 4 ]]; then
    echo "  ✗ python-can ${can_version} is too old; the arm's CAN error-recovery path needs >= 4.0"
    echo "    (can.CanOperationError / can.CanInitializationError)."
    echo "    An apt python3-can (3.3.2) is probably shadowing the pip install:"
    echo "      sudo apt remove python3-can"
    exit 1
else
    echo "  ✓ python-can ${can_version} (>= 4.0 as required)."
fi

echo "  ✓ pip layer installation completed."

echo ""
echo "[next] TRAC-IK overlay (required by agx_arm_moveit): bash ./scripts/setup_trac_ik_overlay.sh"
echo "[next] OmniHand vendor SDK (required by the sdk bridge backend): bash ./scripts/setup_omnihand_sdk.sh"
echo "[next] Build with scripts/colcon_build_system_python.sh"
echo "[next] Full provisioning walkthrough: docs/project/jetson_migration.md"