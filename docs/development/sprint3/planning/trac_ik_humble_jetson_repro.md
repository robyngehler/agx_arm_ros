# Reproducible TRAC-IK Source Install on ROS 2 Humble / Ubuntu 22.04 / Jetson AGX Orin

Target platform:

- NVIDIA Jetson AGX Orin
- Ubuntu 22.04 Jammy
- Kernel 5.15 tegra
- Architecture: `aarch64` / `arm64`
- ROS 2 Humble
- MoveIt 2 Humble
- TRAC-IK source build, because no suitable `ros-humble-trac-ik` apt package was available

## Summary of what worked

The successful build path was:

1. Build TRAC-IK from source in a separate workspace.
2. Avoid Conda / Miniforge Python during the ROS build.
3. Install required system dependencies with apt and rosdep.
4. Patch the newer TRAC-IK MoveIt plugin headers from `.hpp` to Humble-compatible `.h`.
5. Build with `colcon` while explicitly pointing CMake to `/usr/bin/python3`.

The final build completed successfully:

```text
Summary: 3 packages finished
```

The built packages were:

```text
trac_ik
trac_ik_lib
trac_ik_kinematics_plugin
```

## Workspace-level validation in agx_arm_ros

After sourcing the overlay in the order below, the current `agx_arm_ros` workspace validated the expected TRAC-IK behavior on 2026-05-21:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/trac_ik_ws/install/setup.bash
source ~/workspace/agx_arm_ros/install/setup.bash
```

Verified outcomes:

- `ros2 pkg prefix trac_ik_kinematics_plugin` resolved to `~/workspace/trac_ik_ws/install/trac_ik_kinematics_plugin`
- `scripts/moveit_profile_smoke_test.sh` reached `You can start planning now!` for `none`, `agx_gripper`, `revo2` left/right, and `omnihand` left/right without TRAC-IK plugin-load errors
- a live `/compute_ik` request on `nero_arm` returned `MoveItErrorCodes.SUCCESS`

Remaining issue:

- the timeout-driven `move_group` shutdown path on this Humble / aarch64 host still ends in a SIGINT teardown crash, which appears separate from TRAC-IK provisioning

## Problems encountered

### 1. No apt package available

No usable Humble apt package was available via:

```bash
apt-cache search ros-humble-trac-ik
```

So we used a source build.

### 2. Conda / Miniforge hijacked Python

The first build failed with:

```text
ModuleNotFoundError: No module named 'catkin_pkg'
execute_process(/home/user/miniforge3/bin/python3 ...)
```

Root cause:

`ament_cmake_core` was using Miniforge Python instead of system Python.

Fix:

```bash
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'miniforge3' | paste -sd ':' -)
```

Then verify:

```bash
which python3
python3 -c "import sys; print(sys.executable)"
```

Expected:

```text
/usr/bin/python3
/usr/bin/python3
```

### 3. TRAC-IK plugin used newer MoveIt header names

The next build failed with:

```text
fatal error: moveit/kinematics_base/kinematics_base.hpp: No such file or directory
```

Then after fixing that:

```text
fatal error: moveit/robot_model/robot_model.hpp: No such file or directory
```

Root cause:

The TRAC-IK branch used newer MoveIt header names from newer ROS 2 distributions, while MoveIt 2 Humble provides these headers as `.h`.

Fix:

```diff
- #include <moveit/kinematics_base/kinematics_base.hpp>
- #include <moveit/robot_model/robot_model.hpp>
+ #include <moveit/kinematics_base/kinematics_base.h>
+ #include <moveit/robot_model/robot_model.h>
```

The relevant file was:

```text
trac_ik_kinematics_plugin/include/trac_ik/trac_ik_kinematics_plugin.hpp
```

## Reproducible install script

Save this as:

```bash
install_trac_ik_humble_jetson.sh
```

Then run:

```bash
chmod +x install_trac_ik_humble_jetson.sh
./install_trac_ik_humble_jetson.sh
```

Script:

```bash
#!/usr/bin/env bash
set -euo pipefail

# Reproducible TRAC-IK source build for:
# - Ubuntu 22.04
# - ROS 2 Humble
# - Jetson AGX Orin / aarch64
#
# This script intentionally avoids Conda/Miniforge Python and patches
# newer MoveIt .hpp includes back to Humble-compatible .h includes.

WS="${WS:-$HOME/workspace/trac_ik_ws}"
TRAC_IK_REPO="${TRAC_IK_REPO:-https://bitbucket.org/traclabs/trac_ik.git}"
TRAC_IK_BRANCH="${TRAC_IK_BRANCH:-jazzy}"
ROS_SETUP="/opt/ros/humble/setup.bash"

echo "== TRAC-IK Humble source build =="
echo "Workspace: ${WS}"
echo "Repo:      ${TRAC_IK_REPO}"
echo "Branch:    ${TRAC_IK_BRANCH}"
echo

if [ ! -f "${ROS_SETUP}" ]; then
  echo "ERROR: ROS 2 Humble setup file not found at ${ROS_SETUP}"
  echo "Install/source ROS 2 Humble first."
  exit 1
fi

echo "== Step 1: deactivate Conda/Miniforge if present =="
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true

# Remove miniforge paths from current PATH.
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v 'miniforge3' | paste -sd ':' -)"

echo "Python executable:"
which python3
python3 -c "import sys; print(sys.executable)"

if python3 -c "import sys; raise SystemExit(0 if sys.executable == '/usr/bin/python3' else 1)"; then
  echo "OK: using /usr/bin/python3"
else
  echo "ERROR: Python is not /usr/bin/python3. Current Python:"
  python3 -c "import sys; print(sys.executable)"
  echo "Open a clean shell or run:"
  echo "  bash --noprofile --norc"
  echo "  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  exit 1
fi

echo
echo "== Step 2: install system dependencies =="
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-catkin-pkg \
  python3-empy \
  python3-lark \
  python3-numpy \
  python3-yaml \
  python3-setuptools \
  libnlopt-dev \
  libnlopt-cxx-dev \
  libeigen3-dev \
  liborocos-kdl-dev \
  ros-humble-kdl-parser \
  ros-humble-pluginlib \
  ros-humble-moveit-core \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-planning-interface

echo
echo "== Step 3: initialize rosdep if needed =="
sudo rosdep init 2>/dev/null || true
rosdep update

echo
echo "== Step 4: create workspace and clone TRAC-IK =="
mkdir -p "${WS}/src"
cd "${WS}/src"

if [ ! -d "trac_ik/.git" ]; then
  git clone "${TRAC_IK_REPO}" trac_ik
else
  echo "TRAC-IK repository already exists, reusing it."
fi

cd trac_ik
git fetch --all --tags

if git rev-parse --verify "${TRAC_IK_BRANCH}" >/dev/null 2>&1; then
  git checkout "${TRAC_IK_BRANCH}"
elif git rev-parse --verify "origin/${TRAC_IK_BRANCH}" >/dev/null 2>&1; then
  git checkout -B "${TRAC_IK_BRANCH}" "origin/${TRAC_IK_BRANCH}"
else
  echo "ERROR: Branch ${TRAC_IK_BRANCH} not found."
  echo "Available branches:"
  git branch -a
  exit 1
fi

echo
echo "== Step 5: patch MoveIt header names for Humble =="
PLUGIN_HEADER="trac_ik_kinematics_plugin/include/trac_ik/trac_ik_kinematics_plugin.hpp"

if [ ! -f "${PLUGIN_HEADER}" ]; then
  echo "ERROR: Expected plugin header not found:"
  echo "${PLUGIN_HEADER}"
  exit 1
fi

sed -i \
  -e 's|moveit/kinematics_base/kinematics_base.hpp|moveit/kinematics_base/kinematics_base.h|g' \
  -e 's|moveit/robot_model/robot_model.hpp|moveit/robot_model/robot_model.h|g' \
  "${PLUGIN_HEADER}"

echo "Current MoveIt includes in plugin:"
grep -R "#include <moveit/.*\\.\\(hpp\\|h\\)>" -n trac_ik_kinematics_plugin || true

echo
echo "== Step 6: install rosdeps =="
cd "${WS}"
source "${ROS_SETUP}"
rosdep install --from-paths src --ignore-src -r -y

echo
echo "== Step 7: clean and build =="
rm -rf build install log

colcon build \
  --symlink-install \
  --parallel-workers "${PARALLEL_WORKERS:-2}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3

echo
echo "== Step 8: verify =="
source "${WS}/install/setup.bash"

echo "TRAC-IK packages:"
ros2 pkg list | grep trac_ik || {
  echo "ERROR: TRAC-IK packages not visible after sourcing install/setup.bash"
  exit 1
}

echo
echo "Plugin libraries:"
find "${WS}/install" -name "*trac_ik*.so" || true

echo
echo "Plugin XML files:"
find "${WS}/install" -name "*plugin*.xml" | grep trac || true

echo
echo "SUCCESS: TRAC-IK built and sourced successfully."
echo
echo "To use it in future shells, add this to ~/.bashrc:"
echo "  source ${WS}/install/setup.bash"
```

## Optional: save the Humble patch

After a successful build, save the local patch:

```bash
cd ~/workspace/trac_ik_ws/src/trac_ik
git diff > ~/workspace/trac_ik_humble_moveit_headers.patch
```

Later it can be reapplied with:

```bash
cd ~/workspace/trac_ik_ws/src/trac_ik
git apply ~/workspace/trac_ik_humble_moveit_headers.patch
```

## Sourcing order for MoveIt / Nero workspaces

For future terminals:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/trac_ik_ws/install/setup.bash
source ~/workspace/<your_nero_or_moveit_ws>/install/setup.bash
```

TRAC-IK should be sourced before the robot-specific MoveIt workspace.

## MoveIt kinematics.yaml example

Example:

```yaml
nero_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.01
  kinematics_solver_attempts: 5
  solve_type: Distance
```

The group name, here `nero_arm`, must exactly match the MoveIt SRDF planning group.

Check group names with:

```bash
grep -R "group name" ~/workspace/<your_moveit_config_pkg>/config/*.srdf
```

For initial robust testing:

```yaml
solve_type: Distance
kinematics_solver_timeout: 0.01
kinematics_solver_attempts: 5
```

For later faster Servo / online IK testing:

```yaml
solve_type: Speed
kinematics_solver_timeout: 0.003
kinematics_solver_attempts: 1
```

## Final verification commands

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/trac_ik_ws/install/setup.bash

ros2 pkg list | grep trac_ik
find ~/workspace/trac_ik_ws/install -name "*trac_ik*.so"
find ~/workspace/trac_ik_ws/install -name "*plugin*.xml" | grep trac
```