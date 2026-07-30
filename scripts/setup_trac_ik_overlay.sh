#!/usr/bin/env bash
#
# setup_trac_ik_overlay.sh — provision the TRAC-IK overlay workspace that
# agx_arm_moveit depends on.
#
# agx_arm_moveit declares <exec_depend>trac_ik_kinematics_plugin</exec_depend>,
# but there is no ros-humble-trac-ik-kinematics-plugin apt package for Humble on
# arm64. The plugin is therefore built from source in a SEPARATE overlay
# workspace, kept outside this repo's src/ so it stays out of the agx_arm_ros
# colcon graph.
#
# Sources are pinned in config/trac_ik_overlay.repos; the Humble header fix is
# applied from scripts/patches/. Both are idempotent, so re-running this script
# on a provisioned host is safe.
#
# Usage:
#   bash ./scripts/setup_trac_ik_overlay.sh                  # default location
#   TRAC_IK_WS=~/somewhere/trac_ik_ws bash ./scripts/setup_trac_ik_overlay.sh
#
# Afterwards, point the build/runtime wrappers at the overlay:
#   export AGX_ARM_TRAC_IK_OVERLAY="$TRAC_IK_WS/install/setup.bash"
#
# See docs/project/jetson_migration.md and docs/control/environment.md.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
trac_ik_ws="${TRAC_IK_WS:-${HOME}/workspace/trac_ik_ws}"
repos_file="${repo_root}/config/trac_ik_overlay.repos"
patch_file="${repo_root}/scripts/patches/trac_ik_moveit_humble_headers.patch"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

for f in "${repos_file}" "${patch_file}" "${ros_setup}"; do
    if [[ ! -f "${f}" ]]; then
        echo "[error] required file not found: ${f}" >&2
        exit 1
    fi
done

if ! command -v vcs >/dev/null 2>&1; then
    echo "[error] 'vcs' not found. Install it with: sudo apt install -y python3-vcstool" >&2
    exit 1
fi

echo "[1/4] Importing pinned sources into ${trac_ik_ws}/src"
mkdir -p "${trac_ik_ws}/src"
# vcs import is idempotent for an already-checked-out, unmodified repo, but our
# working tree is intentionally patched, so only import when it is missing.
if [[ -d "${trac_ik_ws}/src/trac_ik/.git" ]]; then
    echo "  - trac_ik already checked out, leaving the working tree alone"
else
    vcs import "${trac_ik_ws}/src" < "${repos_file}"
fi

echo "[2/4] Excluding vendor packages that are unused and do not build on Humble"
# trac_ik_examples pulls demo-only deps; trac_ik_python needs SWIG bindings we
# do not use. agx_arm_moveit only loads the C++ kinematics plugin.
for pkg in trac_ik_examples trac_ik_python; do
    if [[ -d "${trac_ik_ws}/src/trac_ik/${pkg}" ]]; then
        touch "${trac_ik_ws}/src/trac_ik/${pkg}/COLCON_IGNORE"
        echo "  - COLCON_IGNORE: ${pkg}"
    fi
done

echo "[3/4] Applying the MoveIt Humble header patch"
# TRAC-IK 2.0.2 includes <moveit/.../*.hpp>, which only exists post-Humble;
# Humble ships the same headers as *.h.
pushd "${trac_ik_ws}/src/trac_ik" >/dev/null
if git apply --reverse --check "${patch_file}" 2>/dev/null; then
    echo "  - already applied, skipping"
elif git apply --check "${patch_file}" 2>/dev/null; then
    git apply "${patch_file}"
    echo "  - applied $(basename "${patch_file}")"
else
    echo "[error] patch does not apply cleanly and is not already applied." >&2
    echo "        Upstream headers may have changed; re-check the pin in" >&2
    echo "        config/trac_ik_overlay.repos against ${patch_file}." >&2
    popd >/dev/null
    exit 1
fi
popd >/dev/null

echo "[4/4] Building the overlay with system Python"
# Conda's Python must not leak into a ROS build: it changes the interpreter
# CMake selects and the resulting plugin fails to load under the ROS runtime.
# Mirrors the hygiene in scripts/colcon_build_system_python.sh.
filtered_path=""
IFS=':' read -r -a path_entries <<< "${PATH}"
for entry in "${path_entries[@]}"; do
    [[ -z "${entry}" ]] && continue
    if [[ "${entry}" == *"/conda/"* || "${entry}" == *"/miniforge"* \
       || "${entry}" == *"/miniforge3"* || "${entry}" == *"/mambaforge"* ]]; then
        continue
    fi
    filtered_path+="${filtered_path:+:}${entry}"
done
export PATH="${filtered_path}"
unset PYTHONHOME PYTHONPATH CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_EXE

set +u
source "${ros_setup}"
set -u

if [[ "$(command -v python3)" != "/usr/bin/python3" ]]; then
    echo "[error] expected system python3 at /usr/bin/python3, found $(command -v python3)" >&2
    exit 1
fi

if command -v rosdep >/dev/null 2>&1; then
    rosdep install --from-paths "${trac_ik_ws}/src" --ignore-src -r -y || \
        echo "  ! rosdep reported unresolved keys; continuing (libnlopt/orocos-kdl come from apt)"
else
    echo "  ! rosdep not found; ensure libnlopt-cxx-dev and liborocos-kdl-dev are installed"
fi

cd "${trac_ik_ws}"
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

echo ""
echo "[done] TRAC-IK overlay built at ${trac_ik_ws}/install"
echo ""
echo "[next] Export the overlay before building or running agx_arm_ros:"
echo "         export AGX_ARM_TRAC_IK_OVERLAY=${trac_ik_ws}/install/setup.bash"
echo "[next] Verify with: ros2 pkg prefix trac_ik_kinematics_plugin"
