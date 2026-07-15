#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
    echo "ROS setup file not found: ${ros_setup}" >&2
    exit 1
fi

filtered_path=""
IFS=':' read -r -a path_entries <<< "${PATH}"
for entry in "${path_entries[@]}"; do
    if [[ -z "${entry}" ]]; then
        continue
    fi
    if [[ "${entry}" == *"/conda/"* || "${entry}" == *"/miniforge"* || "${entry}" == *"/miniforge3"* || "${entry}" == *"/mambaforge"* ]]; then
        continue
    fi
    # Drop the Python user-site bin. With PYTHONNOUSERSITE=1 (set below) a pip
    # "cmake"/"colcon" shim in ~/.local/bin can no longer import its own module
    # (it lives in the now-disabled user-site), so it must not shadow /usr/bin.
    if [[ "${entry}" == "${HOME}/.local/bin" ]]; then
        continue
    fi
    if [[ -n "${filtered_path}" ]]; then
        filtered_path+=":"
    fi
    filtered_path+="${entry}"
done

export PATH="${filtered_path}"
unset PYTHONHOME
unset PYTHONPATH
unset CONDA_DEFAULT_ENV
unset CONDA_EXE
unset CONDA_PREFIX
unset CONDA_PREFIX_1
unset CONDA_PROMPT_MODIFIER
unset CONDA_PYTHON_EXE
unset CONDA_SHLVL
export PYTHONNOUSERSITE=1

set +u
source "${ros_setup}"
set -u

if [[ -n "${AGX_ARM_TRAC_IK_OVERLAY:-}" ]]; then
    if [[ ! -f "${AGX_ARM_TRAC_IK_OVERLAY}" ]]; then
        echo "AGX_ARM_TRAC_IK_OVERLAY does not point to a readable setup.bash: ${AGX_ARM_TRAC_IK_OVERLAY}" >&2
        exit 1
    fi
    set +u
    source "${AGX_ARM_TRAC_IK_OVERLAY}"
    set -u
fi

if [[ "$(command -v python3)" != "/usr/bin/python3" ]]; then
    echo "Expected system python3 at /usr/bin/python3, found $(command -v python3)" >&2
    exit 1
fi

# Drop stale workspace-install entries from the prefix paths. After a clean
# `rm -rf install/` the calling shell often still has the old install/setup.bash
# sourced, so AMENT/CMAKE/COLCON_PREFIX_PATH point at directories that no longer
# exist and colcon prints a warning per package. Only non-existent paths under
# this workspace's install/ are removed; external overlays stay untouched.
filter_stale_install_prefixes() {
    local var_name="$1" entry filtered=""
    local -a entries
    IFS=':' read -r -a entries <<< "${!var_name:-}"
    for entry in "${entries[@]}"; do
        if [[ -z "${entry}" ]]; then
            continue
        fi
        if [[ "${entry}" == "${repo_root}/install"* && ! -d "${entry}" ]]; then
            continue
        fi
        filtered+="${filtered:+:}${entry}"
    done
    if [[ -n "${filtered}" ]]; then
        export "${var_name}=${filtered}"
    else
        unset "${var_name}" || true
    fi
}
filter_stale_install_prefixes AMENT_PREFIX_PATH
filter_stale_install_prefixes CMAKE_PREFIX_PATH
filter_stale_install_prefixes COLCON_PREFIX_PATH

# The OmniHand Pro vendor SDK is upstream input, not a workspace package: it is
# built by its own vendor/OmniHand-Pro-2025/build.sh (that needs the pip
# 'build' module, which usually lives in the user site — hidden here by
# PYTHONNOUSERSITE=1) and consumed at runtime from
# vendor/OmniHand-Pro-2025/build/agibot_hand_pkg via auto-discovery. Skip it in
# workspace builds unless the caller names it explicitly.
skip_args=()
if [[ " $* " != *"omni_hand_pro_2025"* ]]; then
    skip_args=(--packages-skip omni_hand_pro_2025)
fi

cd "${repo_root}"
colcon build "${skip_args[@]}" "$@"