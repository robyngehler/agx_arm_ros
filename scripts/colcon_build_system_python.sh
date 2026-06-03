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

cd "${repo_root}"
colcon build "$@"