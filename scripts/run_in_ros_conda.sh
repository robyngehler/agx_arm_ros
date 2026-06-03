#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
env_name="${AGX_ARM_CONDA_ENV:-agx-arm-runtime}"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --env)
            env_name="$2"
            shift 2
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

if [[ $# -eq 0 ]]; then
    echo "Usage: $0 [--env ENV_NAME] -- <command> [args...]" >&2
    exit 1
fi

if ! command -v conda >/dev/null 2>&1; then
    echo "conda was not found on PATH. Run scripts/setup_agx_arm_runtime_env.sh first." >&2
    exit 1
fi

if [[ ! -f "${ros_setup}" ]]; then
    echo "ROS setup file not found: ${ros_setup}" >&2
    exit 1
fi

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

if [[ -f "${repo_root}/install/setup.bash" ]]; then
    set +u
    source "${repo_root}/install/setup.bash"
    set -u
fi

export PYTHONNOUSERSITE=1
unset PYTHONHOME

conda run --no-capture-output --name "${env_name}" "$@"