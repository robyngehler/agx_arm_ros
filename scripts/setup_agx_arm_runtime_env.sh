#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
env_file="${repo_root}/config/agx_arm_runtime.conda.yaml"
env_name="${1:-agx-arm-runtime}"

if ! command -v conda >/dev/null 2>&1; then
    echo "conda was not found on PATH. Install Miniforge/Conda first." >&2
    exit 1
fi

if [[ ! -f "${env_file}" ]]; then
    echo "Runtime environment file not found: ${env_file}" >&2
    exit 1
fi

echo "[1/2] Updating conda environment '${env_name}' from ${env_file}"
conda env update --name "${env_name}" --file "${env_file}" --prune

vendored_pyagxarm_repo="${repo_root}/vendor/pyAgxArm"
sibling_pyagxarm_repo="$(cd "${repo_root}/.." && pwd)/pyAgxArm"
pyagxarm_repo=""
pyagxarm_label=""

if [[ -f "${vendored_pyagxarm_repo}/pyproject.toml" ]]; then
    pyagxarm_repo="${vendored_pyagxarm_repo}"
    pyagxarm_label="vendored pyAgxArm (${vendored_pyagxarm_repo})"
elif [[ -f "${sibling_pyagxarm_repo}/pyproject.toml" ]]; then
    pyagxarm_repo="${sibling_pyagxarm_repo}"
    pyagxarm_label="sibling pyAgxArm (${sibling_pyagxarm_repo})"
fi

if [[ -n "${pyagxarm_repo}" ]]; then
    echo "[2/2] Installing ${pyagxarm_label} into '${env_name}'"
    conda run --no-capture-output --name "${env_name}" python -m pip install -e "${pyagxarm_repo}"
else
    echo "[2/2] Skipping pyAgxArm editable install; neither vendor/pyAgxArm nor ../pyAgxArm was found"
fi

echo ""
echo "Use scripts/run_in_ros_conda.sh --env ${env_name} -- <command> to run ROS commands in this environment."