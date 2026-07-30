#!/usr/bin/env bash
#
# setup_omnihand_sdk.sh — build the vendored OmniHand Pro SDK so the ROS bridge
# can import it.
#
# vendor/OmniHand-Pro-2025/build/ is gitignored INSIDE the submodule, so a fresh
# clone has no compiled SDK and the bridge's backend_type=sdk path cannot import
# agibot_hand. This script produces it in the exact location the bridge
# auto-discovers (src/agx_arm_ctrl/agx_arm_ctrl/omnihand/sdk_o12_pro.py), so do
# not relocate the output.
#
# Why this wrapper exists instead of calling vendor build.sh directly:
# the vendor CMake does find_package(Python3 COMPONENTS Interpreter Development).
# With Miniforge on PATH it selects conda's Python 3.13 and emits
# agibot_hand_core.cpython-313-*.so, which the ROS runtime (python3.10) cannot
# import. That failure only surfaces later, at bridge startup. This script pins
# the interpreter and strips conda from the environment.
#
# Usage:
#   bash ./scripts/setup_omnihand_sdk.sh              # build
#   bash ./scripts/setup_omnihand_sdk.sh --verify     # only check an existing build
#
# See docs/project/jetson_migration.md and .claude/rules/omnihand-bridge.md.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor_dir="${repo_root}/vendor/OmniHand-Pro-2025"
pkg_dir="${vendor_dir}/build/agibot_hand_pkg"
python_bin="${AGX_ARM_SDK_PYTHON:-/usr/bin/python3.10}"
verify_only=0

if [[ "${1:-}" == "--verify" ]]; then
    verify_only=1
fi

verify_sdk() {
    if [[ ! -d "${pkg_dir}" ]]; then
        echo "  ✗ ${pkg_dir} does not exist" >&2
        return 1
    fi
    local py_tag so
    py_tag="$("${python_bin}" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
    so="${pkg_dir}/agibot_hand/agibot_hand_core${py_tag}"
    if [[ ! -f "${so}" ]]; then
        echo "  ✗ expected extension module not found: ${so}" >&2
        echo "    present instead:" >&2
        ls "${pkg_dir}/agibot_hand/"*.so 2>/dev/null | sed 's/^/      /' >&2 || echo "      (none)" >&2
        echo "    A mismatched cpython tag means the SDK was built against the" >&2
        echo "    wrong interpreter. Rebuild with conda deactivated." >&2
        return 1
    fi
    # The .so has RUNPATH=$ORIGIN, so no LD_LIBRARY_PATH is needed here.
    if ! PYTHONPATH="${pkg_dir}" "${python_bin}" -c "import agibot_hand" 2>/dev/null; then
        echo "  ✗ 'import agibot_hand' failed under ${python_bin}" >&2
        return 1
    fi
    echo "  ✓ agibot_hand imports under ${python_bin} ($(basename "${so}"))"
    return 0
}

if [[ ! -f "${vendor_dir}/build.sh" ]]; then
    echo "[error] vendor SDK sources not found at ${vendor_dir}." >&2
    echo "        Initialize the submodule first:" >&2
    echo "          git submodule update --init --recursive" >&2
    exit 1
fi

if [[ ! -x "${python_bin}" ]]; then
    echo "[error] interpreter not found: ${python_bin}" >&2
    echo "        The ROS runtime is system python3.10; install python3-dev or set" >&2
    echo "        AGX_ARM_SDK_PYTHON to the interpreter the ROS stack runs on." >&2
    exit 1
fi

if [[ "${verify_only}" -eq 1 ]]; then
    echo "[verify] Checking the existing SDK build"
    verify_sdk
    exit $?
fi

echo "[info] Building the OmniHand Pro SDK for ${python_bin}"
echo "[info] CAN backend: SOCKETCAN (Jetson native mttcan; the ZLG USB backend is x86-only)"

# Strip conda so find_package(Python3) cannot pick up a 3.13 interpreter, and so
# the vendor's check_python_package() looks in the right site-packages.
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

# build/setuptools/wheel must be importable by ${python_bin} or the vendor build
# silently skips packaging the Python wheel (it only prints a CMake WARNING).
missing=()
for mod in build setuptools wheel; do
    "${python_bin}" -c "import ${mod}" 2>/dev/null || missing+=("${mod}")
done
if [[ "${#missing[@]}" -gt 0 ]]; then
    echo "[error] ${python_bin} cannot import: ${missing[*]}" >&2
    echo "        Install the pip layer first:" >&2
    echo "          ${python_bin} -m pip install --user -r ${repo_root}/requirements.txt" >&2
    exit 1
fi

cd "${vendor_dir}"
bash ./build.sh -DPython3_EXECUTABLE="${python_bin}" "$@"

echo ""
echo "[verify] Checking the produced SDK"
verify_sdk

echo ""
echo "[done] SDK available at ${pkg_dir}"
echo "[note] The bridge auto-discovers this path; AGX_ARM_OMNIHAND_SDK_DIR is only"
echo "       needed if the built package lives outside the repo checkout."
