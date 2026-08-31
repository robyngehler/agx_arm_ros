#!/bin/bash

set -o pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
timeout_seconds="${MOVEIT_SMOKE_TIMEOUT:-25}"
kill_after_seconds="${MOVEIT_SMOKE_KILL_AFTER:-10}"
base_log_dir="${MOVEIT_SMOKE_LOG_DIR:-$workspace_root/log/moveit_profile_smoke}"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
trac_ik_setup="${TRAC_IK_SETUP:-$HOME/workspace/trac_ik_ws/install/setup.bash}"

if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
    echo "Workspace overlay not found at $workspace_root/install/setup.bash" >&2
    exit 1
fi

if [[ ! -f "$ros_setup" ]]; then
    echo "ROS setup not found at $ros_setup" >&2
    exit 1
fi

set +u
source "$ros_setup"
if [[ -f "$trac_ik_setup" ]]; then
    source "$trac_ik_setup"
fi
source "$workspace_root/install/setup.bash"
set -u
export LC_NUMERIC=en_US.UTF-8

timestamp="$(date +%Y%m%d_%H%M%S)"
run_log_dir="$base_log_dir/$timestamp"
mkdir -p "$run_log_dir"

echo "ROS setup: $ros_setup"
if [[ -f "$trac_ik_setup" ]]; then
    echo "TRAC-IK overlay: $trac_ik_setup"
else
    echo "TRAC-IK overlay: not found, continuing without external overlay"
fi

overall_status=0

run_profile() {
    local name="$1"
    shift

    local log_file="$run_log_dir/${name}.log"
    local exit_code=0
    local ready="no"
    local shutdown_crash="no"
    local tracik_missing="no"
    local gated="no"
    local status="fail"

    timeout --signal=INT --kill-after="${kill_after_seconds}s" "${timeout_seconds}s" \
        ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero use_rviz:=false "$@" \
        >"$log_file" 2>&1
    exit_code=$?

    if grep -q "You can start planning now!" "$log_file"; then
        ready="yes"
    fi

    # A process dying with "exit code -2" is the SIGINT this script sends, not a crash.
    if grep -Eq "Segmentation fault|failed to terminate" "$log_file" \
        || grep -E "process has died" "$log_file" | grep -qv "exit code -2"; then
        shutdown_crash="yes"
    fi

    if grep -Eq "trac_ik_kinematics_plugin|TRAC_IK|According to the loaded plugin descriptions the class|class .* does not exist" "$log_file"; then
        tracik_missing="yes"
    fi

    if [[ "$ready" == "yes" && "$exit_code" -eq 124 ]]; then
        status="ready_timeout"
    elif [[ "$ready" == "yes" && "$exit_code" -eq 137 ]]; then
        status="ready_killed"
    elif [[ "$ready" == "yes" && "$exit_code" -eq 0 ]]; then
        status="ready_clean_exit"
    elif [[ "$ready" == "yes" ]]; then
        status="ready_exit_${exit_code}"
    fi

    if [[ "$ready" != "yes" || "$tracik_missing" == "yes" || "$shutdown_crash" == "yes" ]]; then
        gated="yes"
        overall_status=1
    fi

    printf '%-16s status=%-16s exit=%-4s ready=%-3s gated=%-3s tracik_missing=%-3s shutdown_crash=%-3s log=%s\n' \
        "$name" "$status" "$exit_code" "$ready" "$gated" "$tracik_missing" "$shutdown_crash" "$log_file"
}

run_profile none
run_profile agx_gripper effector_type:=agx_gripper
run_profile revo2_left effector_type:=revo2 revo2_type:=left
run_profile revo2_right effector_type:=revo2 revo2_type:=right
run_profile omnihand_left effector_type:=omnihand omnihand_type:=left
run_profile omnihand_right effector_type:=omnihand omnihand_type:=right

echo "Logs: $run_log_dir"
exit "$overall_status"