#!/usr/bin/env bash
# L3 stop-latency stress: fire emergency stops into a live MIT stream.
#
# The budget this exercises is "an emergency stop reaches the SDK within 20 ms
# while normal work is running" (docs/sprint_refactor/reference/sdk_latency_budget.md).
# The number that answers it is `sdk_queue_wait.safety` in the driver's runtime
# metrics: how long the stop waited behind work already executing. The service
# round trip is NOT that number — it includes up to 0.5 s of feedback
# verification, which is the stop proving itself, not the stop being delayed.
#
# Each cycle: stop -> clear the fault lockout -> re-enable the controller, which
# is also the enable/disable churn the checklist asks for. Requires the driver
# and MIT controller already running with runtime_metrics_enabled:=true, and the
# controller enabled and holding.
#
# Usage: bash scripts/measure_stop_latency.sh [cycles] [seconds_between]
set -uo pipefail

CYCLES="${1:-10}"
GAP="${2:-3}"

echo "stop-latency stress: ${CYCLES} cycles, ${GAP}s apart"
echo "cycle,stop_service_s,stop_success,clear_s,reenable_s"

for i in $(seq 1 "$CYCLES"); do
    t0=$(date +%s.%N)
    stop_out=$(ros2 service call /emergency_stop std_srvs/srv/Trigger "{}" 2>&1)
    t1=$(date +%s.%N)
    ok=$(echo "$stop_out" | grep -o "success=[A-Za-z]*" | head -1 | cut -d= -f2)

    t2=$(date +%s.%N)
    ros2 service call /clear_fault_lockout std_srvs/srv/Trigger "{}" >/dev/null 2>&1
    t3=$(date +%s.%N)

    t4=$(date +%s.%N)
    ros2 service call /mit_controller/enable std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
    ros2 service call /mit_controller/hold_current std_srvs/srv/Empty "{}" >/dev/null 2>&1
    t5=$(date +%s.%N)

    printf "%d,%.3f,%s,%.3f,%.3f\n" \
        "$i" "$(echo "$t1 - $t0" | bc)" "${ok:-unknown}" \
        "$(echo "$t3 - $t2" | bc)" "$(echo "$t5 - $t4" | bc)"

    sleep "$GAP"
done

echo
echo "Now read 'sdk_queue_wait.safety' from the driver log — that is the budget"
echo "number. The service timings above include verification and are not it."
