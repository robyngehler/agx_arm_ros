#!/bin/bash
#
# jetson_performance_mode.sh — take every power-saving knob off this Jetson.
#
# The MIT control loop is a paced 100-200 Hz thread that does bounded SDK work
# per cycle. Anything that lets the platform decide to run slower for a while
# shows up in it as jitter, and a frequency ramp that takes tens of milliseconds
# is several control cycles. So on this unit the answer to every "should the
# platform save power here" question is no.
#
# WHAT IT TURNS OFF, and what each one costs when left on:
#
#   CPU governor      `schedutil` scales frequency with load. A control thread
#                     that is mostly waiting looks like low load, so the cluster
#                     clocks down and the next burst starts slow. Set to
#                     `performance`, which pins every core at its maximum.
#   nvpmodel          the power budget. MAXN (mode 0) is the only one that lets
#                     all twelve cores run at full clock.
#   jetson_clocks     GPU and memory-controller DVFS. MoveIt planning and the
#                     description stack care; the arm loop cares indirectly,
#                     through memory latency.
#   WiFi power save   the radio parks between beacons. Costs interactive SSH its
#                     responsiveness, which is how this unit is operated.
#   USB autosuspend   a USB-CAN FD adapter that has been idle can be suspended.
#                     The hand buses are on such adapters.
#   PCIe ASPM         link power states, same argument one layer down.
#
# WHAT IT COSTS. More heat and more power: the fan will run harder and the unit
# draws its full budget whether or not it is doing anything. That is the trade
# this script exists to make. It is not a setting for a battery.
#
# NOT PERSISTENT unless installed. Everything here is runtime state and a reboot
# restores the defaults; `--install` writes a systemd unit that reapplies it at
# boot.
#
# Usage:
#   ./scripts/jetson_performance_mode.sh --show      # report, change nothing
#   sudo ./scripts/jetson_performance_mode.sh        # apply
#   sudo ./scripts/jetson_performance_mode.sh --install   # apply at every boot
#   sudo ./scripts/jetson_performance_mode.sh --uninstall

set -uo pipefail

UNIT=/etc/systemd/system/jetson-performance-mode.service
SELF="$(readlink -f "${BASH_SOURCE[0]}")"

MODE=apply
case "${1:-}" in
    --show|show)   MODE=show ;;
    --install)     MODE=install ;;
    --uninstall)   MODE=uninstall ;;
    "")            MODE=apply ;;
    -h|--help)     sed -n '2,40p' "$SELF"; exit 0 ;;
    *) echo "usage: $0 [--show|--install|--uninstall]" >&2; exit 2 ;;
esac

wifi_device() {
    nmcli -t -f DEVICE,TYPE dev status 2>/dev/null \
        | awk -F: '$2 == "wifi" { print $1; exit }'
}

report() {
    local dev
    echo "nvpmodel        : $(nvpmodel -q 2>/dev/null | head -1 | sed 's/.*: //')"
    echo "cpu governors   : $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor \
                              2>/dev/null | sort -u | paste -sd, -)"
    echo "cpu0 min/cur/max: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq \
                              /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq \
                              /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq \
                              2>/dev/null | paste -sd/ -)"
    echo "cores online    : $(cat /sys/devices/system/cpu/online 2>/dev/null)"
    dev="$(wifi_device)"
    if [ -n "$dev" ]; then
        echo "wifi power save : $(iw dev "$dev" get power_save 2>/dev/null | sed 's/.*: //') ($dev)"
    else
        echo "wifi power save : no wifi device"
    fi
    echo "usb autosuspend : $(cat /sys/bus/usb/devices/*/power/control 2>/dev/null \
                              | sort | uniq -c | tr -s ' ' | paste -sd' ' -)"
    echo "pcie aspm       : $(sed 's/.*\[\([a-z]*\)\].*/\1/' \
                              /sys/module/pcie_aspm/parameters/policy 2>/dev/null || echo n/a)"
    echo "boot at max     : $(systemctl is-enabled jetson-performance-mode.service 2>/dev/null \
                              || echo 'not installed')"
}

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "Run with sudo (this changes CPU, radio and bus power state)." >&2
        exit 1
    fi
}

apply() {
    local failed=0 dev

    # The power budget first: a governor cannot reach a clock the mode forbids.
    if command -v nvpmodel >/dev/null; then
        nvpmodel -m 0 >/dev/null 2>&1 || echo "  nvpmodel -m 0 failed" >&2
        echo "  nvpmodel: MAXN"
    fi

    # GPU and memory-controller clocks. Runs BEFORE the governor because it
    # writes CPU frequency state too — whichever runs last decides, and the
    # governor is the setting worth being able to read back.
    if command -v jetson_clocks >/dev/null; then
        jetson_clocks >/dev/null 2>&1 && echo "  jetson_clocks: GPU and EMC pinned" \
            || echo "  jetson_clocks failed" >&2
    fi

    local count=0
    for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        if echo performance > "$governor" 2>/dev/null; then
            count=$((count + 1))
        else
            failed=1
        fi
    done
    echo "  cpu governor: performance on $count core(s)"

    dev="$(wifi_device)"
    if [ -n "$dev" ]; then
        iw dev "$dev" set power_save off 2>/dev/null \
            && echo "  wifi: power save off on $dev" \
            || echo "  wifi: could not disable power save on $dev" >&2
        # And on the profile, so NetworkManager does not put it back on the next
        # reconnect. 2 = disable.
        local connection
        connection="$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null \
                      | awk -F: -v d="$dev" '$2 == d { print $1; exit }')"
        if [ -n "$connection" ]; then
            nmcli con modify "$connection" 802-11-wireless.powersave 2 2>/dev/null \
                && echo "  wifi: '$connection' profile set to powersave disabled"
        fi
    fi

    # Per device, because usbcore's default only applies to devices that appear
    # after it changes. The CAN FD adapters are already enumerated.
    local usb=0
    for control in /sys/bus/usb/devices/*/power/control; do
        echo on > "$control" 2>/dev/null && usb=$((usb + 1))
    done
    echo "  usb: autosuspend off on $usb device(s)"
    printf '%s' -1 > /sys/module/usbcore/parameters/autosuspend 2>/dev/null \
        && echo "  usb: autosuspend disabled for devices added later"

    if [ -w /sys/module/pcie_aspm/parameters/policy ]; then
        echo performance > /sys/module/pcie_aspm/parameters/policy 2>/dev/null \
            && echo "  pcie: ASPM policy performance" \
            || echo "  pcie: ASPM policy is locked by the kernel command line"
    fi

    return "$failed"
}

install_unit() {
    cat > "$UNIT" <<UNITFILE
[Unit]
Description=Take every power-saving knob off this Jetson (agx_arm_ros)
After=network.target NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
ExecStart=$SELF
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNITFILE
    systemctl daemon-reload
    systemctl enable --now jetson-performance-mode.service >/dev/null 2>&1
    echo "installed $UNIT and enabled it"
}

case "$MODE" in
    show)
        report
        exit 0
        ;;
    uninstall)
        require_root
        systemctl disable --now jetson-performance-mode.service >/dev/null 2>&1
        rm -f "$UNIT"
        systemctl daemon-reload
        echo "removed $UNIT — the settings stay until the next reboot"
        exit 0
        ;;
esac

require_root
echo "before:"
report
echo
echo "applying:"
apply
rc=$?
echo
echo "after:"
report

if [ "$MODE" = install ]; then
    echo
    install_unit
fi

exit "$rc"
