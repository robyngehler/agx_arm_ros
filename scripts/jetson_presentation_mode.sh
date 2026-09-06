#!/usr/bin/env bash
#
# jetson_presentation_mode.sh {on|off|status} — take off the power saving that
# costs latency or a link, and put it back afterwards.
#
# The line this script draws: it removes every knob that lets the platform stall
# something — a governor that starts a burst on a slow core, a suspended USB-CAN
# adapter, a radio parked between beacons, a machine that sleeps under an idle
# SSH session — but it leaves the CPU idle states alone, so cores reach full
# clock under load and still idle between bursts. Pinning clocks at idle too is
# jetson_clock_boost.sh, deliberately a separate opt-in.
#
# Everything is captured before it is changed. Most of it is runtime state that a
# reboot resets anyway; nvpmodel and the NetworkManager Wi-Fi profile are not, so
# their previous values are kept outside /run and 'off' undoes them after a
# reboot too.
#
# Do not combine with jetson_performance_mode.sh in one session: that one calls
# jetson_clocks without --store, so a boost taken afterwards records boosted
# clocks as the state to restore.

set -euo pipefail

# Runtime state: these knobs reset at reboot, so what they were resets with them.
STATE_DIR=/run/jetson-presentation-mode
SYSFS_STATE="$STATE_DIR/sysfs.tsv"
WIFI_STATE="$STATE_DIR/wifi.tsv"
SLEEP_MARKER="$STATE_DIR/sleep_masked_by_us"

# The power model and the NetworkManager profile survive a reboot, so what they
# were has to survive it too — otherwise 'off' after a reboot cannot undo them.
PERSIST_DIR=/var/lib/jetson-presentation-mode
NVPMODEL_STATE="$PERSIST_DIR/nvpmodel.txt"
NM_STATE="$PERSIST_DIR/networkmanager.tsv"

ASPM_POLICY=/sys/module/pcie_aspm/parameters/policy

if [[ ${EUID} -ne 0 ]]; then
  exec sudo --preserve-env=PATH "$0" "$@"
fi

log() { printf '[jetson-presentation] %s\n' "$*"; }
warn() { printf '[jetson-presentation] WARNING: %s\n' "$*" >&2; }

save_and_write() {
  local path="$1" value="$2" old
  [[ -e "$path" && -r "$path" && -w "$path" ]] || return 0
  old=$(cat "$path" 2>/dev/null || true)
  if ! grep -Fq "${path}"$'\t' "$SYSFS_STATE" 2>/dev/null; then
    printf '%s\t%s\n' "$path" "$old" >> "$SYSFS_STATE"
  fi
  printf '%s' "$value" > "$path" 2>/dev/null || warn "Could not set $path = $value"
}

restore_sysfs() {
  [[ -f "$SYSFS_STATE" ]] || return 0
  tac "$SYSFS_STATE" | while IFS=$'\t' read -r path value; do
    [[ -e "$path" && -w "$path" ]] || continue
    printf '%s' "$value" > "$path" 2>/dev/null || warn "Could not restore $path = $value"
  done
}

disable_sleep() {
  mkdir -p "$STATE_DIR"
  # Runtime masks disappear automatically at reboot.
  systemctl mask --runtime sleep.target suspend.target hibernate.target hybrid-sleep.target suspend-then-hibernate.target >/dev/null 2>&1 || true
  touch "$SLEEP_MARKER"
}

set_power_model_max() {
  command -v nvpmodel >/dev/null 2>&1 || { warn "nvpmodel not found; power model unchanged."; return 0; }

  local current
  # 'NV Power Mode: MAXN' then the mode id on the next line.
  current=$(nvpmodel -q 2>/dev/null | sed -n '2p' | tr -cd '0-9')
  if [[ -n "$current" && ! -f "$NVPMODEL_STATE" ]]; then
    printf '%s\n' "$current" > "$NVPMODEL_STATE"
  fi

  # A ceiling, not a floor: MAXN permits the top clocks, it does not ask for
  # them. Without it the governor cannot reach a clock the model forbids.
  nvpmodel -m 0 </dev/null >/dev/null 2>&1 || warn "Could not set nvpmodel to MAXN"
}

restore_power_model() {
  [[ -f "$NVPMODEL_STATE" ]] || return 0
  command -v nvpmodel >/dev/null 2>&1 || return 0
  local mode
  mode=$(cat "$NVPMODEL_STATE" 2>/dev/null)
  [[ -n "$mode" ]] || return 0
  nvpmodel -m "$mode" </dev/null >/dev/null 2>&1 || warn "Could not restore nvpmodel mode $mode"
}

enable_all_cpus_and_performance_governor() {
  local p avail
  for p in /sys/devices/system/cpu/cpu[0-9]*/online; do
    [[ -e "$p" ]] && save_and_write "$p" 1
  done

  for p in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
    [[ -e "$p" ]] || continue
    avail="${p%/scaling_governor}/scaling_available_governors"
    if [[ -r "$avail" ]] && grep -qw performance "$avail"; then
      save_and_write "$p" performance
    else
      warn "performance governor not available for ${p%/*}"
    fi
  done
  # Best effort: a glob that matches nothing would otherwise end the function
  # non-zero and abort 'on' half applied under set -e.
  return 0
}

disable_runtime_pm() {
  local p

  # Global USB autosuspend + per-device USB runtime PM.
  save_and_write /sys/module/usbcore/parameters/autosuspend -1
  for p in /sys/bus/usb/devices/*/power/control; do
    [[ -e "$p" ]] && save_and_write "$p" on
  done
  for p in /sys/bus/usb/devices/*/power/autosuspend_delay_ms; do
    [[ -e "$p" ]] && save_and_write "$p" -1
  done

  # PCIe-attached peripherals (NVMe, NICs, add-in controllers, ...).
  for p in /sys/bus/pci/devices/*/power/control; do
    [[ -e "$p" ]] && save_and_write "$p" on
  done

  # Any network interface with an exposed runtime-PM control.
  for p in /sys/class/net/*/device/power/control; do
    [[ -e "$p" ]] && save_and_write "$p" on
  done

  # Native Jetson MTT-CAN controllers may be platform devices.
  while IFS= read -r -d '' p; do
    save_and_write "$p" on
  done < <(find /sys/devices -type f -path '*mttcan*/power/control' -print0 2>/dev/null)
  return 0
}

disable_pcie_aspm() {
  # Link power states, one layer below the per-device runtime PM above. Reads as
  # '[default] performance powersave ...' and only the bare token can be written
  # back, so this cannot go through save_and_write.
  if [[ ! -w "$ASPM_POLICY" ]]; then
    warn "PCIe ASPM policy is not writable; it is locked by the kernel command line."
    return 0
  fi
  local current
  current=$(sed -n 's/.*\[\([a-z]*\)\].*/\1/p' "$ASPM_POLICY" 2>/dev/null)
  if [[ -n "$current" ]] && ! grep -Fq "${ASPM_POLICY}"$'\t' "$SYSFS_STATE" 2>/dev/null; then
    printf '%s\t%s\n' "$ASPM_POLICY" "$current" >> "$SYSFS_STATE"
  fi
  printf '%s' performance > "$ASPM_POLICY" 2>/dev/null \
    || warn "Could not set PCIe ASPM policy to performance"
}

disable_wifi_powersave() {
  # Never re-capture: a second 'on' would record the already-disabled state as
  # the original and 'off' would then leave power save off for good. Same reason
  # save_and_write only records a path once.
  [[ -f "$WIFI_STATE" ]] || : > "$WIFI_STATE"
  if ! command -v iw >/dev/null 2>&1; then
    warn "'iw' not installed; Wi-Fi driver power-save state was not changed."
    return 0
  fi

  local iface state
  while read -r iface; do
    [[ -n "$iface" ]] || continue
    state=$(iw dev "$iface" get power_save 2>/dev/null | awk -F': ' '/Power save/ {print $2}')
    if [[ -n "$state" ]] && ! grep -Fq "${iface}"$'\t' "$WIFI_STATE" 2>/dev/null; then
      printf '%s\t%s\n' "$iface" "$state" >> "$WIFI_STATE"
    fi
    iw dev "$iface" set power_save off 2>/dev/null || warn "Could not disable Wi-Fi power save on $iface"
  done < <(iw dev 2>/dev/null | awk '$1=="Interface" {print $2}')
}

restore_wifi() {
  [[ -f "$WIFI_STATE" && -x "$(command -v iw 2>/dev/null || true)" ]] || return 0
  local iface state
  while IFS=$'\t' read -r iface state; do
    [[ -n "$iface" && -n "$state" ]] || continue
    iw dev "$iface" set power_save "$state" 2>/dev/null || true
  done < "$WIFI_STATE"
}

# 'iw' above only reaches the link that is up now. NetworkManager re-applies the
# profile on every reconnect, and the demo is operated over this radio, so the
# profile has to say it too. The profile is persistent, which is why its previous
# value is kept outside /run.
nm_wifi_profiles() {
  # NAME:DEVICE:TYPE, with ':' inside a name escaped as '\:' by -t. Dropping the
  # last two fields leaves the name whether or not it contained one.
  nmcli -t -f NAME,DEVICE,TYPE con show --active 2>/dev/null \
    | awk -F: -v OFS=: '$NF ~ /wireless/ && NF > 2 { NF -= 2; print }' \
    | sed 's/\\:/:/g'
}

disable_nm_wifi_powersave() {
  if ! command -v nmcli >/dev/null 2>&1; then
    warn "'nmcli' not installed; the Wi-Fi profile still re-enables power save on reconnect."
    return 0
  fi
  [[ -f "$NM_STATE" ]] || : > "$NM_STATE"

  local con current
  while read -r con; do
    [[ -n "$con" ]] || continue
    current=$(nmcli -g 802-11-wireless.powersave con show "$con" 2>/dev/null)
    if [[ -n "$current" ]] && ! grep -Fq "${con}"$'\t' "$NM_STATE" 2>/dev/null; then
      printf '%s\t%s\n' "$con" "$current" >> "$NM_STATE"
    fi
    # 2 = disable. Modifying the profile does not touch the live connection, so
    # nothing is torn down under the SSH session running this.
    nmcli con modify "$con" 802-11-wireless.powersave 2 2>/dev/null \
      || warn "Could not disable power save on profile '$con'"
  done < <(nm_wifi_profiles)
}

restore_nm_wifi_powersave() {
  [[ -f "$NM_STATE" ]] || return 0
  command -v nmcli >/dev/null 2>&1 || return 0
  local con value
  while IFS=$'\t' read -r con value; do
    [[ -n "$con" && -n "$value" ]] || continue
    nmcli con modify "$con" 802-11-wireless.powersave "$value" 2>/dev/null \
      || warn "Could not restore power save on profile '$con'"
  done < "$NM_STATE"
}

show_status() {
  echo '=== Power mode ==='
  if command -v nvpmodel >/dev/null 2>&1; then
    nvpmodel -q 2>/dev/null || true
  else
    echo 'nvpmodel not found'
  fi

  echo
  echo '=== CPU ==='
  printf 'online: '
  cat /sys/devices/system/cpu/online 2>/dev/null || true
  for p in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
    [[ -e "$p" ]] && printf '%s: %s\n' "${p%/*}" "$(cat "$p")"
  done

  echo
  echo '=== USB autosuspend ==='
  cat /sys/module/usbcore/parameters/autosuspend 2>/dev/null || true

  echo
  echo '=== PCIe ASPM ==='
  sed -n 's/.*\[\([a-z]*\)\].*/\1/p' "$ASPM_POLICY" 2>/dev/null || echo 'n/a'

  echo
  echo '=== Wi-Fi power save ==='
  if command -v iw >/dev/null 2>&1; then
    local iface
    while read -r iface; do
      [[ -n "$iface" ]] || continue
      printf 'link %s: ' "$iface"
      iw dev "$iface" get power_save 2>/dev/null | sed 's/^\s*//' || true
    done < <(iw dev 2>/dev/null | awk '$1=="Interface" {print $2}')
  fi
  if command -v nmcli >/dev/null 2>&1; then
    local con
    while read -r con; do
      [[ -n "$con" ]] || continue
      printf "profile '%s': powersave=%s (2 = disabled)\n" \
        "$con" "$(nmcli -g 802-11-wireless.powersave con show "$con" 2>/dev/null)"
    done < <(nm_wifi_profiles)
  fi

  echo
  echo '=== Sleep targets ==='
  systemctl is-enabled sleep.target suspend.target hibernate.target hybrid-sleep.target suspend-then-hibernate.target 2>/dev/null || true
  # A report always succeeds: 'status' is the exit code of this function.
  return 0
}

case "${1:-}" in
  on)
    mkdir -p "$STATE_DIR" "$PERSIST_DIR"
    if [[ -f "$SYSFS_STATE" ]]; then
      warn "State already exists in $STATE_DIR; assuming presentation mode is already active."
    else
      : > "$SYSFS_STATE"
    fi
    disable_sleep
    set_power_model_max
    enable_all_cpus_and_performance_governor
    disable_runtime_pm
    disable_pcie_aspm
    disable_wifi_powersave
    disable_nm_wifi_powersave
    log "Presentation mode enabled."
    log "The CPU idle states are untouched: cores reach full clock under load and"
    log "still idle between bursts. Full clocks at idle are jetson_clock_boost.sh."
    log "Most of this resets at reboot; nvpmodel and the Wi-Fi profile do not, so"
    log "run '$0 off' to undo them."
    show_status
    ;;

  off)
    restore_nm_wifi_powersave
    restore_power_model
    restore_wifi
    restore_sysfs
    if [[ -f "$SLEEP_MARKER" ]]; then
      systemctl unmask --runtime sleep.target suspend.target hibernate.target hybrid-sleep.target suspend-then-hibernate.target >/dev/null 2>&1 || true
    fi
    rm -rf "$STATE_DIR" "$PERSIST_DIR"
    log "Presentation mode disabled; saved settings restored where possible."
    ;;

  status)
    show_status
    ;;

  *)
    echo "Usage: $0 {on|off|status}"
    exit 2
    ;;
esac
