#!/usr/bin/env bash
set -euo pipefail

STATE_DIR=/run/jetson-presentation-mode
SYSFS_STATE="$STATE_DIR/sysfs.tsv"
WIFI_STATE="$STATE_DIR/wifi.tsv"
SLEEP_MARKER="$STATE_DIR/sleep_masked_by_us"

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
  echo '=== Wi-Fi power save ==='
  if command -v iw >/dev/null 2>&1; then
    local iface
    while read -r iface; do
      [[ -n "$iface" ]] || continue
      printf '%s: ' "$iface"
      iw dev "$iface" get power_save 2>/dev/null | sed 's/^\s*//' || true
    done < <(iw dev 2>/dev/null | awk '$1=="Interface" {print $2}')
  fi

  echo
  echo '=== Sleep targets ==='
  systemctl is-enabled sleep.target suspend.target hibernate.target hybrid-sleep.target suspend-then-hibernate.target 2>/dev/null || true
}

case "${1:-}" in
  on)
    mkdir -p "$STATE_DIR"
    if [[ -f "$SYSFS_STATE" ]]; then
      warn "State already exists in $STATE_DIR; assuming presentation mode is already active."
    else
      : > "$SYSFS_STATE"
    fi
    disable_sleep
    enable_all_cpus_and_performance_governor
    disable_runtime_pm
    disable_wifi_powersave
    log "Presentation mode enabled. Settings are runtime-only and will also reset on reboot."
    show_status
    ;;

  off)
    restore_wifi
    restore_sysfs
    if [[ -f "$SLEEP_MARKER" ]]; then
      systemctl unmask --runtime sleep.target suspend.target hibernate.target hybrid-sleep.target suspend-then-hibernate.target >/dev/null 2>&1 || true
    fi
    rm -rf "$STATE_DIR"
    log "Presentation mode disabled; saved runtime settings restored where possible."
    ;;

  status)
    show_status
    ;;

  *)
    echo "Usage: $0 {on|off|status}"
    exit 2
    ;;
esac
