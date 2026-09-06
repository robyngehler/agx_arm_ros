#!/usr/bin/env bash
set -euo pipefail

STATE_DIR=/run/jetson-clock-boost
CLOCK_STATE="$STATE_DIR/jetson_clocks.conf"

if [[ ${EUID} -ne 0 ]]; then
  exec sudo --preserve-env=PATH "$0" "$@"
fi

JETSON_CLOCKS=${JETSON_CLOCKS:-/usr/bin/jetson_clocks}
[[ -x "$JETSON_CLOCKS" ]] || JETSON_CLOCKS=$(command -v jetson_clocks || true)

if [[ -z "${JETSON_CLOCKS:-}" || ! -x "$JETSON_CLOCKS" ]]; then
  echo "jetson_clocks not found" >&2
  exit 1
fi

case "${1:-}" in
  on)
    mkdir -p "$STATE_DIR"
    if [[ ! -f "$CLOCK_STATE" ]]; then
      "$JETSON_CLOCKS" --store "$CLOCK_STATE"
    fi

    echo 'Current nvpmodel (not modified by this script):'
    if command -v nvpmodel >/dev/null 2>&1; then
      nvpmodel -q 2>/dev/null || true
    fi

    if [[ "${2:-}" == "--fan" ]]; then
      "$JETSON_CLOCKS" --fan
    else
      "$JETSON_CLOCKS"
    fi
    echo "Jetson clocks boosted. Original clock state stored in $CLOCK_STATE"
    ;;

  off)
    if [[ ! -f "$CLOCK_STATE" ]]; then
      echo "No stored clock state found at $CLOCK_STATE; refusing to guess restore values." >&2
      exit 1
    fi
    "$JETSON_CLOCKS" --restore "$CLOCK_STATE"
    rm -rf "$STATE_DIR"
    echo 'Jetson clock boost disabled; previous clock settings restored.'
    ;;

  status)
    "$JETSON_CLOCKS" --show
    echo
    if command -v nvpmodel >/dev/null 2>&1; then
      nvpmodel -q 2>/dev/null || true
    fi
    ;;

  *)
    echo "Usage: $0 {on [--fan]|off|status}"
    exit 2
    ;;
esac
