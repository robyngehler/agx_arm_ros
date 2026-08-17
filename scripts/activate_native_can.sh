#!/bin/bash
#
# RETIRED — use scripts/activate_duo_can.sh
#
# activate_native_can.sh brought up the two native mttcan arm buses. The supported bring-up is now
# scripts/activate_duo_can.sh, which configures all four Duo buses by physical
# slot: the two hand adapters are identical hardware, so their canN indices
# depend on enumeration order and can swap between boots, which points a hand's
# commands at the other hand. The slot cannot swap.
#
# It carries everything this script set: bitrate and sample points, CAN FD,
# one-shot, restart-ms, txqueuelen, net.core.rmem_max, and the TJA1051T/3 TDC
# offset on the native arm buses.
#
#   sudo ./scripts/activate_duo_can.sh          # all four
#   sudo ./scripts/activate_duo_can.sh arms     # arms only
#   sudo ./scripts/activate_duo_can.sh hands    # hands only
#   ./scripts/activate_duo_can.sh --show        # report state, change nothing
#
# This shim forwards rather than failing, so an operator or a stale runbook is
# not left without a bus. It does not accept this script's old arguments.

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "NOTE: activate_native_can.sh is retired; forwarding to activate_duo_can.sh arms." >&2
if [ "$#" -gt 0 ]; then
    echo "      Ignoring arguments: $* — see activate_duo_can.sh --show." >&2
fi

exec bash "$here/activate_duo_can.sh" arms
