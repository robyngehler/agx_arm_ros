# Proposal: OmniHand CAN-FD Bitrate Validation on Jetson Native mttcan

## Context

The OmniHand Pro bring-up has progressed from USB-CAN driver blocking issues to a real CAN-FD validation stage on the Jetson Orin native CAN controller.

The current setup uses the Jetson native `mttcan` interface exposed as `can_omnihand`. This interface can be configured as CAN FD and reports:

```text
can_omnihand: mtu 72
can <BERR-REPORTING,FD>
bitrate 1000000
dbitrate 3846153
parentdev c310000.mttcan
clock 50000000
```

This confirms that the native Jetson interface is now a real CAN-FD netdevice. The previous blocker, where the USB adapter only exposed classic CAN with `mtu 16`, is no longer the immediate issue for this test path.

However, the OmniHand SDK smoke test still cannot retrieve live hardware information from the hand.

## Current observed failure

After configuring the interface and running the SDK smoke test with the patched SocketCAN backend:

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK

PYTHONPATH=$PWD/build_phase1_socket/omnihand_2025_pkg \
LD_LIBRARY_PATH=$PWD/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_omnihand \
python3.10 python/example/demo_get_hardware_info.py
```

The SDK reports CAN-FD request timeouts:

```text
[ERROR]: CANFD ID: 0x00010101 请求超时
[ERROR]: CANFD ID: 0x00020101 请求超时
```

The printed vendor and device information remains empty/default:

```text
Vendor Info:
Product Model:
Serial Number:
Hardware Version: 0.0.0
Software Version: 0.0.0
Supply Voltage: 0mV
Active Degrees of Freedom: 0

Device Info:
Device ID: 0
Arbitration Bitrate: 125Kbps
Arbitration Sample Point: 75.0%
Data Bitrate: 125Kbps
Data Sample Point: 75.0%
```

These values should be treated as fallback/default SDK output, not as confirmed live hardware state.

## Latest validation on this host

Additional validation on the current Jetson host now shows:

- the SDK SocketCAN path does generate request frames locally; enabling `hand.show_data_details(True)` prints `SND` frames for `0x00010101` and `0x00020101`,
- the repo-local SocketCAN backend has been aligned with the vendor ZLG backend by enabling CAN FD bit-rate switching on transmit,
- but `demo_get_hardware_info.py` still times out, `candump` on `can_omnihand` stays silent, and `ip -details -statistics link show can_omnihand` remains at `TX packets 0` with accumulated `bus-errors` and automatic restarts.

Interpretation:

- missing CAN FD bit-rate switching in the SocketCAN path was not the only blocker,
- the current failure still points below ROS and below the Python demo layer,
- and the remaining suspects stay concentrated in kernel-driver transmission, CAN FD timing mismatch, or physical bus conditions.

## Technical finding: exact 4 Mbit/s data phase is not currently produced

The vendor path appears to expect a CAN-FD data bitrate of `4000000` bit/s.

The Jetson native `mttcan` controller reports a CAN clock of `50000000` Hz. With this clock, the requested `4000000` data bitrate is not represented exactly by the current driver timing constraints. The interface instead reports:

```text
requested dbitrate: 4000000
actual dbitrate:    3846153
```

This is consistent with bit-timing quantization:

```text
50000000 / 13 = 3846153.846...
```

The mismatch from 4 Mbit/s is approximately:

```text
(4000000 - 3846153) / 4000000 = 0.03846 = 3.846 %
```

For CAN FD this is likely too large if the hand is fixed to exactly 4 Mbit/s in the data phase.

## Working hypothesis

The current failure is most likely caused by one of the following:

1. The OmniHand firmware expects `dbitrate=4000000`, while the Jetson native interface runs at `3846153`.
2. The SDK contains hardcoded CAN-FD timing assumptions that do not match the current SocketCAN interface.
3. The SDK is sending requests, but the hand cannot decode them due to data-phase bitrate mismatch.
4. The SDK generates request frames in userspace, but the Jetson SocketCAN path is still not producing observable bus traffic on `can_omnihand`; this must be verified with kernel counters and error frames, not only SDK logs.
5. Less likely, but still possible: physical CAN-FD wiring, termination, power, or hand-side mode is incorrect.

## Recommendation

The next useful step is to investigate whether the vendor SDK allows changing the CAN-FD data bitrate or whether the 4 Mbit/s data phase is hardcoded.

Changing only the Jetson interface is not sufficient unless the hand/SDK side uses the same data bitrate. Both sides must agree on:

```text
arbitration bitrate
data bitrate
arbitration sample point
data sample point
CAN FD frame format
```

Therefore, the next phase should remain below ROS and focus only on SDK plus SocketCAN validation.

## Proposed next tasks

### 0. Reconfigure candidate timings through the repo-owned role script

Before trying more SDK changes, test candidate Jetson-compatible timings without editing the repo baseline:

```bash
cd ~/workspace/agx_arm_ros

python3 scripts/prepare_can_interfaces.py --roles omnihand \
  --omnihand-can-interface can_omnihand \
  --omnihand-bitrate 1000000 \
  --omnihand-dbitrate 2000000 \
  --omnihand-sample-point 0.75 \
  --omnihand-dsample-point 0.75
```

Swap only the timing arguments when trying additional candidate values such as `3846153` or `2500000`.

### 1. Confirm whether the SDK sends frames on `can_omnihand`

Run in terminal 1:

```bash
candump -tz -x can_omnihand
```

Run in terminal 2:

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK

PYTHONPATH=$PWD/build_phase1_socket/omnihand_2025_pkg \
LD_LIBRARY_PATH=$PWD/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_omnihand \
python3.10 python/example/demo_get_hardware_info.py
```

Interpretation:

| Result | Meaning |
|---|---|
| No frames in `candump` | SDK is not sending through the expected SocketCAN interface. |
| TX frames only | SDK sends, but the hand does not answer. Likely bitrate, wiring, power, or protocol mismatch. |
| TX and RX frames | Bus is alive; SDK may fail to parse or validate responses. |
| Error frames | Physical layer, bitrate, dbitrate, or termination issue. |

### 2. Search the SDK for hardcoded bitrate settings

From repo root:

```bash
cd ~/workspace/agx_arm_ros

grep -Rni \
  "4000000\|4M\|4mbps\|dbitrate\|data.*bitrate\|bitrate\|CANFD\|canfd\|125K\|1000K\|1000000" \
  vendor/Omnihand-2025-SDK
```

Also search generated and build directories:

```bash
grep -Rni \
  "4000000\|3846153\|1000000\|125000\|CANFD\|dbitrate" \
  vendor/Omnihand-2025-SDK/build_phase1_socket \
  vendor/Omnihand-2025-SDK/python \
  vendor/Omnihand-2025-SDK/src \
  vendor/Omnihand-2025-SDK/include 2>/dev/null
```

### 3. Search SocketCAN backend implementation

```bash
grep -Rni \
  "socketcan\|PF_CAN\|AF_CAN\|CAN_RAW\|CANFD_MTU\|setsockopt\|bind(" \
  vendor/Omnihand-2025-SDK
```

Relevant findings would include:

- interface name selection,
- CAN-FD frame size handling,
- bitrate assumptions,
- device query IDs,
- request timeout logic,
- CAN-FD enable flags,
- hardcoded arbitration/data bitrate tables.

### 4. Test Jetson-compatible data bitrates only if the SDK/hand can match them

Candidate values that are more compatible with the Jetson `50 MHz` CAN clock include:

```text
1000000 / 3846153
1000000 / 2500000
1000000 / 2000000
1000000 / 1000000
```

Example for `2 Mbit/s` data phase:

```bash
sudo ip link set can_omnihand down
sudo ip link set can_omnihand type can \
  bitrate 1000000 sample-point 0.75 \
  dbitrate 2000000 dsample-point 0.75 \
  fd on restart-ms 100 berr-reporting on
sudo ip link set can_omnihand up

ip -details link show can_omnihand
```

Then rerun the SDK smoke test.

Do not treat this as a valid test unless the SDK or hand is also configured for the same data bitrate.

## Possible implementation direction inside the SDK

If the SDK has a hardcoded data bitrate value, patch it to read runtime configuration from environment variables, similar to the existing `OMNIHAND_SOCKETCAN_IFACE` patch.

Suggested environment variables:

```text
OMNIHAND_SOCKETCAN_IFACE=can_omnihand
OMNIHAND_CAN_BITRATE=1000000
OMNIHAND_CAN_DBITRATE=3846153
OMNIHAND_CAN_SAMPLE_POINT=0.75
OMNIHAND_CAN_DSAMPLE_POINT=0.692
```

However, this only helps if the SDK actually configures the hand or the adapter. If the hand firmware itself is fixed to 4 Mbit/s, then the SDK patch alone will not solve communication.

## Fallback paths if SDK bitrate cannot be changed

If the SDK or hand is fixed to `dbitrate=4000000`, the native Jetson path may not be sufficient with the current `50 MHz` mttcan clock.

Fallback options:

1. Use a CAN-FD adapter that can generate exact `4000000` data bitrate.
2. Use the Agibot-recommended ZLG USBCANFD adapter path.
3. Investigate changing the Jetson mttcan clock so exact 4 Mbit/s is representable.
4. Request an official Agibot/Supplier configuration option for alternative CAN-FD data bitrates on Linux/aarch64.

## Current decision

Proceed with native Jetson CAN-FD validation, but keep it below ROS.

The immediate next goal is not ROS integration. The immediate goal is:

```text
SDK smoke test can read live OmniHand hardware info over can_omnihand.
```

Only after that succeeds should ROS bridge integration continue.

## Acceptance criteria

The native Jetson CAN-FD path is accepted only when all of the following are true:

- `ip -details link show can_omnihand` reports `mtu 72` and CAN FD enabled.
- `candump` confirms SDK requests are sent on `can_omnihand`.
- The hand sends valid response frames.
- `demo_get_hardware_info.py` returns non-default product, serial, hardware, software, and voltage values.
- The chosen arbitration bitrate and data bitrate are documented and reproducible.

Until then, the OmniHand path remains a CAN-FD bring-up issue, not a ROS issue.
