# Proposal: OmniHand CAN-FD Bitrate Validation on Jetson Native mttcan

> ⚠️ **SUPERSEDED (sprint5).** The validated standard is `bitrate 1M / sample-point 0.8 /
> dbitrate 5M / dsample-point 0.8 / fd on` on native `mttcan` with the TJA1051T/3 (Adafruit CAN
> Pal) transceiver. FD+BRS works once the TDC offset is set via sysfs before bring-up:
> `echo "0x800" | sudo tee .../tdc_offset`. The bitrate sweeps in this document never resolved
> BRS failures because the missing TDCR step was the actual blocker, not the data bitrate.
> Current setup: `docs/assets/omnihand/omnihand_canfd_setup.md`. Kept for history.

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
cd ~/workspace/agx_arm_ros/vendor/OmniHand-Pro-2025

PYTHONPATH=$PWD/build/agibot_hand_pkg \
LD_LIBRARY_PATH=$PWD/build/agibot_hand_pkg/agibot_hand:$LD_LIBRARY_PATH \
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

## Superseding update: exact 5 Mbit/s is available, but the external bus path is still dead

The earlier `4 Mbit/s` mismatch hypothesis is no longer the leading explanation on this host.

Current validated state:

- the vendor SDK source clearly targets a `1 Mbit/s` arbitration phase with a `5 Mbit/s` CAN FD data phase,
- Jetson native `mttcan` on this host accepts that timing exactly, including `sample-point 0.800` and `dsample-point 0.800`,
- external-bus tests on both `can0` and `can1` still show `TX packets 0`, `RX packets 0`, SDK timeouts, and no `candump` traffic,
- but an internal `loopback on` test on `can0` successfully transmits and receives a CAN FD frame, increments both `TX` and `RX` counters, and is visible in `candump`.

Interpretation:

- the local Jetson `mttcan` controller path and SocketCAN userspace path are functional,
- the current blocker is no longer the SDK send path or the inability to realize the vendor bitrate,
- the remaining fault is on the external path between the Jetson controller and the OmniHand bus.

## Latest validation on this host

Additional validation on the current Jetson host now shows:

- the SDK SocketCAN path does generate request frames locally; enabling `hand.show_data_details(True)` prints `SND` frames for `0x00010101` and `0x00020101`,
- the repo-local SocketCAN backend has been aligned with the vendor ZLG backend by enabling CAN FD bit-rate switching on transmit,
- but `demo_get_hardware_info.py` still times out, `candump` on `can_omnihand` stays silent, and `ip -details -statistics link show can_omnihand` remains at `TX packets 0` with accumulated `bus-errors` and automatic restarts.

Newer Jetson-native tests on the physical `can0`/`can1` path sharpen that result further:

- after rewiring `can0_in` to RX and `can0_out` to TX, both controllers can be configured to exact `1 Mbit/s / 5 Mbit/s`,
- `cansend can0 123##1551122334455667788` and the same test on `can1` still leave `TX packets 0` and `RX packets 0`,
- the SDK smoke test on `can0` and `can1` still times out with only local `SND` logs,
- switching `can0` into internal `loopback on` mode produces a valid local receive in `candump` and increments `TX` plus `RX` counters.

### Additional timing candidates tested after this proposal

The following candidate data-phase bitrates were applied successfully on the native Jetson `mttcan` interface and re-tested with the SDK smoke test:

| Requested dbitrate | Reported interface state | SDK result | Bus observation |
|---|---|---|---|
| `3846153` | `dbitrate 3846153 dsample-point 0.692` | timeout | `TX packets 0`, `candump` silent, error counters rise |
| `2000000` | `dbitrate 2000000 dsample-point 0.720` | timeout | `TX packets 0`, `candump` silent, error counters rise |
| `2500000` | `dbitrate 2500000 dsample-point 0.750` | timeout | `TX packets 0`, `candump` silent, error counters rise |
| `1000000` | `dbitrate 1000000 dsample-point 0.720` | timeout | `TX packets 0`, `candump` silent, error counters rise |

This narrows the current Jetson-native failure further:

- the link does not start working merely by moving to a Jetson-friendly CAN FD data bitrate,
- the transport remains stuck before any successful SDK-visible exchange,
- and the next likely discriminator is not another arbitrary `dbitrate`, but a transport-path change or a hardware-side confirmation of the hand's expected CAN FD timing.

Interpretation:

- missing CAN FD bit-rate switching in the SocketCAN path was not the only blocker,
- the current failure still points below ROS and below the Python demo layer,
- CAN FD timing mismatch is no longer the main suspect on this host,
- and the remaining suspects are now concentrated on the external electrical path: transceiver presence or enable, board-level routing, external wiring, termination, power, or hand-side mode.

## Historical note: earlier 4 Mbit/s mismatch hypothesis

An earlier hypothesis was that the vendor path expected a CAN-FD data bitrate of `4000000` bit/s.

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

That observation remains mathematically correct, but it is no longer the leading blocker because the vendor source and the newer host tests both support exact `5 Mbit/s` operation on this Jetson path.

## Current technical finding: exact 5 Mbit/s data phase is available on Jetson native mttcan

The vendor SDK now points much more strongly at a `5 Mbit/s` data phase than at `4 Mbit/s`:

- the ZLG backend initializes the data-domain timing block as `5M`,
- the vendored SocketCAN helper macro also uses `dbitrate 5000000`.

On this Jetson host, the native controller accepts that timing exactly:

```text
bitrate 1000000 sample-point 0.800
dbitrate 5000000 dsample-point 0.800
```

This means the native Jetson path is no longer blocked by the inability to realize the vendor CAN FD rate.

## Working hypothesis

The current failure is most likely caused by one of the following:

1. The Jetson local controller path works, but the external CAN electrical path is still not complete or not active.
2. A CAN transceiver, transceiver-enable signal, or board-level routing between Jetson `mttcan` and the physical CAN bus is missing or incorrect.
3. The hand-side bus conditions are still wrong: termination, power, bus polarity at the transceiver side, or required hand-side mode.
4. Less likely now: a remaining SDK-level protocol mismatch after the external bus starts carrying frames.

## Recommendation

The next useful step is no longer another arbitrary `dbitrate` sweep inside Jetson `mttcan`.

The next useful step is to validate the external CAN path between Jetson and OmniHand.

The current evidence already shows that:

- the SDK can generate CAN FD requests,
- Jetson `mttcan` can realize the vendor `1M/5M` timing,
- and the controller can send and receive internally in loopback mode.

What is still missing is proof that frames leave the Jetson over the real external bus and reach the hand.

Both sides still must agree on:

```text
arbitration bitrate
data bitrate
arbitration sample point
data sample point
CAN FD frame format
```

Therefore, the next phase should remain below ROS and focus on hardware-path validation rather than more SDK timing edits.

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
cd ~/workspace/agx_arm_ros/vendor/OmniHand-Pro-2025

PYTHONPATH=$PWD/build/agibot_hand_pkg \
LD_LIBRARY_PATH=$PWD/build/agibot_hand_pkg/agibot_hand:$LD_LIBRARY_PATH \
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
  vendor/OmniHand-Pro-2025
```

Also search generated and build directories:

```bash
grep -Rni \
  "4000000\|3846153\|1000000\|125000\|CANFD\|dbitrate" \
  vendor/OmniHand-Pro-2025/build \
  vendor/OmniHand-Pro-2025/python \
  vendor/OmniHand-Pro-2025/src \
  vendor/OmniHand-Pro-2025/include 2>/dev/null
```

### 3. Search SocketCAN backend implementation

```bash
grep -Rni \
  "socketcan\|PF_CAN\|AF_CAN\|CAN_RAW\|CANFD_MTU\|setsockopt\|bind(" \
  vendor/OmniHand-Pro-2025
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

Current empirical result on this host: all four of these pairings still fail the SDK hardware-info smoke test.

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
