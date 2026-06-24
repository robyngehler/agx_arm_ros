# Proposal: OmniHand Pro Bring-up on Jetson Native CAN-FD

> ⚠️ **SUPERSEDED (sprint5).** The conclusion below — "native mttcan cannot do CAN FD with BRS" —
> was a **transceiver/wiring and TDCR limitation**, not a driver limit. Two things were required:
> (1) a BRS-capable 5 Mbit transceiver (Adafruit CAN Pal, TJA1051T/3), and (2) setting the
> mttcan TDC offset via sysfs (`echo "0x800" | sudo tee .../tdc_offset`) **before** bringing up
> the interface. The devmem path (register `0xC310048`) and the custom DTB/extlinux boot-entry
> approach investigated here were both confirmed dead ends — only the sysfs write works. With
> these two fixes, native FD+BRS works and drives the OmniHand; the ZLG adapter was dropped.
> Current setup: `docs/assets/omnihand/omnihand_canfd_setup.md`. Kept for history.

## Summary

Recent tests moved the OmniHand CAN-FD bring-up from a broad software/SDK problem to a much narrower hardware/driver-layer issue.

The Jetson native `mttcan` interfaces are now correctly exposed on the 40-pin header after configuring the Jetson-IO pinmux. Before this, `can0` and `can1` existed as Linux netdevices, but the header pins were still listed as `unused`, so no external CAN activity was possible. After pinmux configuration, bus activity and CAN errors became visible, confirming that the physical pins are now active.

The current best assumption is:

> The Jetson native CAN-FD path works for Classic CAN and CAN-FD without BRS, but fails when CAN-FD Bitrate Switching is enabled. Since the OmniHand SDK sends CAN-FD frames with BRS enabled, the current native Jetson CAN path is not yet suitable for live OmniHand communication.

In other words: the bus is no longer silent. It is now failing specifically at the BRS / data-phase part. Progress, just with the usual embedded-flavored slap in the face.

---

## Known Required OmniHand CAN-FD Parameters

According to the OmniHand manual and vendor SDK behavior, the expected communication mode is:

```text
CAN-FD
29-bit extended identifiers
Arbitration bitrate: 1 Mbps
Arbitration sample point: 80%
Data bitrate: 5 Mbps
Data sample point: 80%
BRS enabled
Default device ID: 0x01
```

The SDK SocketCAN backend also sends frames with:

```cpp
frame.flags = CANFD_BRS;
```

Therefore, BRS support is not optional for the unmodified vendor SDK path.

---

## Latest Findings

### 1. Previous blocker: pinmux was not configured

The Jetson Expansion Header Tool showed the CAN-related header pins as `unused`:

```text
Pin 29: unused  // CAN0_DIN / CAN0_RX
Pin 31: unused  // CAN0_DOUT / CAN0_TX
Pin 33: unused  // CAN1_DOUT / CAN1_TX
Pin 37: unused  // CAN1_DIN / CAN1_RX
```

After configuring these pins through Jetson-IO and rebooting, the CAN pins became physically active. This explains why earlier tests showed valid `can0`/`can1` netdevices but no external bus activity.

### 2. Hardware wiring and termination were corrected

A previous wiring error was found where the transceiver board had `3.3V` and `GND` reversed. After correcting this and trying a new transceiver, the ground reference improved substantially:

```text
Jetson ↔ Transceiver GND: ~0.6 Ω
Jetson ↔ OmniHand GND:    ~2.0 Ω
Transceiver ↔ OmniHand:   ~2.0 Ω
```

The measured CANH-CANL resistance was approximately `60–68 Ω`, which is plausible for two 120 Ω terminations in parallel. Therefore, gross bus termination failure is unlikely.

### 3. Classic CAN now works

Classic CAN transmission works externally:

```bash
cansend can0 123#1122334455667788
```

Observed result:

```text
TX packets increased
candump showed TX frames
```

This confirms:

```text
Jetson CAN pinmux is active
Basic CAN controller path works
Transceiver path is at least partially functional
Classic CAN frames can leave the Jetson
```

### 4. CAN-FD without BRS works

CAN-FD frames without Bitrate Switching also work:

```bash
cansend can0 123##0551122334455667788
```

Observed result:

```text
candump showed TX CAN-FD frame
TX packets increased
```

This confirms:

```text
SocketCAN CAN-FD frame format works
mttcan accepts and emits CAN-FD frames
The issue is not simply “CAN-FD unsupported”
```

### 5. CAN-FD with BRS fails, even at dbitrate = 1 Mbps

When sending BRS-enabled CAN-FD frames:

```bash
cansend can0 123##1551122334455667788
```

with:

```text
bitrate  1000000
sample-point 0.8
dbitrate 1000000
dsample-point 0.8
fd on
```

Observed result:

```text
ERROR-PASSIVE
bus-errors increased heavily
RX errors increased heavily
TX packets did not increase for the BRS frames
```

This is the most important result: BRS fails even when the nominal data bitrate is not faster than the arbitration bitrate.

Therefore, the issue is likely related to BRS mode itself, not only to the 5 Mbps data rate.

### 6. TDC is not available via current iproute2 / mttcan path

The current `ip link set can0 type can help` output does not expose any TDC options:

```text
No tdc-mode / tdco / tdcv / tdcf options available
```

Trying:

```bash
sudo ip link set can0 type can ... fd on tdc-mode auto
```

fails with:

```text
can: unknown option "tdc-mode"
```

So Transmitter Delay Compensation cannot currently be configured through the standard SocketCAN command line on this setup.

---

## Error Case Evaluation

### Case A: SDK or Python environment issue

Status: unlikely.

The problem is reproducible with raw `cansend`, below the SDK and below ROS. Therefore, the vendor Python package is not the root cause of the current physical communication failure.

### Case B: Wrong CAN interface

Status: unlikely.

Both native Jetson interfaces were tested:

```text
can0 -> c310000.mttcan
can1 -> c320000.mttcan
```

Both can be configured as CAN-FD. After pinmux configuration, `can0` produces physical CAN activity. The remaining problem is mode-specific, not interface-discovery related.

### Case C: Wrong CANH/CANL wiring

Status: unlikely but not impossible.

Multiple wiring permutations were tested. Correct wiring according to the manual and observed termination were verified. Classic CAN and CAN-FD without BRS now produce TX frames, which strongly suggests that CANH/CANL are not fundamentally disconnected.

### Case D: Bad termination

Status: unlikely.

Measured CANH-CANL resistance around `60–68 Ω` is plausible for a correctly terminated two-node bus.

### Case E: Ground reference issue

Status: mostly resolved.

Early measurements showed poor ground continuity, but this was traced back to incorrect transceiver power wiring. After correction, GND measurements improved significantly. Grounding should still be kept short and robust, but it is no longer the leading suspect.

### Case F: Transceiver does not support BRS / CAN-FD properly

Status: likely.

Classic CAN works. CAN-FD without BRS works. CAN-FD with BRS fails. This strongly suggests that the current physical transceiver path or wiring quality cannot handle BRS mode correctly.

Even if a transceiver module claims CAN-FD support, loose Dupont wiring, poor layout, weak grounding, or an unsuitable module can still break BRS behavior.

### Case G: Jetson mttcan BRS / TDC limitation

Status: likely or at least unresolved.

The native Jetson `mttcan` driver accepts CAN-FD and BRS frames, but the bus fails under BRS. TDC options are not exposed through the current `iproute2` path. Since BRS fails even at `dbitrate=1M`, this may involve mttcan driver behavior, timing compensation, or transceiver interaction rather than raw data bitrate alone.

### Case H: OmniHand requires a specific vendor adapter path

Status: plausible.

Agibot recommends ZLG USBCANFD devices. The vendor SDK was originally written for the ZLG backend and sets BRS. The native Jetson SocketCAN path is now close, but still fails at the exact feature required by the SDK.

---

## Final Working Assumption

The current blocker is no longer ROS, Python, the SDK import path, CAN pinmux, or basic CAN-FD support.

The best current assumption is:

> The native Jetson `mttcan` + external transceiver setup can transmit Classic CAN and CAN-FD without BRS, but cannot reliably transmit CAN-FD frames with BRS enabled. Since the OmniHand SDK uses CANFD_BRS and the hand expects 1M/5M CAN-FD communication, the current setup cannot yet support live OmniHand communication.

The likely root cause is one of:

```text
1. Current transceiver module / wiring quality is insufficient for BRS.
2. Jetson native mttcan needs TDC/TDCR or driver-level tuning for BRS, but this is not exposed via current iproute2.
3. The vendor-recommended ZLG adapter path handles BRS/5M timing differently and may be required for reliable bring-up.
```

---

## Recommended Next Steps

### 1. Patch SDK SocketCAN backend to disable BRS temporarily

Change:

```cpp
frame.flags = CANFD_BRS;
```

to:

```cpp
frame.flags = 0;
```

Purpose:

```text
Confirm that SDK traffic becomes electrically clean when BRS is disabled.
```

Expected outcome:

```text
No response from OmniHand is still possible, because the hand likely expects BRS.
But bus error behavior should improve if BRS is truly the failure trigger.
```

This is a diagnostic patch, not the final solution.

### 2. Use a known-good CAN-FD BRS-capable adapter

Recommended primary path:

```text
ZLG USBCANFD-100U-mini or USBCANFD-100U
```

Reason:

```text
This is the vendor-recommended adapter family and matches the SDK’s original design path.
```

### 3. If continuing with Jetson-native CAN-FD

Investigate:

```text
mttcan driver-level TDC/TDCR configuration
iproute2/kernel support for TDC options
Jetson-specific CAN-FD BRS examples
Known-good CAN-FD transceiver modules with short wiring
CAN0↔CAN1 test using two high-quality CAN-FD transceivers
```

### 4. Avoid further ROS-level tests for now

ROS integration should remain blocked until the following raw SocketCAN test succeeds:

```bash
cansend can0 00010101##1
```

with:

```text
bitrate  1000000
sample-point 0.8
dbitrate 5000000
dsample-point 0.8
fd on
```

and without causing massive bus errors.

---

## Conclusion

The investigation made substantial progress:

```text
Pinmux fixed
Wiring errors found and corrected
Grounding improved
Classic CAN validated
CAN-FD without BRS validated
BRS isolated as the current failure trigger
```

The remaining blocker is specifically **CAN-FD with BRS on the native Jetson mttcan + external transceiver path**.

For the fastest reliable OmniHand bring-up, the recommended path is to switch to the vendor-recommended ZLG USBCANFD adapter or another known-good Linux CAN-FD adapter with proven BRS support. Continuing with native Jetson CAN is still possible, but it likely requires lower-level mttcan/TDC/transceiver validation rather than further SDK or ROS debugging.
