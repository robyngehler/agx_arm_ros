# Historical Note: OmniHand CAN FD Transport Bring-up On Jetson

Status: consolidated historical transport note.

Do not use this file as the current bringup guide. The stable operational path lives in
`../../assets/omnihand/omnihand_canfd_setup.md` and the launch surface in
`../../control/bringups/launches.md`.

## What Sprint 2 was trying to answer

The original question was whether the first real OmniHand backend could be validated on Jetson below
ROS before the repo promoted stronger runtime claims.

The investigation intentionally stayed below ROS and focused on:

- Linux CAN FD interface capability
- vendor SDK transport assumptions
- adapter and transceiver constraints
- whether failure was in ROS, Python, driver, or electrical setup

## Historical findings that later mattered

### 1. Forced USB paths were a dead end for the attached adapter

- forcing the attached `a8fa:8598` adapter into `gs_usb` exposed only classic CAN
- the resulting `canX` interface reported `mtu 16` and rejected `fd on`
- this was not a viable baseline for OmniHand Pro CAN FD work

### 2. The vendored ZLG path was only partially useful

- the vendored ZLG kernel module built on Jetson and supported specific USB IDs
- the current adapter was not one of those supported IDs
- the bundled userspace library stack was mixed-architecture and not a clean Jetson baseline

### 3. Native Jetson `mttcan` was not actually the wrong direction

The Sprint 2 proposals temporarily concluded that native `mttcan` failed at CAN FD with BRS.
That conclusion was later proven incomplete.

What was really missing:

- a BRS-capable 5 Mbit transceiver
- the mandatory TDC offset write through sysfs before bringup

### 4. Bitrate sweeps were not the root fix

Several timing candidates were tested, but the real blocker was not the exact `dbitrate` value.
The decisive fix was the TDCR step plus the correct transceiver path, not another bitrate guess.

### 5. The below-ROS validation discipline was still correct

Even though several conclusions were later superseded, the investigation got one important process
decision right:

- validate transport and vendor SDK communication below ROS first
- only promote ROS-side runtime claims after hardware-info retrieval and safe command/readback loops

## Stable outcome that superseded the early proposals

The current validated baseline is now:

- Jetson native `mttcan`
- `bitrate 1M`, `dbitrate 5M`, `fd on`, `one-shot on`
- TJA1051T/3-based transceiver path
- sysfs TDC offset write before bringup
- SocketCAN-backed vendor SDK path on Jetson

That final path is documented in `../../assets/omnihand/omnihand_canfd_setup.md`.

## What historical value remains here

Keep this note only for these lessons:

- why the repo stopped treating forced USB `gs_usb` bringup as a serious path for the attached hand
- why the repo prefers transport debugging below ROS before bridge-level debugging
- why the earlier "native Jetson CAN FD cannot do BRS" conclusion should not be reused without the
  later transceiver and TDCR fixes

If those lessons are fully captured elsewhere later, this file can be deleted too.