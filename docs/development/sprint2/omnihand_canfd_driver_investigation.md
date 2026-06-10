# OmniHand CAN FD Driver Investigation

## Purpose

This note records the current Sprint 2 evidence for bringing up the real OmniHand Pro on the Jetson host.

It separates:

- what is now verified on this machine,
- what remains only a historical or fallback path,
- and what still blocks promotion into the repo-owned ROS runtime.

## Verified Findings

### Real hardware information can now be fetched below ROS over CAN FD

The current validated probe on this host is:

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK
PYTHONPATH=$PWD/build_phase1_socket/omnihand_2025_pkg \
LD_LIBRARY_PATH=$PWD/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can0 \
python3.10 python/example/demo_get_hardware_info.py
```

Observed live output summary:

- `Product Model: OmniHand Pro`
- `Serial Number: R302602032030`
- `Hardware Version: 1.1.1`
- `Software Version: 1.2.15`
- `Supply Voltage: 24000mV`
- `Active Degrees of Freedom: 12`
- `Device ID: 1`
- `Arbitration Bitrate: 1Mbps`
- `Arbitration Sample Point: 80.0%`
- `Data Bitrate: 5Mbps`
- `Data Sample Point: 80.0%`

Interpretation:

- the current Jetson SocketCAN path on `can0` is no longer blocked at the Linux transport or device-enumeration layer,
- the vendor Python example can reach the real hand on this host,
- and the repo-local `aarch64` socket-backed SDK path is now validated for hardware-info retrieval.

### The repo-local low-risk runtime path remains the patched SocketCAN backend

The vendor SocketCAN backend in this repo honors `OMNIHAND_SOCKETCAN_IFACE` instead of assuming `can0`.

That remains the cleanest local host path on Jetson:

1. expose or configure a real CAN FD-capable Linux `canX` interface,
2. verify it accepts `fd on` and reports `mtu 72`,
3. point the SDK at that interface with `OMNIHAND_SOCKETCAN_IFACE`,
4. validate with `demo_get_hardware_info.py` before any ROS-side testing.

### Historical note: the forced `gs_usb` path for `a8fa:8598` is still not a baseline path

- The currently attached OmniHand dongle enumerates as `a8fa:8598`.
- `lsusb -v` reports a vendor-specific USB interface class, not a standard gs_usb-style CAN device.
- Forcing `a8fa:8598` into `gs_usb` creates a `canX` interface, but only as classic CAN.
- On this host the forced interface reports `mtu 16`, exposes no `dbitrate`, and rejects `ip link set ... fd on ...` with `Operation not supported`.

That old forced `gs_usb` path is still not suitable as the repo-local CAN FD baseline.

### The vendored ZLG kernel module does build on this Jetson

The repo contains a vendored CAN FD kernel module source at:

- `vendor/Omnihand-2025-SDK/thirdParty/usbcanfd200_400u_2.10/`

It builds successfully against the current Jetson kernel headers:

```bash
cd vendor/Omnihand-2025-SDK/thirdParty/usbcanfd200_400u_2.10
make module
```

The resulting module advertises:

- `vermagic: 5.15.122-tegra ... aarch64`
- USB aliases for `04cc:1240`
- USB aliases for `3068:0009`

This is a real Jetson-usable path for supported ZLG-style hardware.

### The vendored userspace CAN FD library is not a clean Jetson path

The bundled directory is explicitly:

- `vendor/Omnihand-2025-SDK/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/`

Local `file` checks show:

- `libusbcanfd.so`: `ELF 64-bit ... x86-64`
- bundled `libusb-1.0.so`: `ELF 32-bit ... ARM`

So the packaged userspace ZLG stack is mixed-architecture and is not a trustworthy repo-local install path for Jetson/aarch64.

## Decision

### Current local bring-up path

The current recommended local path on Jetson is now:

1. keep using the socket-backed vendor build on `aarch64`,
2. keep the hand on a real CAN FD SocketCAN interface such as `can0`,
3. validate first with `OMNIHAND_SOCKETCAN_IFACE=<iface> python/example/demo_get_hardware_info.py`,
4. only then promote the same path into repo-owned adapter or ROS-backend work.

What is no longer blocked:

- local build and import of the patched vendor SDK on `aarch64`,
- device enumeration and hardware-info retrieval on the current host,
- confirmation of the live hand's reported `1 Mbps / 5 Mbps` CAN FD timings.

What is still open:

- a safe active-joint command and readback loop on the same validated interface,
- a repo-owned non-mock OmniHand backend in `agx_arm_ctrl`,
- ROS launch validation against the real hand instead of the staged mock bridge.

### Historical fallback path: current adapter `a8fa:8598`

There is no verified repo-local driver installation path for this exact USB ID on Jetson.

The evidence currently supports only two realistic options:

1. obtain a supplier-provided Linux aarch64 driver or library package that explicitly supports `a8fa:8598`, or
2. replace the adapter with a CAN FD adapter already supported by the vendored kernel source or by a known-good SocketCAN FD driver.

Do not continue investing in forced `gs_usb` for `a8fa:8598` unless the supplier explicitly confirms gs_usb/CAN FD compatibility for that exact device and firmware.

### Supported replacement adapter path

The verified replacement path on this host is:

1. use a supported ZLG-style adapter that matches `04cc:1240` or `3068:0009`,
2. build and load `usbcanfd.ko` on Jetson,
3. bring the resulting interface up as CAN FD,
4. run the OmniHand SDK through the patched SocketCAN backend.

This avoids the x86-only bundled userspace library and keeps the runtime on Linux SocketCAN.

## Installation Paths

## Path A: Current validated Jetson SocketCAN path

Use this path first for local hardware work.

### 1. Confirm the interface is real CAN FD

```bash
ip -details link show can0
```

The interface should report a real CAN FD state such as:

- `mtu 72`
- `bitrate 1000000`
- `dbitrate 5000000`
- `sample-point 0.800`
- `dsample-point 0.800`

### 2. Run the validated hardware-info probe

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK
PYTHONPATH=$PWD/build_phase1_socket/omnihand_2025_pkg \
LD_LIBRARY_PATH=$PWD/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can0 \
python3.10 python/example/demo_get_hardware_info.py
```

If this succeeds and returns real model, serial, firmware, voltage, and bitrate information, the host transport path is good enough to proceed to the next below-ROS validation step.

### 3. Keep the next validation below ROS

The next useful discriminator is a safe joint command and readback loop on the same interface.

Do not treat the staged repo-owned ROS bridge as a real-hardware validation path yet; the current ROS bridge remains mock-oriented and is not the proof point for the live hand.

## Path B: Keep the historical `a8fa:8598` adapter

This path is blocked until the supplier provides the correct Linux support package.

Required supplier deliverables:

- exact confirmation that the adapter USB ID is `a8fa:8598`,
- Linux support for Ubuntu 22.04 on aarch64,
- either a kernel driver or a userspace SDK,
- any required udev rules,
- a minimal diagnostic example that can fetch device info from the adapter.

Acceptance checks before using it in this repo:

```bash
lsusb -d a8fa:8598
file <supplier-shared-library>
```

The shared library must be an aarch64 Linux binary, not x86_64.

Only after that should the adapter be validated with:

```bash
ip -details link show <iface>
```

or with the supplier's own diagnostic sample.

If the supplier path does not expose a Linux CAN FD netdevice, keep ROS work blocked and validate the supplier SDK below ROS first.

## Path C: Use a supported ZLG-style CAN FD adapter

This is the current recommended bring-up path on Jetson.

### 1. Install host prerequisites

```bash
sudo apt update
sudo apt install -y build-essential can-utils ethtool linux-headers-$(uname -r)
```

### 2. Build the vendored kernel module

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK/thirdParty/usbcanfd200_400u_2.10
make module
```

### 3. Confirm the module supports your adapter

```bash
modinfo ./usbcanfd.ko | grep '^alias:'
lsusb
```

Your adapter must match one of the advertised USB aliases, currently:

- `04cc:1240`
- `3068:0009`

If it does not, stop there. Do not force-bind an unsupported USB ID into this driver.

### 4. Load the CAN FD module

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK/thirdParty/usbcanfd200_400u_2.10
sudo modprobe can-dev
sudo insmod ./usbcanfd.ko cfg_term_res=1
```

Then verify Linux created CAN interfaces:

```bash
ip -br link show type can
```

### 5. Bring the OmniHand interface up as CAN FD

Use the repo helper:

```bash
cd ~/workspace/agx_arm_ros
bash scripts/omnihand_canfd_activate.sh can_omnihand 1000000 4000000 <iface-or-usb-port>
```

Examples:

```bash
bash scripts/omnihand_canfd_activate.sh can_omnihand 1000000 4000000 can0
bash scripts/omnihand_canfd_activate.sh can_omnihand 1000000 4000000 1-4.3:1.0
```

The interface is acceptable only if this succeeds and `ip -details link show can_omnihand` reports:

- `mtu 72`
- `dbitrate 4000000`
- CAN FD capability

### 6. Run the SocketCAN-based OmniHand smoke test

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK
PYTHONPATH=$PWD/build_phase1_socket/omnihand_2025_pkg \
LD_LIBRARY_PATH=$PWD/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_omnihand \
python3.10 python/example/demo_get_hardware_info.py
```

If import succeeds but communication fails, stay below ROS and debug power, termination, bitrate, dbitrate, and CAN traffic first.

## Parallel Operation Guidance

Parallel operation is not the current blocker.

The recommended split is:

- Nero arm on its existing classic-CAN path,
- OmniHand on its own CAN FD interface.

That is the lowest-risk arrangement because:

- it preserves the working Nero path,
- it isolates hand bring-up noise from the arm bus,
- and it lets the OmniHand SDK be debugged below ROS before any shared launch changes.

## Exit Criteria For ROS Bring-up

Do not move to ROS launch validation until all of the following are true:

- the intended Linux interface is on a verified CAN FD path,
- `ip -details link show` confirms `mtu 72` plus the expected live bitrate state,
- the SDK can fetch live hardware info over that interface,
- and at least one safe hand-side command and readback loop has succeeded below ROS.