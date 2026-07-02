# Proposal: Dual-CAN Bring-up for AgileX Nero Arm and AgiBot OmniHand Pro

> Historical proposal: kept for Sprint 5 design context and transport investigation history.
> Current operational bringup lives in `docs/control/bringup.md` and `docs/assets/omnihand/omnihand_canfd_setup.md`.

## Context

The current host is a Jetson/Ubuntu 22.04 system running an NVIDIA Tegra kernel. The AgileX Nero arm now works over SocketCAN after enabling the missing `gs_usb` kernel driver for the supplied USB-CAN adapter. The AgiBot OmniHand Pro is not yet usable through the same path because the current adapter/driver binding exposes only a classic CAN interface, while the OmniHand SDK path requires CAN FD frames.

This proposal defines a dual-CAN architecture:

- **Nero arm:** keep the existing working classic CAN setup.
- **OmniHand Pro:** isolate on a dedicated CAN FD-capable interface and validate below ROS first using the vendor SDK / SocketCAN path.
- **ROS bridge:** only integrate after the hand interface has proven live CAN FD traffic.

The key design principle is boring but important: **do not debug ROS until the bus can demonstrably exchange the required frame type**. Otherwise we are just arguing with middleware while the electrons laugh quietly.

---

## Current Observations

### Nero arm

The Nero path is operational after enabling `gs_usb` support in the Jetson kernel.

Observed working direction:

```bash
bash can_activate.sh can_nero3 1000000 "1-4.3:1.0"
```

The system can then expose the adapter as a SocketCAN interface, for example:

```text
Interface can_nero3 is connected to USB port 1-4.3:1.0
```

The Nero arm uses classic CAN in the current working setup.

### OmniHand Pro

The OmniHand bring-up is blocked at the Linux CAN interface layer.

Known facts from the current host:

```text
USB CANFD adapter can be forced-bound through gs_usb as can3
USB ID: a8fa:8598
USB port: 1-4.3:1.0
SocketCAN interface exists, but only as classic CAN
ip link reports: mtu 16
No dbitrate field is available
ip link set can3 type can fd on ... returns: Operation not supported
```

For SocketCAN, this is decisive:

- `mtu 16` means classic CAN.
- `mtu 72` means CAN FD-capable interface.
- `fd on` requires the driver/controller to expose CAN FD support.

Therefore, the current forced `gs_usb` path is insufficient for the OmniHand SDK if that SDK uses CAN FD frames.

---

## Technical Basis

Linux SocketCAN distinguishes classic CAN and CAN FD netdevices by MTU:

```text
CAN_MTU   = 16  -> classic CAN
CANFD_MTU = 72  -> CAN FD
```

A CAN FD-capable interface also accepts `dbitrate` and `fd on`, for example:

```bash
sudo ip link set canX up type can bitrate 1000000 dbitrate 5000000 fd on
```

If the command returns `Operation not supported`, the kernel driver or the adapter firmware is not exposing CAN FD support for that netdevice.

The `gs_usb` driver supports Geschwister Schneider / candleLight-compatible USB CAN adapters. However, an individual adapter must still expose CAN FD capability through the expected firmware protocol and the Linux driver must recognize that capability. A forced USB-ID bind is not proof of CAN FD support.

---

## Target Architecture

```text
Jetson host
├── can_nero
│   ├── Adapter: AgileX / GS USB classic CAN adapter
│   ├── Driver: gs_usb
│   ├── Mode: classic CAN
│   ├── Bitrate: 1000000
│   └── Consumer: agx_arm_ros / pyAgxArm / Nero
│
└── can_omnihand
    ├── Adapter: verified CAN FD-capable adapter
    ├── Driver: CAN FD-capable SocketCAN driver or vendor driver
    ├── Mode: CAN FD
    ├── Bitrate: to be confirmed from vendor SDK/docs
    ├── Data bitrate: to be confirmed from vendor SDK/docs
    └── Consumer: isolated OmniHand vendor SDK first, ROS bridge later
```

The two buses must be physically and logically independent. The Nero arm should not share a bus with the hand during first validation.

---

## Proposed Solution Options

### Option A — Recommended: Use a known SocketCAN CAN FD adapter for OmniHand

Use a dedicated USB-CAN FD adapter that exposes a real CAN FD-capable SocketCAN interface.

Acceptance criteria:

```bash
ip -details link show can_omnihand
```

must show:

```text
mtu 72
can <FD>
dbitrate ...
```

A valid configuration command should look like this, using vendor-confirmed rates:

```bash
sudo ip link set can_omnihand down
sudo ip link set can_omnihand type can bitrate <ARB_BITRATE> dbitrate <DATA_BITRATE> fd on restart-ms 100
sudo ip link set can_omnihand txqueuelen 256
sudo ip link set can_omnihand up
```

Candidate adapter families to evaluate:

| Adapter family | Linux path | Notes |
|---|---|---|
| PEAK PCAN-USB FD | `peak_usb` / SocketCAN | Good candidate if driver exists in the current Jetson kernel or can be installed cleanly |
| Kvaser CAN FD | `kvaser_usb` / SocketCAN or vendor stack | Good candidate if available; verify SocketCAN FD behavior |
| candleLight FD-compatible | `gs_usb` / SocketCAN | Only acceptable if the exact firmware/device exposes `mtu 72` and accepts `fd on` |
| Vendor-recommended OmniHand adapter | vendor SDK path | Preferred if the OmniHand SDK expects a specific adapter or protocol |

This is the lowest-risk path because it avoids fighting a forced `gs_usb` binding that already proved classic-only on this host.

### Option B — Use the vendor-supported driver/backend for the OmniHand adapter

If the OmniHand adapter `a8fa:8598` is not meant to be driven by upstream `gs_usb`, use the vendor driver or SDK backend instead of SocketCAN.

This should be evaluated if:

```text
a8fa:8598 is a vendor-specific CAN FD adapter
forced gs_usb binding exposes only mtu 16
vendor SDK contains adapter-specific USB, serial, or libusb code
```

Validation should happen with a vendor-provided diagnostic tool or minimal SDK sample before ROS integration.

### Option C — Patch/backport `gs_usb` support for this adapter

This is only sensible if the adapter is actually compatible with the `gs_usb` protocol and its firmware supports CAN FD, but the current kernel driver does not recognize the device or capability correctly.

Required evidence before attempting this:

```text
Vendor confirms a8fa:8598 is gs_usb/candleLight FD-compatible
Adapter firmware advertises CAN FD support
Upstream or vendor gs_usb patch exists for this USB ID/device
```

Without that evidence, this option is a time sink with decorative Makefiles.

### Option D — Use a second host for first OmniHand validation

Use a standard x86 Ubuntu machine with a known CAN FD adapter and validate the OmniHand SDK there first.

This is useful if Jetson driver work blocks progress. Once the vendor SDK is proven with live traffic, port the known-good CAN FD setup back to the Jetson.

---

## Recommended Path

The recommended path is:

1. Keep the Nero arm on the now-working classic CAN `gs_usb` path.
2. Do not reuse the forced-bound `a8fa:8598` interface for the OmniHand unless it can expose `mtu 72`.
3. Bring up a dedicated CAN FD adapter for the OmniHand.
4. Validate the OmniHand below ROS using `candump`, `cansend`, and the patched vendor SDK.
5. Only then connect the live hand SDK into the ROS bridge.

This avoids destabilizing the working Nero setup while isolating the still-unproven CAN FD path.

---

## Validation Plan

### Phase 1 — Preserve Nero CAN

Document the working Nero adapter mapping:

```bash
cd ~/workspace/agx_arm_ws/src/agx_arm_ros/scripts
bash find_all_can_port.sh
```

Expected example:

```text
Interface can_nero3 is connected to USB port 1-4.3:1.0
```

Activate Nero:

```bash
bash can_activate.sh can_nero 1000000 "<NERO_USB_PORT>"
```

Verify:

```bash
ip -details -statistics link show can_nero
```

Expected:

```text
mtu 16
bitrate 1000000
state ERROR-ACTIVE or UNKNOWN without growing error counters
```

Launch Nero:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/agx_arm_ws/install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=none \
  tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]' \
  follow:=true \
  control:=false
```

### Phase 2 — Bring up OmniHand CAN FD interface

The OmniHand interface must not be accepted unless this works:

```bash
sudo ip link set can_omnihand down || true
sudo ip link set can_omnihand type can bitrate <ARB_BITRATE> dbitrate <DATA_BITRATE> fd on restart-ms 100
sudo ip link set can_omnihand txqueuelen 256
sudo ip link set can_omnihand up

ip -details link show can_omnihand
```

Required output characteristics:

```text
mtu 72
can <FD>
dbitrate <DATA_BITRATE>
```

If output remains:

```text
mtu 16
```

or if configuration fails with:

```text
Operation not supported
```

then that interface is not usable for the current OmniHand SocketCAN SDK path.

### Phase 3 — Passive traffic check

```bash
candump -tz can_omnihand
```

Power-cycle or initialize the hand if the vendor procedure requires it.

Expected:

- Either passive status frames from the hand, or
- no passive frames but no CAN error counter growth.

Check counters:

```bash
ip -details -statistics link show can_omnihand
```

### Phase 4 — Vendor SDK direct validation

The vendor SDK has been patched to read:

```bash
OMNIHAND_SOCKETCAN_IFACE
```

Use it explicitly:

```bash
export OMNIHAND_SOCKETCAN_IFACE=can_omnihand
```

Then run the minimal vendor SDK read/status example first.

Acceptance criteria:

```text
The SDK opens the requested interface
The SDK sends CAN FD frames without local SocketCAN errors
The hand returns valid state/status/version data
No growing tx/rx error counters
No bus-off
```

### Phase 5 — ROS bridge integration

Only after the SDK path is validated:

- replace mock-only bridge behavior with the vendor SDK backend,
- expose hand state as ROS topics,
- add command topics/services,
- add launch parameter for `omnihand_can_port`,
- document required CAN FD bitrates.

---

## Reproducible Nero Kernel Upgrade: `gs_usb` on Jetson Linux R36.2

This section documents the kernel/module work that enabled the Nero USB-CAN adapter.

### Host baseline

Observed host baseline:

```bash
cat /etc/nv_tegra_release
```

```text
# R36 (release), REVISION: 2.0
```

```bash
dpkg -l | grep nvidia-l4t-core
```

```text
nvidia-l4t-core 36.2.0-20231218214829
```

```bash
uname -r
```

```text
5.15.122-tegra
```

Before the upgrade:

```bash
zcat /proc/config.gz | grep CONFIG_CAN_GS_USB
```

```text
# CONFIG_CAN_GS_USB is not set
```

```bash
sudo modprobe gs_usb
```

```text
modprobe: FATAL: Module gs_usb not found in directory /lib/modules/5.15.122-tegra
```

### Required source package

For Jetson Linux R36.2, use the matching NVIDIA Driver Package BSP sources:

```text
Jetson Linux 36.2
Driver Package (BSP) Sources
public_sources.tbz2
```

NVIDIA documents that the Jetson Linux BSP includes the Linux kernel and that Jetson Linux 36.2 uses Linux kernel 5.15 with Ubuntu 22.04. NVIDIA also documents that kernel sources can be obtained either through Git sync or by manually downloading and extracting the release source files.

### Install build tools

```bash
sudo apt update
sudo apt install -y \
  git \
  build-essential \
  bc \
  flex \
  bison \
  libssl-dev \
  libncurses5-dev \
  dwarves \
  pahole \
  rsync
```

### Prepare source tree

Assuming `public_sources.tbz2` has been downloaded to `~/Downloads`:

```bash
mkdir -p ~/nvidia_kernel/Linux_for_Tegra
cd ~/nvidia_kernel

tar xf ~/Downloads/public_sources.tbz2 -C ~/nvidia_kernel
find ~/nvidia_kernel -name "kernel_src.tbz2"
```

Extract kernel sources:

```bash
cd ~/nvidia_kernel/Linux_for_Tegra/source
tar xf kernel_src.tbz2
```

Expected kernel source path:

```bash
~/nvidia_kernel/Linux_for_Tegra/source/kernel/kernel-jammy-src
```

### Configure `gs_usb`

```bash
cd ~/nvidia_kernel/Linux_for_Tegra/source/kernel/kernel-jammy-src
zcat /proc/config.gz > .config
```

Ensure local version matches the running kernel:

```bash
grep LOCALVERSION .config
```

If needed:

```bash
scripts/config --set-str LOCALVERSION "-tegra"
```

Enable `gs_usb` as a module:

```bash
chmod +x scripts/config
scripts/config --module CONFIG_CAN_GS_USB
grep CONFIG_CAN_GS_USB .config
```

Expected:

```text
CONFIG_CAN_GS_USB=m
```

### Build the module

```bash
make olddefconfig
make prepare
make modules_prepare
make M=drivers/net/can/usb modules
```

Verify:

```bash
find . -name "gs_usb.ko"
modinfo drivers/net/can/usb/gs_usb.ko | grep vermagic
uname -r
```

Expected `vermagic` should match:

```text
5.15.122-tegra
```

### Install the module

```bash
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/net/can/usb
sudo cp drivers/net/can/usb/gs_usb.ko /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/
sudo depmod -a
sudo modprobe gs_usb
```

Verify:

```bash
lsmod | grep gs_usb
find /lib/modules/$(uname -r) -name "gs_usb.ko*"
```

### Validate adapter enumeration

Reconnect the USB-CAN adapter:

```bash
lsusb
ip link show
```

Expected for the Nero adapter family:

```text
1d50:606f OpenMoko / Geschwister Schneider CAN adapter
```

The adapter should create a SocketCAN interface such as:

```text
can0
```

Then use the AgileX helper:

```bash
cd ~/workspace/agx_arm_ws/src/agx_arm_ros/scripts
bash find_all_can_port.sh
```

Expected example:

```text
Interface can0 is connected to USB port 1-4.3:1.0
```

Activate:

```bash
bash can_activate.sh can_nero 1000000 "1-4.3:1.0"
```

Check:

```bash
ip -details -statistics link show can_nero
```

---

## Proposed Repository Changes

### 1. Add persistent CAN naming documentation

Create a local setup document:

```text
docs/can/dual_can_setup.md
```

Include:

- physical USB port mapping,
- adapter USB IDs,
- CAN role assignment,
- expected bitrates,
- expected interface names.

### 2. Add udev rules for stable naming

Example concept:

```udev
# Nero classic CAN adapter
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", NAME="can_nero"

# OmniHand CAN FD adapter
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="<vendor>", ATTRS{idProduct}=="<product>", NAME="can_omnihand"
```

Caution: use tested matching attributes from:

```bash
udevadm info -a -p $(udevadm info -q path -n canX)
```

Do not blindly copy the rule above; USB CAN adapters can expose attributes at different parent levels. Because of course they can.

### 3. Add CAN preflight script

Create:

```text
scripts/check_dual_can.sh
```

Responsibilities:

```text
- verify can_nero exists
- verify can_nero has mtu 16
- verify can_nero bitrate is 1000000
- verify can_omnihand exists
- verify can_omnihand has mtu 72
- verify can_omnihand has expected arbitration bitrate
- verify can_omnihand has expected data bitrate
- fail clearly if CAN FD is not active
```

### 4. Add OmniHand SDK live validation script

Create:

```text
scripts/validate_omnihand_socketcan.sh
```

Responsibilities:

```bash
export OMNIHAND_SOCKETCAN_IFACE=can_omnihand
```

Then run the minimal SDK status/version call.

### 5. Gate ROS bridge launch on CAN FD readiness

The ROS launch path should not start the OmniHand bridge if:

```text
can_omnihand is missing
can_omnihand has mtu 16
can_omnihand lacks dbitrate
ip reports no <FD> mode
```

Failing fast here is better than a mock bridge pretending to be useful. A fake green light is still red, just dressed for a meeting.

---

## Open Questions

1. What exact CAN FD arbitration bitrate and data bitrate does OmniHand Pro require?
2. Does the vendor SDK require ISO CAN FD or non-ISO CAN FD?
3. Is `a8fa:8598` intended to be a SocketCAN adapter, a vendor-specific USB protocol device, or a gs_usb-compatible device?
4. Is the current OmniHand adapter firmware CAN FD capable, or only the hardware is CAN FD capable?
5. Does the vendor provide a known-good Linux adapter/driver combination?
6. Does the vendor SDK use raw SocketCAN CAN FD frames or a higher-level transport?

---

## Decision Gate

The OmniHand path is considered ready for ROS integration only when all of the following are true:

```text
can_omnihand exists
can_omnihand has mtu 72
ip link accepts fd on and dbitrate
candump can_omnihand works without error counter growth
vendor SDK can read hand status/version
vendor SDK can send at least one safe no-motion command
```

Until then, the correct working layer is Linux driver + SocketCAN + vendor SDK, not ROS.

---

## References

- NVIDIA Jetson Linux 36.2 release page: https://developer.nvidia.com/embedded/jetson-linux-r362
- NVIDIA Jetson Linux R36.2 Kernel Customization Guide: https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/SD/Kernel/KernelCustomization.html
- Linux SocketCAN documentation: https://www.kernel.org/doc/html/latest/networking/can.html
- Linux `gs_usb` driver source: https://codebrowser.dev/linux/linux/drivers/net/can/usb/gs_usb.c.html
- Linux Kernel Driver Database entry for `CONFIG_CAN_GS_USB`: https://cateee.net/lkddb/web-lkddb/CAN_GS_USB.html
