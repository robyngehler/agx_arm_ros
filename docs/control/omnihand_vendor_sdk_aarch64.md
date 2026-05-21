# OmniHand Vendor SDK On Jetson aarch64

status: ACTIVE_BASELINE
target_host: Jetson Ubuntu 22.04 aarch64
target_kernel: Linux 5.15 tegra

## Purpose

This note defines the repo-local OmniHand SDK baseline for Jetson-class `aarch64` hosts.

It exists to keep the local runtime path explicit and to avoid mixing:

- the upstream vendor x86_64 userspace path,
- partial ZLG-only assumptions,
- and the repo-local SocketCAN-based Jetson bring-up path.

## Local SDK Baseline

The repo-local OmniHand SDK baseline on Jetson is:

- build the vendored SDK with `OMNIHAND_CAN_DRIVER=socket`,
- use a real Linux CAN FD netdevice as the hardware transport,
- select the interface with `OMNIHAND_SOCKETCAN_IFACE`,
- and do not use the bundled `thirdParty/usbcanfd_libusb_x64_1.0.10_250328` package as the default backend on `aarch64`.

Why:

- the bundled ZLG userspace package is named `..._x64_...`,
- the bundled `libusbcanfd.so` is x86_64,
- and the repo-local Jetson bring-up already succeeds through the SocketCAN backend.

## Backend Policy

### SocketCAN backend

This is the default local backend for Jetson `aarch64`.

Use it when:

- Linux exposes a real CAN FD interface,
- `ip link set ... fd on ...` works,
- and `ip -details link show` reports `mtu 72` and a `dbitrate`.

Build command:

```bash
./build.sh -DCMAKE_BUILD_TYPE=Release \
           -DCMAKE_INSTALL_PREFIX=./build/install \
           -DBUILD_PYTHON_BINDING=ON \
           -DBUILD_CPP_EXAMPLES=OFF \
           -DBUILD_ROS_NODE=OFF \
           -DOMNIHAND_CAN_DRIVER=socket
```

Runtime selector:

```bash
export OMNIHAND_SOCKETCAN_IFACE=can_omnihand
```

### ZLG userspace backend

This is not the default local backend on Jetson.

Use it only when:

- a native `aarch64` ZLG SDK is available,
- that SDK contains `zcan.h` and `libusbcanfd.so`,
- and it is passed explicitly with `-DOMNIHAND_ZLG_SDK_PATH=/path/to/sdk`.

If no native `aarch64` package exists, do not use `OMNIHAND_CAN_DRIVER=zlg` on Jetson.

## Adapter Matrix For Jetson

The vendor README recommends:

- `USBCANFD-100U-mini`
- `USBCANFD-100U`
- `USBCANFD-200U`

Repo-local Jetson status:

| Adapter family | Current Jetson/aarch64 status | Notes |
| --- | --- | --- |
| `USBCANFD-100U-mini` | UNVERIFIED | Vendor recommends it, but the repo-local bundled ZLG userspace package is x86_64-only. No verified SocketCAN FD path is recorded here yet. |
| `USBCANFD-100U` | UNVERIFIED | Same status as `100U-mini`: recommended upstream, but no verified native `aarch64` userspace path is bundled in this repo. |
| `USBCANFD-200U` | PARTIALLY VERIFIED | The vendored `usbcanfd200_400u_2.10` kernel module builds on Jetson `5.15.122-tegra` and advertises supported USB IDs `04cc:1240` and `3068:0009`. This is the cleanest current vendor-family option for the repo-local SocketCAN baseline. |
| `USBCANFD-400U` | PARTIALLY VERIFIED | Not listed in the vendor README recommendation line, but the vendored kernel module source targets the same family and builds on Jetson. Treat it as potentially viable only when the USB ID matches the module aliases and the interface exposes real CAN FD semantics. |
| Current dongle `a8fa:8598` | REJECTED_FOR_LOCAL_BASELINE | Forced `gs_usb` binding exposed only classic CAN. No verified repo-local Jetson CAN FD path exists for this USB ID. |

Interpretation:

- `USBCANFD-200U` is the strongest current vendor-family candidate for repo-local Jetson bring-up.
- `100U-mini` and `100U` remain possible only if the supplier provides a real native `aarch64` userspace package or another verified Jetson path.

## Kernel-Module Path That Is Already Verified

The following vendored driver source builds successfully on the current Jetson host:

- `vendor/Omnihand-2025-SDK/thirdParty/usbcanfd200_400u_2.10/`

Observed properties after build:

- target kernel: `5.15.122-tegra`
- architecture: `aarch64`
- module aliases: `04cc:1240` and `3068:0009`

This validates the host-side kernel path for supported adapters in that family.

## Acceptance Rules

An adapter is acceptable for the repo-local SocketCAN baseline only when all of the following are true:

- Linux creates a `canX` interface for it,
- `sudo ip link set <iface> type can bitrate <arb> dbitrate <data> fd on` succeeds,
- `ip -details link show <iface>` reports `mtu 72`,
- the OmniHand SDK can open that interface through `OMNIHAND_SOCKETCAN_IFACE`.

If any of those fail, stay below ROS and do not treat the adapter as part of the local supported baseline.