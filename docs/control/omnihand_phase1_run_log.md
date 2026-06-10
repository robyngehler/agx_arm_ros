# OmniHand Phase 1 Run Log

status: HARDWARE_INFO_CONFIRMED_COMMAND_LOOP_PENDING
last_updated: 2026-06-04
script: scripts/omnihand/phase1_smoke_test.py

## Goal

Capture the first isolated SDK bring-up result before any repo-owned wrapper or ROS bridge is introduced.

## Latest Attempt

- Date: 2026-06-04
- Host: current workspace machine
- Architecture: `aarch64`
- Runtime transport path exercised: socket-backed local vendor build
- Result: the vendor hardware-info example now succeeds on `can0` and returns live OmniHand device information on this host

## Evidence Collected

### Userspace CAN Library Provenance

- Vendored userspace library path: `vendor/Omnihand-2025-SDK/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so`
- Local file inspection result: `ELF 64-bit LSB shared object, x86-64`
- Packaging path: `vendor/Omnihand-2025-SDK/python/CMakeLists.txt` copies that same x64 userspace library into the Python package build output

### Current successful hardware-info probe on this host

Command used:

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK
PYTHONPATH=$PWD/build_phase1_socket/omnihand_2025_pkg \
LD_LIBRARY_PATH=$PWD/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can0 \
python3.10 python/example/demo_get_hardware_info.py
```

Observed result summary:

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

- the repo-local socket-backed SDK path is now validated for live device enumeration on Jetson `aarch64`,
- the current host can retrieve real OmniHand hardware and device metadata over SocketCAN,
- the remaining Phase 1 gap is no longer device identity or basic transport reachability.

### Stock Python SDK Probe On This Host

Command used:

```bash
cd vendor/Omnihand-2025-SDK/python
python3 -c "import sys, platform; print(platform.machine()); sys.path.insert(0, '.'); import omnihand_2025; print('py package import ok')"
```

Observed result:

```text
aarch64
ModuleNotFoundError: No module named 'omnihand_2025.omnihand_2025_core'
```

Interpretation:

- the native Python extension is not available on the current host in the vendored tree
- even if the extension is built later, the vendored userspace CAN library currently present in the repo is x86_64-only
- the stock vendor Python path is therefore blocked on this host without local changes

### Backend Reachability Note

Additional source inspection after the first probe found a vendor-side integration mismatch:

- the C++ source tree includes both `zlg` and `socket` CAN backends
- the Python docs advertise a `cfg_path` parameter
- the actual Python binding in `python/binding.cc` exposes `create_hand(device_id, canfd_id, hand_type)` only
- the factory path in `src/c_agibot_hand_base.cc` constructs `AgibotHandCanO10(canfd_id)` directly
- the reachable constructor path in `src/implementation/c_agibot_hand_can/c_agibot_hand_can.cc` still defaults to `can_driver = "zlg"`

Current implication:

- although `socket_can` source files exist, the current Python-facing bring-up path does not expose a working way to select that backend
- without vendor code changes, the reachable Phase 1 Python path still depends on the ZLG-linked build path

### Local Socket-Backed Build Probe

Local repo changes were applied to continue Phase 1 on `aarch64`:

- the vendored build can now be configured with `-DOMNIHAND_CAN_DRIVER=socket`
- the x64-only ZLG userspace library is no longer linked or copied in that socket-backed mode
- the repo smoke test can target a built `omnihand_2025_pkg` directory directly

Build result:

- socket-backed `omnihand_2025_core` built successfully on `aarch64`
- the unpacked `omnihand_2025_pkg` refresh no longer hard-fails when Python wheel tooling is absent; wheel generation is skipped in that case
- the built package imported successfully from `vendor/Omnihand-2025-SDK/build_phase1_socket/omnihand_2025_pkg`

### Earlier runtime probe after local socket build

Command used:

```bash
python3 scripts/omnihand/phase1_smoke_test.py \
	--hand-type left \
	--canfd-id 0 \
	--sdk-python-dir vendor/Omnihand-2025-SDK/build_phase1_socket/omnihand_2025_pkg \
	--json-output /tmp/omnihand_phase1_socket_probe.json
```

Observed result summary:

- `is_initialized` returned `true`
- the child runtime probe emitted request-send failures on CAN IDs such as `0x00010101`
- no complete 10-joint active vector or 16-joint full-joint vector was returned
- the isolated child process exited with `4` and the parent probe recorded `status: runtime_probe_incomplete`

Interpretation:

- the local host is no longer blocked at build/import time if the SDK is built in socket-backed mode
- the repo-side probe no longer crashes on failed CAN reads; it now records an incomplete runtime result with default vendor/device metadata and empty joint vectors when the device path is not responding
- this earlier result remains useful as evidence for failure handling, but it no longer reflects the current best-known bring-up state on this host
- safe command-response validation still cannot be claimed from this earlier run

## Phase 1 Exit Criteria Status

| Exit Criterion | Current State | Notes |
| --- | --- | --- |
| Device enumeration succeeds | VERIFIED | `demo_get_hardware_info.py` now returns live model, serial, firmware, supply-voltage, and bitrate information over `OMNIHAND_SOCKETCAN_IFACE=can0` on this host. |
| Safe command-response loop succeeds | BLOCKED | Hardware-info retrieval is now validated, but a safe active-joint command and readback loop is still pending. |
| Stable 10-joint naming map exists | BASELINE_CAPTURED | Vendor-declared mapping recorded in `docs/control/omnihand_phase1_joint_map.md`; runtime verification is still pending. |

## Repo-Side Phase 1 Artifacts Created

- `scripts/omnihand/phase1_smoke_test.py`: repo-owned isolated probe entrypoint
- `docs/control/omnihand_phase1_joint_map.md`: vendor-declared left/right active-joint index map with limits
- `docs/control/omnihand_wrapper_integration_plan.md`: updated with the current Phase 1 status
- local vendor hardening for empty CAN readback handling and optional wheel packaging during socket-backed builds

## External Step Still Required To Truly Close Phase 1

The remaining work before Phase 1 can be called complete is now narrower:

1. Validate at least one safe active-joint command and readback loop on the same working SocketCAN path.
2. Preserve the current Jetson SocketCAN path as the local baseline unless the deployment later requires a native `aarch64` ZLG userspace package.
3. Promote the same validated hardware path into the repo-owned non-mock adapter and ROS backend work.

Until then, the repo-side preparation for Phase 1 is complete, live device enumeration is now confirmed, and the remaining block is at the first safe motion/readback step plus backend promotion into the agx_arm runtime.