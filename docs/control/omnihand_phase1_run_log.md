# OmniHand Phase 1 Run Log

status: BUILD_AND_IMPORT_CONFIRMED_RUNTIME_DEVICE_BLOCKED
last_updated: 2026-05-13
script: scripts/omnihand/phase1_smoke_test.py

## Goal

Capture the first isolated SDK bring-up result before any repo-owned wrapper or ROS bridge is introduced.

## Latest Attempt

- Date: 2026-05-13
- Host: current workspace machine
- Architecture: `aarch64`
- Runtime transport path exercised: socket-backed local vendor build
- Result: local build, package refresh, and Python import succeeded on `aarch64`, and the runtime probe now exits cleanly with `runtime_probe_incomplete` when the CAN request path does not return a complete joint-state vector

## Evidence Collected

### Userspace CAN Library Provenance

- Vendored userspace library path: `vendor/Omnihand-2025-SDK/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so`
- Local file inspection result: `ELF 64-bit LSB shared object, x86-64`
- Packaging path: `vendor/Omnihand-2025-SDK/python/CMakeLists.txt` copies that same x64 userspace library into the Python package build output

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

### Runtime Probe After Local Socket Build

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
- the remaining blocker is the actual runtime/device path: either the expected hand is not responding on the active CAN interface, or the chosen bring-up path is still not valid for the current hardware stack
- safe device enumeration and safe command-response validation still cannot be claimed from this run

## Phase 1 Exit Criteria Status

| Exit Criterion | Current State | Notes |
| --- | --- | --- |
| Device enumeration succeeds | BLOCKED | Build/import now work in local socket-backed mode, but the runtime probe still returns incomplete/default data instead of a validated device response. |
| Safe command-response loop succeeds | BLOCKED | Not safe to attempt until the device path returns a complete joint-state vector and stable readback. |
| Stable 10-joint naming map exists | BASELINE_CAPTURED | Vendor-declared mapping recorded in `docs/control/omnihand_phase1_joint_map.md`; runtime verification is still pending. |

## Repo-Side Phase 1 Artifacts Created

- `scripts/omnihand/phase1_smoke_test.py`: repo-owned isolated probe entrypoint
- `docs/control/omnihand_phase1_joint_map.md`: vendor-declared left/right active-joint index map with limits
- `docs/control/omnihand_wrapper_integration_plan.md`: updated with the current Phase 1 status
- local vendor hardening for empty CAN readback handling and optional wheel packaging during socket-backed builds

## External Step Still Required To Truly Close Phase 1

One of the following must happen before Phase 1 can be called complete:

1. The current socket-backed runtime path is validated against an actual responsive hand on the expected CAN interface, and the vendor RPC phase stops crashing.
2. Agibot provides a supported `aarch64` ZLG userspace path if the intended deployment depends on the vendor ZLG backend rather than SocketCAN.
3. The first hardware validation moves to a supported `x86_64` host and the smoke-test script is executed there with the actual adapter and hand.

Until then, the repo-side preparation for Phase 1 is complete, the local build/import barrier is resolved for the socket-backed path, and the remaining block is at the live device/runtime layer.