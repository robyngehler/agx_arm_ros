# Errors And Fixes

## 2026-05-20

### Isolated OmniHand socket bring-up was pinned to `can0`

- Symptom: the current host enumerates the attached USB CANFD adapter as `can2`, but the vendored SocketCAN path in `vendor/Omnihand-2025-SDK` bound unconditionally to `can0`.
- Impact: even the lowest-risk Phase 1 SDK smoke test could not target the attached adapter without renaming interfaces or patching vendor code first.
- Fix: patched the local SocketCAN backend to honor `OMNIHAND_SOCKETCAN_IFACE` and fall back to `can0` only when the environment variable is unset.
- Follow-up: bring the intended CANFD interface up with the correct bitrate and dbitrate, then run the vendor-info/device-info smoke test through the existing `build_phase1_socket` Python package before attempting any ROS2 bridge integration.

### Forced `gs_usb` binding created `can3`, but still not CAN FD

- Symptom: after adding USB ID `a8fa:8598` to `gs_usb`, Linux exposed the OmniHand dongle as `can3` on `1-4.3:1.0`, but `sudo ip link set can3 type can fd on bitrate 1000000 dbitrate 4000000` failed with `RTNETLINK answers: Operation not supported`.
- Impact: the adapter can be brought up as classic CAN only, which is not sufficient for the current OmniHand SDK path and explains why live vendor-info requests still fail on the hand path.
- Fix: none yet at the repo level; this is a host driver or adapter capability limit, not a repo-owned launch or naming issue.
- Follow-up: use a Linux path that exposes CAN FD semantics for the hand adapter, or switch to an adapter that is already supported as CAN FD on this host before resuming SDK or ROS bridge bring-up.

### Vendored ZLG CAN FD path is buildable on Jetson, but not for `a8fa:8598`

- Symptom: the repo needed a concrete answer on whether the current Jetson can host a supported CAN FD driver path for real OmniHand bring-up.
- Impact: without that answer, Sprint 2 could not distinguish between a host limitation and a current-adapter limitation.
- Fix: built `vendor/Omnihand-2025-SDK/thirdParty/usbcanfd200_400u_2.10/usbcanfd.ko` successfully against `5.15.122-tegra` and inspected the resulting module aliases.
- Result: the Jetson host can build and load the vendored ZLG-style CAN FD kernel module, but the module only matches `04cc:1240` and `3068:0009`, not the currently attached `a8fa:8598` adapter.
- Follow-up: either obtain a supplier-provided aarch64 Linux package for `a8fa:8598`, or switch the hand to a supported ZLG-style adapter and use the repo's SocketCAN path.

## 2026-05-17

### Launch and runtime understanding was spread across code and docs

- Symptom: launch order, ROS graph behavior, file composition, and config dataflow had to be reconstructed from several launch files, xacros, and control docs.
- Impact: Sprint 2 context recovery stayed slower than it should be for developers and agents.
- Fix: promoted a stable diagram set into `docs/project/repo_interaction_diagrams.md` and created the Sprint 2 working-note folder for the remaining runtime-baseline work.

### Sprint 2 had no working-note folder yet

- Symptom: the new two-tier docs layout had Sprint 1 working notes, but no matching Sprint 2 folder for checklist and issue tracking.
- Impact: Sprint 2 progress existed only in stable docs and runtime/control docs, which made it harder to record in-flight questions without adding another top-level source.
- Fix: created `docs/development/sprint2/` with `README.md`, `checklist.md`, `errors_and_fixes.md`, and `open_questions.md`.