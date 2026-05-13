# OmniHand Wrapper Integration Plan

source_document: docs/development/sprint1/hand/omnihand_sdk_integration.md
promotion_date: 2026-05-12
status: IN_PROGRESS
decision: WRAPPER_FIRST

## Current Execution Status

- Repo-side Phase 1 artifacts are now in place:
	- `scripts/omnihand/phase1_smoke_test.py`
	- `docs/control/omnihand_phase1_run_log.md`
	- `docs/control/omnihand_phase1_joint_map.md`
- The current workspace host is no longer blocked at build/import time for isolated testing:
	- a local socket-backed vendor build now succeeds on `aarch64`
	- the unpacked Python package refresh now succeeds even when `python -m build` is unavailable locally
	- the built Python package imports successfully on this host
	- the repo smoke test can probe that built package directly
- The remaining Phase 1 blocker is the live runtime/device layer:
	- the isolated probe reaches CAN request traffic
	- failed request traffic now yields a clean `runtime_probe_incomplete` result instead of a process crash
	- the current probe still does not return a complete active-joint vector or validated device identity
	- no safe command-response loop has been validated yet
- Result: Phase 1 repo preparation and local build enablement are complete, but the runtime exit criteria remain blocked on actual device-path stability and hardware response

## Goal

Bring OmniHand up as an isolated vendor-SDK device first, then expose it through a repo-owned wrapper and ROS bridge that fit the current Nero control and MoveIt stack.

This plan intentionally defers direct overlay of the vendor ROS2 packages into `src/`.

## Why Wrapper-First

- The current runtime already integrates end-effectors through thin local wrappers rather than by embedding vendor ROS nodes directly.
- The current hand message surface is Revo2-specific and does not match OmniHand's 10 active joints plus richer diagnostics and tactile data.
- The vendor ROS node is coupled to a transport and topic layout that is not yet validated on this `aarch64` host.
- The vendor asset bundle still needs normalization before it can serve as a local description package.

## Current Constraints

- The vendor README documents Ubuntu 22.04 `x86_64`; this workspace host is `aarch64`.
- The vendored userspace CAN bundle under `thirdParty/` is `usbcanfd_libusb_x64_1.0.10_250328`.
- the stock ZLG-oriented Python packaging path copies that x64 userspace library into the Python package build; the local socket-backed path bypasses it
- `assets/urdf/omnihand_right.urdf` contains absolute local mesh paths.
- `assets/urdf/xacro/finger.xacro` contains a stray literal `y` before one joint declaration.
- Multiple asset files reference `package://omnihand_description/...`, but the vendor tree does not provide a matching ROS package.

## Phase 0: Platform Gate

Objective: determine whether isolated bring-up should happen on this host or move first to `x86_64`.

Tasks:

- confirm whether Agibot provides an `aarch64`-compatible userspace CAN library or build path
- if not, declare `aarch64` blocked for first runtime validation and move the first execution pass to an `x86_64` machine
- keep the SDK vendored in this repo for inspection either way

Exit criteria:

- one documented answer for `aarch64`: supported here, or blocked and deferred to `x86_64`

## Phase 1: Isolated SDK Bring-Up

Objective: validate direct device access before introducing any repo-specific ROS integration.

Tasks:

- use the vendor Python API for first smoke tests
- read hardware information and firmware info
- command one safe active joint and read back joint state
- verify fault handling and clean shutdown behavior
- capture the final tested transport setup, host architecture, and adapter details in a run log

Recommended artifact output:

- one minimal smoke-test script
- one tested environment note with adapter model, architecture, and library provenance
- one joint-name and motor-index mapping table derived from working runtime observations

Exit criteria:

- device enumeration succeeds
- at least one safe command-response loop succeeds
- a stable 10-joint naming map exists for the active joints

Current state:

- the smoke-test entrypoint exists
- the vendor-declared 10-joint naming map is recorded in the repo
- a socket-backed local build/import path now works on `aarch64`
- the current-host runtime probe now fails cleanly with incomplete/default data when the device path does not answer, rather than crashing the process

## Phase 2: Repo-Owned Adapter Layer

Objective: hide the vendor API behind a narrow local interface with no ROS dependencies.

Design constraints:

- keep the vendor SDK behind a thin adapter rather than exposing vendor types across the repo
- keep this layer independent from ROS messages and launch files
- preserve direct access to richer diagnostics so later ROS bridges do not lose tactile or health data

Minimum adapter surface:

- connect and close
- get hardware info
- get joint names
- read active-joint state
- set active-joint targets
- change control mode when required by the device
- read diagnostics, temperatures, currents, and tactile status when available
- stop or safe-disable

Implementation note:

- this adapter can live either as a new repo-owned Python package or as a focused extension inside `pyAgxArm`, but it should remain clearly separate from the existing Revo2 driver path until OmniHand is validated

Exit criteria:

- a single local interface can drive the isolated device without importing vendor types outside the adapter implementation

## Phase 3: ROS Bridge Above The Adapter

Objective: expose the validated adapter to the ROS stack without reusing the Revo2 message contract.

Tasks:

- publish `sensor_msgs/JointState` for the 10 active joints
- publish raw status and diagnostics topics for device health and errors
- publish tactile data through a dedicated raw topic or message family after the adapter surface is stable
- define command topics or actions that match the OmniHand joint model instead of forcing the current six-channel Revo2 schema

Constraints:

- do not retrofit OmniHand onto `agx_arm_msgs/msg/HandCmd.msg`, `HandPositionTimeCmd.msg`, or `HandStatus.msg`
- keep the first ROS bridge small and hardware-truthful; higher-level grasp abstractions can come later

Exit criteria:

- the ROS bridge can command and observe the isolated hand through the local adapter only

## Phase 4: Description And MoveIt Integration

Objective: integrate the hand into the planning and visualization stack only after the direct control path is stable.

Tasks:

- normalize the vendor meshes and xacros into a repo-owned description package
- remove absolute paths and the invalid xacro token from the imported asset set
- define the Nero wrist-to-palm adapter transform and grasp/tool frames
- add a planning-ready joint naming convention aligned with the adapter and ROS bridge
- extend the MoveIt description and control expectations only after the description package is reproducible locally

Exit criteria:

- the description package can be rendered locally without vendor-specific path hacks
- RViz and MoveIt consume the same normalized joint names and frames used by the adapter and ROS bridge

## Deferred For Now

- overlaying the vendor ROS2 `node/` and `node_msg/` packages into `src/`
- treating the vendor URDF assets as a drop-in local description package
- mapping OmniHand onto the existing Revo2-specific hand message contract

## Decision Checkpoint

Revisit overlay only if the wrapper-first path fails for a concrete technical reason that the repo-owned adapter cannot address, not simply because the vendor ROS node already exists.