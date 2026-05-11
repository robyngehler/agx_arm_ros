# Sprint 1 Checklist

## Documentation Setup

- [x] Create `docs/development/sprint1/`.
- [x] Create working `assets/` and `control/` subfolders inside Sprint 1.
- [x] Create a Sprint 1 overview document.
- [x] Create the checklist and errors log.

## Repository And Asset Discovery

- [x] Inventory ROS2 packages in `agx_arm_ros/src`.
- [x] Inventory local SDK/docs in `pyAgxArm`.
- [x] Review related `docs/development/*.md` files.
- [x] Review relevant git history for MoveIt, MIT, URDF, and gravity work.
- [x] Confirm Nero URDF, Xacro, and mesh sources.
- [x] Confirm the current MoveIt package and solver state.
- [x] Confirm MIT controller model assumptions and URDF lookup logic.
- [x] Confirm local Isaac/USD asset presence or gaps.
- [x] Confirm the local OmniHand asset gap.
- [x] Confirm the local AGV/base CAD gap.

## Sprint 1 Deliverables In Working Form

- [x] Draft `repository_asset_inventory.md`.
- [x] Draft `nero_asset_validation.md`.
- [x] Draft `omnihand_asset_validation.md`.
- [x] Draft `agv_cad_inventory.md`.
- [x] Draft `mit_controller_model_inventory.md`.

## Remaining Work Before Sprint 1 Can Be Called Complete

- [x] Canonicalize the Nero description source of truth onto `src/agx_arm_sim/agx_arm_description` and remove the duplicate root package from `colcon` discovery.
- [x] Drop the legacy root-source MIT URDF fallback and keep auto-discovery aligned with the sim-backed description package.
- [x] Remove the remaining `agx_arm_urdf` submodule dependency and keep the pruned Nero/Revo2 asset tree directly in-repo.
- [x] Restrict the active launch/config/documentation surface to Nero-only workspace defaults.
- [ ] Promote stable Sprint 1 outputs from `docs/development/sprint1/` into permanent top-level `docs/assets/` and `docs/control/`.
- [x] Acquire OmniHand vendor SDK/docs or clone the relevant repo into the workspace.
- [x] Acquire OmniHand model artifacts or an authoritative kinematics/interface document.
- [ ] OPEN: Decide whether OmniHand integration should wrap the vendor SDK or selectively overlay the vendor ROS2 packages into `src/`.
- [ ] OPEN: Resolve the current `aarch64` host vs vendor-documented `x86_64` SDK support constraint before attempting local OmniHand build validation.
- [ ] Acquire AGV/base CAD exports, mounting references, and coordinate definitions.
- [ ] Confirm whether more USD variants exist outside the current workspace or need generation from URDF.
- [ ] Validate the current findings against hardware, simulation, or runtime checks once the missing vendor assets are available.

## Open Questions

- [ ] OPEN: Should OmniHand be integrated through a thin local wrapper around the vendor SDK, or by selectively overlaying the vendor ROS2 packages into `src/`?
- [ ] OPEN: Is there an Agibot-supported `aarch64` path for the vendored SDK, or should first bring-up happen on an `x86_64` host?