# Sprint 1 Checklist

Repo-side Sprint 1 work is complete. The remaining unchecked items depend on external assets or live hardware validation.

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
- [x] Promote stable Sprint 1 outputs from `docs/development/sprint1/` into permanent top-level `docs/assets/` and `docs/assets/`.
- [x] Acquire OmniHand vendor SDK/docs or clone the relevant repo into the workspace.
- [x] Acquire OmniHand model artifacts or an authoritative kinematics/interface document.
- [x] Decide the near-term OmniHand integration approach: isolate the vendor SDK first, then integrate through a thin local wrapper; do not overlay the vendor ROS2 packages into `src/` yet.
- [x] Document the isolated OmniHand bring-up and wrapper integration plan in the stable docs tree.
- [x] Establish and validate a repo-local socket-backed `aarch64` build/import path for isolated OmniHand probing, while keeping the stock vendor ZLG userspace path marked `x86_64`-only.
- [ ] Acquire AGV/base CAD exports, mounting references, and coordinate definitions.
- [ ] Confirm whether more USD variants exist outside the current workspace or need generation from URDF.
- [ ] Complete hardware-backed OmniHand validation on a responsive device path once the actual hand and adapter are available.
- [ ] Validate the current findings against broader simulation/runtime checks once the missing vendor assets are available.

## Remaining Open Questions

- [ ] OPEN: Is there an Agibot-supported live-hardware `aarch64` path for the vendored SDK, or should first validated device access happen on an `x86_64` host?
- [ ] OPEN: Which repo-owned ROS interface should be introduced above the future OmniHand adapter once standalone validation is complete?