# Development Errors And Fixes

Cross-sprint issues that created documentation drift or masked the current runtime baseline.

## Canonical launch surface drift

Problem: active repo-level docs still promoted compatibility wrappers such as `start_single_agx_arm_moveit.launch.py`, even though the operational source of truth had moved to `docs/control/bringup.md` and the component multiplexer.

Current fix:

- active repo-level launch guidance now points to `start_agx_arm_components.launch.py mode:=moveit_mit`
- `start_agx_arm_moveit.launch.py` remains the lower-level combined wrapper
- `start_single_agx_arm_moveit.launch.py` is treated as a compatibility alias only

See `../control/bringup.md` and `../project/repo_interaction_diagrams.md`.

## OmniHand SDK environment drift

Problem: some active docs implied that ROS launches needed manual `PYTHONPATH` or `LD_LIBRARY_PATH` exports for the OmniHand SDK, which conflicted with the bridge's repo-local auto-discovery path and caused users to fall back to brittle shell overrides.

Current fix:

- active ROS bringup docs now state that the normal repo path needs no manual env export
- `sdk_python_dir` or `AGX_ARM_OMNIHAND_SDK_DIR` are documented only as fallback overrides when the built vendor package lives outside the repo checkout
- the bridge runtime error text now matches that behavior

See `../assets/omnihand/omnihand_solo_bringup_and_load_test.md` and `../control/bringup.md`.

## Shared arm+hand bus operating profile

Problem: cross-sprint development summaries reduced native CAN stabilization to "`one-shot on` is the stable baseline", while the current shared arm+hand teach workflow documents a narrower caveat: under live arm load, the hand may need a workflow-specific native CAN profile and lower MIT command rate.

Current fix:

- the development overview now distinguishes the arm-stable native CAN baseline from the shared-bus operational caveat
- the runtime-sensitive detail remains anchored in `../control/teach_and_run.md`

See `../control/teach_and_run.md` and `sprint5/planning/can_transport_decision.md`.

## Historical asset docs outside development

Problem: `docs/assets/omnihand/` still carried historical proposal and run-log documents, which made the stable asset layer look like a mixed archive instead of a current runtime/reference surface.

Current fix:

- the historical OmniHand migration proposal and old phase run log were removed from `docs/assets/omnihand/`
- the surviving vendor-declared joint-order reference was promoted under the stable filename `omnihand_active_joint_map.md`
- active references now point to `omnihand_vendor_sdk_aarch64.md`, `omnihand_active_joint_map.md`, and `omnihand_asset_validation.md`

See `../assets/omnihand/omnihand_vendor_sdk_aarch64.md` and `../assets/omnihand_asset_validation.md`.