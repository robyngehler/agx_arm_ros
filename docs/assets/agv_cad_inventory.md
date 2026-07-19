# AGV CAD Inventory

promotion_origin: Sprint 1 repository and asset discovery pass
promotion_date: 2026-05-12

component: AGV/base CAD, mount geometry, and base-description artifacts
repository_or_source: current workspace (`agx_arm_ros`, `vendor/pyAgxArm`)
inspection_date: 2026-05-11
status: MISSING
found_artifacts:
- no AGV/base CAD repository, ROS2 description package, STEP export, STL export, OBJ export, MJCF model, or mounting-reference doc was found in the workspace
- the only mechanical helper assets found locally are arm-side meshes and a camera-stand flow inside `src/agx_arm_sim/agx_arm_description`, which are not AGV/base assets
missing_artifacts:
- STEP or native CAD export of the AGV/base assembly
- simplified collision geometry inputs
- arm mounting transform definition
- base frame convention (`agv_base_link`, `arm_mount_link`) backed by local mechanical data
- any local package analogous to `agv_base_description` or `agv_base_bringup`
interface_notes:
- roadmap assumptions about a custom AGV/base are not yet backed by files in this workspace
- no local data was found that can answer AGV coordinate conventions, usable collision simplification levels, or mount pose values
risks:
- Sprint 9 planning is blocked without at least one geometric export plus a mount transform reference
- controller-side gravity assumptions for a mounted arm cannot be validated until the arm mounting orientation is known
- later perception placement and camera-mount planning may need AGV CAD context that does not yet exist locally
recommended_next_action:
- collect the AGV/base CAD export set and any mounting drawings into the workspace
- define a first minimal artifact set: one visual mesh/export, one collision simplification sketch, one measured arm mount transform
- reopen this document once the first files exist locally
related_sprint: 1

## Search Result Summary

The following file types were checked across the current workspace and none were found for AGV/base use:

- `*.step`
- `*.stl`
- `*.obj`
- `*.mjcf`

This means the roadmap statement that AGV/base CAD is available should currently be treated as an external assumption, not a local repository fact.