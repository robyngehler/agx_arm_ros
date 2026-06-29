# OmniHand Pro 2025 (O12) description — imported vendor assets

These xacro files and the sibling `../../meshes_pro/` STL meshes are imported from the
**official AgibotTech OmniHand Pro description**
(`o12_hand_description-o12_t3`, upstream repo now `AgibotTech/agillink_omnihand_sdk`,
formerly `OmniHand-Pro-2025`), vendored locally under
`vendor/OmniHand-Pro-2025/description/urdf/o12_hand_description-o12_t3/`. Licensed under
**Mulan PSL v2** (see the vendor package).

## What is verbatim vs. adapted

- `const.xacro`, `material.xacro`, `thumb.xacro`, `index.xacro`, `middle.xacro`,
  `ring_pinky.xacro` — copied **verbatim** from the vendor `assets/urdf/xacro/`.
- `hand.xacro` — **adapted** to this repo's integration seam (see its header). Only the
  seam changed: macro contract `hand name mirror *origin`, palm parented to
  `${name}_base_link`, `mirror_mesh` derived from `mirror`, and repo-local include/mesh
  paths. Vendor link frames, joint limits, mimic ratios, and meshes are unchanged.
- meshes in `../../meshes_pro/` — the vendor **visual** STLs (`assets/meshes/*.STL`). The
  vendor `assets/urdf/xacro/` variant uses primitive collisions, so the `collision/*_col.STL`
  set is not imported.

## Joint model

12 active DoF: `thumb_roll, thumb_abad, thumb_mcp, thumb_pip, index_abad, index_mcp,
index_pip, middle_abad, middle_mcp, middle_pip, ring_mcp, pinky_mcp`. The remaining finger
joints are vendor `mimic` joints (underactuated coupling): `thumb_dip`, `index_dip`,
`middle_dip`, and ring/pinky `pip`+`dip` driven by their `mcp`. This matches
`agx_arm_ctrl/omnihand/models.py` `O12_PRO` and the skill/bridge layer.

## Updating from upstream

Re-import the vendor `assets/urdf/xacro/{const,material,thumb,index,middle,ring_pinky}.xacro`
and `assets/meshes/*.STL`, then re-apply only the seam edits in `hand.xacro`. If joint
names/limits change, also update the MoveIt surfaces that mirror them:
`agx_arm_moveit/config/agx_arm.srdf.xacro` (`omnihand_group`),
`moveit_controllers_omnihand_{right,left}.yaml`, `initial_positions.yaml`,
`agx_arm.ros2_control.xacro`, and `agx_arm_ctrl/omnihand/models.py`.
