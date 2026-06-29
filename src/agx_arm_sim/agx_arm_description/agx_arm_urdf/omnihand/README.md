# OmniHand Assets

This directory contains the canonical OmniHand description assets owned by this repository.

- Source provenance: normalized from `vendor/OmniHand-Pro-2025/assets/urdf` and `vendor/OmniHand-Pro-2025/assets/meshes`.
- Current scope: display, `display_control.launch.py`, and MoveIt simulation.
- Explicitly open for later: hardware ROS bridge, CAN transport, and runtime device validation.

The left and right hand xacro entrypoints are:

- `urdf/omnihand_left_hand.xacro`
- `urdf/omnihand_right_hand.xacro`

The public ROS surface in this repository uses local left/right naming such as `left_thumb_roll_joint` and `right_index_pip_joint`.