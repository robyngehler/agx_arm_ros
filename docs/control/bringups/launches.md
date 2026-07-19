# Launches

status: ACTIVE_BASELINE
last_updated: 2026-07-19

How to start the current system. Two steps: bring up the baseline that owns the runtime graph, then
add the matching tool or demo on top.

Conventions: `can_nero_right` and `can_nero_left` are the native side buses, each carrying its arm
and its OmniHand. Arms are separated by ROS namespace (`left_arm` and `right_arm`); each driver
publishes unprefixed `joint1..7` inside its namespace. `execution_profile`
(`src/agx_arm_ctrl/config/execution_profiles.yaml` and `duo_motion_registry.yaml`) is the source of
truth for slice composition, prefixes, frames, side buses, and gravity URDF derivation.

## 0. CAN buses

```bash
sudo bash scripts/activate_native_can.sh
# or: sudo bash scripts/activate_native_can.sh right
ip -details link show can_nero_right
```

For a shared arm-plus-hand side bus under load, first keep `one-shot on` and deepen the queue:

```bash
TX_QUEUE_LEN=1000 sudo bash scripts/activate_native_can.sh right
```

Details and alternatives: `../../assets/omnihand/omnihand_canfd_setup.md`.

## 1. Teach or MIT baseline

Arm driver plus MIT controller. This is the dependency for the teach manager. Full argument
rationale lives in `../teach_and_run.md`.

| Goal | Command |
|---|---|
| single right arm | `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can_nero_right gravity_arm_side:=right` |
| single left arm | `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can_nero_left gravity_arm_side:=left` |
| right arm plus OmniHand | `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can_nero_right gravity_arm_side:=right effector_type:=omnihand omnihand_type:=right launch_omnihand_bridge:=true omnihand_backend_type:=sdk` |
| both arms | run once per side with `namespace:=left_arm` and `namespace:=right_arm` |
| both arms plus both OmniHands | run once per side with the matching `namespace:=...`, `effector_type:=omnihand`, `omnihand_type:=...`, `launch_omnihand_bridge:=true`, and `omnihand_backend_type:=sdk` |

Rules of thumb:

- always pass `gravity_arm_side`
- when a hand is mounted, also pass `effector_type:=omnihand`
- keep the teach loop unprefixed; do not add `input_joint_prefix` on this baseline

## 2. MoveIt or components baseline

This is the canonical wrapper for planning and execution (`mode:=moveit_mit`), RViz soft-target
debug (`mode:=debug_soft_target`), and the vendor-driver-only path (`mode:=manual_vendor`). The
selected `execution_profile` resolves the side-specific wiring.

| Goal | Command |
|---|---|
| right arm, MoveIt plus MIT | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_arm follow:=true use_rviz:=true planning_pipelines:=ompl` |
| left arm, MoveIt plus MIT | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=left_arm follow:=true use_rviz:=true planning_pipelines:=ompl` |
| right arm plus OmniHand | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_hand follow:=true use_rviz:=true planning_pipelines:=ompl omnihand_backend_type:=sdk` |
| left arm plus OmniHand | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=left_hand follow:=true use_rviz:=true planning_pipelines:=ompl omnihand_backend_type:=sdk` |
| both arms | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=duo_arm follow:=true use_rviz:=true planning_pipelines:=ompl` |
| both arms plus both OmniHands | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=duo_hand follow:=true use_rviz:=true planning_pipelines:=ompl omnihand_backend_type:=sdk` |
| planning only, no hardware | any of the above with `follow:=false use_mit_controller:=false` |
| RViz soft-target debug | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=debug_soft_target execution_profile:=<profile> follow:=true` |

Notes:

- `duo_hand` is validated offline; live hardware validation is still pending
- the OmniHand SDK path needs no manual `PYTHONPATH` or `LD_LIBRARY_PATH`
- multi-arm slices merge per-arm feedback into `/feedback/prefixed_joint_states` for `move_group`

## 3. Description only

```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true gui:=true use_rviz:=true
```

## Tools

Teach manager on top of the MIT baseline:

```bash
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
  --arm-config src/agx_arm_coordination/config/arm_config.yaml \
  --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
```

For both arms, add `--arms left_arm right_arm` or let the manager auto-detect complete namespaced
MIT stacks.

MoveIt anchor transitions use the same `execution_profile` family as the components baseline. See
`teach_and_run.md` for the full flow.

## Demos

Hefeweizen coordinator: start the components baseline with `mode:=moveit_mit`, then run:

```bash
ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py
```

The staged dry-run and live sequence remains in `teach_and_run.md`.

## References

- `../environment.md`: build, test, and runtime wrapper rules
- `teach_and_run.md`: teach loop, gravity, bus details, and coordinator-facing motion flow
- `../../assets/control/single_vs_multi_arm_control_chain.md`: per-arm versus Duo interaction analysis
- `../../assets/omnihand/omnihand_canfd_setup.md`: CAN transport and hardware bringup details
- `../../sprint6/planning/duo_both_hands_moveit_gap.md`: current duo-hand validation gap