# Bringup — Launch Paths

How to start the current system. Two steps: bring up the **baseline** that owns the runtime graph,
then add the **tool** or **demo** on top. Details and rationale live behind the links — this file
stays command-first.

Conventions: `can_nero_right`/`can_nero_left` are the native side buses, each carrying its arm and its
OmniHand (o12_pro, 12 active joints). Arms are separated by ROS **namespace** (`left_arm`/`right_arm`);
each driver publishes unprefixed `joint1..7` inside its namespace. `execution_profile`
(`src/agx_arm_ctrl/config/execution_profiles.yaml` + `duo_motion_registry.yaml`) is the single source
of truth for slice composition, prefixes/frames, side buses, and the gravity URDF derivation — change
the preset, not ad-hoc command lines.

## 0. CAN buses (always first)

```bash
sudo bash scripts/activate_native_can.sh          # both sides (or: ... right | left)
# shared arm+hand bus: TX_QUEUE_LEN=1000 sudo bash scripts/activate_native_can.sh right
ip -details link show can_nero_right              # expect: mtu 72, fd, one-shot, 1M/5M
```

Details/alternatives: [omnihand_canfd_setup.md](../assets/omnihand/omnihand_canfd_setup.md).

## 1. Teach / MIT baseline (`start_nero_mit_controller.launch.py`)

Arm driver + MIT controller (freedrive, `hold_current`, FollowJointTrajectory). This is the
dependency for the teach manager. Full argument rationale (gravity mount, articulated hand payload,
calibration gating, shared-bus tuning): [teach_and_run.md](teach_and_run.md).

| Goal | Command |
|---|---|
| single right arm | `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can_nero_right gravity_arm_side:=right` |
| single left arm | `... can_port:=can_nero_left gravity_arm_side:=left` |
| right arm + OmniHand | `... can_port:=can_nero_right gravity_arm_side:=right effector_type:=omnihand omnihand_type:=right launch_omnihand_bridge:=true omnihand_backend_type:=sdk` |
| both arms (run twice) | `... namespace:=left_arm  can_port:=can_nero_left  gravity_arm_side:=left`  **and**  `... namespace:=right_arm can_port:=can_nero_right gravity_arm_side:=right` |
| both arms + both OmniHands (run twice) | left/right rows above, each with `effector_type:=omnihand omnihand_type:=<side> launch_omnihand_bridge:=true omnihand_backend_type:=sdk` |

Rules of thumb:

- always pass `gravity_arm_side` (body mount) and, with a hand mounted, `effector_type:=omnihand`
  (hand mass, articulated by default) — otherwise gravity compensation is wrong
- no `input_joint_prefix` on this baseline (teach loop is unprefixed)
- namespaced instances are fully independent (own bus, own gravity URDF, own bridge) — bus tuning
  (`TX_QUEUE_LEN`, `command_retry_*`, `joint_read_rate`) applies per side

## 2. MoveIt / components baseline (`start_agx_arm_components.launch.py`)

The canonical wrapper for planning/execution (`mode:=moveit_mit`), RViz soft-target debug
(`mode:=debug_soft_target`), and the vendor-driver-only path (`mode:=manual_vendor`). The
`execution_profile` resolves everything side-specific — including the side CAN bus, so `can_port` is
only needed as an override.

| Goal | Command |
|---|---|
| right arm, MoveIt+MIT | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_arm follow:=true use_rviz:=true planning_pipelines:=ompl` |
| left arm, MoveIt+MIT | `... execution_profile:=left_arm` (same flags) |
| right arm + OmniHand | `... mode:=moveit_mit execution_profile:=right_hand follow:=true use_rviz:=true planning_pipelines:=ompl omnihand_backend_type:=sdk` |
| left arm + OmniHand | `... execution_profile:=left_hand` (same flags) |
| both arms | `... mode:=moveit_mit execution_profile:=duo_arm follow:=true use_rviz:=true planning_pipelines:=ompl` |
| both arms + both OmniHands | `... mode:=moveit_mit execution_profile:=duo_hand follow:=true use_rviz:=true planning_pipelines:=ompl omnihand_backend_type:=sdk` |
| planning only, no hardware | any of the above with `follow:=false use_mit_controller:=false` |
| soft-target RViz debug | `... mode:=debug_soft_target execution_profile:=<same profiles> follow:=true` |

Notes:

- `duo_hand` brings up per-side MIT stacks, per-side OmniHand bridges, and MoveIt groups
  `left_arm`/`right_arm`/`both_arms`/`left_hand`/`right_hand` (validated offline; hardware run
  pending — see [duo_both_hands_moveit_gap.md](../development/sprint6/planning/duo_both_hands_moveit_gap.md))
- the OmniHand SDK path needs no manual `PYTHONPATH`/`LD_LIBRARY_PATH`
  ([details](../assets/omnihand/omnihand_solo_bringup_and_load_test.md))
- multi-arm slices merge per-arm feedback into `/feedback/prefixed_joint_states` for `move_group`
  (arm joints get side prefixes, hand joints pass through)

## 3. Description only (model + TF, no runtime)

```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true gui:=true use_rviz:=true
```

## Tools

**Teach manager** (on baseline 1; identical invocation with or without hands attached — it only
records/plays the 7 arm joints, see [teach_and_run.md](teach_and_run.md)):

```bash
# single arm
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
  --arm-config src/agx_arm_coordination/config/arm_config.yaml \
  --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
# both arms (one keyboard UI, one clock): add --arms left_arm right_arm
# (or omit it: started without --arms, the manager auto-detects complete
#  namespaced MIT stacks already in the graph and rebinds itself)
```

**MoveIt anchor transitions** (`t` mode in the teach manager) run on baseline 2 with the matching
`execution_profile` — flow in [teach_and_run.md](teach_and_run.md).

Playback/conversion CLIs and gravity calibration helpers: [teach_and_run.md](teach_and_run.md),
`src/agx_arm_mit_tools`.

## Demos

**Hefeweizen coordinator (DAG):** baseline 2 with `mode:=moveit_mit`, then
`ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py`. Staged dry-run/live sequence:
[teach_and_run.md](teach_and_run.md).

## Overrides & references

Direct wrappers (`start_agx_arm_moveit.launch.py`, `start_single_agx_arm_rviz.launch.py`) accept
explicit `custom_model`/`custom_model_xacro_args`/`input_joint_prefix`/`can_port` when a preset is
not enough — prefer changing the preset.

- Teach loop, gravity/mount/bus details, duo-vs-parallel routing → [teach_and_run.md](teach_and_run.md)
- Per-arm vs Duo interaction analysis → [single_vs_multi_arm_control_chain.md](../assets/control/single_vs_multi_arm_control_chain.md)
- CAN transport → [omnihand_canfd_setup.md](../assets/omnihand/omnihand_canfd_setup.md)
- duo_hand design + validation state → [duo_both_hands_moveit_gap.md](../development/sprint6/planning/duo_both_hands_moveit_gap.md)
