# Bringup — which launch + which arguments

The single source of truth for **how to start the system**: one arm, an arm + OmniHand (either side),
or both arms. Pick the surface by what you want to do, then copy the matching command.

- **Teach / MIT playback** (freedrive record, anchor capture, direct MIT replay) →
  `agx_arm_mit_controller/start_nero_mit_controller.launch.py` + `agx_arm_teach_manager`. See
  [teach_and_run.md](teach_and_run.md).
- **MoveIt planning + execution, or RViz soft-target debug** →
  `agx_arm_ctrl/start_agx_arm_components.launch.py` (a `mode` × `execution_profile` multiplexer).
- **Coordinated activities (DAG)** → `agx_arm_coordination/start_hefeweizen_demo.launch.py` on top of a
  running MoveIt slice. See [teach_and_run.md](teach_and_run.md) Step D.

Conventions: `can_nero_right` (`can0`) and `can_nero_left` (`can1`) are the native `mttcan` side buses;
each carries its arm (classic frames) and its OmniHand (FD/BRS). The OmniHand Pro (`o12_pro`) exposes
**12 active joints**. Arms are separated by **ROS namespace** (`left_arm`/`right_arm`), not by a joint
prefix — the driver publishes unprefixed `joint1..7` within its namespace.

## 0. CAN pre-configure (once, before any launch)

```bash
sudo bash scripts/activate_native_can.sh          # both sides (or: ... right | left)
# For a shared arm+hand bus (first tests): TX_QUEUE_LEN=1000 sudo bash scripts/activate_native_can.sh right
ip -details link show can_nero_right              # expect: mtu 72, fd, one-shot, bitrate 1M, dbitrate 5M
```

Rationale + alternatives (USB adapters): [omnihand_canfd_setup.md](../assets/omnihand/omnihand_canfd_setup.md),
[can_transport_decision.md](../development/sprint5/planning/can_transport_decision.md).

## 1. Teach / MIT bringup (`start_nero_mit_controller.launch.py`)

Brings up the arm driver + MIT controller (freedrive, `hold_current`, FollowJointTrajectory). Body-mounted
arms are tilted, so pass `gravity_arm_side` — it bakes the real mount into the gravity model (see
[teach_and_run.md](teach_and_run.md) for why). With the OmniHand mounted, also pass `effector_type:=omnihand`
so the ~1 kg hand is folded into the gravity model (mount tilt alone is not enough). Do **not** pass
`input_joint_prefix` for the teach loop.

> **Shared arm+hand bus:** the `right arm + OmniHand` row puts the arm and the hand on one bus
> (`can_nero_right`). For first tests, keep `one-shot on` and just deepen the TX ring (`TX_QUEUE_LEN=1000`)
> and keep `control_rate_hz` at 100; only fall back to `ONE_SHOT=off` if the hand still starves when the MIT
> controller runs — see [teach_and_run.md](teach_and_run.md) (bus load).

| Goal | Command |
|---|---|
| single right arm | `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can_nero_right gravity_arm_side:=right` |
| single left arm | `... can_port:=can_nero_left gravity_arm_side:=left` |
| right arm + OmniHand | `... can_port:=can_nero_right gravity_arm_side:=right effector_type:=omnihand omnihand_type:=right launch_omnihand_bridge:=true omnihand_backend_type:=sdk` |
| both arms (run twice) | `... namespace:=left_arm  can_port:=can_nero_left  gravity_arm_side:=left`  **and**  `... namespace:=right_arm can_port:=can_nero_right gravity_arm_side:=right` |

Then drive it with the teach manager (`joint1..7` are the names on `feedback/joint_states`):

```bash
# single arm
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
  --arm-config src/agx_arm_coordination/config/arm_config.yaml \
  --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
# both arms simultaneously (one keyboard UI, both captured on one clock)
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
  --arm-config src/agx_arm_coordination/config/arm_config.yaml \
  --arms left_arm right_arm \
  --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
```

## 2. MoveIt / debug bringup (`start_agx_arm_components.launch.py`)

One multiplexer over `mode` × `execution_profile`. The profile resolves the Duo model slice, prefixes,
and (for `duo_arm`) per-side driver bring-up from the registry.

- `mode`: `debug_soft_target` (RViz sliders → MIT soft targets), `moveit_mit` (move_group + per-arm MIT),
  `manual_vendor`.
- `execution_profile`: `right_arm`, `left_arm`, `right_hand`, `left_hand`, `duo_arm`.

| Goal | Command |
|---|---|
| right arm, MoveIt+MIT | `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_arm can_port:=can_nero_right follow:=true use_rviz:=true planning_pipelines:=ompl` |
| right arm soft-target debug | `... mode:=debug_soft_target execution_profile:=right_arm can_port:=can_nero_right follow:=true` |
| right arm + OmniHand, MoveIt | `... mode:=moveit_mit execution_profile:=right_hand can_port:=can_nero_right follow:=true use_rviz:=true planning_pipelines:=ompl omnihand_backend_type:=sdk omnihand_device_id:=1 omnihand_canfd_id:=0` |
| both arms, MoveIt+MIT (live) | `... mode:=moveit_mit execution_profile:=duo_arm follow:=true use_rviz:=true planning_pipelines:=ompl` |
| both arms, planning only (no hw) | `... mode:=moveit_mit execution_profile:=duo_arm follow:=false use_rviz:=true planning_pipelines:=ompl` |

The `right_hand`/`left_hand` SDK paths use the repo's normal OmniHand auto-discovery path and do not need
manual `PYTHONPATH` or `LD_LIBRARY_PATH` exports during ROS launch. Use `sdk_python_dir` or
`AGX_ARM_OMNIHAND_SDK_DIR` only when the built vendor package lives outside the repo checkout; see
[omnihand_solo_bringup_and_load_test.md](../assets/omnihand/omnihand_solo_bringup_and_load_test.md). The
`duo_arm` slice launches each side's driver itself (per-side `can_port` from the registry) and merges
per-arm feedback into `/feedback/prefixed_joint_states` for `move_group`.

## 3. Model-only validation (RViz, no hardware)

```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true gui:=true use_rviz:=true
```

## 4. Lower-level overrides

Use the profiles above by default. Only when a preset is not enough, the direct wrappers
(`start_agx_arm_moveit.launch.py`, `start_single_agx_arm_rviz.launch.py`) accept explicit `custom_model`,
`custom_model_xacro_args`, and `input_joint_prefix`. The `execution_profile` presets exist precisely so you
do not wire these by hand.

## See also

- Teach loop, gravity-mount tuning, duo-vs-parallel routing → [teach_and_run.md](teach_and_run.md)
- Per-arm vs Duo interaction analysis → [single_vs_multi_arm_control_chain.md](../assets/control/single_vs_multi_arm_control_chain.md)
- CAN transport → [../assets/omnihand/omnihand_canfd_setup.md](../assets/omnihand/omnihand_canfd_setup.md)
