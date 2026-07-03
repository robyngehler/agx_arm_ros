# Bringup — Ground Truth And Launch Paths

The operational source of truth for how to start the current system.

Use this file in two steps:

1. choose the **baseline** that owns the runtime graph you need
2. add the matching **tool** or **demo** on top of that baseline

`start_agx_arm_components.launch.py` is the canonical wrapper for MoveIt, RViz soft-target debug, and the
normal per-arm MIT runtime. Its `execution_profile` is the repo-owned preset surface. That preset, backed
by `src/agx_arm_ctrl/config/execution_profiles.yaml`, decides the Duo slice, arm/hand composition,
prefix/frame defaults, and the `custom_model`/`custom_model_xacro_args` that the downstream MIT launch uses
to derive the gravity URDF. When a different mounted slice is wanted, change the profile/config source of
truth instead of rebuilding the same launch from ad hoc per-command overrides.

The main categories below are:

- **Baselines**: long-lived launch surfaces that own the core runtime graph
- **Tools**: focused CLIs or helper nodes that assume a baseline already exists
- **Demos**: higher-level workflows layered on top of one of the baselines

Conventions: `can_nero_right` (`can0`) and `can_nero_left` (`can1`) are the native `mttcan` side buses;
each carries its arm (classic frames) and its OmniHand (FD/BRS). The OmniHand Pro (`o12_pro`) exposes
**12 active joints**. Arms are separated by **ROS namespace** (`left_arm`/`right_arm`), not by a joint
prefix — the driver publishes unprefixed `joint1..7` within its namespace.

## Baselines

### 1. Native CAN baseline

Bring the side buses up once before any runtime launch:

```bash
sudo bash scripts/activate_native_can.sh          # both sides (or: ... right | left)
# For a shared arm+hand bus (first tests): TX_QUEUE_LEN=1000 sudo bash scripts/activate_native_can.sh right
ip -details link show can_nero_right              # expect: mtu 72, fd, one-shot, bitrate 1M, dbitrate 5M
```

Rationale + alternatives (USB adapters): [omnihand_canfd_setup.md](../assets/omnihand/omnihand_canfd_setup.md),
[can_transport_decision.md](../development/sprint5/planning/can_transport_decision.md).

### 2. MIT control baseline (`start_nero_mit_controller.launch.py`)

Brings up the arm driver + MIT controller (freedrive, `hold_current`, FollowJointTrajectory). Body-mounted
arms are tilted, so pass `gravity_arm_side` — it bakes the real mount into the gravity model (see
[teach_and_run.md](teach_and_run.md) for why). With the OmniHand mounted, also pass `effector_type:=omnihand`
so the ~1 kg hand is folded into the gravity model (mount tilt alone is not enough); by default the hand
rides **articulated** (live finger pose from combined feedback, `gravity_hand_payload:=static` restores the
frozen rigid payload). With a custom gravity URDF the stale hand-less
`config/nero_gravity_calibration.json` is no longer auto-applied (pass `calibration_file` explicitly for a
matching calibration). Do **not** pass `input_joint_prefix` for the teach loop.

> **Shared arm+hand bus:** the `right arm + OmniHand` row puts the arm and the hand on one bus
> (`can_nero_right`). For first tests, keep `one-shot on` and just deepen the TX ring (`TX_QUEUE_LEN=1000`)
> and keep `control_rate_hz` at 50; only fall back to `ONE_SHOT=off` if the hand still starves when the MIT
> controller runs — see [teach_and_run.md](teach_and_run.md) (bus load). The bridge itself now verifies and
> re-sends dropped hand commands (`command_retry_*`) and polls the hand readback at `joint_read_rate`
> (20 Hz default) instead of per publish tick.

| Goal | Command |
|---|---|
| single right arm | `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can_nero_right gravity_arm_side:=right` |
| single left arm | `... can_port:=can_nero_left gravity_arm_side:=left` |
| right arm + OmniHand | `... can_port:=can_nero_right gravity_arm_side:=right effector_type:=omnihand omnihand_type:=right launch_omnihand_bridge:=true omnihand_backend_type:=sdk` |
| both arms (run twice) | `... namespace:=left_arm  can_port:=can_nero_left  gravity_arm_side:=left`  **and**  `... namespace:=right_arm can_port:=can_nero_right gravity_arm_side:=right` |

This baseline is the dependency for the teach tools below. See [teach_and_run.md](teach_and_run.md) for the
full teach loop.

### 3. Components baseline (`start_agx_arm_components.launch.py`)

This is the canonical wrapper for the normal operational graph around `agx_arm_ctrl`, `agx_arm_moveit`,
and the MIT controller runtime.

- `mode` chooses the broad runtime family:
  - `manual_vendor`: driver-centric runtime without the MoveIt wrapper
  - `debug_soft_target`: RViz sliders driving soft MIT targets
  - `moveit_mit`: MoveIt planning/execution plus the per-arm MIT runtime
- `execution_profile` chooses the repo-owned mounted slice. It is the first place to look when the wrong
  arm/hand/body combination comes up.

Ground truth for `execution_profile`:

- it resolves the Duo model slice and the corresponding `custom_model_xacro_args`
- it resolves whether the slice is arm-only or arm+OmniHand
- it resolves the side-specific prefixes and frames that MoveIt and the MIT runtime expect
- in `moveit_mit`, it also determines the `custom_model` + prefix combination the downstream MIT launch uses
  to derive the gravity URDF; that is why `execution_profile:=right_hand` already yields the right-side
  body-mounted OmniHand gravity slice without spelling out `gravity_arm_side:=right` on the top-level wrapper
- if a different mounted slice or effector composition is desired, update `execution_profiles.yaml` rather
  than encoding a second truth in one-off shell commands

Operational matrix:

- single arm planning: `mode:=moveit_mit execution_profile:=right_arm` or `left_arm`
- single arm + OmniHand planning: `mode:=moveit_mit execution_profile:=right_hand` or `left_hand`
- dual-arm planning/execution: `mode:=moveit_mit execution_profile:=duo_arm`
- RViz soft-target debug for the same slices: `mode:=debug_soft_target` with the same `execution_profile`

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
[omnihand_solo_bringup_and_load_test.md](../assets/omnihand/omnihand_solo_bringup_and_load_test.md).

The `duo_arm` slice launches each side's driver itself (per-side `can_port` from the registry) and merges
per-arm feedback into `/feedback/prefixed_joint_states` for `move_group`.

### 4. Description-only baseline

Use this when only the assembled model and TF tree are needed, not the live runtime:

```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true gui:=true use_rviz:=true
```

## Tools

### Teach tools on top of the MIT control baseline

Start the MIT control baseline first, then add the teach manager or one-shot tools.

Teach manager:

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

Other focused tools stay in the same baseline family:

- recorded trajectory playback and conversion: see [teach_and_run.md](teach_and_run.md)
- gravity validation and calibration helpers: see `src/agx_arm_mit_tools` and [teach_and_run.md](teach_and_run.md)

### MoveIt-backed anchor-transition tests on top of the components baseline

Start the components baseline with `mode:=moveit_mit` and the matching `execution_profile`, then use the
teach manager's transitions mode as described in [teach_and_run.md](teach_and_run.md).

## Demos

### Coordinated activities (DAG)

Start the components baseline first, then add the coordinator demo surface.

- planning/execution substrate: components baseline with `mode:=moveit_mit`
- coordinator/demo layer: `ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py`
- teach pipeline that feeds the activity catalogue: [teach_and_run.md](teach_and_run.md)

For the staged dry-run/live sequence and how anchors, recorded trajectories, and hand skills fit together,
use [teach_and_run.md](teach_and_run.md) instead of duplicating the whole flow here.

## Lower-level overrides

Use the profiles above by default. Only when a preset is not enough, the direct wrappers
(`start_agx_arm_moveit.launch.py`, `start_single_agx_arm_rviz.launch.py`) accept explicit `custom_model`,
`custom_model_xacro_args`, and `input_joint_prefix`. The `execution_profile` presets exist precisely so you
do not wire these by hand. The same rule applies to Duo arm/hand composition and the resulting gravity slice:
change the preset/config source of truth first, not the everyday top-level command line.

## See also

- Teach loop, gravity-mount tuning, duo-vs-parallel routing → [teach_and_run.md](teach_and_run.md)
- Per-arm vs Duo interaction analysis → [single_vs_multi_arm_control_chain.md](../assets/control/single_vs_multi_arm_control_chain.md)
- CAN transport → [../assets/omnihand/omnihand_canfd_setup.md](../assets/omnihand/omnihand_canfd_setup.md)
