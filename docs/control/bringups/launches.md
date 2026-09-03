# Launches

status: ACTIVE_BASELINE
last_updated: 2026-07-19

How to start the current system. Two steps: bring up the baseline that owns the runtime graph, then
add the matching tool or demo on top.

Conventions: **one bus per device.** `can_nero_right` and `can_nero_left` are the native arm buses;
`hand_right` and `hand_left` are the hands' own USB-CAN FD adapters. Each hand's interface is
declared in `duo_motion_registry.yaml` and never derived from its arm's `can_port` — a bridge with
no interface of its own now refuses to start rather than falling back onto the arm bus. (Until
2026-08-13 a side bus carried both its arm and its OmniHand; that shared topology is still
selectable as a degraded mode, but it is no longer what a bring-up does by default.)
Arms are separated by ROS namespace (`left_arm` and `right_arm`); each driver
publishes unprefixed `joint1..7` inside its namespace. `execution_profile`
(`src/agx_arm_ctrl/config/execution_profiles.yaml` and `duo_motion_registry.yaml`) is the source of
truth for slice composition, prefixes, frames, side buses, and gravity URDF derivation.

## Operator entry points

The scripts below are the supported way to run a demo. They wrap the launch lines
in the rest of this page, wait for the surfaces the next step needs, and run the
activity through the existing `run_activity` client. The raw commands stay
documented here because the scripts are a convenience, not a new contract.

```bash
sudo bash scripts/activate_stack.sh            # buses up and verified
./scripts/unpack_bottom_unit.py --slow         # bottom unit out of its packing pose
./scripts/pack_bottom_unit.py --slow           # and back into it
./scripts/unpack_top_unit.py                   # top unit into Functional_Init_Both_V03
./scripts/pack_top_unit.py                     # and back
./scripts/start_tea_demo.py                    # tea_pour_duo_v2
```

Each waits for the stack, prints what is live and how many operator steps the
activity has, then blocks on Enter. Common options: `--from-id N` resumes at
operator step N, `--no-prompt` skips the Enter gate, `--dry-run` brings the stack
up and sends nothing, `--log-dir PATH` keeps the launch logs somewhere you choose.
The bottom-unit scripts also take `--speed fast|slow` (`--fast` / `--slow`).

**Ctrl+C goes to `run_activity`, not to the stack.** The first press cancels the
activity, the second escalates to the unit emergency stop. The launches are torn
down only after the client has returned, so the coordinator still has a driver to
unwind against.

**`--from-id` counts operator steps, not graph nodes.** One step is one dispatch
batch, so a synchronized arm-plus-hand pair is a single step. A step that replays
a taught path is refused as a resume point — a replay commands taught joint
angles from wherever the arm stands — and the refusal names the nearest earlier
step that plans its own approach. On `tea_pour_duo_v2` those are steps 3, 6, 9,
10, 12, 18 and 19 of 21; the pack and unpack flows have none, so any of their
steps is a valid resume point.

## 0. CAN buses

`activate_stack.sh` is the entry point: it calls `activate_duo_can.sh` for the
bring-up, then samples each bus and judges it — RX advancing on the arms, the
controller ERROR-ACTIVE, error counters flat — and runs a bounded
`rmmod`/`modprobe`/reactivate cycle when a bus does not pass.

```bash
sudo bash scripts/activate_stack.sh            # activate, verify, recover if needed
sudo bash scripts/activate_stack.sh --recover  # go straight to the driver reload
bash scripts/activate_stack.sh --show          # report state, change nothing
```

The bring-up itself, all four buses matched by physical slot (the two hand
adapters are identical hardware, so their `canN` indices can swap between boots):

```bash
sudo bash scripts/activate_duo_can.sh          # all four
sudo bash scripts/activate_duo_can.sh arms     # or: hands
bash scripts/activate_duo_can.sh --show        # report state, change nothing
```

Arms only, by name:

```bash
sudo bash scripts/activate_duo_can.sh
# or: sudo bash scripts/activate_duo_can.sh arms
ip -details link show can_nero_right
```

`activate_duo_can.sh` sets `txqueuelen 1000` itself. The kernel default of 10 was measured to cost
roughly double per transmitted frame on an arm bus (`move_mit` 0.61 ms mean falling to 0.38 ms), so
a bus brought up by hand should be deepened the same way:

```bash
TX_QUEUE_LEN=1000 sudo bash scripts/activate_duo_can.sh arms
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

Tea pour (`tea_pour_left_v1`) — the first end-to-end demo, left arm plus left hand, both sides live.
**Validated on hardware 2026-08-17**, three complete runs across two bring-ups, with this composition.

```bash
# 1. components baseline — note the profile: the tea launch owns the hand bridges
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  execution_profile:=duo_hand_external_bridge \
  payload_mass_kg:=1.0 \
  use_rviz:=false

# 2. hand bridges, skill controllers and the coordinator
ros2 launch agx_arm_coordination start_tea_demo.launch.py backend_type:=sdk

# 3. run it
ros2 run agx_arm_coordination run_activity --activity tea_pour_left_v1
```

**Use `duo_hand_external_bridge`, not `duo_hand`.** `start_tea_demo.launch.py` starts the hand
bridges itself, so the generic `duo_hand` profile — which sets `launch_omnihand_bridge: true` —
gives you two bridges per hand fighting over one adapter. The two profiles are otherwise identical.
The generic `duo_hand` profile keeps its own separate validation status; the tea run does not
transfer to it, because it was not the profile used.

**`payload_mass_kg` is not optional here.** It defaults to `0.0`, which preloads no second gravity
model — and the activity's `payload_update: attach` on action 70 is then *refused*, aborting the
run. The demo's teapot value is `1.0`.

Expect ~93 s per activity. Full runbook, CPU budget and stop behaviour:
[`tea_demo.md`](tea_demo.md); the run record is
[`../../sprint6/evidence/tea_pour_left_v1_2026-08-17.md`](../../sprint6/evidence/tea_pour_left_v1_2026-08-17.md).
`Ctrl+C` on either the coordinator or the client cancels the activity and pins the arm rather than
just exiting; with no activity running, the coordinator exits on the first interrupt.

Hefeweizen coordinator: start the components baseline with `mode:=moveit_mit`, then run:

```bash
ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py
```

The staged dry-run and live sequence remains in `teach_and_run.md`.

## References

- `../environment.md`: build, test, and runtime wrapper rules
- `tea_demo.md`: the tea-pour demo runbook (chain, CPU budget, stop behaviour)
- `teach_and_run.md`: teach loop, gravity, bus details, and coordinator-facing motion flow
- `../../assets/control/single_vs_multi_arm_control_chain.md`: per-arm versus Duo interaction analysis
- `../../assets/omnihand/omnihand_canfd_setup.md`: CAN transport and hardware bringup details
- `../../sprint6/planning/decision_record.md` §3: how the duo-hand profile was closed, and its validation state