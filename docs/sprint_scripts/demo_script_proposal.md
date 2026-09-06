# Demo Script Proposal

status: PROPOSAL
last_updated: 2026-09-02
scope: stable operator scripts for stack activation, demo start, resume, and recovery

## Goal

This proposal defines a small operator-facing script layer for the current Duo
demo stack.

The scripts should:

- activate and verify the four-bus CAN stack
- start the required ROS launches for pack, unpack, and tea-demo runs
- wait for an explicit Enter before the first motion command is sent
- stop safely on `Ctrl+C`
- resume a run from a numbered step after an interruption
- stay callable from later Python or JavaScript wrappers without adding a new
  ROS-specific UI layer first

The intent is to reuse the current runtime surfaces and add only the smallest
code and activity changes needed for stable operation.

## Current anchors

The proposal builds on surfaces that already exist in the repository.

- `scripts/activate_duo_can.sh` already brings up all four CAN interfaces by
  fixed physical slot, not by unstable `canN` numbering.
- `agx_arm_coordination run_activity` already owns the safe cancel path: the
  first `Ctrl+C` cancels the activity, and a second interrupt escalates to the
  unit emergency stop.
- `src/agx_arm_coordination/launch/start_tea_demo.launch.py` already owns the
  tea-demo hand bridges, skill controllers, and coordinator.
- `src/agx_arm_coordination/launch/start_coordination.launch.py` already brings
  up the coordinator alone for activities that do not need the tea-demo hand
  stack.
- `src/agx_arm_coordination/config/activities/unit_unpack_fast_v1.yaml` and
  `unit_unpack_slow_v1.yaml` already contain the bottom-unit motion path, but
  both are round-trip activities today: they unpack and then pack back in the
  same graph.
- `src/agx_arm_coordination/config/activities/tea_pour_duo_v2.yaml` already
  contains the top-unit path between the packing pose and
  `Functional_Init_Both_V03` as part of the tea-demo sequence.
- `PerformActivity.metadata_json` is already the smallest coordinator entry
  point for new run-time controls such as resume.

These anchors are strong enough that the new script layer can stay thin. The
root change is not a new launch architecture; it is a small amount of activity
refactoring plus one shared resume mechanism.

## Proposed operator surface

The user-facing script names should match the operator verbs directly.

| Script | Purpose | Launch stack | Activity |
| --- | --- | --- | --- |
| `activate_stack` | Verify and activate CAN; recover missing interfaces if needed | none | none |
| `unpack_bottom_unit` | Move from the bottom packing pose into the working pose | components + coordinator | new bottom unpack activity |
| `pack_bottom_unit` | Move from the bottom working pose back into the packing pose | components + coordinator | new bottom pack activity |
| `unpack_top_unit` | Move from the top packing pose into `Functional_Init_Both_V03` | components + coordinator | new top unpack activity |
| `pack_top_unit` | Move from `Functional_Init_Both_V03` back into the top packing pose | components + coordinator | new top pack activity |
| `start_tea_demo` | Bring up the tea-demo stack and run `tea_pour_duo_v2` | components + tea-demo coordination launch | `tea_pour_duo_v2` |

The existing ROS launch file name `start_tea_demo.launch.py` stays unchanged.
The operator script can still be named `start_tea_demo`; the distinction is the
path and entry point, not the string alone.

## Core decisions

### 1. Keep CAN activation shell-based, and move demo orchestration to Python

`activate_stack` should remain a shell script, because its job is system-level:
network interfaces, kernel modules, and `sudo` calls.

The demo scripts should be Python CLIs with one shared implementation module.
That gives us:

- better process control than shell pipelines
- clean `Ctrl+C` forwarding
- Enter gating without fragile subshell tricks
- one place for readiness checks and step mapping
- direct reuse from future Python or JavaScript wrappers

Thin shell shims may still be added for convenience, but the logic should live
in one Python surface.

### 2. `--from_id` must mean step number, not raw `action_no`

The operator contract should be a linear step count `1..N`.

It must not be the raw graph `action_no`, because synchronized groups are one
operator step even when the activity graph represents them as several nodes.
In `tea_pour_duo_v2`, for example, these pairs must resume together:

- step 4: `both_arms_to_can_grip_idle` and `left_hand_can_pre_grip`
- step 8: `left_hand_can_grip` and `both_arms_to_can_adjust_while_grip`
- step 17: `both_arms_to_functional_init` and `left_hand_fist`

The smallest correct model is therefore:

- collapse each dispatch batch into one operator-visible step
- number those steps from `1` upward
- let `--from_id N` mean: start at batch `N`, not at graph node `action_no=N`

This keeps the CLI stable and keeps synchronized motion atomic at resume time.

### 3. Resume should be implemented once in the coordinator path

The resume mechanism should be coordinator-owned, not script-owned.

The scripts should not rewrite YAML files, splice activities on disk, or guess a
resume path from log text. Instead:

- `run_activity` grows a CLI option `--from-id`
- the client passes it through `metadata_json`
- the coordinator resolves the requested start step against the activity graph
- the same shared helper computes the linear step mapping for all scripts

A reasonable metadata shape is:

```json
{
  "resume": {"from_step": 8},
  "playback": {"mode": "tempo_scale", "speed_scale": 0.6}
}
```

The playback override stays as it is today; resume becomes one more run-time
override next to it.

### 4. Enter gating belongs in the wrapper, not in launch files

The launches should keep doing one thing: start ROS nodes.

The new scripts should:

- bring the required launches up
- wait until the stack is actually ready
- print a short summary of what is live and what activity will run
- block on Enter before sending the `run_activity` goal

This matters for two reasons.

- The same script must also run non-interactively later from a web wrapper.
- The readiness barrier is a property of the operator workflow, not of the ROS
  launch description.

Each script should therefore support both:

- interactive mode: wait for Enter
- non-interactive mode: `--no-prompt` or `--auto-start`

### 5. `Ctrl+C` must flow through `run_activity` first

The existing `run_activity` client already has the right safety semantics. The
wrapper should reuse them rather than replacing them.

During motion, the process model should be:

1. the wrapper starts the background launches
2. the wrapper waits for readiness
3. the wrapper starts `run_activity` in the foreground
4. `Ctrl+C` is delivered to `run_activity` first
5. only after the client returns does the wrapper tear the background launches
   down

This preserves the current cancel and e-stop ladder instead of killing the
coordinator or MoveIt stack out from under it.

### 6. Re-arm after e-stop or bus recovery must stay explicit

The scripts may automate detection and guidance, but they should not silently
clear lockouts by default.

After an e-stop or a recovery, the arm stack may require:

- `/left_arm/clear_fault_lockout`
- `/right_arm/clear_fault_lockout`
- `/unit_safety/rearm`

The default behavior should be:

- detect that the stack is latched or fault-locked
- print the exact re-arm commands or call them only after explicit operator
  confirmation
- refuse to send new motion before the re-arm step succeeded

This keeps the recovery boundary visible and avoids an automatic re-enable after
an unverified physical stop.

## Activity changes

### Bottom unit

The current bottom-unit graphs are round trips. For stable scripts and stable
resume they should be split into one-way activities.

Proposed new activities:

- `unit_unpack_bottom_fast_v1`
- `unit_unpack_bottom_slow_v1`
- `unit_pack_bottom_fast_v1`
- `unit_pack_bottom_slow_v1`

They should reuse the existing actions from `config/catalogue.d/unit_unpack_v1.yaml`.

Expected one-way step counts:

- bottom unpack fast: 3 steps
- bottom unpack slow: 5 steps
- bottom pack fast: 3 steps
- bottom pack slow: 5 steps

The first node should remain a move to the declared start pose instead of
assuming the current pose. That is what makes the activity repeatable after an
unclean prior stop.

### Top unit

The top-unit motion should be added as two new activities using the existing tea
catalogue actions.

Proposed new activities:

- `unit_unpack_top_v1`
- `unit_pack_top_v1`

They should reuse:

- `both_arms_to_packing_pose`
- `both_arms_to_functional_init`

The recommended shape is the same as the bottom-unit flows: move to the declared
start pose first, then move to the target pose. That makes the activity runnable
from an arbitrary state and gives resume a stable first step.

Expected one-way step counts:

- top unpack: 2 steps
- top pack: 2 steps

### Tea demo

The tea-demo activity itself stays `tea_pour_duo_v2`.

The new work is not a new tea-demo graph. It is:

- wrapper orchestration around the current tea-demo launches
- batch-based resume on the existing 17-step sequence
- operator-friendly start and recovery behavior

The current right-hand constraint remains unchanged: the right hand is out of
service and the activity commands it nowhere.

## Launch composition

### Bottom and top pack or unpack scripts

Use:

- `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py`
- `mode:=moveit_mit`
- `execution_profile:=duo_hand`
- `follow:=true`
- `planning_pipelines:=ompl`
- `use_rviz:=false`

Then start:

- `ros2 launch agx_arm_coordination start_coordination.launch.py`

Rationale:

- these flows are arm-only, so the tea-demo hand skill controllers are not
  needed
- `duo_hand` keeps both OmniHands in the model, so gravity and collision geometry
  stay correct
- `duo_arm` would remove the hand mass and geometry from planning and gravity,
  which is unnecessary risk for a packing workflow

### Tea demo script

Use the existing documented composition:

- `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py`
- `mode:=moveit_mit`
- `execution_profile:=duo_hand_external_bridge`
- `follow:=true`
- `planning_pipelines:=ompl`
- `use_rviz:=false`
- `payload_mass_kg:=1.0`

Then start:

- `ros2 launch agx_arm_coordination start_tea_demo.launch.py backend_type:=sdk`

Rationale:

- the tea-demo launch owns the hand bridges itself
- `duo_hand` would double-start those bridges and violate the one-owner rule for
  the vendor SDK session
- `payload_mass_kg:=1.0` is required because the grip action attaches the teapot
  gravity model during the sequence

## Readiness contract

A script should not declare success because a process exists. It should wait for
the ROS surfaces the next step actually needs.

### Common readiness checks

- `/move_action` available
- `execute_activity` available
- `/left_arm/emergency_stop` available
- `/right_arm/emergency_stop` available
- `/unit_safety/rearm` available
- combined `feedback/joint_states` active

### Additional tea-demo readiness checks

- `/left_hand/perform` available
- `/right_hand/perform` available

If the script is interactive, it should print the ready state and then wait for
Enter. If it is non-interactive, it should continue immediately once the same
checks pass.

## `activate_stack` proposal

`activate_stack` should be a stable wrapper around the existing four-bus bring-up.

Normal path:

1. inspect current interface state
2. confirm that `can_nero_right`, `can_nero_left`, `hand_right`, and
   `hand_left` are present on the expected physical devices
3. run the standard activation path
4. verify that every required interface is up with the expected name

Recovery path when an interface is missing or bound to the wrong device:

1. retry the standard activation once
2. if a hand interface is missing, reload `peak_usb` and reactivate
3. if an arm interface is missing, reload `mttcan` and reactivate
4. if needed, bring the link down first before reactivation
5. fail hard if the required interface still does not exist after recovery

This script should assume `sudo` is available without a password prompt, as
planned for the deployment host.

## Script CLI proposal

For operator simplicity and future wrappers, every Python demo script should use
the same small set of options.

Recommended common options:

- `--from-id N` or `--from_id N`: start from operator step `N`
- `--no-prompt`: do not wait for Enter once ready
- `--dry-run`: bring the stack up, validate readiness, do not send the activity
- `--timeout-sec T`: readiness timeout
- `--log-dir PATH`: optional run log directory

Bottom-unit scripts also need a speed selector. The cleanest shape is:

- `--speed fast`
- `--speed slow`

If exact user-facing flag names matter more than parser cleanliness, aliases such
as `--fast` and `--slow` can still map onto the same internal option.

## Recovery model

The scripts should recognize two kinds of recovery.

### Soft recovery

The activity was canceled, but the stack is still healthy.

Expected script behavior:

- keep or restart the stack cleanly
- print the last completed step and the next valid `--from-id`
- allow the operator to relaunch from that step directly

### Hard recovery

The run ended in e-stop, transport failure, or fault lockout.

Expected script behavior:

1. stop using the current runtime instance
2. run `activate_stack` in recovery mode if the bus state is suspect
3. require explicit re-arm of lockout and unit safety
4. only then allow a resumed activity run

The scripts should never pretend that a hard recovery is equivalent to a normal
cancel.

## Implementation phases

### Phase 1: activity cleanup

- add the four one-way bottom-unit activities
- add the two one-way top-unit activities
- add tests that validate their graphs and expected step counts

### Phase 2: shared step mapping and resume

- add one shared helper that collapses an activity into operator-visible steps
- add `--from-id` to `run_activity`
- extend the coordinator to accept resume via `metadata_json`
- reject invalid resume requests early and clearly

### Phase 3: operator wrappers

- add the shared Python orchestration module under `scripts/`
- add `unpack_bottom_unit`, `pack_bottom_unit`, `unpack_top_unit`,
  `pack_top_unit`, and `start_tea_demo`
- make them wait for readiness and then block on Enter unless `--no-prompt` is
  set

### Phase 4: stack activation hardening

- add `activate_stack`
- fold the current `activate_duo_can.sh` path into it
- add driver reload and link-reset recovery paths for missing interfaces

### Phase 5: docs and hardware closure

- update `docs/control/bringups/launches.md`
- update `docs/control/bringups/tea_demo.md`
- document the pack or unpack flows once the final script names and activities
  land
- capture the hardware validation record for cancel, resume, and recovery

## Validation ladder

### L1

- unit tests for the new activity graphs
- unit tests for the step-collapse helper
- unit tests for `--from-id` validation and metadata parsing
- unit tests for wrapper decision logic that does not need ROS

### L2

- mock integration tests for readiness waiting
- mock integration tests for Enter gating and `--no-prompt`
- mock integration tests for `Ctrl+C` forwarding and orderly shutdown
- mock integration tests for resume on the tea-demo step model

### L3

- hardware check for `activate_stack`
- hardware run of bottom unpack and bottom pack, slow first
- hardware run of top unpack and top pack
- hardware tea-demo run on the current `tea_pour_duo_v2`
- hardware cancellation mid-run followed by `--from-id` resume
- hardware CAN recovery followed by explicit re-arm and resumed motion

## Out of scope for the first cut

The first cut should stay narrow.

- no new web service or ROS API surface
- no activity graph redesign beyond the small top or bottom pack and unpack split
- no automatic recovery that clears fault lockout without operator intent
- no attempt to generalize resume for arbitrary branching DAGs beyond the shipped
  demo flows

## Summary

The stable path is:

- split the existing pack or unpack motion into one-way activities
- add one batch-based resume mechanism in the coordinator path
- wrap the current launches in a shared Python supervisor
- keep CAN bring-up and low-level recovery in a dedicated shell script

That approach reuses the existing safety behavior, keeps the changes small, and
gives the next web-wrapper step a script surface that is already stable enough
to call directly.