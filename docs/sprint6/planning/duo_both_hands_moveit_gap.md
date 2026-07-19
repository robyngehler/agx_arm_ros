# Gap: MoveIt planning groups for duo_arm + both OmniHands

Status: **implemented as `execution_profile:=duo_hand`** (2026-07-15, hardware validation pending —
see the closing section). The analysis below is kept as the record of what was blocked and why; all
three gates are closed:

- **Gate 1** — registry `allowed_effector_types` extended to `[none, omnihand]`
  (`duo_runtime_contract` messages generalized).
- **Gate 3** — `omnihand_group` in `agx_arm.srdf.xacro` is per-side instantiable
  (`group_name`/`parent_group`/`parent_link`/`link_prefix` params, legacy defaults keep the
  single-side profiles byte-identical); the dual-arm branch instantiates it per side as
  `left_hand`/`right_hand` with per-side tip frames + link prefixes, gated by the new
  `include_left_omnihand`/`include_right_omnihand` xacro args.
- **Gate 2** — `_moveit_config_builder` derives those per-side args from the `arm_instances`
  (each carries `effector_type`/`omnihand_type`), so the duo slice no longer loses the intra-hand
  collision ACM; `include_end_effector_groups` stays false for duo (the legacy single-side gates are
  not reused).
- New `duo_hand` profile in `execution_profiles.yaml` (per-instance omnihand + side bus), launch
  choices extended on all four wrapper launches.

Verified offline (`validate_duo_hand.py`, 2026-07-15): profile resolution (per-instance
effector/side/can_port), SRDF has `left_hand`+`right_hand`+`both_arms` with **no duplicate groups**,
per-side eef parents, 2×325 IntraHand ACM entries + per-side flange pairs; pinocchio+FCL on the full
both-arms-both-hands URDF: 0 self-collisions at open, right fist, left fist, and both fists
(hand-vs-arm/body pairs stay active: 1626 checked pairs). Single-side `right_hand` SRDF regression:
unchanged (`hand` group kept).

---

## Original analysis (2026-07-04, for the record)

## What already works (MIT/teach baseline, no gap)

`start_nero_mit_controller.launch.py` run twice (namespaced `left_arm`/`right_arm`), each with
`effector_type:=omnihand omnihand_type:=<side> launch_omnihand_bridge:=true`, works today with no
code changes:

- gravity URDF resolution is per-instance (`gravity_arm_side` + `hand_payload_mode`) — verified
  offline both sides independently produce a correctly side-prefixed, 26-DoF articulated model
- the OmniHand bridge is namespaced identically to the arm driver (`namespace` is forwarded end to
  end in `start_single_agx_arm.launch.py` -> `start_omnihand_bridge.launch.py`), so
  `feedback/omnihand/joint_states`, `control/joint_states`, etc. do not collide between sides
  (`/left_arm/...` vs `/right_arm/...`)
- each side's CAN interface auto-resolves from the registry (`arm.sides.<side>.can_port`), so the
  hand rides its own side's bus, mirroring the already-documented single-side congestion behavior
  (`command_retry_*`, `joint_read_rate`) independently per side
- teach_manager's arm recording/playback/transitions-status code only ever looks up its own
  `--source-joints` keys in the (superset) `feedback/joint_states` position map — the extra hand
  joint entries merged in by `effector_type:=omnihand` are silently ignored, so hands present or
  not does not change arm-recording behavior

See `docs/control/bringups/launches.md` for the resulting launch-matrix row and `docs/control/bringups/teach_and_run.md` for
the teach-manager caveat (it never records or replays hand joints; hand gestures are driven by a
separate tool such as `omnihand_exerciser`).

## What is blocked (MoveIt/components baseline: `mode:=moveit_mit execution_profile:=duo_arm`)

Two independent gates currently reject `effector_type:=omnihand` on the `both_arms` MoveIt profile,
and even removing both would not yet produce a correct SRDF:

**Gate 1 — `duo_runtime_contract.validate_duo_both_arms_contract`.** Reads
`motion_profiles.both_arms.restrictions.allowed_effector_types` from the registry (`[none]` today)
and raises before any launch happens. This is the easy part to lift once Gate 2/3 are fixed — it is
a one-line registry change plus updating the per-instance check to accept `omnihand`.

**Gate 2 — `_resolve_include_end_effector_groups` in `_moveit_config_builder.py`.** Hardcoded:
`if custom_model and moveit_profile == both_arms: return "false"`. This unconditionally disables
*all* SRDF end-effector content for any duo bring-up, regardless of `effector_type` — including the
**intra-hand collision ACM entries**, not just the MoveIt `hand` planning group. If Gate 1 alone were
lifted without touching this, a duo+hands bring-up would hit exactly the RC2 class of bug fixed for
single-arm bring-ups in `teaching_demo_03-07-2026-debug-moveit_plan_error.md` (a fist invalidates
the whole `both_arms` planning start state) — except worse, since the URDF would carry hand geometry
that the SRDF has zero self-collision exceptions for at all.

**Gate 3 — `agx_arm.srdf.xacro`'s `omnihand_group` macro is single-side-only.** It hardcodes
`<group name="hand">` and `<end_effector name="${side}_omnihand_eef" ... parent_group="${planning_group_name}">`
(a single group name, a single top-level `planning_group_name`, a single `end_effector_parent_link`
xacro arg). Calling it twice (once per side) as currently written would emit two `<group name="hand">`
blocks with the same name — SRDF/MoveIt would only recognize one, silently dropping the other side's
hand group and its intra-hand ACM protection. Needs:
- a `group_name` param (e.g. `${side}_hand`) instead of the hardcoded `"hand"`
- `parent_group="${side}_arm"` instead of the single `${planning_group_name}` (only correct for the
  single-arm profiles where `planning_group_name == moveit_profile`)
- a per-side `end_effector_parent_link` (currently one xacro arg for the whole file); the natural
  values are `left_arm_nero_tool0` / `right_arm_nero_tool0`, already resolvable from
  `ARM_SIDES[side]["tip_frame"]` in `_multi_arm_runtime.py`, same source the existing
  `left_arm_tip_frame`/`right_arm_tip_frame` srdf_mappings already use
- the intra-hand ACM block (currently bundled inside the same gated macro) should probably emit
  whenever hand geometry exists for that side, independent of whether a MoveIt-controllable `hand`
  group is wanted — otherwise a future "collision-safe but not directly hand-planned" duo+hands
  bring-up is stuck with the same gate

## What would still need building on top, once the SRDF gap is closed

- `execution_profiles.yaml`: a `duo_hands` profile (`custom_model_xacro_args` with
  `use_left_hand`/`use_right_hand: true`, `arm_instances` with per-side `effector_type: omnihand`,
  `omnihand_type: left`/`right`, `launch_omnihand_bridge: true` — mirrors `right_hand`'s pattern,
  applied per instance instead of at the top level)
- `_moveit_config_builder.py`: thread the per-side hand flags from `arm_instances` (which already
  carries `omnihand_type` per instance, see `_build_mit_trajectory_execution`) into the new SRDF
  per-side args instead of the current single top-level `effector_type`/`omnihand_type`
- the hand FJT controller registration (`omnihand_controller_path`/`omnihand_controller_joint_names`
  in `_multi_arm_runtime.py`) is **already per-instance** and needs no change — confirmed by reading
  `_build_mit_trajectory_execution`, which already loops `arm_instances` and registers one hand
  controller per instance that carries `omnihand_type`

## Suggested order if picked up

1. Registry: extend `allowed_effector_types` for `both_arms` to include `omnihand` (Gate 1)
2. SRDF: parameterize `omnihand_group` (group name, parent_group, per-side tip frame) and split ACM
   emission from MoveIt-group emission (Gate 3), then stop forcing
   `include_end_effector_groups:="false"` for the duo+custom_model case when hands are requested
   (Gate 2)
3. `execution_profiles.yaml` + `_moveit_config_builder.py`: add `duo_hands`, thread per-side flags
4. Validate offline exactly like the single-arm SRDF fix: generate the duo+hands SRDF via `xacro`,
   parse for duplicate group names, run the pinocchio+FCL fist-pose collision repro per side
5. Hardware: `t` transitions mode from the teach manager against the `duo_hands`
   `execution_profile`, fist on either hand, confirm arm planning is unaffected

---

## Remaining hardware validation (open, 2026-07-15)

Steps 1–4 above are done (profile is named `duo_hand`, singular, matching `left_hand`/`right_hand`).
Step 5 is open and additionally blocked by a hardware finding: **the left arm currently sends
nothing on `can_nero_left`** (interface UP/ERROR-ACTIVE, `candump` completely silent while
`can_nero_right` streams normal feedback) — check power/wiring/connector side assignment before
retesting `duo_arm`/`duo_hand`. The related software bug (side profiles silently defaulting to the
right bus, so `left_hand` connected the RIGHT arm) is fixed: side profiles now inherit
`arm.sides.<side>.can_port` from the registry, explicit `can_port:=...` still wins.
