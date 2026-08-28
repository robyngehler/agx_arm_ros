# Sprint 6 Target
status: RESUMING_ON_V02_CONTRACTS
last_updated: 2026-08-18

Sprint 6 is the coordinated dual-arm plus dual-hand task layer.

> **Resuming.** The V02 refactor's Runtime RC closed 2026-08-17, so coordinated
> demo work is unblocked. Remaining Phase-4/5/6 refactor items are follow-up
> engineering; pull one forward only when it blocks an interface the demo
> actually uses.
>
> **Read [`planning/decision_record.md`](../planning/decision_record.md) §6
> before resuming.** Several premises of this sprint changed underneath it: each
> device now owns its CAN bus so same-side arm and hand motion runs in parallel
> and step-and-settle is a selectable degraded mode; a hand command carries the
> authority it was issued under and an unclaimed hand executes nothing; the MIT
> controller consumes device authority rather than a hand-window boolean; and
> the coordinator admits one activity at a time with atomic sync groups.
>
> **The demo has since run.** `tea_pour_left_v1` completed end to end on
> hardware on 2026-08-17, three times across two bring-ups, on the *existing* taught data — the
> re-teach that was expected to be needed first was not. See
> `../evidence/tea_pour_left_v1_2026-08-17.md`. What remains is calibration
> (tactile thresholds for Hefeweizen, the payload mass) and resilience work
> (the stop ladder mid-motion, coordinator-crash containment).
>
> Canonical refactor plan: `docs/sprint_refactor/planning/integration_plan.md`;
> its rationale: `docs/sprint_refactor/planning/decision_record.md`.

## Main goal

Build the orchestration and hand-skill layer on top of the existing arm and hand control so the
repo can execute the first coordinated dual-arm plus dual-hand task: pouring a Hefeweizen.

## Scope

1. An OmniHand skill controller that turns semantic skills such as `grasp_glass_until_contact`,
	`release_glass`, and `open_hand` into vendor-SDK gestures or presets and confirms them with
	tactile feedback.
2. A coordinator that runs an Activity-DAG and dispatches arm trajectories and hand skills via a
	performer-helper with clean fault propagation.

## Foundation already in place

- arm control through the MIT controller layer
- both-arms MoveIt planning baseline
- OmniHand vendor-SDK bridge backend
- native CAN baseline and shared-bus operating guidance from Sprint 5

## What this sprint adds

- semantic hand-skill abstraction and tactile-confirmed grasp/release controller
- YAML-backed activity graph and catalogue
- coordinator node and performer routing
- dual-arm teach flow, anchor capture, and recorded-trajectory conversion into catalogue entries

## Current status

- architecture and MVP decisions landed
- hand-skill controller, performer routing, and coordinator package implemented
- graph model, scheduler, and resource logic validated
- O12 Pro hand model migration landed
- hardware validation still pending for tactile thresholds, grasp presets, shared-bus timing under
	sustained motion, and the full demo escalation ladder

## Historical planning and evidence kept in this sprint surface

- `../checklist.md`
- `../errors_and_fixes.md`
- `../open_questions.md`
- `../planning/`
- `../reference/`

### The New Activity Flow:

> **Implemented 2026-08-27** as `tea_pour_duo_v2`
> (`agx_arm_coordination/config/activities/tea_pour_duo_v2.yaml`, actions in
> `config/catalogue.d/tea_pour_duo_v2.yaml`, runbook in
> `docs/control/bringups/tea_demo.md`). 20 nodes in 17 dispatch steps, with three
> of the five "simultaneously" steps as `sync_flag` groups. It validates and
> plans offline; **not run on hardware**.
>
> **The right hand is out of service** (2026-08-27), so the activity commands it
> nowhere: no `can_prep` beside the staging move, no closing `zero` or `heart` —
> it is assumed to sit flat and stay there. The right **arm** still moves, which
> means its support replay and its half of the two duo takes run with a flat hand
> where they were taught with a shaped one.
>
> **Step 5 ends at `Functional_Init_Both_V03`**, with the left hand closing to
> `fist` beside that move. The two-handed heart is dropped. The unused actions
> (`right_hand_*`, `left_hand_heart`, `left_hand_zero`, `both_arms_to_heart_top`)
> stay defined, so restoring any of them is an edge change in the activity.
>
> One deviation from the text below: step 2's last line names
> `Tee-Can_Pre_Grip_Adjust_L` twice, which would be a move to the pose the
> previous line already reached. The activity uses `Tee-Can_Adjust-While-Grip_L`
> there — it exists in `arm_config.yaml`, is otherwise unreferenced, and differs
> from `Tee-Can_Pre_Grip_Adjust_L` in the left arm only, which is the shape of a
> "move while the hand grips" step. Correct the line here or the activity,
> whichever was meant.

1. Prepare for Work (Assumption: Robot comes from packing pose with flat hands)
- move to `Functional_Init_Both_V03`
- move to `Prep_Tee-Can_Grip` and similtaneously trigger right hand to `can_prep`
- play `Prep_Tee-Can_4Grip_Right` with `speed_scale = 1` and `smoothing = 0.5`
2. Grip Tee Can
- move to `Tee-Can_Grip_Idle_L` and similtaneosly trigger left hand to `can_pre_grip`
- move to `Tee-Can_Pre_Grip_L`
- play `Tee-Can_Grip_Move_L` with `speed_scale = 1` and `smoothing = 0.5`
- move to `Tee-Can_Pre_Grip_Adjust_L`
- trigger left hand to  `can_grip_V01` and similtaneously move to `Tee-Can_Adjust-While-Grip_L` **and** update gravity with tee can (~1 kg)
3. Pour Tee Can
- play `Tee-Can_Lift_Post_Grip_L` with `speed_scale = 1` and `smoothing = 0.5`
- play `Tee-Can_GoTo_Pour_Init` with `speed_scale = 1` and `smoothing = 0.5`
- play `Tee-Can_Pour_V01` with `speed_scale = 1` and `smoothing = 0.5`
4. Bring Can back
- move to `Tee-Can_Post_Grip_L`
- move to `Tee-Can_Place_L`
- trigger left hand to `can_pre_grip` **and** update gravity to nominal
- play `Tee-Can_Release_Motion` with `speed_scale = 1` and `smoothing = 0.5`
- play `Tee-Can_Post_Place_Adjust` with `speed_scale = 1` and `smoothing = 0.5`
5. Go back to Idle
- move to `Functional_Init_Both_V03` and similtaneously trigger both hands to `zero`
- move to `Heart_Both_Top_V01` and similtaneously trigger both hands to `heart`
