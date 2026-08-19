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
> hardware on 2026-08-17, twice in one stack, on the *existing* taught data — the
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
