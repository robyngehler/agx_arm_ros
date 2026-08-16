# Sprint 6 Target
status: PAUSED_PENDING_V02_REFACTOR
last_updated: 2026-08-11

Sprint 6 is the coordinated dual-arm plus dual-hand task layer.

> **Paused.** The V02 refactor takes priority on safety, CPU relief, and
> parallel operation; the demo is not meaningful before those land. The active
> entrypoint is `docs/sprint_refactor/`, canonical plan
> `docs/sprint_refactor/planning/integration_plan.md`.
>
> Two premises of this sprint have changed. Each device now has its own CAN bus
> (arms `can_nero_left`/`can_nero_right`, hands `hand_left`/`hand_right` on
> USB-CAN FD adapters), so the
> **step-and-settle and hand-window planning notes here are superseded** —
> same-side arm and hand motion runs in parallel and step-and-settle is a
> selectable degraded mode. The hand message surface is also being consolidated
> into one abstract hand contract.
>
> Sprint 6 resumes and adapts to the resulting contracts once the refactor
> phases land. Its remaining hardware-validation items become regression
> criteria for the refactor rather than independent work.

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
