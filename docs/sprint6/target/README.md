# Sprint 6 Target
status: ACTIVE_SPRINT_ENTRYPOINT
last_updated: 2026-07-18

Sprint 6 is the coordinated dual-arm plus dual-hand task layer.

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
