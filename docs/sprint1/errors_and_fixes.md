# Sprint 1 Errors And Fixes

Historical issue summary for the repository and asset discovery phase.

## Duplicate description ownership

Problem:

- the workspace exposed both a root `agx_arm_description` package and the sim-backed
	`src/agx_arm_sim/agx_arm_description` package

Fix:

- keep the sim-backed package as canonical
- remove the duplicate root package and align launch compatibility there

## Description package and RViz compatibility drift

Problem:

- the current control-side RViz path expected a compatibility interface that the canonical sim-backed
	description package did not provide yet

Fix:

- add the control-compatible display launch path in the canonical description package and repoint
	the runtime-facing RViz path there

## OmniHand vendor path was not a drop-in local runtime surface

Problem:

- the vendor SDK and asset bundle were useful for discovery, but not a clean repo-owned ROS runtime
	surface on their own

Fix:

- keep the SDK vendored for isolated validation
- choose the wrapper-first integration direction
- avoid overlaying the vendor ROS packages directly into the main workspace