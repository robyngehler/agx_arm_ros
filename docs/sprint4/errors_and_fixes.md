# Sprint 4 Errors And Fixes

Historical issue summary for the first Duo body system baseline.

## `duo_body_description` initially could not build cleanly

Problem:

- the package install rules referenced files that did not exist, which blocked the intended
	ROS-native validation path

Fix:

- clean up the package install stanza
- rerun the package-scoped build plus structural validation

## Package policy and code drifted apart

Problem:

- `src/duo_body_description` already existed in the codebase, but the stable policy docs had not yet
	acknowledged it as a staging package

Fix:

- document the staging role explicitly in the stable docs and agent mirrors while keeping the
	canonical long-term ownership in the existing `agx_arm_*` packages

## Launch naming and shared-vs-per-arm execution needed clarification

Problem:

- canonical MoveIt launch naming and the actual multi-arm runtime contract were temporarily misaligned

Fix:

- move the real package-local MoveIt implementation under `start_moveit.launch.py`
- document the per-arm MIT execution split and the shared planning-level wrapper contract