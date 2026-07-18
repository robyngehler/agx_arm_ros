# Duo System Integration Direction

Status: historical direction note.

This note captures the transition from a single-arm-first mindset to the documented body-mounted Duo
system direction.

## Direction change

The repo stopped treating the body-mounted multi-arm system as a much-later add-on and instead
staged it in this order:

1. `body + right arm + right OmniHand`
2. mirror the left side into the same top-level description and bringup surfaces
3. generalize the current single-arm RViz, MoveIt, and controller-facing surfaces in place
4. only then widen further into later coordinated runtime and demo work

## What this note still explains

- why `src/duo_body_description` exists as a staging package
- why the repo chose `left_arm_` and `right_arm_` prefixes
- why planning stayed shared while MIT execution remained per-arm
- why the first hand-aware path landed per-arm instead of inventing a premature dual-hand action surface

## Historical value

Keep this note only as the rationale record behind the current Duo staging package and the first
multi-arm wrapper contracts. The active stable behavior now lives in the control, project, and later
sprint docs.