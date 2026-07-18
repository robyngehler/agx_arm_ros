# Sprint 4 Target

status: HISTORICAL_ENTRYPOINT
last_updated: 2026-07-18

Sprint 4 was the first body-mounted Duo system baseline.

## Main goal

Take the earlier single-arm Nero baseline and stage it into the first Duo body system slice around:

- `src/duo_body_description` as a documented staging package
- right-first then mirrored left-side body composition
- prefix-safe multi-arm MoveIt and MIT integration
- the first hand-aware per-arm config profiles

## What Sprint 4 settled

- `src/duo_body_description` became the documented staging package for the body-mounted Duo system
- the first right-side and mirrored left-side body slices validated structurally
- the first prefixed multi-arm MoveIt and MIT wrapper contracts landed
- hand-aware per-arm profiles (`left_hand`, `right_hand`) landed without forking package ownership
- a central dual-arm soft-stop helper and a fixed-pose OmniHand gravity payload slice were added

## Historical evidence kept in this sprint surface

- `../checklist.md`
- `../errors_and_fixes.md`
- `../open_questions.md`
- `../evidence/duo_system_integration_direction.md`