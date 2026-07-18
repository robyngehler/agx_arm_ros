# Sprint 3 Target

status: HISTORICAL_ENTRYPOINT
last_updated: 2026-07-18

Sprint 3 was the arm-only MoveIt and MIT hardening phase that prepared the later Duo body system
work without reopening the main runtime ownership.

## Main goal

Establish a stable Nero planning and controller baseline around:

- canonical `nero_arm` and `nero_tool0` naming
- TRAC-IK as the selected MoveIt IK baseline
- a reproducible pose-planning smoke test
- minimal prefix-safe Duo description groundwork for later multi-arm work

## What Sprint 3 settled

- `nero_arm` became the canonical planning group
- `nero_tool0` became the canonical flange alias while `tcp_link` stayed the TCP/planning frame
- TRAC-IK replaced KDL in the active MoveIt baseline
- the repo gained a reproducible OMPL pose-planning smoke test
- the first `src/duo_body_description` staging surfaces landed for the later Duo system sprint

## Historical evidence kept in this sprint surface

- `../checklist.md`
- `../errors_and_fixes.md`
- `../open_questions.md`
- `../evidence/trac_ik_humble_jetson_repro.md`
- `../evidence/stage2_mit_follow_joint_trajectory_proposal.md`