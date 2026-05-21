# Sprint 3 Open Questions

## Remaining Open Questions

- Which representative pose-planning cases beyond the verified live `/compute_ik` call are sufficient to qualify the TRAC-IK + OMPL baseline for Nero?
- What is the narrowest executable validation path on this host for confirming MoveIt-to-MIT joint ordering, timing, and units without depending on live hardware?
- Does the timeout-driven `move_group` teardown crash persist if the launch is reduced to the OMPL pipeline only, or is the current multi-pipeline teardown path part of the trigger?
- Is the SIGINT teardown crash reproducible in a smaller Humble/aarch64 MoveIt setup outside this workspace, or is there a workspace-local lifetime issue layered on top of the upstream class-loader warning?