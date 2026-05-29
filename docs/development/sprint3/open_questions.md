# Sprint 3 Open Questions

## Resolved This Pass

- The narrowest executable validation path on this host for MoveIt-to-MIT joint ordering, timing, and unit assumptions is a package-scoped `agx_arm_mit_controller` test run. On 2026-05-28, `test_follow_joint_trajectory_validation` and `test_trajectory_buffer` passed locally; together with `config/nero_mit_controller_defaults.yaml`, they confirm the current action path keeps standard ROS `JointTrajectory` semantics in radians, radians per second, and seconds without requiring live hardware.
- A first representative TRAC-IK + OMPL pose-planning task is now verified locally. `src/agx_arm_moveit/scripts/plan_pose_smoke_test.py` computed FK for the `home` state and then succeeded with an `ompl` pose-plan request to a near-home `tcp_link` target offset by `+3 cm` in `x`.
- The timeout-driven `move_group` teardown crash persists even when `demo.launch.py` is reduced to `planning_pipelines:=ompl`, so the current failure does not depend on Pilz or CHOMP being loaded alongside OMPL.

## Remaining Open Questions

- Is the SIGINT teardown crash reproducible in a smaller Humble/aarch64 MoveIt setup outside this workspace, or is there a workspace-local lifetime issue layered on top of the upstream class-loader warning?
- Which additional profile, obstacle, and execution-safety variants are sufficient beyond the new near-home OMPL pose-plan smoke test to close the remaining Sprint 3 planning-path gap?