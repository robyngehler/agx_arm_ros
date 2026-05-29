# Sprint 3 Errors And Fixes

## 2026-05-21

- Problem: the roadmap expects `nero_arm` and `nero_tool0`, but the working MoveIt surface only exposed `arm` and `tcp_link`, which would have forced rename churn across the current launch and RViz flows.
- Fix: remove the `arm` compatibility alias from the active MoveIt surface, move `nero_tool0` into the canonical Nero description package, update `display_control.launch.py` to publish `tcp_link` from `nero_tool0`, and make the MIT gravity model prefer `nero_tool0` while falling back to legacy flange names.
- Problem: the MoveIt SRDF still carried stale `end_effector` collision disables for the `none` profile, producing repeated unknown-link warnings during startup.
- Fix: remove the stale `end_effector` collision disables and point end-effector parent groups at `nero_arm`.
- Problem: the repo had not yet selected or provisioned the roadmap's intended IK plugin, so the working MoveIt package still depended on KDL.
- Fix: switch the MoveIt config, package metadata, install script, and user docs to TRAC-IK, and document the Humble / Jetson source-build fallback plus required overlay source order in `planning/trac_ik_humble_jetson_repro.md`.
- Problem: a full sim-only profile sweep across `none`, `agx_gripper`, `revo2` left/right, and `omnihand` left/right originally reached MoveIt startup for every profile, but every profile logged a missing TRAC-IK plugin load.
- Fix: source `/opt/ros/humble/setup.bash`, `~/workspace/trac_ik_ws/install/setup.bash`, and `~/workspace/agx_arm_ros/install/setup.bash` in that order. The updated sweep helper now sources the external TRAC-IK overlay automatically when it exists.
- Verification: `ros2 pkg prefix trac_ik_kinematics_plugin` resolves to `~/workspace/trac_ik_ws/install/trac_ik_kinematics_plugin`, the six-profile sim sweep no longer logs TRAC-IK plugin-load failures, and a live `/compute_ik` request against the current `tcp_link` pose returned `MoveItErrorCodes.SUCCESS` for `nero_arm`.
- Problem: a package-scoped `ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero use_rviz:=false` smoke run reached the planning-ready state, but the timeout-driven shutdown path on this Humble host ended with a `move_group` teardown crash and forced SIGKILL.
- Current assessment: the shutdown crash is separate from TRAC-IK provisioning and naming. On this Humble/aarch64 host, SIGINT teardown triggers a `class_loader` warning about unloading libraries with live objects, followed by a `move_group` segmentation fault in `rclcpp::CallbackGroup` destruction after MoveItCpp / TrajectoryExecutionManager teardown.

## 2026-05-28

- Problem: Sprint 3 still had no reproducible repo-owned check for a representative OMPL pose-planning request beyond startup readiness and `/compute_ik`.
- Fix: add `src/agx_arm_moveit/scripts/plan_pose_smoke_test.py`, which computes FK for the `home` state, tries a small set of near-home pose offsets, and sends a `moveit_msgs/srv/GetMotionPlan` request with `pipeline_id=ompl` until one candidate succeeds.
- Verification: `/usr/bin/python3 src/agx_arm_moveit/scripts/plan_pose_smoke_test.py` succeeded locally with `candidate=x_plus_3cm`, `points=21`, and `pipeline=ompl`.
- Problem: it was still unclear whether the known `move_group` teardown crash required the default multi-pipeline configuration.
- Fix: add an explicit `planning_pipelines` override to `agx_arm_moveit/launch/move_group.launch.py` and forward it through `demo.launch.py`.
- Verification: a timeout-driven `ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=none use_rviz:=false planning_pipelines:=ompl` run still produced the same `class_loader` warning plus `rclcpp::CallbackGroup` segmentation fault on SIGINT teardown, so the crash reproduces even in the reduced OMPL-only path.