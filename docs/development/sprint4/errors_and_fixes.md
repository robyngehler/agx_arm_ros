# Sprint 4 Errors And Fixes

## 2026-05-28: `duo_body_description` could not be built into the active ROS overlay

Problem:

- `src/duo_body_description/CMakeLists.txt` tried to install an `rviz/` directory that does not exist in the package.
- That blocked `colcon build --packages-select duo_body_description`, so the intended ROS-native `xacro`, `check_urdf`, and launch validation could not run even in a correct shell.

Fix:

- Remove the nonexistent `rviz/` directory from the package install stanza.
- Rebuild `duo_body_description` in a ROS-capable shell and rerun the intended `xacro`, `check_urdf`, and headless launch validation.

Verification:

- `colcon build --packages-select duo_body_description` completed successfully.
- `xacro` + `check_urdf` succeeded for the `right`, `left`, and `both` profiles.
- `ros2 launch duo_body_description display_duo_system.launch.py gui:=false use_rviz:=false ...` started successfully for the `right` and `left` slices.

## 2026-05-28: `duo_body_description` conflicted with the existing package-structure policy

Problem:

- The repository policy still documented `agx_arm_description` as the only discoverable description package.
- `src/duo_body_description` already existed in `src/`, which made the code and docs disagree.

Fix:

- Document `src/duo_body_description` explicitly as a Sprint 3 and Sprint 4 staging package in `AGENTS.md`, `docs/project/`, the global development docs, and the `.github` mirrors.
- Keep the long-term canonical description ownership in `src/agx_arm_sim/agx_arm_description` so the staging package is a documented exception with an exit path rather than an accidental fork.

## 2026-05-28: link-name collisions between the Nero chain and OmniHand base links

Problem:

- The current OmniHand descriptions already use `left_base_link` and `right_base_link`.
- A naive prefixed Nero chain using only `left_` and `right_` would collide conceptually with those hand base links and make the combined body system harder to reason about.

Fix:

- Use `left_arm_` and `right_arm_` as the current Nero chain prefixes.
- Keep the current OmniHand `left_base_link` and `right_base_link` names unchanged.

## 2026-05-28: missing ROS tooling in the current Windows shell

Problem:

- The active Windows PowerShell environment did not expose `xacro` or `ros2` on `PATH`.
- That blocked direct execution of the new Duo system Xacro validation from this shell.

Fix:

- Keep editor diagnostics as the immediate static validation.
- Record package-scoped ROS validation as the next step in a ROS-capable shell rather than pretending it already happened.

Status update:

- A later Linux ROS shell pass in this workspace completed the queued package-scoped validation; the remaining gap is now RViz visualization and physical mount measurement, not basic ROS tool availability.