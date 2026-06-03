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

## 2026-06-03: the canonical MoveIt launch name still pointed at a legacy implementation surface

Problem:

- `start_moveit.launch.py` had already become the intended neutral package-local entrypoint, but the real implementation still lived in `demo.launch.py`.
- That left the docs, naming, and implementation ownership misaligned just as Sprint 4 grew from a single prefixed arm slice into a shared multi-arm MoveIt wrapper.

Fix:

- Move the actual package-local MoveIt implementation into `src/agx_arm_moveit/launch/start_moveit.launch.py`.
- Reduce `src/agx_arm_moveit/launch/demo.launch.py` to a pure compatibility alias.
- Update the Sprint 4 notes and package docs so `start_moveit.launch.py` is documented as the canonical package-local MoveIt surface and `demo.launch.py` is documented as legacy compatibility only.

Verification:

- `python3 -m py_compile src/agx_arm_moveit/launch/start_moveit.launch.py`

## 2026-06-03: the first dual-arm MoveIt runtime still lacked an explicit documented shared-vs-per-arm contract

Problem:

- The multi-arm runtime wrapper could already launch one MIT controller per arm and route shared MoveIt planning across both arms, but Sprint 4 notes still described the shared surfaces as if wider runtime generalization had not landed.

Fix:

- Document the namespace-scoped per-arm runtime split in `agx_arm_ctrl` and the canonical `start_moveit.launch.py` plus `start_agx_arm_moveit.launch.py` launch hierarchy.
- Record that the combined wrapper now merges per-arm feedback back into one prefixed MoveIt/RViz stream.

Verification:

- The 2026-06-03 headless `moveit_profile:=both_arms` validation through `start_agx_arm_components.launch.py mode:=moveit_mit` reached `You can start planning now!` with `left_arm` and `right_arm` MIT controller instances running concurrently.