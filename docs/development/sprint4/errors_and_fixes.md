# Sprint 4 Errors And Fixes

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