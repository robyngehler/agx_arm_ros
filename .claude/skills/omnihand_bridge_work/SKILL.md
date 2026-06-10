---
name: omnihand_bridge_work
description: "Use when adding or refactoring the OmniHand bridge, its launch surface, messages, or bridge-adjacent docs in agx_arm_ros."
---

# omnihand_bridge_work

Use this skill when the task changes the OmniHand bridge, its command or feedback surface, or the docs
that define that contract.

## Load First

- `.claude/rules/repository-structure.md`
- `.claude/rules/package-naming.md`
- `.claude/rules/omnihand-bridge.md`
- `.claude/rules/local-agent-workflow.md`

## Workflow

1. Confirm whether the change belongs in `agx_arm_ctrl`, `agx_arm_msgs`, or stable docs.
2. Preserve the shared agx_arm-centric control surface unless the task explicitly changes the public contract.
3. Keep OmniHand-specific diagnostics in `agx_arm_msgs` and hand-only debug topics under `feedback/omnihand/*`.
4. Keep the bridge in `agx_arm_ctrl` during Sprint 2 unless a new package boundary is explicitly justified.
5. Update `docs/control/` and any mirrored `.claude/rules/` when the runtime contract changes.
6. Validate with diagnostics plus a package-scoped build.

## Output Checklist

- bridge and launch wiring follow current package boundaries
- command and feedback surfaces match the current docs
- `.claude/` mirrors stay consistent with stable docs
- `colcon build --packages-select agx_arm_ctrl agx_arm_msgs` or an equivalent narrow validation was run
