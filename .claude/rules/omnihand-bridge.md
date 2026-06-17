---
paths:
  - "src/agx_arm_ctrl/**"
  - "src/agx_arm_msgs/**"
  - "docs/assets/omnihand/**"
---

# OmniHand Bridge Contract

*Use when modifying the OmniHand bridge, its topics, launch arguments, messages, or package placement.
Captures the current bridge contract.*

The OmniHand bridge stays repo-owned and agx_arm-centric.

## Placement Rule

- keep the bridge in `src/agx_arm_ctrl` in the current baseline
- revisit a dedicated package only after a non-mock backend proves a clear dependency or public-contract boundary

## Public ROS Contract

- prefer shared `control/joint_states` for coordinated arm-plus-hand command flows
- keep combined `feedback/joint_states` as the canonical follow-mode state
- publish hand-only debug and diagnostics under `feedback/omnihand/*`
- keep `control/omnihand/joint_trajectory` as a bridge-specific compatibility surface while the longer-term action or controller contract is still open
- keep `control/omnihand/stop` as the hand-specific safe-stop surface

## Message Rules

- use standard ROS messages where they already fit, such as `sensor_msgs/JointState`
- keep OmniHand-specific diagnostics in `agx_arm_msgs`
- do not force OmniHand onto Revo2-specific command or status messages

## Backend Rules

- keep the SDK or vendor transport below ROS
- treat vendor ROS topics as backend input references only, not as the public repo contract
- keep the mock backend and real backend behind the same repo-owned bridge surface when possible

## Validation Rule

For bridge changes, run diagnostics on the touched files and at least one package-scoped build such as
`colcon build --packages-select agx_arm_ctrl agx_arm_msgs`.
