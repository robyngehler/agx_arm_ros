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

> **V02 refactor in progress.** Each device now has its own CAN bus (arms
> `can0`/`can1` native, hands `can2`/`can3` on USB-CAN FD adapters), so same-side
> arm and hand motion run in parallel and the shared-bus hand window is a
> selectable degraded mode, not normal operation. The hand command surface is
> moving to one abstract, owned hand contract. Before changing the bridge, read
> `docs/sprint_refactor/planning/integration_plan.md` (constraints C1 and C5,
> phases 2A-2D and 4D). The rules below describe the current baseline; do not
> build new work on the surfaces marked legacy.

## Placement Rule

- keep the bridge in `src/agx_arm_ctrl` in the current baseline
- revisit a dedicated package only after a non-mock backend proves a clear dependency or public-contract boundary

## Public ROS Contract

- shared `control/joint_states` is the current arm-plus-hand command flow and is
  legacy: the V02 target is one abstract hand command carrying owner identity,
  control epoch, and sequence
- keep combined `feedback/joint_states` as the canonical follow-mode state
- publish hand-only debug and diagnostics under `feedback/omnihand/*`
- keep `control/omnihand/joint_trajectory` as a bridge-specific compatibility surface while the longer-term action or controller contract is still open
- keep `control/omnihand/stop` as the hand-specific safe-stop surface

## Message Rules

- use standard ROS messages where they already fit, such as `sensor_msgs/JointState`
- keep hand diagnostics in `agx_arm_msgs`
- do not extend the Revo2-specific messages for OmniHand, and do not add a third
  OmniHand-only message either: the V02 target consolidates `HandCmd`,
  `HandPositionTimeCmd`, `HandStatus`, `GripperStatus`, and `OmniHandStatus`
  into one abstract hand contract that must fit any hand
- keep fields statically defined; no runtime-variable structure in control paths

## Backend Rules

- resolve each hand's SocketCAN interface from its own registry entry; never
  derive it from the arm's `can_port` and never fall back silently on a hardware
  profile
- select the interface through explicit backend construction, not through the
  process-global `OMNIHAND_SOCKETCAN_IFACE` environment variable
- keep the SDK or vendor transport below ROS
- treat vendor ROS topics as backend input references only, not as the public repo contract
- keep the mock backend and real backend behind the same repo-owned bridge surface when possible

## Validation Rule

For bridge changes, run diagnostics on the touched files and at least one package-scoped build such as
`colcon build --packages-select agx_arm_ctrl agx_arm_msgs`.
