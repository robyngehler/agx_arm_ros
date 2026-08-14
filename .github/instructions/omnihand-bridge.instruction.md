---
description: "Use when modifying the OmniHand bridge, its topics, launch arguments, messages, or package placement. Captures the current bridge contract."
---

# OmniHand Bridge Contract

The OmniHand bridge stays repo-owned and agx_arm-centric.

> **V02 refactor in progress.** Each device has its own CAN bus — arms on
> `can_nero_left` / `can_nero_right` (native), hands on `hand_left` / `hand_right`
> (USB-CAN FD adapters) — so same-side arm and hand motion run in parallel and
> the shared-bus hand window is a selectable degraded mode, not normal operation.
> Validated on hardware 2026-08-13: both bridges on their own bus, zero CANFD
> timeouts. Before changing the bridge, read
> `docs/sprint_refactor/planning/integration_plan.md` (constraints C1 and C5,
> phases 2A-2D and 4D).

## Placement Rule

- keep the bridge in `src/agx_arm_ctrl` in the current baseline
- revisit a dedicated package only after a non-mock backend proves a clear dependency or public-contract boundary

## Public ROS Contract

- shared `control/joint_states` is the current arm-plus-hand command flow and is legacy: the V02 target is one abstract hand command carrying owner identity, control epoch, and sequence
- resolve each hand's SocketCAN interface from its own registry entry, never from the arm's `can_port`, and select it through explicit backend construction rather than the process-global `OMNIHAND_SOCKETCAN_IFACE`
- keep combined `feedback/joint_states` as the canonical follow-mode state
- publish hand-only debug and diagnostics under `feedback/omnihand/*`
- `<side>_omnihand_controller/follow_joint_trajectory` is the **production** hand execution path, not a debug or development surface: lower measured latency than a direct `HandCmd`, synchronization with arm trajectories, and the trajectory semantics later motion primitives need. Do not remove it to save CPU
- keep `control/omnihand/joint_trajectory` as a bridge-specific compatibility surface while the longer-term action or controller contract is still open
- `control/omnihand/stop` **cancels** the pending target and holds the current pose. It is not a latching device stop: a hand re-arms on the next command, and only the unit generation can latch it STOPPED. Do not describe it as an emergency stop or rely on it to keep a hand down

## Message Rules

- use standard ROS messages where they already fit, such as `sensor_msgs/JointState`
- keep OmniHand-specific diagnostics in `agx_arm_msgs`
- do not force OmniHand onto Revo2-specific command or status messages

## Backend Rules

- **a hand has no serialized SDK owner yet.** The one-owner-per-device invariant and the four priority lanes (`sdk_worker.py`) are implemented for the *arms* only; the bridge still reaches the hand SDK straight from its timer, its subscriptions, and its service handlers. Treat that as a known gap being closed in phase 2C, not as a pattern to copy into new code
- keep the SDK or vendor transport below ROS
- treat vendor ROS topics as backend input references only, not as the public repo contract
- keep the mock backend and real backend behind the same repo-owned bridge surface when possible

## Validation Rule

For bridge changes, run diagnostics on the touched files and at least one package-scoped build such as `colcon build --packages-select agx_arm_ctrl agx_arm_msgs`.