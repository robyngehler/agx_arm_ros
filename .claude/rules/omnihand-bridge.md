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

> **V02 refactor in progress.** Each device has its own CAN bus — arms on
> `can_nero_left` / `can_nero_right` (native), hands on `hand_left` / `hand_right`
> (USB-CAN FD adapters) — so same-side arm and hand motion run in parallel and
> the shared-bus hand window is a selectable degraded mode, not normal operation.
> Validated on hardware 2026-08-13: both bridges on their own bus, zero CANFD
> timeouts. The hand command surface is moving to one abstract, owned hand
> contract. Before changing the bridge, read
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
- a hand has **two** production motion primitives, and they may never command it
  at once:
  - `<side>_omnihand_controller/follow_joint_trajectory` — the primary
    **trajectory-execution** path. Lower measured latency than a direct
    `HandCmd`, synchronizes with arm trajectories, carries the semantics later
    motion primitives need. Do not remove it to save CPU
  - reactive **contact-seeking** motion (the skill controller) — a grasp that
    ends where the tactile sensor says rather than where the clock does. It
    cannot be decomposed into trajectory goals without losing that closed loop,
    so do not try to route it through the action
- exclusivity comes from **device authority, not topic separation**. Both
  primitives claim `control/omnihand/claim_device` (never plain `claim_device` —
  the arm driver owns that name in the same namespace) and release afterwards.
  The bridge is fail-closed: an unclaimed hand executes nothing
- an owner declares itself as `<primitive>:<node>`. The primitive half is how the
  bridge tells a trajectory command from a reactive one; the node half is how it
  notices a commander that died still holding a claim
- claim and release advance the device epoch, so a command issued under the
  previous owner cannot execute after the handover. A grasp that ends holding
  keeps the claim — the hold *is* the reactive primitive still owning the hand
- keep `control/omnihand/joint_trajectory` as a bridge-specific compatibility surface while the longer-term action or controller contract is still open
- `control/omnihand/stop` **cancels** the pending target and holds the current
  pose. It is not a latching device stop: a hand re-arms on the next command,
  and only the unit generation can latch it STOPPED. Do not describe it as an
  emergency stop or rely on it to keep a hand down

## Message Rules

- use standard ROS messages where they already fit, such as `sensor_msgs/JointState`
- keep hand diagnostics in `agx_arm_msgs`
- do not extend the Revo2-specific messages for OmniHand, and do not add a third
  OmniHand-only message either: the V02 target consolidates `HandCmd`,
  `HandPositionTimeCmd`, `HandStatus`, `GripperStatus`, and `OmniHandStatus`
  into one abstract hand contract that must fit any hand
- keep fields statically defined; no runtime-variable structure in control paths

## Cadence Rules

- the hand's cadence is its own. Do not forward the arm's `pub_rate` to the
  bridge; bringups pass `hand_pub_rate` and `hand_joint_read_rate`
- publication is driven by new data, never by a timer: a joint sample when a
  readback lands, status when the state it reports changes (plus a heartbeat),
  tactile at the rate the sensor is read. `pub_rate` is a ceiling that can
  throttle publication further, and cannot make anything faster
- announce a settled command immediately. `FollowJointTrajectory` holds its goal
  until it sees a status sample stamped after its command, so anything that
  delays that verdict slows the production hand path

## Backend Rules

- **a hand has no serialized SDK owner yet**, and the reason is subtler than it
  looks. Its timer, subscriptions and service handlers all reach the SDK
  directly, but they run on one thread because the bridge uses `rclpy.spin`, so
  the measured attribution really is a single thread. What is missing is not
  serialization — it is that the property is incidental (one edit to a
  `MultiThreadedExecutor` silently ends it, and two sibling nodes in this package
  already use one) and that there are no lanes, so a stop waits behind whatever
  the executor is doing and a 17 ms status read blocks the claim service. Closed
  in phase 2C; do not copy the pattern into new code
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
