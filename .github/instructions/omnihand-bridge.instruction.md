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
> phases 2A-2C and 4D).

## Placement Rule

- keep the bridge in `src/agx_arm_ctrl` in the current baseline
- revisit a dedicated package only after a non-mock backend proves a clear dependency or public-contract boundary

## Public ROS Contract

- shared `control/joint_states` is the current arm-plus-hand command flow and is legacy: the V02 target is one repo-owned abstract hand command carrying the command authority stamp — `owner_id`, `device_epoch`, `unit_safety_epoch`, `sequence`. The standard ROS messages themselves are not modified
- resolve each hand's SocketCAN interface from its own registry entry, never from the arm's `can_port`, and select it through explicit backend construction rather than the process-global `OMNIHAND_SOCKETCAN_IFACE`
- keep combined `feedback/joint_states` as the canonical follow-mode state
- publish hand-only debug and diagnostics under `feedback/omnihand/*`
- a hand has **two** production motion primitives, and they may never command it at once: `<side>_omnihand_controller/follow_joint_trajectory` is the primary **trajectory-execution** path (lower measured latency than a direct `HandCmd`, synchronizes with arm trajectories, carries the semantics later motion primitives need — do not remove it to save CPU), and reactive **contact-seeking** motion (the skill controller) is the second, ending where the tactile sensor says rather than where the clock does, so it cannot be decomposed into trajectory goals
- exclusivity comes from **device authority, not topic separation**. Both primitives claim `control/omnihand/claim_device` (never plain `claim_device` — the arm driver owns that name in the same namespace) and release afterwards. The bridge is fail-closed: an unclaimed hand executes nothing
- an owner declares itself as `<primitive>:<node>`: the primitive half tells a trajectory command from a reactive one, the node half is how a commander that died still holding a claim is noticed
- claim and release advance the device epoch, so a command issued under the previous owner cannot execute after the handover. A grasp that ends holding keeps the claim
- **a command carries the authority it was issued under.** `agx_arm_msgs/DeviceCommandStamp` (`owner_id`, `device_epoch`, `unit_safety_epoch`, `sequence`) is embedded by `AuthorizedJointTrajectory` on `control/omnihand/authorized_trajectory` and by `HandJointTarget` on `control/omnihand/joint_target`. A commander fills it from the claim response and restarts its sequence at each claim
- the bridge admits on the stamp the command **arrived with**, and never substitutes a missing field from its own state. A stamp the bridge builds itself is current by construction, which is why the stale-epoch and out-of-order checks could not refuse anything before this
- shared `control/joint_states` and `control/omnihand/joint_trajectory` remain as migration-only compatibility surfaces. They self-stamp, so they cannot reject a stale or reordered command — do not build new callers on them
- `control/omnihand/stop` **cancels** the pending target and holds the current pose. It is not a latching device stop: a hand re-arms on the next command, and only the unit generation can latch it STOPPED. Do not describe it as an emergency stop or rely on it to keep a hand down

## Message Rules

- use standard ROS messages where they already fit, such as `sensor_msgs/JointState`
- keep hand diagnostics in `agx_arm_msgs`
- do not extend the Revo2-specific messages for OmniHand, and do not add a third OmniHand-only message either: the V02 target consolidates `HandCmd`, `HandPositionTimeCmd`, `HandStatus`, `GripperStatus`, and `OmniHandStatus` into one abstract hand contract that must fit any hand
- keep fields statically defined; no runtime-variable structure in control paths

## Cadence Rules

- the hand's cadence is its own. Do not forward the arm's `pub_rate` to the bridge; bringups pass `hand_pub_rate` and `hand_joint_read_rate`
- publication is driven by new data, never by a timer: a joint sample when a readback lands, status when the state it reports changes (plus a heartbeat), tactile at the rate the sensor is read. `pub_rate` is a ceiling that can throttle publication further, and cannot make anything faster
- announce a settled command immediately. `FollowJointTrajectory` holds its goal until it sees a status sample stamped after its command, so anything that delays that verdict slows the production hand path

## Backend Rules

- **every hand bridge owns one `SdkWorker`.** Steady-state SDK calls are serialized through it on a declared priority lane, exactly as the arms do, and acquisition runs on its own paced thread so it is paced independently from ROS publication. Landed 2026-08-15, validated at L3
- **a ROS callback must never call the vendor SDK directly.** A timer, subscription or service handler submits to the worker and reads the result the acquisition thread stored. The old property — one thread by accident, because the bridge used `rclpy.spin` — is gone, and so is the constraint that came with it: this node may now use a `MultiThreadedExecutor` safely
- **safety work uses the safety lane.** A stop preempts queued ordinary work rather than waiting behind it; measured at ~2 ms while the worker was saturated. The lane does not preempt the call already in flight, so the bound is that call's duration — 36.9 ms worst case on this hand, above the arms' 20 ms budget and not yet a declared hand stop budget (`docs/sprint_refactor/open_questions.md`)
- **shutdown stops both.** Stop the acquisition thread and the worker, and join them, before the node goes away
- state any recovery or exception rule explicitly; nothing may be inferred from executor behaviour
- keep the SDK or vendor transport below ROS
- treat vendor ROS topics as backend input references only, not as the public repo contract
- keep the mock backend and real backend behind the same repo-owned bridge surface when possible

## Validation Rule

For bridge changes, run diagnostics on the touched files and at least one package-scoped build such as `colcon build --packages-select agx_arm_ctrl agx_arm_msgs`.