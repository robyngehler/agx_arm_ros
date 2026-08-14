# Refinement Proposal

**Scope:** Immediate refinements after the first full-system load tests.

**Status (2026-08-14):** section 4 is applied and section 1 is decided — the
`FollowJointTrajectory` action is recorded as the production hand path in
`AGENTS.md`, the bridge rules, and the integration options table. Sections 2, 3
and 5 are now owned by phase 2C of
[`integration_plan.md`](integration_plan.md), which absorbed the former phase 5C
so the bridge is not optimised in two places; phase 5 keeps only the
before/after close-out. Read this file for the reasoning, the plan for the work.

## 1. Keep FJT as the primary hand execution path

Treat `FollowJointTrajectory` as the main production path for real hand motion, not as debug/development-only.

Rationale to preserve in the architecture:
- lower measured latency than direct `HandCmd.msg`;
- better synchronization with arm trajectories;
- easier parallel execution and feedback-driven progression;
- required trajectory semantics for future Physical AI / motion primitives.

`HandCmd`-style interfaces may remain for simple discrete commands or low-level/vendor-specific control, but should not be the primary coordinated-motion path.

## 2. Escalate OmniHand bridge optimization

Phase 2C should own OmniHand runtime/transport efficiency.

Required next steps:
- profile CPU per thread and per SDK call;
- separate ROS publication cost from vendor-SDK polling cost;
- decouple hand feedback publication rate from the arm `pub_rate`;
- split joint readback, tactile, status, heartbeat, and command verification scheduling;
- publish new samples on new readback, with only low-rate heartbeat where required;
- stop unnecessary idle polling and recurring hold traffic;
- remove SDK read-before-write for complete hand targets.

## 3. Preserve trajectory execution while reducing duplicate command paths

Do not remove the FJT controller to save CPU.

Instead:
- keep one authoritative FJT-based production command path;
- identify and remove redundant parallel hand-command surfaces;
- prevent simultaneous FJT and direct-command ownership of the same hand;
- use owner/epoch/sequence admission consistently across the chosen paths.

## 4. Correct documentation and agent guidance

Update stale guidance:

- remove claims that hand FJT is debug/development-only;
- remove remaining shared arm/hand CAN wording;
- clarify that hand SDK ownership/serialization is still pending;
- correct `control/omnihand/stop` wording if it is not a latched device stop;
- assign bridge runtime optimization to Phase 2C, public hand-contract consolidation to Phase 4, and final measurement only to Phase 5.

## 5. Immediate execution order

1. Apply the docs/agent corrections.
2. Profile the current OmniHand bridge under full-system load.
3. Refactor bridge scheduling and polling without changing the FJT production path.
4. Re-run the same full-system CPU/CAN load test.
5. Only after measured improvement, proceed with broader parallel-scheduling and coordinator work.

## Exit criterion

The next slice is successful when the FJT hand path remains fully functional while OmniHand bridge CPU load is substantially reduced and no duplicate command authority exists.
