# Critical CPU Paths (pre-refactor evidence)

last_updated: 2026-08-11

Runtime CPU-load hot spots identified while debugging the shared-CAN hand window on the Jetson
(2026-07-24 … 2026-07-27). They are recorded here as input to the Phase 0 baseline instrumentation and
the refactor's performance goals. None of these is a *correctness* bug — the shared-bus handshake worked —
but together they starve the Jetson enough that, under the full teach stack, CPU stalls overflow the CAN
RX socket buffer and drop hand response frames (see `docs/sprint6/errors_and_fixes.md`, 2026-07-27, and
the `sprint6/planning/step_and_settle_implementation_summary.md` §8e finding). Verify each with the
Phase 0 CPU / loop-duration / SDK-call instrumentation before acting.

## What changed since this note was written

- **Each device now has its own CAN bus** (arms on native `can_nero_left`/`can_nero_right`, hands on
  FD-capable USB adapters `hand_left`/`hand_right`). Bus contention is no longer the reason to bound hand traffic — CPU is. And
  because arm and hand may now run *in parallel*, the peak CPU load this note describes is reached more
  often, not less. The RX overflow below was host-side socket overflow caused by CPU starvation, so it
  remains a live risk under the new topology.
- **The MIT loop runs at 100 Hz, not 50 Hz** (commit `15ac809`). Path 3 below therefore costs twice what
  is written there. 100 Hz is the stability *minimum*; the target is 200–250 Hz, so the control rate is
  a fixed requirement and not available as a CPU lever (integration plan C2).

Measure the Phase 0 baseline against the *current* rates and the four-bus topology, not against the
numbers in the symptom list below.

## Symptoms observed

- `agx_arm_ctrl_single`: `publish-loop overrun: 202 ms gap (> 200 ms)`, and `is_ok() reads false but
  kernel feedback frames are still advancing … local starvation, suppressions=25`.
- `ip -s -d link show`: ~108k RX `dropped` per bus over a session; `arbit-lost` only 9–10 (so the drops
  are host-side socket overflow, not bus arbitration).
- A driver service callback (`prepare_hand_window`) blocked long enough to itself cause a publish-loop
  overrun, i.e. the 200 Hz publish thread and the service thread contend for the same GIL.

## Identified hot paths

1. **The whole 200 Hz feedback/publish batch — not the per-joint reads alone.**
   *(Reformulated 2026-08-11 after measurement.)* The batch costs 1.10 ms mean
   against a 5 ms period; the per-joint `get_motor_states` calls named below are
   only ~0.11 ms of that, about a tenth. The remaining 90 % is pose, arm status,
   effector status, leader publication and message construction/serialisation.
   Optimising the reads alone recovers little; the batch as a whole is the
   target. Original text follows.

   `agx_arm_ctrl_single_node._publish_joint_states` calls `get_motor_states(joint_index)` once per joint
   (7×) plus `get_joint_angles()` every cycle, i.e. ~8 SDK calls × 200 Hz × 2 arms ≈ 3.2k blocking SDK
   calls/s, all under the CPython GIL. This is the dominant single-node load and the source of the
   publish-loop overruns. Candidate: decouple the ROS republish rate from the SDK read rate; batch the
   per-joint reads; drop `pub_rate` (it is NOT a bus lever, only a republish rate).
2. **CAN RX drain is coupled to that same stalling thread.** When the publish/parse thread stalls (GIL,
   heavy publish), nothing drains the CAN RX socket, so the ~208 KB kernel buffer (~125 ms at 2150 f/s)
   overflows and the kernel drops frames. Mitigated for now by raising `net.core.rmem_max` to 4 MB in
   `activate_native_can.sh`; the real fix is to not stall the drain. Still applies per interface under
   the four-bus topology.
3. **Two MIT controllers at 100 Hz with pinocchio gravity comp over 19 payload joints each.** The
   gravity model "articulates 19 payload joints from live feedback" per side; RNEA per control cycle ×
   2 arms ≈ 200 RNEA/s today, ≈ 400–500/s at the 200–250 Hz target. Because the rate is a requirement,
   the lever is per-call cost: precompute or cache the unchanged-configuration term, reduce the
   articulated payload DoF, share one model instance per process, or compile the gravity path.
4. **Two OmniHand bridges doing blocking CANFD request/response polling.** Every joint readback
   (`joint_read_rate`, 20 Hz) and status/tactile read is a real request/response on the bridge thread.
   The bus-contention argument for bounding this is gone; the CPU and GIL argument is not, and the
   bridges may now poll *while* both arms stream MIT setpoints. Candidate: async/pipelined SDK access,
   poll only under active hand ownership, low idle rate.

   **Superseded 2026-08-14 — this entry named the wrong culprit.** Two
   measurements took it apart:

   *The ROS half* (`scripts/profile_hand_bridge.py`, mock backend): 41.5 % of a
   core, because the publish timer ran at the **arm's** 200 Hz while the hand's
   data changed at 20 Hz. Binding publication to new readbacks cut it to 7.3 %
   and made it flat across `pub_rate`; on hardware the whole stack fell from
   814 % to 526 % of a core, because the subscribers had been paying for the
   over-publication too.

   *The other half* (per-thread census on hardware, L3): **not the blocking
   request/response polling this entry describes.** Every SDK call the bridge
   makes sums to ~5 % of a core. The remaining ~100 % per hand is a single
   vendor thread that never sleeps (`wchan=0`), present only when the SDK
   session is open, inside compiled C++ we never call. Bounding our polling
   divides the 5 %. See `errors_and_fixes.md`, 2026-08-14.
5. **`move_group` + `rviz2`.** The planning-scene monitor logs "complete state of the robot is not yet
   known" on a busy loop until every hand joint is populated, and rviz rendering is heavy. Candidate:
   quantify their share; consider a headless bringup for teach.
6. **Historical (kept as a pattern):** the external MIT controller used to dead-man-flood the gate at
   50 Hz during a window because it read the intentional feedback silence as a dead bus. It was fixed by
   standing down on `feedback/hand_window_active`; that coupling is itself removed in Phase 2. The
   lesson survives the topology change: an expected silence must be *signalled* out-of-band, never
   inferred from missing feedback.

## Relation to the refactor

Phase 1's per-side authority and serialized SDK access make the driver's SDK usage explicitly
rate-limited and single-owner, which addresses paths 1, 2, and 4. Path 3 is a MIT controller concern and
gets more expensive as the control rate rises toward the target, so it is now a first-class Phase 5 item
rather than an optional cleanup. Path 5 is a bringup-composition concern. Capture all of them in the
Phase 0 baseline scenarios — including the newly possible parallel same-side arm-plus-hand scenario — so
the refactor can show a measured before/after.
