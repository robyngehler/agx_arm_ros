# Critical CPU Paths (pre-refactor evidence)

Runtime CPU-load hot spots identified while debugging the shared-CAN hand window on the Jetson
(2026-07-24 … 2026-07-27). They are recorded here as input to the Phase 0 baseline instrumentation and
the refactor's performance goals. None of these is a *correctness* bug — the shared-bus handshake works —
but together they starve the Jetson enough that, under the full teach stack, CPU stalls overflow the CAN
RX socket buffer and drop hand response frames (see `docs/sprint6/errors_and_fixes.md`, 2026-07-27, and
the `sprint6/planning/step_and_settle_implementation_summary.md` §8e finding). Verify each with the
Phase 0 CPU / loop-duration / SDK-call instrumentation before acting.

## Symptoms observed

- `agx_arm_ctrl_single`: `publish-loop overrun: 202 ms gap (> 200 ms)`, and `is_ok() reads false but
  kernel feedback frames are still advancing … local starvation, suppressions=25`.
- `ip -s -d link show`: ~108k RX `dropped` per bus over a session; `arbit-lost` only 9–10 (so the drops
  are host-side socket overflow, not bus arbitration).
- A driver service callback (`prepare_hand_window`) blocked long enough to itself cause a publish-loop
  overrun, i.e. the 200 Hz publish thread and the service thread contend for the same GIL.

## Identified hot paths

1. **Driver publish loop at `pub_rate` = 200 Hz with per-joint Python SDK reads.**
   `agx_arm_ctrl_single_node._publish_joint_states` calls `get_motor_states(joint_index)` once per joint
   (7×) plus `get_joint_angles()` every cycle, i.e. ~8 SDK calls × 200 Hz × 2 arms ≈ 3.2k blocking SDK
   calls/s, all under the CPython GIL. This is the dominant single-node load and the source of the
   publish-loop overruns. Candidate: decouple the ROS republish rate from the SDK read rate; batch the
   per-joint reads; drop `pub_rate` (already noted: it is NOT a bus lever, only a republish rate).
2. **CAN RX drain is coupled to that same stalling thread.** When the publish/parse thread stalls (GIL,
   heavy publish), nothing drains the CAN RX socket, so the ~208 KB kernel buffer (~125 ms at 2150 f/s)
   overflows and the kernel drops frames — including hand CANFD responses. Mitigated for now by raising
   `net.core.rmem_max` to 4 MB in `activate_native_can.sh`; the real fix is to not stall the drain.
3. **Two MIT controllers at 50 Hz with pinocchio gravity comp over 19 payload joints each.** The gravity
   model "articulates 19 payload joints from live feedback" per side; RNEA per control cycle × 2 arms.
   Candidate: precompute/cache, reduce articulated payload DoF, or share a model.
4. **Two OmniHand bridges doing blocking CANFD request/response polling.** Every joint readback
   (`joint_read_rate`, 20 Hz) and status/tactile read is a real request/response on the bridge thread;
   under load these are exactly the calls that `请求超时`. Candidate: async/pipelined SDK access, lower
   idle poll rate, back off cleanly when a window on the *other* side owns the bus.
5. **`move_group` + `rviz2`.** The planning-scene monitor logs "complete state of the robot is not yet
   known" on a busy loop until every hand joint is populated, and rviz rendering is heavy. Candidate:
   quantify their share; consider a headless bringup for teach.
6. **Fixed (kept as a pattern):** the external MIT controller used to dead-man-flood the gate at 50 Hz
   during a window because it read the intentional feedback silence as a dead bus. Now it stands down on
   `feedback/hand_window_active`. Lesson for the refactor: an expected silence must be *signalled*
   out-of-band, never inferred from missing feedback.

## Relation to the refactor

The refactor's per-side authority / serialized SDK access (Phase 1) should make the driver's SDK usage
explicitly rate-limited and single-owner, which directly addresses paths 1, 2, and 4. Path 3 is a MIT
controller concern; path 5 is a bringup-composition concern. Capture all of them in the Phase 0 baseline
scenarios (idle, dual-arm hold, one/two MIT arms, one hand window) so the refactor can show a measured
before/after.
