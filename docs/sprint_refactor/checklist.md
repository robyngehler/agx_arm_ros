# Sprint Refactor - Checklist

Phases follow `planning/integration_plan.md`, which is the canonical plan.
Binding constraints C1 (one CAN bus per device), C2 (MIT rate is a requirement),
C3 (pinned submodule vs development checkout), C4 (test ladder), C5 (message
policy), C6 (instrumentation form), C7 (bus topology is one declared fact), and
C8 (the two arms speak different protocol tiers, permanently) are defined there.

Priority: safety, CPU relief, and parallel operation come before demo work.
`docs/sprint6/` adapts afterwards.

## Sprint setup

- [x] Create the `docs/sprint_refactor/` surface on branch
      `ROS2_Duo_System_V02`.
- [x] Move the coordination refactor proposal into the sprint surface.
- [x] Cross-check the proposal against the current code and record the entry
      points that will drive the migration.
- [x] Re-verify the cross-check against the working tree (2026-08-11) and fold
      the four-bus topology into the plan.
- [x] Freeze the open decisions: wrappers as degraded mode, hardware slot for
      the safety checks, instrumentation form, refactor-before-demo priority.
- [x] Freeze the remaining contract decisions: no separate hand lease, epoch in
      `MoveMITMsg`, in-phase message migration, MoveIt hand FJT debug-only,
      degraded-mode removal reviewed at Phase 5 close-out.

## Phase 0 - Hygiene, harness, safety baseline

### 0A Guidance and topology hygiene

- [x] Repoint the sprint entrypoints from `docs/sprint6/` to
      `docs/sprint_refactor/`: `CLAUDE.md`, `.claude/rules/context-routing.md`,
      `.github/copilot-instructions.md`.
- [x] Correct the ROS contract rules that now point the wrong way: `AGENTS.md`
      shared `control/joint_states` and the "do not map onto Revo2 messages"
      rule, plus the `CLAUDE.md` mirror.
- [x] Update `.claude/rules/omnihand-bridge.md` for the per-device bus, hand
      ownership, and the C5 consolidation.
- [x] Banner the operational docs that describe the shared bus and hand window
      as normal operation: `docs/control/bringups/teach_and_run.md`,
      `docs/control/bringups/tea_demo.md`,
      `docs/assets/omnihand/omnihand_solo_bringup_and_load_test.md`.
- [x] Mark the `docs/sprint6/` step-and-settle planning notes superseded and
      record that sprint6 adapts after the refactor.
- [x] Update `scripts/activate_native_can.sh` header, which still documents
      can0/can1 as shared side buses.
- [x] Correct all `pyAgxArm` references to `vendor/pyAgxArm` and document the C3
      workflow.
- [x] Record the MIT control-rate requirement (>= 100 Hz, target 200-250 Hz).
- [ ] Remove the stray untracked `vendor/Omnihand-2025-SDK/` checkout (legacy
      non-Pro `o10` SDK, no longer used; the runtime targets
      `vendor/OmniHand-Pro-2025`).

### 0B Regression harness and test ladder

- [x] Add an L2 mock integration test covering coordinator -> arm driver -> hand
      bridge for one activity including a hand action.
- [x] Encode the C4 test ladder as a `.claude/skills/` workflow with a
      `.github/skills/` mirror, following the `commit-quality` skill's shape.
- [~] Define the `tea_pour_left_v1` regression criteria — **deferred by
      decision**; the L2 harness is the standing regression net until the demo
      is re-taught against the new contracts.

### 0C Honest velocity and stop semantics

- [x] Provide a trustworthy velocity source (derived from timestamped positions)
      with an explicit validity flag.
- [x] Verify it against the protocol value: settled from the MIT pcaps — the
      firmware reports 0 (+/-1) while the joints move, so there is no protocol
      value to compare against and the derived source is the only one.
- [x] Separate `commanded` from `feedback_verified` in stop reporting.
- [x] Define and implement the coordinator response to a `commanded`-only stop,
      including the Ctrl+C stop ladder from commit `8e8fc44`.
- [ ] Audit the undocumented `current *= -1` vendor mutation in `driver.py:541`
      (origin traced to `cea1cb9`; sign still unconfirmed against a known load).
- [x] Extend `test_emergency_stop_verify.py` for the new outcomes.

### 0D Baseline instrumentation

- [x] Add in-node log counters for loop duration, callback duration, SDK call
      origin with thread id (`agx_arm_ctrl/runtime_metrics.py`, off by default).
- [x] Record the external tooling recipe as a runnable script:
      `scripts/measure_can_baseline.sh` reports per-interface rates and drops
      over a window plus process CPU, and sends nothing on any bus.

### 0E Hardware baseline (L3, safety slot)

Captured in `reference/phase0_baseline.md`, nine scenarios across three grants:
communication-only, then hand gestures plus one minimal arm move, then fault
injection.

- [x] Capture idle with no ROS nodes running: ~5430 frames/s of drain before any
      of our code runs.
- [x] Capture one arm driver, no motion: 71.6 % of one core, 198 Hz loop,
      1587 SDK calls/s, publish batch 1.10 ms mean / 2.73 ms max.
- [x] Capture two MIT arms under load (pcaps): 2849 f/s per bus, MIT at
      100 Hz per joint, feedback rate unchanged from idle.
- [x] Capture dual-arm hold, one MIT arm, one hand action.
- [x] Capture same-side arm-and-hand in parallel: both completed, no drops,
      arm buses unaffected — the case C1 exists to allow.
- [ ] Capture both sides arm-and-hand in parallel (blocked: `hand_left`
      cable fault makes its half meaningless until replaced).
- [x] Capture bus-fault and recovery: detection took ~2 s after a 1.6 s
      starvation misdiagnosis, the loop stalled 10 s in one gap, and the
      "recovery succeeded" claim was the bus returning on its own.
- [x] Record per-interface CAN counters and RX drop counts for every captured
      scenario (`scripts/measure_can_baseline.sh`).
- [x] Settle the wire-level velocity question (`evidence/*.pcap`,
      `scripts/analyze_can_pcap.py`): the firmware does not report velocity.

## Phase 1 - Device authority, serialized SDK access, unit guard

### 1A Device authority, epoch, serialized SDK

The rules and the mechanism are separated deliberately: the model is built and
proven at L1 first, then routed through the runtime. A checked box in the first
group means the behaviour is decided and tested, not that the driver uses it
yet.

Rules and mechanism (L1, `agx_arm_ctrl/device_authority.py`, `sdk_worker.py`):

- [x] Per-device `device_epoch` plus a unit-wide `unit_safety_epoch`, with an
      L1 test that an arm recovery does not invalidate the same-side hand and
      that a unit stop does.
- [x] Admission at the boundary: state, owner, both epochs, and a per-epoch
      sequence watermark, each with a structured reject reason.
- [x] Single-commander ownership: claim, release, and safety revoke, each
      bumping the device epoch.
- [x] Separate fault acknowledge from verified rearm — acknowledging clears the
      latch and arms nothing, and `rearm` refuses without positive evidence.
- [x] Serialized SDK worker: one named thread per device, safety lane ahead of
      queued motion, stale-epoch work dropped instead of delivered late,
      superseded setpoints replaced, and a call that never ran distinguishable
      from one that failed.

Routed through the runtime:

- [x] Validated on hardware, both arms (`reference/phase1a_hardware_validation.md`):
      enable readback, per-tier MIT bounds, the four rejection paths, a full
      MIT hold with zero rejections, and the authority through e-stop,
      lockout-clear and a disable/enable cycle.
- [x] Each arm driver builds its own authority and publishes it latched on
      `feedback/authority` (`AgxDeviceAuthority`), derived from the gates the
      driver already acts on — enable readback, feedback readiness, fault
      lockout, recovery, hand window — with the epochs coming from the
      authority's own transitions rather than from those gates.
- [x] The two hand transport authorities. Each bridge publishes
      `feedback/authority` for `hand_left`/`hand_right`, observes the unit
      generation, and offers `claim_device`. They are *transport* authorities:
      what they own is the hand's SDK session and CAN transport, not the
      meaning of a grasp, which stays with the skill controller.
- [ ] Give a hand a device-level stop of its own. `control/omnihand/stop`
      cancels the pending target — a cancel, not a latching stop, and the skill
      flow depends on that — so a hand can currently only be stopped through the
      unit generation, where an arm can latch its own. Belongs with the
      consolidated hand contract (4D).
- [x] Measure the per-call SDK latency that the worker's safety lane has to
      queue behind, before routing anything through it
      (`reference/sdk_latency_budget.md`). Individual calls are almost all
      sub-millisecond; the hazard is the driver's composite retry loops, which
      would turn a 1 ms call into a 5 s block on the stop path. Rule: one SDK
      call per worker task, never a loop. Budget: stop reaches the SDK within
      20 ms under normal load.
- [x] Measure `connect`/`disconnect` under a real bus fault: `connect` 10 ms,
      **`disconnect` ~1 s in a single call**. That is why recovery does not
      share the worker with the safety lane — no task granularity shortens it.
- [x] Take recovery off the acquisition path. Inline it cost a 13.1 s
      publish-loop gap; re-provoked with recovery on its own thread there are
      **no overruns at all** and 2384 authority publications across the fault.
      The recovery still takes 13.1 s — that is the vendor's `disconnect`, three
      times — but it no longer costs the loop.
- [x] Run the L3 stop-latency stress test. **The budget is met**: an emergency
      stop reaches the SDK in 0.94 ms worst case on the right arm and 0.55 ms on
      the left, against a 20 ms budget, with a 100 Hz MIT stream running. Eight
      stop-and-re-enable cycles per arm, 8/8 verified in feedback, no
      escalations, no CAN errors. This required routing the stop onto the safety
      lane first — until then the lane had no users and the budget was a
      statement about nothing.
- [x] Route the **hot path** through the worker. The MIT setpoint is submitted
      as one bounded task (mode bracket plus the per-joint frames), keyed so a
      superseded setpoint is dropped while queued instead of delivered late, and
      stamped with the device epoch so a recovery discards what was issued
      before it. The ingress gate stopped reading the SDK in the same change —
      it decides on the publish loop's latest acquisition and refuses when none
      is fresh, which also closes the case where a command was admitted for an
      arm whose acquisition loop had stopped. L1 only; see
      `reference/sdk_latency_budget.md`.
- [ ] Route the remaining SDK call sites: the hand-window and enable/disable
      service handlers, and the quarantined legacy motion paths
      (`move_j|p|l|c|js`, the `control/joint_states` follow). Off the hot path
      and, for the legacy paths, off by default — but a development profile
      that enables them puts a second writer on the session, so the counter
      cannot be read as unconditional until they move.
- [x] Measure what the queue in front of every setpoint costs — and act on it.
      The first routing sent the setpoint as one task and hardware refused it:
      6.4 ms mean and 21.4 ms worst case of non-preemptible work, more than the
      whole stop budget, while the acquisition loop lost half its rate. The
      setpoint is now a *cycle* (one queue entry, one frame at a time, safety
      drained between frames), the worker has four priority lanes instead of
      two, and acquisition runs on its own justified cadence rather than
      following the ROS publish rate. Verified on both arms.
- [x] Stop latency measured directly rather than inferred from what a stop can
      queue behind. `sdk_queue_wait` is now recorded per lane, because one
      aggregate averaged the safety lane together with diagnostic reads that are
      meant to wait.
- [x] Test the `txqueuelen` hypothesis. Raising both arm buses from the kernel
      default of 10 to 1000 roughly halves the per-frame cost (`move_mit` mean
      0.61 -> 0.38 ms, max 10.5 -> 5.4 ms). It does not remove the tail: ~5 ms
      worst case survives as arbitration against the arm's own feedback push,
      which is what to attack for the 200-250 Hz target.
- [ ] Make `activate_duo_can.sh` set the TX queue depth. The measurements above
      were taken after a manual `ip link set`, so the supported bring-up does
      not yet produce the configuration they describe.
- [ ] Correct the inflated SDK call counts recorded earlier in the sprint. Two
      double-counting defects were found and fixed; durations were never
      affected, but totals quoted before 2026-08-13 are roughly 2× on reads.
- [x] Reject stale epoch and out-of-order sequence on the live command path.
      Verified on hardware: an unstamped command, one carrying a superseded
      device generation, and one from another commander are each refused with
      their own reason, while the legitimate stream runs at 100.2/s — including
      after a rogue command stamped sequence 999999, which does not advance the
      watermark because refusal happens before it.
- [x] Separate hardware readiness from permission. `motion_ready` says the
      device is ready; `may_command(owner)` answers whether *this* commander
      may stream, using the same checks as admission minus the sequence. Reading
      one as the other is how a controller gets told yes and then has every
      command refused for want of an owner.
- [x] Unit safety, part 1 of 2 — **detection only**: make a second allocator
      detectable: generations carry the
      writer that minted them, an observer refuses to mint at all, and an
      equal-generation contradiction is counted with the stop winning rather
      than silently dropped.
- [x] Unit safety, part 2 of 2 — **the actual fix**: one writer
      (`agx_arm_ctrl/unit_safety_node.py`), devices as observers, `AgxUnitSafety`
      latched on `/unit_safety`, and `RequestUnitStop` so a device can ask for a
      generation without ever waiting for one. A device still stops itself
      unilaterally on its own epoch. Verified on hardware: an e-stop on the right
      arm now stops the left arm through the generation, and with the writer not
      running the right arm still stops itself. See
      `planning/unit_safety_writer_spec.md`.
- [x] Add the `srv/` directory to `src/agx_arm_msgs` (`RequestUnitStop.srv`).
- [x] Refuse a **new top-level activity** while unit safety is unknown. The
      writer heartbeats, and staleness is the only usable signal because the
      latched generation outlives the writer. A running activity is untouched —
      that is the point of the split — and the L2 graph now includes the writer,
      because a coordinated graph needs one.
- [x] Claim ownership for the MIT controller, and gate on ownership as well
      as readiness. Enabling takes command, disabling gives it back; streaming
      into a device held by someone else would be a flood of refusals, not a
      control strategy.
- [x] Freeze one command stamp for every commandable device — `owner_id`,
      `device_epoch`, `unit_safety_epoch`, `sequence` — before changing any
      ABI, so the migration happens once (`open_questions.md`).
- [x] Extend `MoveMITMsg` with that stamp, and migrate producer, consumer,
      docs and tests in the same change set. `ClaimDevice` gives the controller
      one commander identity; the driver admits on commander, both generations
      and the sequence.
- [x] Make the MIT controller consume the authoritative device state instead of
      `feedback/hand_window_active`, and abort on authority loss. It aborts on
      losing motion *and* on any epoch change, because a new epoch means the
      in-flight work was issued against a device state that no longer exists.
      Superseded 2026-08-13: the legacy-gate fallback is no longer the default
      — a missing authority is now a refusal, and the fallback survives only in
      a named development profile. Validated on hardware:
      100.2/s while holding, 0 in 8 s after an emergency stop, back to 93.8/s
      after the stop was cleared, with no operator step in between.
- [x] Retire `feedback/hand_window_active` as a controller input. The arm
      authority already reports an open window as STANDBY with the reason
      naming it, plus the device, the generation and the owner — strictly more
      than the boolean said. The subscription, its callback, its state field,
      the control-loop gate and the now-unreachable `HAND_WINDOW` execution
      state are gone, and the obsolete test with them; the driver-side mapping
      is covered where it now lives. The driver still *publishes* the topic for
      other consumers.
- [x] Make CAN recovery report what it verified — 0E showed "recovery
      succeeded" for a bus that returned on its own. The re-arm result was
      being discarded; the log line now names feedback and the enable readback
      separately, and a restored bus with an unconfirmed enable is an error.
- [x] Hardware-boundary command validation for MIT: duplicate or unknown joint
      indexes, empty commands, non-finite values, and values the protocol
      cannot encode are refused whole, before the SDK sees them. Rejections are
      counted per reason and logged rate-limited, because a malformed stream
      arrives at the control rate.
- [x] Bound the MIT values per **firmware tier**, not per arm model. The first
      version applied the default tier's per-joint torque table to both arms,
      which against the 1.11 arm would have refused legitimate commands on
      joints 5-7 and admitted impossible ones on joints 1-2 (L3, 2026-08-12).
- [x] Publish each device's control envelope (`AgxDeviceCapability`, latched)
      and fit the controller's configured limits to *its own* arm before
      commanding, instead of discovering the mismatch as runtime refusals. A
      refused MIT command leaves the firmware on its previous setpoint, so
      under a dual-arm activity the old behaviour meant one arm moving and one
      frozen. Verified on hardware: `[20]*7` becomes `[16]*7` on the left arm
      and `[20,20,16,16,8,8,8]` on the right.
- [ ] Preflight a synchronized `both_arms` execution against **both** devices
      before either side starts, and fail it as a whole if either cannot encode
      the requested envelope. Per-arm clamping covers independent operation;
      this is the coordinated case, and it is coordinator work.
- [ ] Promote the joint-limit check from flagged to refused. A position past a
      joint's *configured* limit is currently warned and still forwarded:
      refusing mid-stream would freeze a running impedance loop at its last
      setpoint, and no hardware session has yet shown the controller never
      legitimately crosses a limit.
- [x] Replace the unassigned `AgxArmStatus.err_status` with a documented
      structured representation: `fault_code` (the vendor's raw 16-bit code,
      whose bits the per-joint flags decode) plus `any_fault`, derived once so
      no consumer re-derives it. The old field published 0 for every arm in
      every state, so the MIT controller's `arm_fault_active` gate could not
      fire — coverage in appearance only. It now consumes `any_fault`.
- [x] Fix the enable readback: a contradicted enable used to warn and return
      success, leaving `enable_flag` stale. The readback now decides both the
      flag and the return value, with a short settle window for a lagging frame.
- [x] Fix the firmware-version parsing: there was no `NeroFW.V112` branch at
      all, so a 1.12 arm ran on the 1.11 protocol, and versions were compared
      as strings. `resolve_nero_firmware` parses numerically and logs the tier,
      which nothing recorded before.
- [x] Make forced e-stop recovery independent of the optional normal-recovery
      setting. `bus_recovery_enabled` turns off the *watchdog*; it used to also
      remove the last resort of an emergency stop that could not confirm the arm
      stopped. Two decisions, no longer one switch.
- [x] Make the device authority mandatory rather than fail-open. A namespace
      typo, a QoS mismatch and an old driver are indistinguishable from the
      controller, so absence is now a refusal; the legacy gates survive only in
      a named development profile. The launch derives `expected_device_id` from
      the same `can_port` as the driver, so a controller cannot be gated by the
      other arm's authority. Verified on hardware: the standard launch still
      streams at 100.0/s.
- [x] Quarantine the unauthenticated arm-motion paths (`control/move_j|p|l|c|js`
      and the arm half of the shared `control/joint_states` follow). They carry
      no commander and no generation, so nothing can establish that a command on
      them is current or that its sender may move this arm. Off by default;
      refusals are counted per path and logged rate-limited. Effector control is
      deliberately untouched — separate devices, separate contract (4D).
- [x] Stress-validate MIT streaming plus e-stop and enable/disable churn, both
      arms. Recovery was **not** exercised in the same run: it takes the SDK
      session off the worker by design, so it is a different regime and its
      13.1 s window is already measured and declared an exception to the budget.
- [x] Confirm one SDK thread per arm with the counter under a full stack. Every
      steady-state window reports one thread on both arms. The only calls ever
      attributed to `MainThread` are the construction sequence (`connect`,
      `get_firmware`, `enable`, `set_speed_percent`, `set_tcp_offset`), which
      runs before the node serves anything.

Exercised on hardware 2026-08-12. The enable readback confirmed on the first
attempt on both arms, so the stricter check introduces no spurious failures. The
protocol tier is now recorded, and it is **not the same on both arms**: right
1.06 (default tier), left 1.11 (`NeroFW.V111`). See `errors_and_fixes.md` and
`open_questions.md` — the tiers differ in MIT frame encoding, so nothing may
assume the two arms are protocol-identical.

### 1B Feedback snapshot and driver CPU reduction

- [x] Separate acquisition cadence from publication cadence. Two threads: the
      arm's read cadence is `acquisition_rate_hz` (100 Hz, justified by the
      consumers), and publication takes the latest snapshot with `pub_rate` as a
      ceiling rather than a rate — a snapshot is published once and the loop
      waits for a newer one. One immutable snapshot per cycle, and the command
      path decides on it instead of reading the SDK itself.
- [x] Pace both loops on the monotonic clock, not on `Node.create_rate`. A ROS
      rate is a timer the executor services, so pacing a hardware I/O loop with
      one makes its cadence depend on ROS middleware load: two rate objects on
      the single-threaded executor dropped acquisition from 98/s to **39/s** and
      cost the setpoint path its timing. Found by measuring the split, not by
      reasoning about it.
- [x] Make per-thread CPU attributable at all (`name_os_thread`,
      `scripts/measure_thread_cpu.sh`). Python thread names never reach the OS,
      so every thread showed up under the process name and "which thread is
      burning CPU" could only be guessed.
- [x] Measure the arm-driver CPU against the 0E baseline. At rest **50.6 % of a
      core** (4.2 % of the machine) against 0E's 71.6 %; under MIT hold 98.0 %
      of a core (8.2 % of the machine), acquisition 99.7/s, publish batch
      99.3/s, zero loop overruns. The comparison is not like-for-like — 0E ran
      a 200 Hz coupled loop and no MIT — so it is a direction, not a delta.
- [ ] **Reduce what the measurement actually named.** The largest single
      consumer is the rclpy executor thread: 23.4 % at rest, 26.3 % under load,
      against 3.6 % for the acquisition loop and 14.5 % for publication. The
      sprint assumed the per-joint SDK reads, then the publish batch; both were
      wrong. Fewer ROS entities or a different executor is the lever, and
      nothing here has attacked it yet.
- [x] Measure the RX drain under a provoked fault (right arm, 14.9 s link-down,
      90 s sampled at 5 Hz). **Zero dropped, zero missed, zero errors, zero
      bus-off** across the whole window. The feared overflow cannot happen in
      this fault mode: a down link delivers nothing, so there is nothing to
      buffer — 2257 f/s before, 0 during, 2232 f/s after. The premise in the
      earlier version of this item was also wrong: the socket is drained by the
      vendor SDK's own reader thread (`driver_context._read_loop`), not by our
      loop, so a stalled publisher does not stop the drain.
- [x] Re-verify under fault that the acquisition loop survives recovery, now
      that acquisition and publication are separate threads: "recovery finished
      after 17.1s; the acquisition loop kept publishing throughout".
- [x] Close a one-owner violation the healthy-path measurements could not see.
      `get_last_send_error` was read straight off the session, and it only runs
      when a send was actually dropped — so every healthy window reported one
      SDK thread and the fault window reported three. Found by provoking the
      fault, not by reading the code.
- [ ] Test the drain in the fault mode that can actually overflow: the bus stays
      **up** while the reader is stopped or starved, which is what a TX-stall
      recovery does. The link-down case measured above does not exercise it.
- [ ] Fix the socket receive buffer, which does not do what the repo believes.
      `activate_duo_can.sh` raises `net.core.rmem_max` to 4 MB, but that is only
      a ceiling an application must then request, and `python-can` never calls
      `setsockopt(SO_RCVBUF)` — so every CAN socket actually gets
      `net.core.rmem_default`, measured at 208 KB. Commit `e69daa2` ("raise the
      RX socket buffer so CPU stalls don't drop hand responses") therefore did
      not take effect. Either raise `rmem_default` too, or set the option in the
      comm layer under the C3 vendor workflow.

### 1C One active unit activity — the small guard

Pulled ahead of Phase 2 on review: the rule must hold before parallelism exists.

- [x] `READY` accepts one activity; `EXECUTING` rejects every further goal with
      a structured reason. The goal callback refuses at the door; the claim
      inside execute is authoritative, because two goals can pass the door
      check at once on a reentrant callback group.
- [x] One authoritative unit activity state and failure reason
      (`agx_arm_coordination/unit_activity.py`), replacing a running-flag that
      nothing consulted before dispatching.
- [x] No polling-loop or event-queue work here — that stays in Phase 3.
- [x] L2 regression: a second goal sent while an activity runs is rejected.
      Two client processes cannot show this — the mock activity finishes in
      under a second, less than process startup jitter — so the probe sends the
      second goal from the same process the moment the first is accepted.

Exit gate met at L2 with mock backends and an arm double. Nothing here touches
hardware.

## Phase 2 - Parallel operation

### 2A Declare the topology, model the four buses

- [x] Add `bus_topology` to the registry as the single declared fact (C7) and
      `omnihand.sides.<side>.can_port`; schema version bumped to 3.
- [x] Remove the bridge's derivation of its interface from the arm `can_port`
      and delete the built-in fallback to the arm buses. A hardware backend now
      refuses to start without its own declared interface rather than opening
      the arm's, where no hand ever answers.
- [x] Derive the hand-bus default from the declared topology instead of typing
      `shared` into four launch files. The handshake and the interface were two
      independent settings, and the combination that shipped was the wrong one
      in both halves.
- [x] **Live hardware validation of `duo_hand`**, which the docs carried as
      "validated offline; live hardware validation is still pending". Both hand
      bridges resolve to `hand_left`/`hand_right`, zero CANFD timeouts, hand
      feedback at 167/s (right) and 126/s (left), and each device on its own
      bus: hand buses 25 f/s each, arm buses 2200 f/s RX with 703 f/s TX per arm
      under a dual MIT hold. No CAN errors on any of the four.
- [ ] Replace process-global `OMNIHAND_SOCKETCAN_IFACE` selection with explicit
      backend construction.
- [ ] Adopt `scripts/activate_duo_can.sh` as the supported bring-up and retire
      `activate_native_can.sh` / `omnihand_canfd_activate.sh`.
- [ ] Rewrite the operational docs bannered in 0A for the four-bus reality.

### 1B/2C measurement: what the full stack actually costs

Measured 2026-08-14 on the real bring-up (`components.launch`, `duo_hand`,
MoveIt + RViz, both arms, both hands), percent of **one** core — the ceiling
that binds, because these are GIL-bound Python nodes and a 12-core machine has
headroom the individual node cannot use. Desktop load was ~20 % of a core
(browser, editor, shell) and is excluded from these figures.

| node (both sides where applicable) | arms idle | both arms MIT at 100 Hz |
| --- | --- | --- |
| **omnihand_bridge** | **319.2 %** | **324.7 %** |
| agx_arm_ctrl_single | 140.1 % | 182.3 % |
| rviz2 | 115.9 % | 118.5 % |
| omnihand_follow_joint_trajectory | 88.1 % | 89.1 % |
| agx_arm_mit_controller | 80.1 % | 99.7 % |
| agx_arm_shared_can_recovery | 35.1 % | 33.5 % |
| agx_arm_joint_state_merger | 27.2 % | 26.7 % |
| move_group | 8.6 % | 8.3 % |
| **total** | **814.5 %** (67.9 % of machine) | **882.9 %** (73.6 % of machine) |

Two things this settles:

- **The hand bridges are the system's CPU problem**, at ~160 % of a core each
  for 25 frames/s on the wire, and they cost the same whether a hand is doing
  anything or not. That is 2C's target and it is now measured on a working
  four-bus stack rather than inferred.
- **The arm driver's acquisition loop is not.** Both acquisition threads
  together are 5.7 %. An earlier plan to attack the rclpy executor inside the
  driver was dropped on this evidence: it would have optimised a fifth of a
  core inside a system spending eight.

An earlier run of the same scenario, before the hand wiring was fixed, is kept
as a caution: the bridges cost 289.6 % of a core while transmitting **nothing
at all** on any of the four buses. A node can be the largest consumer in the
system purely by failing.

### 2B Parallel resource model, handoff derived not configured

- [ ] Derive the scheduler's bus tokens from `bus_topology`; under
      `dedicated_per_device`, `<side>_arm` and `<side>_hand` stop sharing one.
- [ ] Derive `handoff_enabled` from the same value; nothing reads it directly.
- [ ] Remove the MIT stand-down on `feedback/hand_window_active`.
- [ ] Keep `prepare_hand_window` / `resume_arm_control` only as the
      `shared_per_side` implementation.
- [ ] Fix the stale TX-loss warning that blames "hand-frame arbitration loss on
      the shared bus".
- [ ] Add tests for the newly reachable interleavings.

### 2C Hand arbitration and transport efficiency

Moved ahead of the coordinator rewrite: 115 % of a core for 27 frames/s makes
this the largest measured CPU consumer in the system.

- [ ] Implement the frozen contract: single-goal arbitration plus `owner_id`,
      `device_epoch` and `sequence`; no separate lease.
- [ ] Reject stale-epoch and out-of-order hand commands at the bridge boundary.
- [ ] Profile the O12 Pro backend at SDK-call level: which calls, how many per
      setpoint, how long each blocks.
- [ ] Eliminate the read-before-write round trip in the full-joint command path.
- [ ] Decouple command verification, joint readback, tactile and status into
      separate schedules.
- [ ] Stop polling entirely while no hand action is active.
- [ ] Remove the recurring post-grasp hold traffic.
- [ ] Bound and record the SDK round trips per commanded setpoint.
- [ ] Measure the hand-bridge CPU reduction against the 115 % baseline.
- [ ] Validate parallel same-side arm and hand motion without CAN RX drops.
- [ ] Verify the `shared_per_side` topology still executes an activity.

## Phase 3 - Event-driven coordinator and strict synchronization

The exclusivity guard landed in 1C; this is the conversion itself.

- [ ] Replace polling and `time.sleep` with event-driven completion handling and
      a low-rate deadline watchdog.
- [ ] Make SIGINT with no activity in flight exit rather than spin (0B finding).
- [ ] Extend the 1C guard to the full unit activity state machine, with cleanup
      as part of completion.
- [ ] Migrate the Ctrl+C stop ladder and replay planning onto the event model
      without weakening them.
- [ ] Make `sync_flag` merge strict: merge-or-fail, never independent fallback.
- [ ] Add cleanup deadlines and structured failure reasons for child shutdown.
- [ ] Validate concurrent-goal rejection, cancellation, cleanup, and the
      parallel interleavings from 2C.

## Phase 4 - Registry, manifest, and contract consolidation

- [ ] Define the resolved manifest contract and schema/version bump.
- [ ] Reduce execution profiles to selection-only composition.
- [ ] Generate MoveIt controller config, joint-state merger inputs, and launch
      parameter dictionaries from the resolved manifest.
- [ ] Move coordinator resource claims to manifest-driven data.
- [ ] Keep the MoveIt hand FJT path non-default in coordinated production
      profiles.
- [ ] Remove or quarantine the legacy unprefixed `moveit_controllers.yaml`.
- [ ] Consolidate `HandCmd`, `HandPositionTimeCmd`, `HandStatus`,
      `GripperStatus`, and `OmniHandStatus` into one abstract hand contract per
      C5, with a caller migration note (4D).
- [ ] Carry owner identity, control epoch, and sequence in hand commands.
- [ ] Validate joint values at the bridge; remove SDK read-before-write; reject
      partial commands without a valid cache.
- [ ] Distinguish `commanded`, `delivery_verified`, and `contact_confirmed`
      completion.
- [ ] Add manifest-hash consistency checks across runtime nodes.
- [ ] Validate source/install path resolution and fail-closed behavior.
- [ ] Add a repository check for duplicated authoritative values.

## Phase 5 - Runtime consolidation and close-out measurements

- [ ] Make MIT action completion event-driven with one trajectory sampler.
- [ ] Decompose the MIT tick before optimising it (trajectory sample, feedback
      snapshot, gravity/RNEA, command construction, ROS publish, action
      feedback/tolerance, locking/executor). "Gravity dominates" is a hypothesis;
      the same assumption about the arm driver's SDK reads was measured wrong.
- [ ] Then reduce whatever the decomposition names, so the rate can rise toward
      200-250 Hz (C2).
- [ ] Split OmniHand bridge timers by command verification, tactile, and status
      semantics.
- [ ] Bound executor thread counts and keep each vendor SDK session in its own
      process.
- [ ] Remove duplicate hand-joint aggregation from arm driver output.
- [ ] Re-run CPU and CAN baselines, including the parallel scenarios, and compare
      them against 0E.

## Phase 6 - Unit contract skeleton

- [ ] Freeze the unit-level public contract skeleton for later multi-unit work.

## Every phase

- [ ] `tea_pour_left_v1` still runs after the phase closes.
- [ ] L1 and L2 pass before any hardware run; L3 evidence recorded or its absence
      stated explicitly.

## Documentation follow-through

- [ ] Promote only stable runtime-contract changes into `docs/assets/`.
- [ ] Promote only stable operational changes into `docs/control/`.
- [ ] Update `docs/project/` if package boundaries, ownership, or generated
      artifact policy change.
