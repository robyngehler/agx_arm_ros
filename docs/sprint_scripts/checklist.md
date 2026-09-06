# Sprint Scripts — operator script layer status

Target: one operator entry point per verb — activate the stack, unpack and pack
each unit, start the tea demo — over the launches and the `run_activity` client
that already exist. No new ROS package and no new ROS surface.

Source proposal: `demo_script_proposal.md`. The lifecycle split that followed it:
`script_refactor_proposal.md`.

## Done

| Area | State |
| --- | --- |
| Operator step model | landed — `graph_model.operator_steps`; one dispatch batch is one step, so a sync pair is one step. `tea_pour_duo_v2` is **21 steps**, not the 17 the proposal assumed |
| `tea_pour_duo_v2` step contract under test | landed — the sequence is asserted step by step, so an inserted node fails a test rather than renumbering an operator's flow silently |
| One-way pack/unpack activities | landed — 4 bottom + 2 top, at 3/3/5/5/2/2 steps; every one starts by moving to its declared start pose |
| Resume in the coordinator | landed — `{"resume": {"from_step": N}}`, seeded before planning; skipped steps are not pre-planned |
| Resume refusals | landed — past the end, and onto a taught replay, naming the nearest earlier planned step |
| `run_activity --from-id N` | landed — `--from_id` as an alias; declaring it beside a `resume` block in `--metadata-json` is refused, not merged |
| CAN activation with verification and recovery | landed — `scripts/activate_stack.sh`, including the `rmmod mttcan` / `modprobe` / reactivate cycle |
| The operator scripts | landed — `scripts/demo_stack.py` plus one CLI per verb: four pack/unpack flows, the wave, the tea demo, and the block restack |
| Stack lifecycle split from activity execution | landed — `start_demo_stack.py` owns the launches and stays alive; every activity script attaches to it. A cancelled activity now leaves the stack up, which is what makes the `--from-id` it prints usable |
| Unit identity | landed — `AGX_UNIT` (`top`, `bottom`, `stacking`), written by `isolate_ros_graph.sh --unit` beside the ROS domain it derives (41/42/50). The stack profile follows it — `duo_hand`, `duo_arm`, `duo_gripper` — and an activity refuses to run on the unit it was not written for |
| Sequential bring-up | landed — components are waited for before coordination is started, each phase reported separately. The coordinator's action clients only wait for their servers at dispatch, so `/execute_activity` says nothing about the arms |
| Orderly shutdown | landed — `stop_demo_stack.py` signals the supervisor named in `~/.cache/agx_demo_stack/<unit>.json`; the supervisor stops coordination, waits, then components, escalating SIGINT → SIGTERM → SIGKILL per launch. No `pkill ros2` |
| Persistent logs | landed — `logs/demo_stack/<unit>_<timestamp>/`, one file per launch, instead of a temp dir that disappeared with the run |
| `wave_after_unpack_v1` runnable by script | landed — `scripts/wave.py`, top unit, between unpack and pack or on its own |
| `block_restack_v1` runnable by script | landed — `scripts/start_block_restack.py` on the `duo_gripper` stack, waiting for both gripper trajectory servers. 63 operator steps, no replay, so any step is a resume point |
| Every shipped activity checked on load | landed — `test_shipped_activities` sweeps `config/activities/`, so a new activity is covered without anyone remembering to add it |
| One command from SSH login to READY | landed — `scripts/start_demo_session.sh` runs the cold order (platform, CAN, stack) and waits for READY, putting the supervisor in tmux session `agx-stack` rather than owning it. Refuses without tmux, and skips bus activation under a live stack |
| The resume point machine-readably | landed — `<log_dir>/last_activity.json` carries `completed_step` and `resume_from_id` from the same `operator_resume` call that prints the prose, so a non-terminal caller does not scrape it |
| A resume that follows a replay goes **back**, not forward | fixed 2026-09-06 — `next_resume_step` skipped a taught replay to reach the next planned step, so a cancel after step 8 of the tea demo suggested step 11 and would have poured with the can still on the table. It now names the nearest earlier planned step, the same one `resume_seed` names when the replay is asked for directly; the two no longer give opposite answers to the same situation. The cost is re-running one anchor move, which plans from the current state |
| CAN health as data | landed — `activate_stack.sh --show --json` / `--verify-only --json`, no sudo, same verdicts as the table |

## Not done

| Item | Why |
| --- | --- |
| **Hardware validation** | Nothing below has been run against the arms. See the gate. |
| Event-based recording and its playback (proposal §8.2, §9) | Belongs to the teach loop, not to this layer; follows the Piper gripper's own event work |
| Recording → catalogue conversion for gripper events (§10) | Follows the above |
| Browser operator UI | proposed, not implemented — `demo_web_ui_proposal.md`, rewritten 2026-09-06 against the unit model. It covers all four layers (platform, CAN, stack, activity), not only the ROS stack, and needs one prerequisite in this layer: `_execute()` must write its resume point machine-readably so the UI does not scrape the printed hint |
| Headless operation over an access point | evaluated, not configured — see `headless_operation.md`. SSH, mDNS and key login are in place and the radio supports AP mode; the AP profile, the ROS environment for non-interactive SSH and the hostname are not done. **tmux is not installed** (checked on `top`, 2026-09-06) and everything that survives a dropped SSH session depends on it. Graph isolation and unit identity **are** applied on `top` (`AGX_UNIT=top`, domain 41, localhost-only), unverified on `bottom` and `stacking`; the power-saving knobs still have to be run per session or per unit |
| A dropped SSH session still orphans the stack | the supervisor holds the launches, so a SIGHUP takes it and leaves them. The state file makes what is left findable; it does not stop it happening. tmux remains the answer (`headless_operation.md` §7) |
| `--stop-stack-on-cancel` | not implemented. A cancelled activity leaves the stack up, deliberately; a presentation mode that ends everything on one Ctrl+C would be a flag on the activity scripts |
| The automatic recovery trigger's calibration | `activate_stack.sh` judges a bus on RX advancing and flat error counters. The reported first-start symptom is *messages rising but MoveIt never starts*, and that state has never been measured — so `--recover` runs the chain unconditionally until it has been |

## Found by the activity sweep, not fixed

Three shipped activities load and schedule but cannot be planned: they name
anchor poses that were re-captured under other names and no longer exist in
`arm_config.yaml`. They are quarantined in `test_shipped_activities` with the
reason, and the quarantine itself is asserted, so one that gets re-anchored fails
the test rather than sitting in the list.

| Activity | Missing anchors |
| --- | --- |
| `tea_pour_left_v1` | `Can_Grip_Idle_L`, `Can_Pre_Grip_L`, … — re-captured as `Tee-Can_*`. Already documented as unrunnable in `docs/control/bringups/tea_demo.md` |
| `hefeweizen_pour_v1` | `Pre_Grip_L`, `grasp_L`, … — the pose set that predates that re-capture. **Not** documented anywhere |
| `both_arms_pregrasp_grasp_retract_v1` | the same pose set |

Re-anchoring them is a judgement about which current pose replaced which old one,
so it is left to whoever captured them.

## Hardware validation gate

**Top unit, 2026-09-06: items 1, 3, 6 and 7 passed.** One `start_demo_session.sh`
brought the platform, the buses and the stack up — all four buses ERROR-ACTIVE
with flat error counters, the arms pushing 2126 and 2188 frames/s — and
`unpack_top_unit.py`, `wave.py` and `pack_top_unit.py` then ran against **one**
bring-up, which is what the lifecycle split exists for. The reteached
`wave_after_unpack_v1` completed all 7 steps; the stale expectations are in
`test_wave_after_unpack_v1.py`, not in the activity.

**Item 7 passed on the third attempt, 2026-09-06.** `stop_demo_stack.py` reported
`stopped`, and both arm drivers ended as `process has finished cleanly` where the
attempt before had them dying on signal inside the hold. The shutdown hold now
completes, which is the whole point of the rung.

Still noisy, and not on the hold path: the `mit_tools` helpers and both MIT
controllers exit -2 with a `KeyboardInterrupt` out of `spin()`, and `move_group`
exits -11. None owns a CAN session or asserts a hold, but launch reports each as
`ERROR ... process has died`, which trains an operator to read past exactly the
line that would carry a real one.

**What the two earlier attempts found:**
The supervisor SIGINT'd each launch's whole process group, but `ros2 launch`
forwards SIGINT to its nodes itself — so every node received two. The first
started the arm driver's `hold_on_shutdown`, the second arrived inside
`_assert_firmware_hold` and ended it as a `KeyboardInterrupt`: both drivers
exited on signal with the `MOVE-J(current_q)` assertion incomplete, leaving the
firmware on its last setpoint rather than on the hold the ladder specifies.
Shutdown is a rung like any other, and this teardown skipped it.

The teardown now signals the launch process only and leaves the group to SIGKILL,
where nothing is left to interrupt. The first attempt was run under `sudo`, which
has no `AGX_UNIT` and a different home, so it reported a running stack as stopped
and exited 0; the operator scripts refuse root now, and a supervisor that is not
found names the path it looked in.

**The stop then waited on a process that had already exited.** The supervisor is
the process of a tmux pane held open by `remain-on-exit`, so once it exits it
stays defunct until that pane is closed — and `os.kill(pid, 0)`, which
`StackState.alive()` used, succeeds for the whole of that time. Measured
2026-09-06: pid 19555 in state `Z`, launches down, `all launches are down`
printed, stop still reporting `still shutting down`. `alive()` reads the process
state and calls a zombie exited; the same reading previously refused a fresh
bring-up and told activity scripts a dead stack was up.

The state file is now removed after the launches rather than before, so its
absence does not report "no stack" while the arms are still coming down.

Items 2, 4, 5, 8-13 remain open; items 10 and 11 are still the ones that decide
whether this layer is worth anything.

Per unit, after `isolate_ros_graph.sh --unit <this one>` and a new session.

1. `sudo bash scripts/activate_stack.sh` on a cold boot — buses up, verified
2. the same during the failing first-start state: `--show` only, to capture the
   error counters that would separate it from a healthy bus
3. `./scripts/start_demo_session.sh` from a fresh SSH login — platform, buses,
   then the supervisor in tmux `agx-stack`, ending at READY. Nothing is
   commanded; this replaces the old `--dry-run`. Then `--status`, and a second
   run against the running stack: it must skip bus activation, not repeat it
4. top and bottom both up: `isolate_ros_graph.sh --show` on each counts only its
   own nodes. This is the pair that shares a router; `stacking` stands alone
5. bottom: `unpack_bottom_unit.py --slow`, then `pack_bottom_unit.py --slow`, then
   the fast variants
6. top: `unpack_top_unit.py`, `wave.py`, `pack_top_unit.py` — three activities
   against **one** bring-up, which is the point of the split
7. `stop_demo_stack.py`: coordination exits, then components, state file gone
8. stacking: `start_demo_stack.py` comes up on `duo_gripper` with no flag, then
   `start_block_restack.py`
9. tea: `start_demo_stack.py --stack tea`, then `start_tea_demo.py` end to end
10. Ctrl+C mid-activity: the activity cancels, and the stack is **still up**
    afterwards
11. `--from-id N` against that same stack, using the number the script printed —
    and check `<log_dir>/last_activity.json` carries the same `resume_from_id`
12. an emergency stop, then the explicit re-arm, then a resumed run
13. an activity against the wrong stack and on the wrong unit — both must be
    refused before anything is sent

Items 10 and 11 are what decide whether this layer is worth anything: everything
else is a shorter way to type commands that already worked.

## Open questions

- **Does a resume need a planned approach of its own?** Today a resume is refused
  onto a replay and allowed onto an anchor move, on the grounds that an anchor
  move plans from the current state. That is true of the *planner*, but nothing
  checks how far the arms are from where the previous run left them. A resume
  after the arms were moved by hand is a long planned motion nobody watched
  start. Backing a resume up to the anchor before a replay (fixed 2026-09-06)
  makes this matter more, not less: that anchor move is now the thing that
  guarantees the replay's start pose, so it is the motion whose length nobody
  checks.
- **Where does `--speed` belong?** It picks between two activities today
  (`unit_unpack_bottom_fast_v1` / `_slow_v1`), which is honest but means the step
  numbers differ between them. An operator who resumes a slow run with `--fast`
  gets a different flow at the same step number.
