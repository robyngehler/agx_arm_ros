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
| Unit identity | landed — `AGX_UNIT`, written by `isolate_ros_graph.sh --unit` beside the ROS domain. The stack profile follows it and an activity refuses to run on the unit it was not written for |
| Sequential bring-up | landed — components are waited for before coordination is started, each phase reported separately. The coordinator's action clients only wait for their servers at dispatch, so `/execute_activity` says nothing about the arms |
| Orderly shutdown | landed — `stop_demo_stack.py` signals the supervisor named in `~/.cache/agx_demo_stack/<unit>.json`; the supervisor stops coordination, waits, then components, escalating SIGINT → SIGTERM → SIGKILL per launch. No `pkill ros2` |
| Persistent logs | landed — `logs/demo_stack/<unit>_<timestamp>/`, one file per launch, instead of a temp dir that disappeared with the run |
| `wave_after_unpack_v1` runnable by script | landed — `scripts/wave.py`, top unit, between unpack and pack or on its own |
| `block_restack_v1` runnable by script | landed — `scripts/start_block_restack.py` on the `duo_gripper` stack, waiting for both gripper trajectory servers. 63 operator steps, no replay, so any step is a resume point |
| Every shipped activity checked on load | landed — `test_shipped_activities` sweeps `config/activities/`, so a new activity is covered without anyone remembering to add it |

## Not done

| Item | Why |
| --- | --- |
| **Hardware validation** | Nothing below has been run against the arms. See the gate. |
| Event-based recording and its playback (proposal §8.2, §9) | Belongs to the teach loop, not to this layer; follows the Piper gripper's own event work |
| Recording → catalogue conversion for gripper events (§10) | Follows the above |
| Headless operation over an access point | evaluated, not configured — see `headless_operation.md`. SSH, mDNS, key login and tmux are in place and the radio supports AP mode; the AP profile, the ROS environment for non-interactive SSH and the hostname are not done. Graph isolation, unit identity and the power-saving knobs have scripts that **still have to be run on each unit** |
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

## Hardware validation gate (not started)

Per unit, after `isolate_ros_graph.sh --unit <this one>` and a new session.

1. `sudo bash scripts/activate_stack.sh` on a cold boot — buses up, verified
2. the same during the failing first-start state: `--show` only, to capture the
   error counters that would separate it from a healthy bus
3. `./scripts/start_demo_stack.py` in tmux — components ready, then coordination
   ready, then READY. Nothing is commanded; this replaces the old `--dry-run`
4. with the other unit's stack also up: `isolate_ros_graph.sh --show` on both
   counts only its own nodes
5. bottom: `unpack_bottom_unit.py --slow`, then `pack_bottom_unit.py --slow`, then
   the fast variants
6. top: `unpack_top_unit.py`, `wave.py`, `pack_top_unit.py` — three activities
   against **one** bring-up, which is the point of the split
7. `stop_demo_stack.py`: coordination exits, then components, state file gone
8. tea: `start_demo_stack.py --stack tea`, then `start_tea_demo.py` end to end
9. Ctrl+C mid-activity: the activity cancels, and the stack is **still up**
   afterwards
10. `--from-id N` against that same stack, using the number the script printed
11. an emergency stop, then the explicit re-arm, then a resumed run
12. an activity against the wrong stack and on the wrong unit — both must be
    refused before anything is sent

Items 9 and 10 are what decide whether this layer is worth anything: everything
else is a shorter way to type commands that already worked.

## Open questions

- **Does a resume need a planned approach of its own?** Today a resume is refused
  onto a replay and allowed onto an anchor move, on the grounds that an anchor
  move plans from the current state. That is true of the *planner*, but nothing
  checks how far the arms are from where the previous run left them. A resume
  after the arms were moved by hand is a long planned motion nobody watched
  start.
- **Where does `--speed` belong?** It picks between two activities today
  (`unit_unpack_bottom_fast_v1` / `_slow_v1`), which is honest but means the step
  numbers differ between them. An operator who resumes a slow run with `--fast`
  gets a different flow at the same step number.
