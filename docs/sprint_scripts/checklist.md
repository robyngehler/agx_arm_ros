# Sprint Scripts — operator script layer status

Target: one operator entry point per verb — activate the stack, unpack and pack
each unit, start the tea demo — over the launches and the `run_activity` client
that already exist. No new ROS package and no new ROS surface.

Source proposal: `../demo_script_proposal.md`.

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
| The five operator scripts | landed — `scripts/demo_stack.py` plus one CLI per verb |

## Not done

| Item | Why |
| --- | --- |
| **Hardware validation** | Nothing below has been run against the arms. See the gate. |
| Event-based recording and its playback (proposal §8.2, §9) | Belongs to the teach loop, not to this layer; follows the Piper gripper's own event work |
| Recording → catalogue conversion for gripper events (§10) | Follows the above |
| The automatic recovery trigger's calibration | `activate_stack.sh` judges a bus on RX advancing and flat error counters. The reported first-start symptom is *messages rising but MoveIt never starts*, and that state has never been measured — so `--recover` runs the chain unconditionally until it has been |

## Hardware validation gate (not started)

1. `sudo bash scripts/activate_stack.sh` on a cold boot — buses up, verified
2. the same during the failing first-start state: `--show` only, to capture the
   error counters that would separate it from a healthy bus
3. `./scripts/unpack_bottom_unit.py --slow`, then `pack_bottom_unit.py --slow`
4. the fast variants of both
5. `unpack_top_unit.py`, `pack_top_unit.py`
6. `start_tea_demo.py --dry-run` — stack up, no goal sent
7. `start_tea_demo.py` end to end
8. Ctrl+C mid-run: the activity must cancel and the launches must survive it long
   enough for the coordinator to unwind
9. `--from-id N` after that cancel, using the number the script printed
10. an emergency stop, then the explicit re-arm, then a resumed run

Items 8 and 9 are what decide whether this layer is worth anything: everything
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
