# Demo Web UI Proposal

status: PROPOSAL
last_updated: 2026-09-06
scope: a browser operator layer over the existing scripts, and the SSH session it lives in

Rewritten 2026-09-06 against the script layer as it actually stands. The original
draft assumed one machine holding a `--top` and a `--bottom` stack, an operator
layer that begins at the ROS stack, and a cancel that ends the run. All three
readings are superseded and corrected in place below:

| Superseded reading | Current state |
| --- | --- |
| `start_demo_stack.py --top` / `--bottom` on one host | a unit **is** one machine; identity is `AGX_UNIT`, and there are three of them (`top`, `bottom`, `stacking`). No `--top` flag exists |
| the UI starts at the ROS stack | a cold unit needs the platform knobs and the CAN buses first, and the CAN activation refuses to run once the stack holds the sockets |
| `CANCEL ACTIVITY` is one SIGINT | the cancel path is a three-press ladder: cancel, emergency stop, abandon |
| a cancelled run ends there | a cancelled run prints the step to resume from, and resuming is the reason the stack now outlives the activity |

## Goal

A minimal browser interface over the operator scripts, so a demo is run from a
laptop without typing script names into an SSH terminal.

The UI must:

- run locally on one unit and control only that unit;
- be reachable only through an SSH tunnel;
- trigger the existing scripts and never reimplement what they do;
- cover the whole cold-start order, not only the ROS stack;
- show state derived from the unit, not from which button was pressed;
- offer cancel, emergency stop, re-arm and resume;
- survive the browser closing, the tunnel dropping and its own restart.

## 0. What the UI has to control — four layers, in order

The demo does not start at the ROS stack. A cold unit needs all four, in this
order, and `activate_stack.sh` enforces the third-before-fourth part itself: it
refuses to reload the CAN driver while an arm driver or hand bridge holds a
socket.

| Layer | Script | Root | When |
| --- | --- | --- | --- |
| platform latency | `scripts/jetson_presentation_mode.sh {on\|off\|status}` | yes (re-execs under sudo) | once per boot, before the demo |
| platform clocks | `scripts/jetson_clock_boost.sh {on\|off\|status}` | yes (re-execs under sudo) | optional, opt-in; draws full power at idle |
| CAN buses | `scripts/activate_stack.sh [arms\|hands\|--all] [--show\|--recover]` | yes, except `--show` | once per boot, and again after a bus fault |
| ROS stack | `scripts/start_demo_stack.py`, `scripts/stop_demo_stack.py` | no | once per demo; outlives every activity |
| activities | `unpack_*.py`, `wave.py`, `pack_*.py`, `start_tea_demo.py`, `start_block_restack.py` | no | many per stack |

`scripts/start_demo_session.sh` runs rows 1, 3 and 4 of that table in order and
waits for READY. The UI's platform, CAN and START STACK controls do the same
steps individually; where the two disagree, that script is the reference.

**`jetson_performance_mode.sh` must not appear in the UI.** It calls
`jetson_clocks` without `--store`, so a `jetson_clock_boost.sh on` afterwards
records already-boosted clocks as the state to restore and `off` never gets back.
Offering both buttons beside each other is the most likely way that happens.

`scripts/isolate_ros_graph.sh --unit <name>` is **not** a UI button. It edits
`~/.bashrc` and stops the ROS daemon; it is one-time provisioning per unit, done
before there is a UI to press.

## 1. Architecture — one UI per unit

```text
Laptop browser
    ↓  http://localhost:8080
SSH tunnel  -L 8080:localhost:8000
    ↓
Jetson 127.0.0.1:8000   (this unit only)
    ↓
demo_ui.py
    ↓
scripts/*.sh, scripts/*.py
    ↓
ROS 2 stack / activities
```

Python + Flask + plain HTML/CSS + minimal JavaScript. No React, no Node, no
rosbridge, no X11 forwarding, no direct ROS control from the browser.

**One UI instance per unit, and it controls no other unit.** `top` and `bottom`
are separate Jetsons on one router, each with `ROS_LOCALHOST_ONLY=1` and its own
`ROS_DOMAIN_ID`; the graph does not cross the network, deliberately, because both
units publish `/left_arm/feedback/joint_states` and offer `/execute_activity`
under the same names. A single UI driving both is either a ROS graph on the WiFi
— which the isolation exists to prevent — or `ssh other-unit '<command>'`, which
has no ROS environment on these units and is documented as not being fixed
(`headless_operation.md` §2).

Two units therefore means two tunnels on two local ports:

```bash
ssh -L 8080:localhost:8000 nvidia@<top>       # → http://localhost:8080
ssh -L 8081:localhost:8000 nvidia@<bottom>    # → http://localhost:8081
```

The UI reads `AGX_UNIT` at startup and names it in the page title and in a header
band, so two browser tabs are not confusable. If `AGX_UNIT` is unset the UI
refuses to start — the same refusal `resolve_unit()` already makes.

## 2. Network and SSH

Bind only to:

```text
127.0.0.1:8000
```

Do not expose port 8000 on the LAN or WiFi interface.

**The UI has no authentication, and that is only acceptable because of the bind.**
Anyone with a shell on the Jetson can reach it, and it drives passwordless `sudo`
(§0). Two bounds follow, both mandatory:

- every command the UI can run is a fixed entry in an allowlist in `demo_ui.py`;
- **no request parameter reaches a command line as free text.** Arguments are
  chosen from closed sets (`--speed fast|slow`, `--from-id <int>`), validated as
  their own type, never interpolated as strings.

The tunnel is also the failure case worth naming: if the SSH link drops, the stop
button goes with it. The UI is a convenience layer over the scripts, never the
only stop path — see §7 and §11.

## 3. tmux — and what the UI must *not* own

The UI itself runs in tmux, started from an interactive SSH session so it has the
ROS environment (`~/.bashrc` sources ROS only for interactive shells):

```bash
tmux new -A -s demo-ui
./scripts/demo_ui.py
# detach: Ctrl+B, D
```

**The UI must not hold the stack supervisor as its own child.** The supervisor
owns the launches and is responsible for their ordered teardown; if it dies
without running that teardown, the launches keep going with a live arm driver and
nobody supervising it, and the next run finds the buses held
(`headless_operation.md` §7). A UI that crashes, is restarted, or is killed with
the tmux session would do exactly that to a stack it owned.

So the UI starts the supervisor **in its own detached tmux session** and tracks it
through the state file, not through a `Popen` handle:

```bash
tmux new-session -d -s agx-stack './scripts/start_demo_stack.py'
```

This has a second payoff: the UI is then correct about a stack an operator
started by hand in tmux, and a restarted UI re-attaches to the running demo
instead of reporting OFFLINE beside a moving arm.

## 4. UI controls

Sections follow the four layers of §0, top to bottom, so the page reads in the
order the demo is run.

```text
UNIT: TOP                              [header band, from AGX_UNIT]

PLATFORM
  PRESENTATION MODE   ON | OFF | STATUS
  CLOCK BOOST         ON | OFF | STATUS

CAN BUSES
  SHOW                (no sudo, safe at any time)
  ACTIVATE            (all four)
  RECOVER             (reload chain; refused while the stack is up)

STACK
  START STACK         [ demo | demo --grippers | tea ]
  STOP STACK

ACTIVITY
  <the activities of this unit only>
  CANCEL ACTIVITY
  RESUME AT STEP N    (offered only after a run that did not complete)

SAFETY
  EMERGENCY STOP
  RE-ARM
```

The activity buttons come from this unit and the running stack, not from a fixed
list:

| `AGX_UNIT` | stack | activities offered |
| --- | --- | --- |
| `top` | `demo` (`duo_hand`) | `unpack_top_unit`, `wave`, `pack_top_unit` |
| `top` / `bottom` | `tea` | `start_tea_demo` |
| `bottom` | `demo` (`duo_arm`) | `unpack_bottom_unit`, `pack_bottom_unit`, each `--speed fast\|slow` |
| `stacking` | `demo` (`duo_gripper`) | `start_block_restack` |

Buttons whose precondition is not met are disabled with the reason shown, not
hidden: an activity with no stack up, a `tea` activity against a `demo` stack, a
CAN recovery while the stack holds the sockets. The scripts refuse all of these
anyway — `require_unit()`, the `state.stack != spec.stack` check, the socket
check in `activate_stack.sh` — so the UI is repeating a refusal to save a click,
never inventing one. **Where the two disagree, the script wins.**

## 5. Reuse the existing scripts

The UI calls the scripts. It does not build `ros2 launch` or `ros2 run` command
lines of its own, and it does not read the activity YAML.

```text
sudo ./scripts/jetson_presentation_mode.sh on|off|status
sudo ./scripts/jetson_clock_boost.sh on|off|status
sudo ./scripts/activate_stack.sh --all
     ./scripts/activate_stack.sh --show
sudo ./scripts/activate_stack.sh --recover

     ./scripts/start_demo_stack.py [--stack tea] [--grippers]     (in tmux, §3)
     ./scripts/stop_demo_stack.py

     ./scripts/unpack_top_unit.py    --no-prompt [--from-id N]
     ./scripts/wave.py               --no-prompt [--from-id N]
     ./scripts/pack_top_unit.py      --no-prompt [--from-id N]
     ./scripts/unpack_bottom_unit.py --no-prompt --speed slow|fast [--from-id N]
     ./scripts/pack_bottom_unit.py   --no-prompt --speed slow|fast [--from-id N]
     ./scripts/start_tea_demo.py     --no-prompt [--from-id N]
     ./scripts/start_block_restack.py --no-prompt [--from-id N]
```

**`--no-prompt` is not optional.** Every activity script waits on
`input("Press Enter to start")` by default. Launched from the UI with stdin
closed, that `input()` raises `EOFError`, the wrapper reports "not started" and
returns 130 — every activity fails identically and for a reason no log explains.
Pass `--no-prompt` and give the child `stdin=DEVNULL`.

The confirmation the prompt provided moves into the browser: the UI asks before
dispatching, because a button that moves two arms should not be a single click.

## 6. Process handling and ownership

Two ownership models, because the two kinds of process have different failure
modes.

**The stack supervisor** is not a UI child (§3). Its truth is
`~/.cache/agx_demo_stack/<unit>.json` — `unit`, `stack`, `pid`, `log_dir`,
`started`, `execution_profile` — plus `os.kill(pid, 0)`. Read it every poll; do
not cache it. `running_supervisor()` in `scripts/demo_stack.py` already does
exactly this, including clearing a file whose supervisor is gone, and the UI
should import it rather than reimplement it.

**An activity** is a UI child, because the cancel ladder needs its process group.
Start it with `start_new_session=True` so the UI's own group is never signalled,
and write its pgid, activity name and start time to a small state file of the
UI's own beside the stack's. A UI that is restarted mid-activity can then still
cancel the run it did not start — without that file it can only watch.

Short-running commands (`--show`, `status`, the sudo knobs) run with a timeout
and their output is captured. Long-running ones are never run inside a request
handler; the handler starts them and returns, and the page polls.

Refuse a second launch of anything already running: one activity at a time, one
supervisor per unit (`run_supervisor()` already refuses the second), one platform
command at a time.

## 7. Cancellation — a ladder of three, not one signal

`run_activity_client` counts interrupts. The wrapper sets SIGINT to `SIG_IGN` for
itself and spawns the client in its own process group, so a group SIGINT reaches
the client and only the client:

```text
1st SIGINT   cancel the activity, wait for the coordinator to confirm it unwound
2nd SIGINT   escalate to the unit emergency stop
3rd SIGINT   KeyboardInterrupt — the client leaves while the coordinator unwinds
```

The UI exposes the first two and **not** the third:

- `CANCEL ACTIVITY` sends one SIGINT to the activity's process group.
- While a cancel is in progress the same button becomes `FORCE EMERGENCY STOP`
  and sends the second. It is a distinct label with a confirmation, because it is
  a different action, not an impatient repeat of the first.
- There is no third button. Abandoning the client mid-unwind leaves the
  coordinator finishing a cancel with nothing watching it; an operator who needs
  that has the tmux pane.

Do not stop components or coordination while a cancel is in progress. After a
cancel the stack stays up — that is what makes the printed resume point usable —
unless `STOP STACK` is pressed explicitly.

## 8. Emergency stop and re-arm — required, not optional

**This unit has no mechanical emergency stop.** The arm is either powered or it
is not, and a de-energized Nero has no brakes. A software stop the operator can
reach in one click is therefore the operator's only bounded stop, and it belongs
in the UI as its own control — the §7 ladder only exists while an activity is
running, and an arm can be holding a pose with no activity in flight.

`EMERGENCY STOP` calls the services directly, independent of any activity:

```text
/left_arm/emergency_stop     std_srvs/srv/Trigger
/right_arm/emergency_stop    std_srvs/srv/Trigger
```

`RE-ARM` is the explicit counterpart, exactly what the activity wrapper already
prints after a failed run — and explicit is the point: nothing re-arms an arm
because a page was refreshed.

```text
/left_arm/clear_fault_lockout    std_srvs/srv/Trigger
/right_arm/clear_fault_lockout   std_srvs/srv/Trigger
/unit_safety/rearm               std_srvs/srv/Trigger
```

Both are rendered in a visually separate band, and `RE-ARM` is disabled while an
activity is running.

**State the limit in the operator runbook, not only here:** the UI's stop path
runs over an SSH tunnel and dies with it. The guaranteed stop remains removing
arm power, and it drops the arm.

## 9. Resume — the reason the stack outlives the activity

A UI that can cancel but not resume throws away what the lifecycle split was for.
After a run that did not complete, `demo_stack.py` computes the operator step to
resume from with the coordinator's own step model (`operator_steps`,
`next_resume_step`) and prints `resume with --from-id N`.

**Do not scrape that line.** The step model must have one owner, and a UI parsing
prose is a second one that drifts on the next wording change.

**Landed 2026-09-06.** `_execute()` writes `<log_dir>/last_activity.json` after
every run, completed or not, from the same `_resume_point()` call the prose comes
from:

```json
{
  "script": "wave", "activity": "wave_after_unpack_v1",
  "unit": "top", "stack": "demo", "from_id": null,
  "exit_code": 130, "completed_action": "both_arms_to_wave_both_init_v02",
  "total_steps": 7, "completed_step": 3, "resume_from_id": 5,
  "finished": "2026-09-06T10:55:28"
}
```

The log dir comes from the stack's own state file, so the UI knows where to look
without being told. It offers `RESUME AT STEP 5`, which re-runs the same script
with `--from-id 5` — verified against `check_from_id`, which accepts 5 and
refuses 4 for this activity because step 4 replays a taught path.

Where `resume_from_id` is null the UI says so and offers a plain re-run: a step
that replays a taught path is not a resume point, and `check_from_id()` refuses
it before anything is sent.

## 10. Status display

Derive state from the unit. A button press is not evidence.

```text
UNIT      top          profile duo_hand      domain 41

PLATFORM  presentation ON | OFF | UNKNOWN
          clock boost  ON | OFF | UNKNOWN

CAN       can_nero_left   UP / RX advancing / errors flat
          can_nero_right  UP / RX advancing / errors flat
          hand_left       UP / errors flat
          hand_right      UP / errors flat

STACK     OFFLINE | STARTING | READY | STOPPING | ERROR
          pid 12345   started 14:02   logs logs/demo_stack/top_20260906-140233/

ACTIVITY  IDLE | RUNNING <name> | CANCELLING | COMPLETED | FAILED (exit N)
          last failure: exit 130, step 4 completed, resume from 5
```

Sources, in order of preference — a derived fact beats a remembered one:

| Field | Source |
| --- | --- |
| unit, profile, log dir, supervisor pid | `~/.cache/agx_demo_stack/<unit>.json` via `running_supervisor()` |
| stack READY | the state file **and** the ROS surfaces it must serve — `COMPONENT_SERVICES`, `COMPONENT_TOPICS`, `COORDINATION_ACTIONS` — reusing `_StackWatcher.missing()` |
| CAN health | `./scripts/activate_stack.sh --show --json` — per interface: state, rx/s, error delta, tec/rec, verdict, reason, plus `healthy` and `failed`. No sudo, changes nothing |
| platform | `jetson_presentation_mode.sh status`, `jetson_clock_boost.sh status` |
| activity | the UI's own child state file (§6) plus the result file (§9) |

A stack whose state file says READY but whose surfaces do not answer is `ERROR`,
not `READY` — that is the state a held bus produces, and it is the one worth
distinguishing.

Poll no faster than 1 s, and run the ROS graph query through **one** long-lived
node, not a fresh `rclpy.init()` per request. A recent-log tail from `log_dir` is
useful and stays secondary.

## 11. Server shutdown

```text
1. Cancel the activity if one is running, and wait for it.
2. STOP STACK in the UI; wait for OFFLINE.
3. Optionally: presentation mode OFF, clock boost OFF.
4. tmux attach -t demo-ui   →  Ctrl+C  →  exit
5. Exit SSH.
```

Optional cleanup:

```bash
tmux kill-session -t demo-ui
tmux kill-session -t agx-stack
```

Do not use broad `pkill python`, `pkill ros2` or `killall`. Killing the UI never
stops a stack — by §3 it does not own one.

## 12. Operator workflow

### Cold start

```bash
ssh -L 8080:localhost:8000 nvidia@<unit>

./scripts/start_demo_session.sh          # platform, buses, stack — waits for READY

tmux new -A -s demo-ui
./scripts/demo_ui.py
# Ctrl+B, D
```

Then everything else from `http://localhost:8080`: the activities, CANCEL,
RESUME, STOP STACK.

`start_demo_session.sh` (landed 2026-09-06) is the same four-layer order as §0,
without a browser: it runs `jetson_presentation_mode.sh on`, then
`activate_stack.sh`, then puts `start_demo_stack.py` in detached tmux session
`agx-stack` and waits for READY. `--status` reports all four layers and changes
nothing. It is listed here rather than replaced by a button because the first
session on a unit is the one where the UI is most likely to be the thing that is
broken — and because it is what the UI's START STACK does anyway.

### Between demos

The stack stays up across activities. Only `STOP STACK` ends it.

## 13. Acceptance criteria

Reachability and independence:

- the UI is reachable through the tunnel and on no other interface;
- closing the browser stops nothing;
- detaching tmux stops nothing;
- **killing the UI mid-demo leaves the stack running and the arms supervised**,
  and a restarted UI reports that stack as READY and can still cancel the
  activity it did not start;
- the operator can still run every script by hand if the UI is broken.

Correctness against the script layer:

- the UI refuses to start without `AGX_UNIT`, and offers only that unit's
  activities;
- a `tea` activity against a `demo` stack is refused before anything is sent;
- a CAN recovery is refused while the stack holds the sockets;
- no activity ever hangs waiting for stdin;
- no request parameter reaches a command line as free text.

Safety and recovery — the ones that decide whether this layer is worth building:

- `CANCEL ACTIVITY` cancels the motion and **leaves the stack up**;
- a second press escalates to the emergency stop, from a distinct labelled
  control;
- `EMERGENCY STOP` works with no activity running;
- `RE-ARM` is explicit, and never happens as a side effect of a page load;
- after a cancelled run, `RESUME AT STEP N` runs the same activity from the step
  the coordinator's own step model named, and the arms complete the demo.

## 14. Deliberately not in the UI

- `isolate_ros_graph.sh` — one-time provisioning, edits `~/.bashrc`;
- `jetson_performance_mode.sh` — corrupts the clock-boost restore state (§0);
- anything that edits an activity, a recording or a pose;
- freedrive, teach recording, or any direct MIT command;
- a third cancel press (§7);
- control of any unit other than this one (§1).

## Scope

The UI is an operator convenience layer, not a control architecture.

```text
Browser UI
    ↓
operator scripts
    ↓
ROS 2
```

The scripts stay authoritative for lifecycle, activity execution, refusals and
the step model. The UI's own additions are exactly three: it puts the platform
and CAN layers in the same place as the ROS ones, it exposes the safety ladder as
labelled controls instead of counted keystrokes, and it makes the resume point
clickable. Everything else it does is calling a script and reading a file.

## Prerequisites

Landed 2026-09-06, exercised off hardware:

1. `<log_dir>/last_activity.json` — the machine-readable resume point (§9).
2. `activate_stack.sh --show --json` / `--verify-only --json` (§10).
3. `scripts/start_demo_session.sh` — the cold order as one command (§12), which
   is also the reference for what the UI's platform, CAN and START STACK
   controls have to do.

Still open:

4. **`sudo apt install tmux` on each unit.** It is not installed on `top`
   (checked 2026-09-06), and §3 depends on it entirely — as does
   `start_demo_session.sh`, which refuses to start without it.
5. Confirm passwordless `sudo` for the three root scripts on each unit. The
   intended hardware environment has it, but a UI whose sudo prompts for a
   password hangs a request handler forever.

None of this has been run on hardware. The whole hardware validation gate in
`checklist.md` is still open, and a UI over an unvalidated script layer validates
neither.
