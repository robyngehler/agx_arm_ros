# Running the Jetson headless over SSH

status: EVALUATION
last_updated: 2026-09-05
scope: what is in place for operating a unit with no monitor, and what is missing

Measured on the unit 2026-09-01. Items 4 and 6 have since been acted on and are
marked; the rest is a report. §4 and §6 were extended 2026-09-05 for the
three Duo units, two of which share a router.

## What already works

| Piece | State |
| --- | --- |
| SSH server | `ssh.service` enabled and active, listening on `0.0.0.0:22` and `[::]:22` |
| Key login | `~/.ssh/authorized_keys` present |
| mDNS | `avahi-daemon` enabled and active |
| WiFi | connected as a client to `agx-7ax-nju`, `192.168.31.50/24`; a second profile `agx-7ax-cym` is saved, both autoconnect |
| Wired fallback | `eno1` up at `192.168.209.231/24` |
| tmux | installed |
| AP capability | the radio reports `AP` among its supported interface modes, so the hardware can do it |

So an interactive `ssh nvidia@192.168.31.50` works today, and the demo scripts
run in it.

## What is missing

### 1. There is no access-point profile — the Jetson joins one, it does not make one

Both saved WiFi connections are `802-11-wireless.mode: infrastructure`. The
current arrangement is *Jetson and laptop on an external router*, not *Jetson is
the network*. If the intent is that the unit brings its own network to a demo,
that profile does not exist yet.

The radio supports it, so this is a configuration step, not a hardware question:
an `nmcli` connection with `802-11-wireless.mode ap`, `ipv4.method shared`, and a
lower `autoconnect-priority` than the two client profiles so it is the fallback
rather than the default. **Untested on this unit** — an AP profile that
autoconnects ahead of the client profiles would take the unit off the network it
is currently reachable on, so it wants a wired session or a monitor for the first
attempt.

### 2. `ssh <host> '<command>'` has no ROS environment — accepted, not fixed

`~/.bashrc` returns for non-interactive shells at lines 5-9 and sources ROS at
lines 121-123. Verified against a clean environment:

```
env -i bash  -c 'command -v ros2'   -> not found
env -i bash -lc 'command -v ros2'   -> not found
env -i bash -ic 'command -v ros2'   -> found
```

So an interactive session works and a scripted one does not:
`ssh jetson './scripts/start_tea_demo.py --no-prompt'` fails before it starts.
That is exactly the form a laptop-side wrapper, a cron entry or a remote
watchdog would use.

**Decided 2026-09-01: left as it is.** The unit is operated by opening an
interactive session and typing in it, not by firing single remote commands, and
an interactive session has the full environment. Moving the sourcing above the
guard would change the environment of every non-interactive shell on the machine
to fix a workflow nobody uses.

What this rules out, so it is not rediscovered later: a laptop-side wrapper, a
cron entry, or a remote watchdog that starts a demo with `ssh jetson '<command>'`.
Any of those needs the sourcing moved first.

### 3. The hostname is `ubuntu`

mDNS therefore advertises `ubuntu.local`, which collides with every other stock
Ubuntu machine on the same network — the normal situation at a demo, and the
case where a name instead of an IP matters most.

### 4. Power saving, WiFi included — handled by a script

Measured before: `nvpmodel` MAXN but the CPU governor `schedutil` on all twelve
cores, cpu0 idling at 883 MHz against a 2.2 GHz maximum; WiFi power save on; USB
autosuspend `auto` on four devices, two of which are the CAN FD adapters; PCIe
ASPM at `default`.

`scripts/jetson_performance_mode.sh` takes all of it off — power mode, CPU
governor, GPU and memory clocks, WiFi radio, USB autosuspend, PCIe link states —
and `--install` writes a systemd unit so a reboot does not restore the defaults.
`--show` reports without changing anything.

The governor is the one that reaches the arm. The MIT loop is a paced thread
doing bounded work per cycle, so it reads as low load; `schedutil` clocks the
cluster down and the next burst starts on a slow core, with the ramp costing
several control cycles.

The trade: the unit draws its full budget and the fan runs harder whether or not
it is doing anything.

Two further scripts cover the same ground with a restore path, for a demo the
unit is meant to come out of again. They split on one line: what costs latency or
a link, and what costs heat.

`scripts/jetson_presentation_mode.sh {on|off|status}` takes off everything that
lets the platform stall something — `nvpmodel` to MAXN, `performance` governor,
all cores online, USB/PCI/net runtime PM, PCIe ASPM, WiFi power save, the sleep
targets. **It leaves the CPU idle states alone**, so cores reach full clock under
load and still idle between bursts. `nvpmodel` here is a ceiling, not a floor: it
permits the top clocks rather than asking for them, and a governor cannot reach a
clock the power model forbids.

`scripts/jetson_clock_boost.sh {on|off|status}` is the rest — `jetson_clocks`,
which pins CPU min to max, pins GPU and memory clocks, and **disables the CPU
idle states**. That is the one that draws the full budget whether or not the unit
is doing anything, so it is a separate opt-in.

WiFi power save is set twice on purpose: with `iw` for the link that is up now,
and in the NetworkManager profile so a reconnect does not put it back. The demo
is operated over that radio, so the profile is the half that matters — and the
profile, like `nvpmodel`, survives a reboot. Their previous values are therefore
kept in `/var/lib/jetson-presentation-mode` rather than `/run`, so `off` still
undoes them after a reboot. Everything else is runtime state that a reboot resets
on its own, and its saved values live in `/run` beside it.

Do not mix these with `jetson_performance_mode.sh` in one session: that one calls
`jetson_clocks` without `--store`, so a `jetson_clock_boost.sh on` afterwards
records the already-boosted clocks as the state to restore.

### 5. The unit boots to `graphical.target` — it runs a desktop nobody sees

`systemctl get-default` returns `graphical.target`, so the machine starts the
full GNOME session: display manager, compositor, and the desktop services behind
them. With no monitor attached, all of that runs and renders for nobody.

SSH does not depend on it, so headless works today — this is about what the
machine spends on itself. This repository already treats CPU here as a budget:
`use_rviz:=false` is documented in the tea runbook as "a CPU decision, not
cosmetic". Switching to `multi-user.target` returns that budget.

```bash
sudo systemctl set-default multi-user.target     # text boot, no desktop
sudo systemctl set-default graphical.target      # back again
```

The trade is that plugging a monitor in later gives a console, not a desktop,
until the target is switched back and the machine rebooted. **Not changed** —
this is a decision about how the unit is used, not a defect.

### 6. The ROS graph on the network — done, and it is per unit

Was `ROS_LOCALHOST_ONLY=0` with no domain id, so any ROS 2 machine joining the
same AP would have joined the graph, and discovery ran as multicast over WiFi.

**Every unit says which unit it is, and the rest follows from that.**
`scripts/isolate_ros_graph.sh --unit <name>` writes one managed block into
`~/.bashrc` above the ROS sourcing — `AGX_UNIT`, `ROS_LOCALHOST_ONLY=1` and the
`ROS_DOMAIN_ID` derived from the unit — backs the file up, and stops the `ros2`
daemon, which otherwise keeps serving the graph of the domain it was started in.
`--show` reports, `--revert` removes it.

| `AGX_UNIT` | What it is | Profile | Domain |
| --- | --- | --- | --- |
| `top` | tea-demo installation, upper | `duo_hand` | 41 |
| `bottom` | tea-demo installation, lower | `duo_arm` | 42 |
| `stacking` | the solo unit, AGX grippers, block restack | `duo_gripper` | 50 |

`--domain` overrides the derived one.

`AGX_UNIT` is the same identity the script layer uses:
`scripts/start_demo_stack.py` takes the execution profile from it and every
activity script refuses to run on the unit it was not written for. Which unit a
machine is was previously encoded separately in the script the operator typed,
the profile the stack came up with, and the domain — three places that could each
be wrong on their own.

Only `top` and `bottom` share a router and therefore have a conflict to resolve.
`stacking` stands on its own and is configured the same way regardless, because
the identity is worth more than the isolation: a unit that declares what it is
cannot be brought up as the wrong one.

The network is only how a unit is reached; the graph stays on it. Discovery and
traffic are confined to loopback, so nothing on the WiFi can join and nothing
leaves for it — which also takes the flakiest part of multi-machine ROS,
multicast discovery over WiFi, out of the picture.

**With `top` and `bottom` on one router this is a safety property, not hygiene.**
The stack names its topics by side, not by unit: both units publish
`/left_arm/feedback/joint_states`, both offer `/right_arm/emergency_stop` and
`/execute_activity`, and `/tf` is global with identical frame names on both. On a
shared graph one trajectory command reaches two arm drivers, and each unit's
MoveIt collision-checks against the other's poses. Nothing needs the network:
each unit starts its own launches and runs `run_activity` against them locally
(`scripts/demo_stack.py`). The domain id is the redundant half — it still
separates the units if `ROS_LOCALHOST_ONLY` is unset for a debugging session, and
localhost-only still separates them if both end up on the same domain.

Domains: keep them under 101, where the DDS ports start reaching into the
ephemeral range, and off 77, which the L2 activity harness claims and refuses to
share.

The cost, stated plainly: no laptop-side RViz, `ros2 topic echo` or rqt against a
unit any more. The demo stacks run with `use_rviz:=false` in any case. The
exports are in `~/.bashrc`, so an interactive SSH session has them and
`ssh <host> '<command>'` does not — see §2.

## 7. A dropped connection orphans the stack — and that one is ours

The supervisor starts its launches with `start_new_session=True` on purpose, so
it decides when they stop and in which order rather than a terminal signal
reaching all of them at once. The same property means a SIGHUP from a dropped SSH
session kills **only the supervisor**: its teardown never runs, and the launches
keep going with a live arm driver and nobody supervising it. The next run then
finds the buses held.

Splitting the supervisor from the activity scripts moved this rather than fixing
it — but it also left a trace. The supervisor writes
`~/.cache/agx_demo_stack/<unit>.json` with its pid and log directory, so a second
SSH session can find what is still running instead of guessing;
`stop_demo_stack.py` reads it, and clears it when the supervisor it names is
gone. Not `/run/user/<uid>`, which systemd removes when the user's last login
session ends — precisely the dropped-SSH case.

`start_demo_stack.py` warns when it is started over SSH outside tmux or screen
and names the command to use. It is a warning rather than a refusal: an operator on a
wired link with a monitor beside them does not need it.

**Run the stack inside tmux when working over SSH:**

```bash
tmux new -A -s stack
./scripts/start_demo_stack.py
# detach with ctrl-b d; the stack survives the disconnect
tmux attach -t stack
```

Activities run from a second pane against that stack. Only the supervisor has to
survive a disconnect — an activity that loses its terminal loses `run_activity`
with it, and the coordinator's own cancel path takes over from there.

## Where it stands

Done: confining the ROS graph to loopback has a script (§6), and every
power-saving knob has one (§4). **Both still have to be run on each unit** — they
are not applied yet, and the graph isolation has to be verified on *both* units,
not on one.

Open, in the order they are worth doing:

1. `./scripts/isolate_ros_graph.sh --unit top|bottom|stacking` on each of the
   three units, then `--show` on top and bottom with the other one's stack up
2. run `sudo ./scripts/jetson_performance_mode.sh --install`
3. rename the host away from `ubuntu`, so `.local` resolves to this machine and
   not to whichever stock Ubuntu box booted first (§3) — with three units this is
   also what tells them apart in an SSH session
4. add the AP profile, from a wired session, at a lower autoconnect priority than
   the two client profiles (§1) — this is what makes the unit independent of a
   room's network
5. switch the boot target once nobody needs the desktop (§5)

Deliberately not done: moving the ROS sourcing for non-interactive SSH (§2), and
anything that would put the ROS graph back on the network.
