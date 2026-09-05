# Running the Jetson headless over SSH

status: EVALUATION
last_updated: 2026-09-05
scope: what is in place for operating the unit with no monitor, and what is missing

Measured on the unit 2026-09-01. Items 4 and 6 have since been acted on and are
marked; the rest is a report. §4 was extended 2026-09-05 with the reversible
demo-mode scripts.

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
unit is meant to come out of again: `scripts/jetson_presentation_mode.sh
{on|off|status}` (CPU governor, CPU hotplug, USB/PCI/net runtime PM, WiFi power
save, sleep targets) and `scripts/jetson_clock_boost.sh {on|off|status}`
(`jetson_clocks`, which pins CPU min to max and disables the CPU idle states).
Both save what they change under `/run`, so the saved state and the settings
disappear together at reboot.

They do **not** set `nvpmodel`, and a governor cannot reach a clock the power
model forbids — check `nvpmodel -q` reads MAXN before relying on them, or run
`jetson_performance_mode.sh` instead. Do not mix the two families in one session:
`jetson_performance_mode.sh` calls `jetson_clocks` without `--store`, so a
`jetson_clock_boost.sh on` afterwards records the already-boosted clocks as the
state to restore.

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

### 6. The ROS graph on the network — done

Was `ROS_LOCALHOST_ONLY=0` with no domain id, so any ROS 2 machine joining the
same AP would have joined the graph, and discovery ran as multicast over WiFi.

**Set 2026-09-01: `ROS_LOCALHOST_ONLY=1` in `~/.bashrc`, above the ROS sourcing.**
The network is only how the unit is reached; the graph stays on it. DDS
discovery and traffic are confined to loopback, so nothing on the WiFi can join
and nothing leaves for it — which also takes the flakiest part of multi-machine
ROS, multicast discovery over WiFi, out of the picture entirely.

The cost, stated plainly: no laptop-side RViz, `ros2 topic echo` or rqt against
this unit any more. Reverse it by unsetting the variable if that is ever wanted.
A backup of the previous `~/.bashrc` is beside it.

## 7. A dropped connection orphans the stack — and that one is ours

The demo scripts start their launches with `start_new_session=True` on purpose,
so a terminal Ctrl+C reaches `run_activity` and its cancel ladder instead of the
stack it is cancelling against. The same property means a SIGHUP from a dropped
SSH session kills **only the wrapper**: its teardown never runs, and the launches
keep going with a live arm driver and nobody supervising it. The next run then
finds the buses held.

`demo_stack.py` now warns when it is started over SSH outside tmux or screen and
names the command to use. It is a warning rather than a refusal: an operator on a
wired link with a monitor beside them does not need it.

**Run the demos inside tmux when working over SSH:**

```bash
tmux new -A -s demo
./scripts/start_tea_demo.py
# detach with ctrl-b d; the run survives the disconnect
tmux attach -t demo
```

## Where it stands

Done: the ROS graph is confined to loopback (§6), and every power-saving knob has
a script that turns it off (§4) — **that script still has to be run**, it is not
applied yet.

Open, in the order they are worth doing:

1. run `sudo ./scripts/jetson_performance_mode.sh --install`
2. rename the host away from `ubuntu`, so `.local` resolves to this machine and
   not to whichever stock Ubuntu box booted first (§3)
3. add the AP profile, from a wired session, at a lower autoconnect priority than
   the two client profiles (§1) — this is what makes the unit independent of a
   room's network
4. switch the boot target once nobody needs the desktop (§5)

Deliberately not done: moving the ROS sourcing for non-interactive SSH (§2), and
anything that would put the ROS graph back on the network.
