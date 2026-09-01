# Running the Jetson headless over SSH

status: EVALUATION
last_updated: 2026-09-01
scope: what is in place for operating the unit with no monitor, and what is missing

Measured on the unit 2026-09-01, read-only. Nothing here was changed.

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

### 2. `ssh <host> '<command>'` has no ROS environment

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

Two ways out, and they are not equivalent:

- move the three `source` lines above the interactivity guard, or into a file a
  non-interactive shell reads. Fixes every remote command at once, and changes
  the environment of every non-interactive shell on the machine.
- have the demo scripts source the workspace themselves. Narrower, but each
  script then has to know where the workspace is, and `ros2 launch` inside it
  still needs the environment.

### 3. The hostname is `ubuntu`

mDNS therefore advertises `ubuntu.local`, which collides with every other stock
Ubuntu machine on the same network — the normal situation at a demo, and the
case where a name instead of an IP matters most.

### 4. WiFi power save is on

`iw dev wlP1p1s0 get power_save` reports `Power save: on`, and the connection
leaves `802-11-wireless.powersave` at `default`. This is the usual cause of a
laggy or dropping SSH session over WiFi on this platform. It costs power to turn
off, which is a trade rather than an obvious fix.

### 5. The unit boots to `graphical.target`

Headless that still works — SSH does not depend on it — but it runs a desktop
session nobody looks at. This repository already treats CPU as a budget on this
machine: `use_rviz:=false` is documented as "a CPU decision, not cosmetic".
`multi-user.target` would return that budget, at the cost of needing a target
change before anyone plugs a monitor in again.

### 6. No `ROS_DOMAIN_ID`, and discovery is multicast over WiFi

`ROS_LOCALHOST_ONLY=0`, no domain id, no DDS profile. Two machines on one AP
discover each other, which is what a laptop running RViz wants — and so does any
other ROS 2 machine that joins the same AP. Worth a deliberate decision before a
demo where several laptops are present, not automatically a defect.

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

## Suggested order

1. rename the host and pin an address or a name you can rely on
2. move the ROS sourcing so remote commands work
3. turn WiFi power save off and measure whether the session steadies
4. add the AP profile, from a wired session, at a lower autoconnect priority
5. decide the domain id
6. switch the boot target once nobody needs the desktop

Items 1-3 are what make headless *comfortable*; item 4 is what makes it
*independent of a room's network*. Nothing above has been changed on the unit.
