# Script Refactor Proposal

## Goal

Refactor the current demo scripts so that ROS stack lifecycle and activity execution are clearly separated.

The intended operator flow is:

```text
start_demo_stack.py --top|--bottom
        ↓
components ready
        ↓
coordination ready
        ↓
READY FOR ACTIVITIES
        ↓
./unpack_*.py / ./wave.py / ./pack_*.py / ...
        ↓
stop_demo_stack.py --top|--bottom
```

### QUESTION:
Do we need a --top/ --bottom flag for `stop_demo_stack.py`?

The stack must stay alive across multiple activities and must be safe to operate over SSH/tmux.

---

## 1. Split stack lifecycle from activity execution

Refactor `demo_stack.py`.

Do **not** let `pack_*`, `unpack_*`, `wave`, etc. start or stop ROS launches anymore.

Activity scripts should only:

1. verify that the required stack is already ready;
2. execute the existing `ros2 run agx_arm_coordination run_activity --activity ...`;
3. forward optional arguments such as `--from-id`;
4. propagate Ctrl+C to `run_activity`;
5. return the actual activity exit code.

Keep the existing activity definitions and resume logic where useful.

---

## 2. Add one shared stack supervisor

Create:

```text
start_demo_stack.py --top
start_demo_stack.py --bottom
```

Prefer one implementation with a target-specific configuration instead of duplicating lifecycle code.

Example internal structure:

```python
STACKS = {
    "top": StackSpec(...),
    "bottom": StackSpec(...),
}
```

### Top stack

Use the existing top-unit component configuration with:

```text
mode:=moveit_mit
execution_profile:=duo_hand
follow:=true
planning_pipelines:=ompl
```

For pack/unpack/wave, no OmniHand SDK backend is required. Do not add `omnihand_backend_type:=sdk` unless a later activity explicitly requires it.

### Bottom stack

Use the validated bottom-unit execution profile instead of `duo_hand`.

Use the existing configuration appropriate for that unit:

```text
duo_arm
```
-> we are using right now

or, when the installed grippers must be represented/controlled:

```text
duo_gripper
```
-> maybe later, mark what we need to change to use this profile

Do not silently reuse the top-unit profile.

Keep the Top/Bottom profile selection explicit in the stack configuration so it cannot drift between scripts.

---

## 3. Enforce sequential bring-up

Do not start Components and Coordination back-to-back.

Required sequence:

```text
1. start agx_arm_ctrl/start_agx_arm_components.launch.py
2. wait for COMPONENT readiness
3. start agx_arm_coordination/start_coordination.launch.py
4. wait for COORDINATION readiness
5. print READY FOR ACTIVITIES
6. remain alive as stack supervisor
```

### Component readiness

Check at least the ROS surfaces needed by the respective unit, e.g.:

```text
/left_arm/feedback/joint_states
/right_arm/feedback/joint_states
/left_arm/emergency_stop
/right_arm/emergency_stop
/unit_safety/rearm
/move_action
```

Add gripper action servers only for a `duo_gripper` bottom stack.

### Coordination readiness

Wait separately for:

```text
/execute_activity
```

Do not use one combined readiness check after both launches have already started.

Fail clearly if either launch exits during startup.

---

## 4. Keep the supervisor alive

`start_demo_stack.py` must remain running after readiness.

It owns the launched process groups and is responsible for their cleanup.

Recommended use:

```bash
tmux new -A -s demo_top
./start_demo_stack.py --top
```

and analogously for bottom.

Print a clear final state:

```text
TOP DEMO STACK READY
```

or

```text
BOTTOM DEMO STACK READY
```

Do not exit after readiness.

---

## 5. Add explicit stack shutdown

Create:

```text
stop_demo_stack.py --top
stop_demo_stack.py --bottom
```

The stop command should signal the corresponding running supervisor instead of independently searching and killing arbitrary ROS processes.
-> Do we really need --top/--bottom?

Recommended implementation:

- supervisor writes PID/state file under `/run/user/<uid>/agx_demo_stack/`;
- include target (`top` / `bottom`) and child PIDs/process groups;
- refuse a second supervisor for the same target;
- `stop_demo_stack.py` requests orderly supervisor shutdown.

Shutdown order:

```text
1. stop Coordination with SIGINT
2. wait for clean exit
3. stop Components with SIGINT
4. wait for clean exit
5. SIGTERM/SIGKILL only as timeout fallback
6. remove PID/state file
```

Do not use broad `pkill ros2` / `killall` commands.

---

## 6. Ctrl+C semantics for activity scripts

Preserve the existing `run_activity` cancellation semantics.

When an activity is running:

```text
Ctrl+C
   ↓
SIGINT reaches run_activity
   ↓
activity cancel / safety unwind
   ↓
run_activity exits
   ↓
activity wrapper returns
```
--> can we really use ctrl+c over ssh in a tmux shell?

The wrapper must **not consume SIGINT before the child receives it**.

Do not tear down Components or Coordination while `run_activity` is still cancelling.

For the persistent-stack workflow, a cancelled activity should stop the motion but leave the stack available unless an explicit stack stop is requested.

If a presentation-specific "Ctrl+C also shuts down the complete stack" mode is still desired, implement it only after `run_activity` has fully returned, preferably behind an explicit flag such as:

```text
--stop-stack-on-cancel
```

Do not make launch teardown race against activity cancellation.

---

## 7. Simplify the individual activity scripts

Files such as:

```text
pack_top_unit.py
unpack_top_unit.py
pack_bottom_unit.py
unpack_bottom_unit.py
wave.py
```

should become thin activity launchers.

They should define only:

- required target: `top` or `bottom`;
- activity name;
- optional speed/activity variant;
- description;
- optional resume arguments.

Before executing, verify that the matching stack supervisor exists and `/execute_activity` is available.

If the wrong or no stack is running, fail immediately with a useful message, e.g.:

```text
BOTTOM demo stack is not ready.
Start it first with:
  ./start_demo_stack.py --bottom
```

---

## 8. Logging

Use persistent per-run logs instead of temporary directories.

Suggested layout:

```text
logs/demo_stack/
  top_<timestamp>/
    components.log
    coordination.log
  bottom_<timestamp>/
    components.log
    coordination.log
```

Print the log directory during startup.

Activity output should remain attached to the operator terminal/tmux pane.

---

## 9. Acceptance test

Validate both units independently.

### Top

```bash
./start_demo_stack.py --top
./unpack_top_unit.py
./wave.py
./pack_top_unit.py
./stop_demo_stack.py --top
```

Verify:

- Components start before Coordination.
- Stack stays alive between activities.
- No OmniHand SDK backend is unnecessarily started.
- Ctrl+C during an activity cancels motion cleanly.
- A later activity can still be started after a normal completion/cancel.
- Explicit stop cleanly terminates Coordination and then Components.

### Bottom

Equivalent test with:

```bash
./start_demo_stack.py --bottom
...
./stop_demo_stack.py --bottom
```

Verify that the correct bottom execution profile (`duo_arm` / `duo_gripper` as required by the installed setup) is used and that no top-specific `duo_hand` configuration leaks into it.

---

## Scope

Keep this refactor small.

Do not redesign the ROS coordinator, activity model, safety state machine, or resume mechanism.

The task is only to make demo process ownership and lifecycle deterministic:

```text
one persistent stack supervisor
+
many short-lived activity clients
+
one explicit orderly shutdown path
```
