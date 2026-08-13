---
name: test-ladder
description: Choose and run the right validation level for a change in agx_arm_ros — unit, mock integration, or hardware end-to-end — and state the level the evidence came from. Consult before validating any change and before claiming something works.
---

# Test Ladder

## Why this exists

This repository can lie to itself in one specific way: a change is exercised
against mocks, reported as "validated", and the CAN, timing, or motion behaviour
it actually affects is never touched. `AGENTS.md` requires the gap to be stated
explicitly. This skill makes that mechanical.

The platform decides what is **possible**. The ladder decides what is
**required**.

## The three levels

| Level | What it exercises | Where it runs | Required when |
| --- | --- | --- | --- |
| **L1 unit** | pure logic, no ROS spin, no I/O | any platform | every commit |
| **L2 mock integration** | a real ROS graph on mock backends, no hardware | any platform; the primary gate off-`aarch64` | before any hardware run |
| **L3 hardware end-to-end** | real arms, hands, CAN, real timing | `aarch64` with granted hardware access | every phase gate; any CAN/timing/motion claim |

Rules:

- L1 and L2 must pass **before** hardware is touched, on every platform.
- L2 is **never** a substitute for L3 when the claim is about CAN behaviour,
  loop timing, CPU load, or physical motion.
- L3 is preferred over L2 for acceptance evidence once hardware access is
  granted on this platform.
- A result that could not reach the level its claim needs says so, in the commit
  message and in the doc entry.
- A measurement whose method can fail silently is not evidence. `ros2 topic hz`
  cannot answer "did it stop publishing?" — its buffered output loses the last
  seconds when killed. Use `scripts/count_topic_messages.py`, which counts over
  a fixed window and prints once. This is not hypothetical: it produced two
  confident zeros during Phase 1A, one supporting a wrong conclusion.

## Choosing the level

Start from what the change can break, not from what is convenient to run.

- pure function, parsing, routing, limit maths, state-machine transitions → **L1**
- node wiring, topic and service names, action contracts, ordering between
  nodes, launch composition, config resolution → **L2**
- anything touching a CAN interface, a control rate, a stop path, a real device,
  or a CPU budget → **L3**

A change often needs more than one level. Adding an epoch field to a command is
L1 for the validation logic, L2 for the fact that the driver and the controller
still agree, and L3 for whether a stale command is actually rejected on the bus.

## Running each level

### L1 — package unit tests

From a system-Python ROS shell, not Conda:

```bash
colcon test --packages-select <pkg>
colcon test-result --verbose --test-result-base build/<pkg>
```

Or directly while iterating:

```bash
python3 -m pytest src/<pkg>/test/<file>.py -q
```

### L2 — mock integration

The harness lives in `src/agx_arm_coordination/test/test_l2_activity_integration.py`
and brings up a real ROS graph: both hand bridges on the **mock** backend, both
skill controllers, an arm test double (`l2_arm_double.py` — the production arm
driver has no mock backend), and the real coordinator running
`hands_open_release_v1`.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/agx_arm_coordination/test/test_l2_activity_integration.py -q
```

It needs the workspace overlay installed; it skips itself when `ROS_DISTRO` is
unset. Build first if the change touched a package:

```bash
bash ./scripts/colcon_build_system_python.sh --packages-select <pkg>
```

### L3 — hardware end-to-end

**Ask before every hardware-touching action.** Hardware access is not implied by
being on the Jetson, and is granted per session.

Once granted:

1. bring up the interfaces — arms on native CAN, hands on their own USB-CAN FD
   adapters (one bus per device);
2. run the bringup for the profile under test;
3. run `tea_pour_left_v1` as the end-to-end regression benchmark;
4. record per-interface CAN counters (`ip -s -d link show`) and CPU
   (`tegrastats`, `pidstat`) alongside the functional result.

Failure to reach L3 is a reportable state, not a silent omission.

## Reporting the level

Every claim carries its level. In a commit message, one clause is enough:

- ✅ "Validation: L2 — the ordering is pinned by the mock harness; the CAN effect is unmeasured until the hardware baseline runs."
- ✅ "Validation: L3 on the left side — 20 cycles, no RX drops on either interface."
- ❌ "Validation: tested, works."
- ❌ "Validation: all tests pass." (which tests, at which level, proving what?)

In documentation, an entry that records a measurement names the level, the date,
and the scenario. An entry that could not be measured says so rather than
implying it was.

## Before you commit

Follow [`.claude/skills/commit-quality/SKILL.md`](../commit-quality/SKILL.md)
(skill `commit-quality`). The level this skill produced is exactly the evidence
clause that skill asks for.
