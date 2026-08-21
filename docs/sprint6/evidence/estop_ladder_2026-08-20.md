# Stop ladder on the wire — two runs, 2026-08-20

The first hardware evidence that the arm emergency stop pins a **moving** arm,
and the first evidence of any kind about which frames it does and does not put
on the bus. Closes the sprint-6 item *verify the stop ladder on hardware,
mid-replay*; the mid-hand-window half stays open.

Contract and reasoning: `docs/sprint_refactor/reference/emergency_stop_ladder.md`.

## What was run

`scripts/l3_estop_pcap_run.py` on the four-bus stack — both arms native, both
hands on USB-CAN FD, `bus_topology=dedicated_per_device`. Hands on the **mock
backend**, deliberately: the evidence lives entirely on the arm buses, and a
mock hand means the arm walks the whole `tea_pour_left_v1` graph with no object
to drop.

The script captures both arm buses, waits for a named graph node to start, fires
`/left_arm/emergency_stop` and `/right_arm/emergency_stop` five seconds into it,
and keeps recording.

| run | trigger node | what the left arm was doing |
| --- | --- | --- |
| 1 | 160 `left_arm_teapot_handle_release` | recorded trajectory replay, hand empty |
| 2 | 110 `left_arm_pour_tea` | recorded trajectory replay, payload at height |

## Result

Both arms answered `stop=verified` in both runs. All four captures verdict
**clean**.

| run | arm | settle peak | hold on the wire | electronic stop |
| --- | --- | --- | --- | --- |
| 1 | left | 0.034 rad/s | +10.6 ms | none |
| 1 | right (idle) | 0.048 rad/s | +8.1 ms | none |
| 2 | left | 0.021 rad/s | +12.9 ms | none |
| 2 | right (idle) | 0.000 rad/s | +7.6 ms | none |

"Hold on the wire" is the delay from the service call to the `0x151` mode frame
carrying MOVE-J with MIT off; the joint-position payload follows within a further
1–3 ms. The idle right arm is pinned by the same ladder without ever having been
in MIT.

The rest of the stack unwound as designed: the unit stop generation was
allocated once and the second request was idempotent against it, both hand
bridges held their measured pose, both MIT controllers went FAULTED, a post-stop
`move_mit` was refused as `not_ready`, and MoveIt aborted with
`INVALID_GOAL: device authority changed`.

## What these captures are

**Trimmed to ±2 s around the stop** by `scripts/trim_pcap_window.py`, from full
captures of 51–83 s. The full files were 31 MB and are not in git; they stayed
in the run's `logs/estop_*/` directory on the Jetson. The window keeps every
frame the verdict depends on at 1.8 MB.

One consequence to read correctly: `analyze_can_pcap.py` defaults to a 5 s
window, so its "MIT command frames in the 5s before the stop" line reports what
the trimmed file holds (~1400) rather than what the bus carried (~3500). The
frame *rate* before the stop is unchanged.

```text
test_run_estop/     run 1 — can_nero_{left,right}.pcap, run.json, terminals.md
test_run_estop_2/   run 2 — same layout
```

`run.json` carries the stop timestamp, both service responses and the full
activity feedback log. `terminals.md` is the operator's terminal record.

## Re-verifying

```bash
python3 scripts/analyze_can_pcap.py --stop-at $(python3 -c \
  "import json;print(json.load(open('docs/sprint6/evidence/test_run_estop/run.json'))['stop_unix_ts'])") \
  docs/sprint6/evidence/test_run_estop/can_nero_*.pcap
```

Exit status is 0 when every capture is clean, 3 when one is not.

## What this does not cover

- **The retry ladder.** Both stops verified on the first attempt, so
  `ESTOP_HOLD_ATTEMPTS` and the `no_hold_commanded` outcome — the two things
  that replaced the removed vendor rungs — still have unit-level evidence only.
  Needs a deliberately provoked unverified stop.
- **Mid-hand-window.** No hand window exists on this topology. That case belongs
  to the degraded `shared_per_side` mode.
- **Coordinator-crash containment.** A commanded stop and an abrupt process
  death are different failure modes.
