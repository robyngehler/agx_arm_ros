# Hand Skill Backend Mapping

Design for the **OmniHand skill controller** (`agx_arm_ctrl`, `omnihand_skill_controller`, one per
side). It turns a *semantic* `skill_name` into vendor-SDK action + tactile-confirmed completion, and
holds/​releases according to policy. This is the "missing piece 1+2" from the sprint target.

**Hard rule:** the public layer (graph, catalogue, action metadata) carries only `skill_name`. The
`skill_name → vendor gesture / preset / joint sequence` mapping lives **here, inside the backend**,
and can change without touching any activity graph.

## 1. Inputs the controller already has

The OmniHand bridge (`omnihand_backend_type:=sdk`) already provides, per side:

- command: active 10-joint targets via the vendor `set_all_active_joint_angles(...)` path
- `feedback/omnihand/joint_states` — current finger joints
- `feedback/omnihand/status` — hand/finger status
- `feedback/omnihand/tactile_raw` — per-sensor tactile values (the grasp-confirmation signal)

The skill controller subscribes to the tactile + status streams and issues commands through the
same bridge surface (it does not open its own vendor SDK session).

## 2. Skill table (semantic → backend)

| `skill_name` | backend action | completion condition |
|---|---|---|
| `open_hand` | drive active joints to the open preset | all monitored joints within open tolerance |
| `grasp_glass_until_contact` | close toward the glass preset, slow | `contact_score ≥ contact_threshold` for `stable_samples`, then stop+hold |
| `grasp_bottle_until_contact` | close toward the bottle power-grasp preset, slow | as above (more sensors) |
| `release_glass` | open from hold to the release preset | joints reach open tolerance |
| `release_bottle` | open from hold to the release preset | joints reach open tolerance |
| `stop_hand` | freeze current joint targets | immediate |

The presets (target joint vectors / vendor gestures) are backend constants, **calibrated on
hardware**; only `skill_name` is public. If a vendor custom skill is added later, keep the same
`skill_name`.

## 3. Tactile-confirmed close (the core of grasp)

```
contact_score = aggregate(tactile_raw[s] for s in contact_sensors)   # e.g. normalized mean/max
loop at control rate while CLOSING_UNTIL_CONTACT:
    if tactile stale or hand error      -> FAILED (fault)
    step joints toward the grasp preset (bounded step, optional current/force cap)
    if contact_score >= contact_threshold:
        if held for >= stable_samples   -> stop motion, GRASP_HOLDING (success)
    if elapsed > timeout_sec            -> FAILED (timeout)
```

- `contact_sensors`, `contact_threshold`, `stable_samples`, `timeout_sec` come from the action
  metadata (per object).
- `stable_samples` debounces transient touches.
- a current/force cap is an optional extra safety stop.

## 4. State machine

```
IDLE
 ├─ open_hand            → OPENING            → (open reached) → IDLE
 ├─ grasp_*_until_contact→ CLOSING_UNTIL_CONTACT
 │                          ├─ contact stable → GRASP_HOLDING   (success; hold internally)
 │                          └─ timeout/fault  → FAILED
 ├─ release_*            → RELEASING          → (open reached) → IDLE
 └─ stop_hand            → freeze             → IDLE
GRASP_HOLDING: keep the grasp targets, run passive contact monitoring, publish status
FAILED: structured failure result; coordinator aborts + cancels children
```

## 5. completion_policy / fallback_policy (behavior, not commands)

Interpreted by the controller; never carry vendor commands:

| key | values (examples) | meaning |
|---|---|---|
| `completion_policy.on_success` | `hold_internal`, `finish_when_open` | after success: keep holding vs finish |
| `completion_policy.passive_contact_monitoring` | `true`/`false` | watch for slip while holding |
| `fallback_policy.on_cancel` | `stop_and_hold`, `stop_motion` | what cancel does |
| `fallback_policy.on_timeout` | `stop_and_hold`, `report_failure` | what timeout does |
| `fallback_policy.on_contact_loss` | `abort_activity`, `warn` | slip while holding |

**Hold is internal**: after a confirmed grasp the controller stays in `GRASP_HOLDING` and publishes
status; the coordinator does not model hold as a long-running action (so it does not block arm + hand
resources on the same side bus). Releases are normal `Gripper` actions in the graph; cancel/emergency
behavior comes from `fallback_policy`, not from extra `hold_*` graph nodes.

## 6. Passive slip monitoring while holding

In `GRASP_HOLDING`, keep computing `contact_score`:

- drop below a **warn** threshold → publish a warning event (`feedback/omnihand/*` / coordinator event)
- drop below a **critical** threshold with `on_contact_loss: abort_activity` → fail the activity

This is passive monitoring, not a long-running coordinator action (proposal §10.2).

## 7. Transport (MVP) — folded behind PerformAction

Per the architecture decision (§8), the MVP does **not** add a dedicated `HandSkill.action`. A
`Gripper` catalogue action is dispatched as a `PerformAction` whose metadata carries the skill
fields: `skill_name`, `contact_sensors[]`, `contact_threshold`, `stable_samples`, `timeout_sec`,
`completion_policy`, `fallback_policy`. The coordinator-internal performer routes
`Gripper + {left,right}_hand` to the matching `omnihand_skill_controller`, which returns a structured
result (`success`, `final_contact_score`, `final_state`) and feedback (`state`, `progress`,
`contact_score`). A typed `HandSkill.action` can be promoted later if the metadata contract grows.

## 8. Calibration to do on hardware (placeholders today)

- which `contact_sensors` are reliable per object (glass vs bottle)
- `contact_threshold` and `stable_samples` that are robust across repeated grasps
- the grasp presets (open / glass / bottle power-grasp / release joint vectors)
- optional current/force caps
- warn vs critical slip thresholds

Record measured values and the final presets in `hefeweizen_validation_log.md`.
