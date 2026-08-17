# Sprint 6 — Minimal Dynamic Payload Adjustment Proposal

> **Implemented 2026-08-17, with one correction.** §1–§9 and §11 are built as
> written. The payload CoM offset is **`[0.15, 0.0, 0.0]`**, not the
> `[0.0, 0.0, +0.15]` this document specifies below: the OmniHand mounts through
> a rotated flange joint and reaches along the tool0 frame's **+x**, so a
> z-offset would place the mass 0.15 m sideways off the wrist. The `[0,0,0.15]`
> reading is **superseded (2026-08-17)**; the FK evidence and the resulting
> torque error are in `../reference/payload_gravity_model.md`. The `+0.15 m from
> the flange` *magnitude* is unchanged, and remains an unmeasured estimate until
> the §10 L3 static check runs.
>
> Two further deviations, both narrower than the text: the service parameter is
> `payload_com_xyz` (read in whatever frame `payload_parent_link` names, resolved
> from the URDF) rather than `payload_com_xyz_flange`; and the payload launch
> arguments are forwarded through the production `moveit_mit` bringup only, not
> the RViz debug launches.

## Goal

Add the minimum functionality required for the first `tea_pour_left_v1` demo so
the MIT gravity feedforward knows when the left arm is carrying the teapot.

Required physical model:

```text
payload mass:        1.0 kg
payload CoM:         [0.0, 0.0, +0.15] m from the arm flange frame
payload shape:       cylinder approximation, r=0.06 m, h=0.15 m
```

For the current controller only mass and CoM affect gravity compensation. The
cylinder inertia should nevertheless be inserted into the derived URDF so the
payload description is physically complete:

```text
Ixx = Iyy = 0.002775 kg m²
Izz         = 0.001800 kg m²
```

The implementation must stay narrow:

- no general object-state manager;
- no new dynamics controller;
- no runtime URDF rebuild on every action;
- no inference from gesture/preset names;
- no payload state in DeviceAuthority;
- no modification of the normal FJT / MIT command path.

The existing Pinocchio gravity path remains the single source of gravity torque.

---

# 1. Important existing semantics

The current hand mapping is:

```text
pre_grip_handle
    -> can_pre_grip

grip_handle
    -> can_grip_V01

release_handle
    -> can_pre_grip
```

Therefore:

> **A hand preset must never implicitly modify the arm payload.**

`can_pre_grip` is used both before grasping and for releasing. Only one of those
uses means "the teapot is no longer attached".

Payload state must instead be an explicit **action-level side effect** in the
coordination catalogue.

---

# 2. Minimal action metadata contract

Add one optional metadata field:

```yaml
payload_update: attach | detach
```

Semantics:

```text
field absent
    -> do not touch the current payload state

payload_update: attach
    -> same-side arm switches to its configured payload gravity model

payload_update: detach
    -> same-side arm switches back to the unloaded/base gravity model
```

This one field is both the explicit trigger and the requested state transition.
Do not derive a payload transition from:

- `skill_name`;
- `target_preset`;
- `can_grip_V01`;
- `can_pre_grip`;
- generic hand open/close state.

That keeps the physical task semantics in the activity/action layer rather than
inside a reusable hand gesture.

---

# 3. Apply it to `tea_pour_left_v1`

## 3.1 Pre-grip — no payload change

Keep:

```yaml
left_hand_pre_grip_handle:
  actiontype_id: Gripper
  robot_id: left_hand
  metadata:
    skill_name: pre_grip_handle
    ...
```

There is deliberately **no** `payload_update`.

The hand moves to `can_pre_grip`, but the teapot is still on the table and not
carried.

---

## 3.2 Grip — attach configured teapot payload

Change:

```yaml
left_hand_grip_handle:
  actiontype_id: Gripper
  robot_id: left_hand
  metadata:
    skill_name: grip_handle
    payload_update: attach
    ...
```

The existing mapping remains:

```text
grip_handle -> can_grip_V01
```

After the successful hand action, the coordinator switches the **left arm**
gravity model to:

```text
Arm + OmniHand + 1.0 kg teapot
CoM = +0.15 m along the configured flange-frame Z axis
```

This update must complete before action 70 is marked completed, so action 80
(`left_arm_to_teapot_post_grip`) cannot start under the unloaded model.

---

## 3.3 Release — detach payload despite using `can_pre_grip`

Change:

```yaml
left_hand_release_handle:
  actiontype_id: Gripper
  robot_id: left_hand
  metadata:
    skill_name: release_handle
    payload_update: detach
    ...
```

The hand skill may continue to map to:

```text
release_handle -> can_pre_grip
```

That reuse is intentional.

After the successful release action, the coordinator switches the left arm back
to its unloaded/base gravity model before action 150 is marked completed and
before action 160 withdraws the arm.

---

# 4. MIT controller: preload two models, switch a reference

The current MIT controller already owns:

```text
gravity_model
-> compute_gravity(actual q, hand joint state)
-> scale/sign/calibration
-> MoveMITMsg.torque
```

Do not change that control path.

At startup construct two Pinocchio models:

```text
gravity_model_base
    = current generated gravity URDF
      (arm + existing OmniHand payload)

gravity_model_loaded
    = same gravity URDF
      + one fixed teapot payload link
```

Runtime state:

```python
payload_attached = False
gravity_model = gravity_model_base
```

Expose one small service:

```text
~/payload_attached
std_srvs/SetBool
```

Behavior:

```text
False -> gravity_model_base
True  -> gravity_model_loaded
```

The service only swaps the active model reference under the controller's existing
`state_lock`.

It must be:

- idempotent;
- bounded;
- non-motion-generating;
- successful only when the requested model exists.

No controller restart and no Pinocchio model construction should occur during
the action transition.

---

# 5. Reuse the existing gravity-URDF generation path

`gravity_launch_utils.py` already:

- expands the Duo xacro;
- selects the active arm slice;
- includes/folds the OmniHand payload;
- writes a temporary gravity-only URDF.

Extend that existing path rather than creating another description subsystem.

Add one small helper, conceptually:

```python
derive_fixed_payload_urdf(
    base_gravity_urdf_path,
    parent_link,
    mass_kg,
    com_xyz,
    inertia,
) -> str
```

It should:

1. parse the already generated gravity URDF;
2. append one fixed child link containing the payload inertial;
3. attach it to the configured arm flange link;
4. return a second temporary URDF path.

Suggested payload parameters:

```yaml
payload_mass_kg: 1.0
payload_com_xyz_flange: [0.0, 0.0, 0.15]
payload_cylinder_radius_m: 0.06
payload_cylinder_height_m: 0.15
```

Default parent link should derive from the already existing joint prefix:

```text
left_arm_nero_tool0
right_arm_nero_tool0
```

for the Duo model, with an explicit override parameter only if another gravity
URDF uses a different flange link.

Do not duplicate the Duo mount rotation. The loaded model is derived from the
same already-correct base gravity URDF, so its mount orientation, arm inertials,
and OmniHand model remain identical.

---

# 6. Coordinator: apply payload transition as a post-success action effect

The coordinator already has:

- catalogue action metadata;
- the completed child action ID;
- side information for hand actions;
- MIT-controller service clients per side;
- a bounded synchronous service-call pattern.

Add one `SetBool` client per MIT controller:

```text
/{side}_arm/mit_controller/payload_attached
```

and one narrow helper:

```python
_apply_payload_update(child) -> None
```

Pseudo-flow:

```text
hand child completes successfully
        |
        v
load action metadata
        |
        v
payload_update present?
   | no              | yes
   v                 v
no-op          attach / detach
                       |
                       v
             call same-side MIT service
                       |
              +--------+--------+
              |                 |
           success           failure
              |                 |
              v                 v
       continue completion   abort activity
```

### Required ordering

For a successful hand child:

```text
1. hand action reports success
2. inspect payload_update
3. if requested, update MIT payload model
4. only after successful update:
       mark graph node completed
5. resume/release any hand-window bookkeeping
6. scheduler may admit the next arm action
```

The exact placement of hand-window resume may follow the existing topology logic,
but the invariant is:

> **No downstream arm action may start until the requested payload transition has
> succeeded.**

A payload-update service failure is an activity failure, not a warning. Running
the lift with the wrong gravity model is worse than stopping the demo.

---

# 7. Why the update belongs to the coordinator, not the hand controller

Do not make `omnihand_skill_controller` call an arm service.

The hand controller owns:

```text
hand motion / grasp execution
```

The coordinator owns:

```text
task semantics and cross-device consequences of a successful action
```

"After this grasp, the arm now carries a 1 kg teapot" is a task-level fact.

This also preserves reuse:

```text
can_pre_grip
    -> used before grasp: no payload change
    -> used for release: detach payload

can_grip_V01
    -> could theoretically be reused elsewhere without automatically attaching
       a teapot
```

---

# 8. Failure and restart semantics for the MVP

Keep the state deliberately simple.

## Process start

```text
payload_attached = False
```

The first `tea_pour_left_v1` activity starts with the teapot on the table, so this
matches the demo's physical precondition.

## Activity failure after attach

Do **not** automatically clear the payload.

If the activity aborts while the hand is still holding the teapot, retaining the
loaded gravity model is the physically safer approximation.

Payload state changes only on an explicit:

```text
payload_update: attach
payload_update: detach
```

or an operator/service command.

## MIT-controller restart during an active grasp

Do not add persistence for Sprint 6.

A controller restart invalidates the running activity/control context anyway.
Before motion is resumed, the operator must re-establish the physically correct
payload state through the same service.

Document this limitation; do not introduce a payload database merely to survive
a process restart.

---

# 9. No torque blending in the first implementation

The payload switch changes gravity feedforward discretely.

For this demo both transitions occur while the arm is effectively stationary:

```text
attach:
    final handle seat
    -> hand closes
    -> payload model attaches
    -> lift begins

detach:
    teapot is already placed on table
    -> hand releases
    -> payload model detaches
    -> arm withdraws
```

Therefore implement the direct model switch first.

Only if hardware shows a noticeable impulse should a later tiny blend be added:

```text
tau_g =
    tau_base
    + alpha * (tau_loaded - tau_base)
```

with `alpha` ramped over a short bounded interval.

Do not add that complexity prophylactically.

---

# 10. Tests

## L1 — model correctness

At several joint configurations verify:

```text
base model
loaded model
loaded - base
```

Requirements:

- both return exactly seven arm torques;
- adding the fixed payload adds no DoF;
- detach returns exactly to the original base-model output;
- loaded model includes the existing OmniHand contribution rather than replacing
  it.

---

## L1 — metadata semantics

Test all three cases:

```text
left_hand_pre_grip_handle
    payload_update absent
    -> no service call

left_hand_grip_handle
    payload_update=attach
    -> SetBool(True)

left_hand_release_handle
    payload_update=detach
    -> SetBool(False)
```

This explicitly proves that the shared `can_pre_grip` preset does not determine
payload state.

---

## L2 — coordinator ordering

Mock a successful grip.

Require:

```text
hand result success
-> payload service success
-> action 70 completed
-> action 80 becomes runnable
```

If the payload service fails:

```text
action 70 is NOT completed
action 80 is NOT dispatched
activity fails
```

Repeat equivalently for release/action 150 before action 160.

---

## L3 — static payload check

With the real left arm:

1. hold the arm in the grip pose without teapot;
2. record `gravity_feedforward`;
3. attach the ~1 kg teapot;
4. issue `payload_attached=true`;
5. confirm feedforward changes in the expected direction and the arm does not sag;
6. set the teapot down and release;
7. issue `payload_attached=false`;
8. confirm feedforward returns to the base-model value.

Use conservative poses before running the full replay.

---

## L3 — first `tea_pour_left_v1`

Run the existing graph unchanged except for the two action metadata flags.

Expected payload state:

```text
10..60     base
70 success loaded
80..140    loaded
150 success base
160..170   base
```

Specifically:

```text
30 left_hand_pre_grip_handle
   -> can_pre_grip
   -> NO payload update

70 left_hand_grip_handle
   -> can_grip_V01
   -> ATTACH

150 left_hand_release_handle
    -> can_pre_grip
    -> DETACH
```

Acceptance:

- no visible sag after lift;
- no excessive jump at attach/detach;
- no torque-limit rejection;
- payload service transitions exactly twice;
- no downstream arm action executes before its required payload transition;
- full pour replay completes with loaded gravity compensation active.

---

# 11. Files to touch

Keep the implementation surface small:

```text
src/agx_arm_mit_controller/agx_arm_mit_controller/gravity_launch_utils.py
    derive loaded gravity URDF from the existing resolved URDF

src/agx_arm_mit_controller/agx_arm_mit_controller/mit_controller_node.py
    preload base/loaded models
    SetBool payload service
    atomic active-model switch

src/agx_arm_coordination/agx_arm_coordination/coordinator_node.py
    payload service clients
    payload_update post-success hook

src/agx_arm_coordination/config/catalogue.d/tea_pour_left_v1.yaml
    attach flag on left_hand_grip_handle
    detach flag on left_hand_release_handle
```

Potential launch/config parameters may require the corresponding MIT controller
launch/profile YAML to forward the four payload geometry parameters. Do not
modify the activity DAG itself; its existing order already provides the required
grip -> carry -> place -> release boundaries.

---

# 12. Non-goals

Not part of this Sprint-6 patch:

- multiple simultaneously selectable payload profiles;
- automatic object recognition;
- tactile inference of payload attachment;
- persistence across controller restart;
- MoveIt PlanningScene attached-object integration;
- inertia/Coriolis feedforward;
- full inverse dynamics;
- payload-aware trajectory retiming;
- generalized post-action effect framework;
- cross-arm payload transfer.

If one of those becomes necessary later, the action-level `payload_update`
boundary can be generalized without changing the current demo semantics.

---

# Final target

The intended first-demo behavior is deliberately simple:

```text
startup
    payload = base

left_hand_pre_grip_handle / can_pre_grip
    payload unchanged

left_hand_grip_handle / can_grip_V01 succeeds
    payload = teapot_1kg

all carry + pour + return actions
    Pinocchio uses loaded model

left_hand_release_handle / can_pre_grip succeeds
    payload = base

withdraw
    Pinocchio uses base model
```

This adds the missing physical state transition without coupling gravity
compensation to vendor gestures or introducing a new runtime subsystem.
