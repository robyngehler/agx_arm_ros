# Plan: Runtime Gravity Payload API (deferred)

> **Superseded in part (2026-08-17).** The narrow slice this plan covers shipped
> as the Sprint-6 MVP instead: two preloaded Pinocchio models and a
> `std_srvs/SetBool` `~/payload_attached` switch, driven by an action-level
> `payload_update` flag in the catalogue. See
> `sprint6_dynamic_payload_adjustment_proposal.md` and
> `../reference/payload_gravity_model.md`.
>
> Still open and still wanted here: the general `SetGravityPayload.srv` with a
> runtime-settable mass/COM/tensor, `reference: flange|palm` resolution,
> per-action payload parameters, and clear-on-release tracking. The MVP hardcodes
> one payload per bringup and knows only attached/detached.
>
> The warning below — *verify the frame axis direction via FK on the generated
> URDF, do not guess* — was the one that mattered: the MVP proposal specified a
> flange **z** offset, and the hand actually reaches along the flange's **x**.

Status: **planned, not started** (2026-07-03). Follow-up to the hand-aware gravity work in
`debug_recordings.md` (P1'/P1''). Implement when the teach/transition debugging is done.

## Goal

A runtime API on the MIT controller that loads an additional payload (mass + COM + optional inertia
tensor) into the gravity model, updates it, and clears it — reference either the flange (no EE) or the
palm (OmniHand mounted), with sensible defaults, callable from a grip action **only when explicitly
flagged**.

## Mechanics

Add the payload as a `pin.Inertia` onto an existing link instead of regenerating the URDF:

```
payload = pin.Inertia(mass, lever, I_3x3)
model.inertias[joint_id] = base_inertia_snapshot + payload
```

- `joint_id` = parent joint of the resolved reference frame (flange frame / `<side>_palm` link),
  `lever` = frame placement x custom frame_offset x com, expressed in the joint frame.
- Base inertia snapshotted at load -> updates are idempotent, `mass = 0` restores the baseline exactly.
- `computeGeneralizedGravity` reads inertias per call: no model rebuild, no extra cost in the 50 Hz loop.
  A palm payload rides the wrist/arm kinematics automatically.
- Rotational inertia is **mathematically irrelevant for the gravity term** (RNEA at zero
  velocity/acceleration; g(q) depends only on mass + COM). It is carried anyway for forward
  compatibility: full dynamics feedforward (M(q)q'' + C + g) for heavier moved objects, and payload
  export to planning/time parametrization. Pinocchio accepts the tensor for free.

## Service (uses standard messages — geometry_msgs/Inertia carries exactly mass + com + 6 tensor terms)

```
# agx_arm_msgs/srv/SetGravityPayload.srv
string reference                  # "" = auto (palm if the model articulates hand joints, else flange)
                                  # | "flange" | "palm"
geometry_msgs/Pose frame_offset   # custom frame relative to the resolved reference frame
                                  # (identity default). Deliberately a static pose, NOT a TF frame id:
                                  # the 50 Hz gravity path must stay free of TF lookups; a thin client
                                  # layer can resolve TF and feed the resolved pose here later.
bool use_default_com              # true: use the reference default COM and ignore payload.com
                                  # (explicit flag instead of a NaN/zero sentinel — com=[0,0,0] is a
                                  # legitimate explicit value)
geometry_msgs/Inertia payload     # m = 0.0 clears; com + ixx..izz in the custom frame, tensor about COM
---
bool success
string message                    # resolved frame, effective COM in the joint frame, total mass
```

Defaults (parameters, verified against the URDF before hardcoding axis signs):

- `payload_default_flange_offset` = `[0, 0, 0.05]` — flange center + 5 cm along the positive tool axis
- `payload_default_palm_offset` ~ `[0, 0, 0.03]` — palm center + offset perpendicular to the palm plane
  toward finger curl. **Verify the palm frame axis direction via FK on the generated URDF, do not guess.**
- optional `payload_mass_kg` / `payload_reference` / `payload_com_offset` launch params for statically
  known payloads (e.g. a flange-mounted camera).

`reference: palm` on a hand-less bring-up (no `effector_type:=omnihand`) fails hard
(`success=false`, clear message) instead of silently falling back to flange.

## Controller integration

- Service `~/set_payload` on the MIT controller; application under `state_lock` (control loop reads the
  model under the same lock).
- Validation: mass >= 0, upper bound (~3 kg safety bound like `feedforward_model`), finite values,
  positive-semidefinite tensor with triangle inequality (a broken grip action must not inject
  unphysical inertia — harmless for gravity, not for later dynamics FF).
- Persistence: payload survives freedrive/enable transitions until explicitly cleared (the grip persists
  too).
- Duo: service is per controller instance (namespaced) -> per-side payloads work automatically.
- Log + diagnostics: active payload logged on set ("payload 0.55 kg @ right_palm + [0,0,0.03]").

## Grip-action integration (phase 2, agx_arm_coordination) — opt-in only

The performer calls `~/set_payload` **only** when the grip action carries an explicit flag:

```yaml
# catalogue.yaml grip action (all fields optional except apply)
payload:
  apply: true               # without this flag the performer NEVER calls set_payload
  mass_kg: 0.55
  reference: palm           # ""|flange|palm
  frame_offset: {xyz: [...], rpy: [...]}   # optional, else identity
  com: [0.0, 0.0, 0.03]     # optional; omitted -> use_default_com: true
  inertia: [ixx, ixy, ixz, iyy, iyz, izz]  # optional; omitted -> zeros (point mass)
  clear_on_release: true    # default true when apply is set
```

- Release/abort/timeout paths clear the payload, but only if this action set it (the performer tracks
  "I set it" per resource so it never clears a manually-set payload).

## Optional later stage: dynamic grip_center

Centroid of the fingertip frames (`*_tip` fixed joints) via FK at the current finger pose, lever
refreshed at ~10 Hz. Small effect vs the static palm offset (few cm lever at <1 kg); decide after
hardware experience with phase 1.

## Validation plan

1. Unit tests (fake pin, like `test_gravity_model.py`): inertia composition, set/set/clear idempotence,
   tensor validation, custom-frame rotation of com + tensor.
2. Offline on the Jetson: 0.5 kg @ flange+5 cm vs analytic m*g*lever at 2-3 poses; clear restores the
   baseline bit-exactly; regression test that g(q) is identical with/without tensor; palm offset
   direction checked via FK.
3. Hardware: freedrive neutrality with a known weight (full bottle) in the fist, with and without the
   payload set; compare `~/gravity_feedforward` vs measured effort.

## Phases

- Phase 1: `agx_arm_msgs` srv + `gravity_model.set_payload/clear_payload` + `mit_controller_node`
  service/params + tests/docs.
- Phase 2: catalogue schema + performer call in `agx_arm_coordination` (opt-in flag, clear-on-release).
