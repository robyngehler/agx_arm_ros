# Carried-payload gravity model (2026-08-17)

How a picked-up object enters the MIT controller's gravity feedforward, and the
URDF measurement that fixed the payload offset's axis.

## The mechanism

Two Pinocchio models are built at startup and one reference is switched:

```text
gravity_model_base     the generated gravity URDF (arm + body mount + OmniHand)
gravity_model_loaded   the same URDF + one fixed payload link on the flange
```

`~/payload_attached` (`std_srvs/SetBool`) points `gravity_model` at one of them
under the controller's `state_lock`. Nothing is built, planned or moved at the
transition; `compute_gravity` reads whichever model is current on its next call.

The loaded model adds a **fixed** link, so it has the same joints, the same q
layout and the same articulated hand-joint names as the base. The controller
refuses a loaded model whose joint set differs — a model swap may change the mass
the arm compensates for, never which joints it compensates.

Attaching is refused when no loaded model exists (`payload_mass_kg: 0.0`, or a
failed derivation). The service never answers "applied" without applying: the
coordinator treats success as proof the lift may proceed.

## The payload offset is along the flange's +x, not +z

The Sprint-6 proposal specified `payload_com_xyz_flange: [0.0, 0.0, 0.15]` —
0.15 m along the flange frame's z. **That axis is wrong for this assembly**, and
the shipped default is `[0.15, 0.0, 0.0]`.

Measured by FK on the generated left-arm gravity URDF
(`duo_system.urdf.xacro`, `use_left_arm`/`use_left_hand`), positions expressed in
the `left_arm_nero_tool0` frame at q = 0:

| frame | x | y | z |
| --- | --- | --- | --- |
| `left_arm_nero_tool0` (= `left_arm_link7`) | 0.000 | 0.000 | 0.000 |
| `left_arm_omnihand_flange` | 0.032 | 0.000 | −0.024 |
| `left_palm` | 0.044 | 0.000 | −0.024 |
| `left_index_tip` | 0.255 | −0.013 | −0.054 |
| `left_middle_tip` | 0.265 | −0.013 | −0.030 |
| `left_ring_tip` | 0.255 | −0.012 | −0.010 |
| `left_pinky_tip` | 0.248 | −0.012 | 0.010 |
| `left_thumb_tip` | 0.146 | 0.135 | −0.115 |

The hand mounts through `left_arm_omnihand_flange`, whose origin carries
`rpy="-1.5708 0 -1.5708"` relative to `link7`. The consequence is that the hand
reaches along **tool0's +x**: the palm is at x = 0.044 and the fingertips at
x ≈ 0.25. A grasped object therefore sits between them, around x ≈ 0.15 — which
is where `[0.15, 0, 0]` puts it.

`[0, 0, 0.15]` would place the 1 kg mass 0.15 m sideways off the wrist, 0.212 m
away from the intended point and nowhere near the grasp. Sampling 400 random
poses in ±1.5 rad, the two placements disagree by up to **2.08 N·m** on joint 1,
against a `torque_limit` of 8.0 N·m.

The deferred `planning/gravity_payload_api_plan.md` already warned about exactly
this ("verify the frame axis direction via FK on the generated URDF, do not
guess"). The parameter is named `payload_com_xyz` rather than
`payload_com_xyz_flange` because it is read in whatever frame
`payload_parent_link` names, and the parent link is resolved from the URDF.

## Mass and lever are placeholders

`payload_mass_kg: 1.0` and the 0.15 m lever are the proposal's figures for the
teapot. Neither has been measured on hardware. The axis above is a fact from the
description; the magnitude is an estimate, and the L3 static payload check is
what turns it into a measurement.

The cylinder inertia (r = 0.06 m, h = 0.15 m, giving
Ixx = Iyy = 2.775·10⁻³, Izz = 1.8·10⁻³ kg·m²) does not affect gravity at all —
`computeGeneralizedGravity` is RNEA at zero velocity and acceleration, so g(q)
depends only on mass and CoM. It is written into the URDF so the payload
description stays usable for a later dynamics feedforward.

## Validation level

L1 only. Model derivation, the no-new-DoF property, exact detach, service
refusals and coordinator ordering are unit-tested; the derivation and the
26-DoF-preserving property were also exercised against the real generated Duo
gravity URDF. **No hardware run.** The gravity effect on a live arm is the L3
static payload check in
`planning/decision_record.md` §4.
