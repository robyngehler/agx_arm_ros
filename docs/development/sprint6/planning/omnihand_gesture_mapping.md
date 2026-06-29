# OmniHand Gesture Mapping and Skill Selection

**Status:** Sprint 6 discovery — gesture inventory and backend preset selection  
**Target:** Finalize gesture presets for the hand skill controller (`omnihand_skill_controller`)

> [!WARNING]
> **O10-era document — partially superseded by the OmniHand Pro (O12) migration.**
> The hardware we own and the live bridge default is now **`o12_pro` (12 active
> joints)**, not the O10 10-joint hand this inventory was written for. The 18
> vendor gestures and 10-element vectors below are **O10-only** and must not be
> reused for the Pro as-is — the Pro adds real `thumb_pip` and `*_mcp` curl joints
> and drops `ring_abad`/`pinky_abad`. See
> [proposal_omnihand_pro_migration.md](../../../assets/omnihand/proposal_omnihand_pro_migration.md)
> §5.4 (joint order) and §9 (skill-layer migration).
>
> Source of truth for live presets:
> - `o12_pro`: `src/agx_arm_ctrl/config/omnihand_pro_gestures.yaml` (12 joints;
>   currently only the `zero` + `fist_vendor_demo` vendor bootstrap — calibrated
>   `open`/grasp poses still pending hardware measurement)
> - `o10` (mock only): `src/agx_arm_ctrl/config/omnihand_gestures.yaml`
>
> Load them model-aware: `resolve_gesture_presets(side, get_hand_model("o12_pro"))`.
> A calibrated O12 grasp/skill table still needs to be produced on the Pro
> hardware (proposal §9.2) before the skill controller can use named grasp poses.

---

## 1. Vendor SDK Gesture Library

The OmniHand 2025 SDK provides **18 pre-recorded gestures** via hardcoded joint position vectors. These are stored as 10-element arrays corresponding to the hand's active joints:

```
[thumb_roll, thumb_abad, thumb_mcp, index_abad, index_pip, middle_pip, ring_abad, ring_pip, pinky_abad, pinky_pip]
```

### Complete Gesture Inventory

| ID | Gesture Name | English Description | Physical Meaning | Vendor Code |
|---|---|---|---|---|
| 0 | RESET | Rest/Open | All joints to zero (open hand) | `[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]` |
| 1 | PAPER | Paper/Hand Flat | Palm open, fingers extended | `[0.58, -0.21, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` |
| 2 | FIST1 | Fist (style 1) | Loose fist, all fingers flexed | `[0.43, -0.3, 0.66, 0.0, 1.48, 1.48, 0.0, 1.48, 0.0, 1.48]` |
| 3 | FIST2 | Fist (style 2) | Tight fist, all fingers fully flexed | `[0.5, -1.0, 0.75, 0.0, 1.48, 1.48, 0.0, 1.48, 0.0, 1.48]` |
| 4 | OK | OK Sign | Thumb-index loop, other fingers open | `[0.03, -1.51, 0.7, -0.16, 0.85, 0.21, 0.07, 0.153, 0.107, 0.1]` |
| 5 | ONE_HANDED_FINGER_HEART | One-handed Heart | Index and middle form heart shape | `[0.8, -0.4, 0.47, 0.0, 0.82, 1.48, 0.0, 1.48, 0.0, 1.48]` |
| 6 | LIKE | Thumbs Up | Thumb extended, other fingers curled | `[0.27, 0.0, 0.0, 0.0, 1.48, 1.48, 0.0, 1.48, 0.0, 1.48]` |
| 7 | ILY | I Love You Sign | Thumb, index, pinky extended | `[0.33, 0.0, 0.0, -0.1, 0.0, 1.48, 0.07, 1.48, 0.11, 0.0]` |
| 8 | NUM1 | Number 1 | Index extended, others curled | `[0.32, -1.12, 0.79, -0.06, 0.0, 1.48, 0.0, 1.48, 0.0, 1.48]` |
| 9 | NUM2 | Number 2 | Index and middle extended | `[0.48, -1.5, 0.79, -0.16, 0.0, 0.0, 0.0, 1.48, 0.0, 1.48]` |
| 10 | NUM3 | Number 3 | Index, middle, ring extended | `[0.64, -1.48, 0.81, -0.16, 0.0, 0.0, 0.09, 0.0, 0.09, 1.48]` |
| 11 | NUM4 | Number 4 | All fingers extended (salute) | `[0.64, -1.48, 0.81, -0.16, 0.0, 0.0, 0.07, 0.0, 0.15, 0.0]` |
| 12 | NUM6 | Number 6 | Thumb and pinky extended | `[0.40, 0.0, 0.0, 0.0, 1.48, 1.48, 0.05, 1.48, 0.17, 0.0]` |
| 13 | NUM8 | Number 8 | All fingers curled, thumb down | `[0.40, 0.0, 0.0, 0.0, 0.0, 1.48, 0.0, 1.48, 0.0, 1.48]` |
| 14 | HAND_HEART1 | Two-hand Heart (part 1) | Thumb and fingers curved | `[-0.03, -1.36, 0.0, 0.0, 0.65, 0.65, 0.0, 0.65, 0.0, 0.65]` |
| 15 | HAND_HEART2 | Two-hand Heart (part 2) | Softer heart shape | `[0.30, -0.1, 0.66, 0.0, 1.1, 1.1, 0.0, 1.1, 0.0, 1.1]` |
| 16 | HAND_HEART3 | Two-hand Heart (part 3) | Thumbs up heart variation | `[0.0, -1.56, 0.46, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` |
| 17 | CLASPING | Prayer/Clasping | Hands together, palms touching | `[0.5, -0.8, 0.2, -0.16, 0.6, 0.6, 0.17, 0.6, 0.17, 0.6]` |

---

## 2. How the Bridge Currently Accesses Gestures

The existing `omnihand_bridge_node.py` in `src/agx_arm_ctrl/agx_arm_ctrl/` provides:

- **Direct joint angle control:** `set_all_active_joint_angles(positions)`
- **Via JointState messages:** `control/joint_states` topic with position arrays
- **Via JointTrajectory:** `control/omnihand/joint_trajectory` topic with time-parameterized motion

**Current implementation:**
- No dedicated gesture preset API exists in the bridge
- Presets must be set as raw joint position vectors via the above interfaces
- No gesture enum or named gesture service exists yet

---

## 3. Accessing Gesture Presets in the Skill Controller

For the Sprint 6 `omnihand_skill_controller`, use this pattern:

```python
from enum import IntEnum

class OmniHandGesture(IntEnum):
    """Repo-owned gesture presets backed by vendor calibration."""
    OPEN = 0           # Reset position for open hand
    FIST_POWER = 3     # Power grasp (FIST2)
    GRASP_GLASS = 2    # Lighter grasp for delicate objects (FIST1)
    # ... other selections below

# Backend gesture lookup table
GESTURE_PRESETS = {
    OmniHandGesture.OPEN: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    OmniHandGesture.FIST_POWER: [0.5, -1.0, 0.75, 0.0, 1.48, 1.48, 0.0, 1.48, 0.0, 1.48],
    OmniHandGesture.GRASP_GLASS: [0.43, -0.3, 0.66, 0.0, 1.48, 1.48, 0.0, 1.48, 0.0, 1.48],
    # ...
}

# In the skill controller, drive toward a preset:
target_positions = GESTURE_PRESETS[OmniHandGesture.FIST_POWER]
self.backend.apply_joint_targets(target_dict, "grasp_closing")
```

### Access via Bridge Methods

1. **Direct position array** → `apply_joint_targets(target_map, control_mode)`
2. **Trajectory** → `apply_trajectory(trajectory_msg)` with preset as final point
3. **JointState topic** → publish positions via `control/joint_states`

---

## 4. Sprint 6 Gesture Selection for the Skill Controller

### MVP Skill Set

Based on the hand skill backend mapping (section 2 of `hand_skill_backend_mapping.md`), we need:

| Skill Name | Selected Vendor Gesture | Usage | Hardware Calibration Needed |
|---|---|---|---|
| `open_hand` | RESET (0) | Open hand to rest | ✓ Verify mechanical limits |
| `grasp_glass_until_contact` | FIST1 (2) | Light, controlled close for delicate objects | ✓ Sensor thresholds |
| `grasp_bottle_until_contact` | FIST2 (3) | Power grasp for larger objects | ✓ Sensor thresholds |
| `release_glass` | RESET (0) | Open from hold state | ✓ Joint velocity limits |
| `release_bottle` | RESET (0) | Open from hold state | ✓ Joint velocity limits |
| `stop_hand` | (internal, no preset) | Freeze current position | N/A |

### Rationale for Selection

- **RESET (0):** Universal open position; mechanical zero reference.
- **FIST1 (2):** Gentle flexion, suitable for thin-walled objects (glasses). Tested vendor preset.
- **FIST2 (3):** Maximum controlled flexion for sturdy objects (bottles). Tested vendor preset.
- **Avoided (for now):** Symbolic gestures (LIKE, ILY, NUM*, HAND_HEART*) — no grasp semantics.

### Not Selected (Out of Scope for MVP)

- All NUM* gestures (1, 4, 6, 8) — No grasp semantics, reserved for demo/teleoperation
- All HAND_HEART variants — Two-hand signatures, not relevant to single-hand grasp
- PAPER (1) — Less stable grasp than FIST1
- OK (4), ONE_HANDED_FINGER_HEART (5), CLASPING (17) — Specialized poses, not core grasping

---

## 5. Implementation Roadmap

### Phase 1: Backend Constants (This Sprint)

**Single source of truth:** the named presets now live in
`src/agx_arm_ctrl/config/omnihand_gestures.yaml`. Do **not** define a second
`GESTURE_PRESETS` copy in the skill controller; load them via
`agx_arm_ctrl.omnihand_bridge_node.resolve_gesture_presets(side)`:

```python
from agx_arm_ctrl.omnihand.models import get_hand_model
from agx_arm_ctrl.omnihand_bridge_node import resolve_gesture_presets

# Pass the model so the right per-model preset file + mirror convention are used.
model = get_hand_model("o12_pro")
presets = resolve_gesture_presets(hand_side, model)  # right = canonical, left = mirrored
target_positions = presets["zero"]                   # o12_pro has no calibrated "open" yet
```

Two corrections captured while wiring this up:

- **Convention is RIGHT-hand, not left.** The vendor `demo_set_motion.py` menu
  claims the presets are "only for the left hand", but the demo calls
  `create_hand(EHandType.RIGHT)` and every preset value fits the right-hand
  limits while falling out of range for the left. The config stores the
  canonical right-hand vectors; `resolve_gesture_presets("left")` mirrors them
  via `SDK_LEFT_POS_DIRECTION` so there is no second left-hand copy to maintain.
- **`open` ≠ all zeros.** The all-zeros pose (vendor RESET, exposed as `zero`) is
  the motor reference: fingers extend but the thumb rolls in and adducts across
  the palm and looks bent. The genuinely flat open palm is the vendor PAPER
  vector, now exposed as `open`.

### Phase 2: Hardware Calibration

On the Duo hardware, measure and record:

1. **Mechanical validation** — joints reach open position without binding
2. **Contact sensor thresholds** — glass vs bottle grasp detection thresholds
3. **Closure speed** — bounded step size in closing loop (safety)
4. **Hold stability** — transient contact debounce count

Record findings in `hefeweizen_validation_log.md`.

### Phase 3: Future Expansion

If additional object types are needed:
- Define new presets (e.g., `grasp_sphere`, `grasp_thin_handle`)
- Test on hardware
- Add to the backend preset table
- Update the skill action metadata to include the new skill names

---

## 6. Vendor Gesture Source

**Location:** `vendor/OmniHand-Pro-2025/python/example/demo_set_motion.py`  
**Enum:** `class Gesture(Enum)` (lines 8–27)  
**Presets:** `get_gesture_positions(gesture)` (lines 52–73)

All values are in **radians** and respect the hardware joint limits defined in the SDK API docs.

---

## 7. API Reference: How to Use Presets in Code

### Direct Control via Bridge

```python
# In skill controller, after importing the backend
target_positions = GESTURE_PRESETS["grasp_glass"]
target_map = {
    "right_thumb_roll_joint": target_positions[0],
    "right_thumb_abad_joint": target_positions[1],
    # ... (10 joints total)
}
self.backend.apply_joint_targets(target_map, "grasp_closing")
```

### Trajectory-Based Motion

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

msg = JointTrajectory()
msg.joint_names = [...]  # 10 joint names
point = JointTrajectoryPoint()
point.positions = GESTURE_PRESETS["grasp_bottle"]
point.time_from_start = Duration(sec=2)  # 2-second close
msg.points = [point]
trajectory_pub.publish(msg)  # Triggers bridge motion
```

---

## 8. Notes for Hardware Validation

- **Test environment:** OmniHand on Duo body (Hefeweizen target)
- **Safety:** Always command open position first; validate joint ranges before power-on
- **Tactile feedback:** Presets alone are insufficient for grasp confirmation; calibrate contact sensor thresholds per object
- **Vendor calibration:** The preset values are from vendor demo code; they are **recommendations** and may need fine-tuning per hardware batch

---

## See Also

- [Hand Skill Backend Mapping](./hand_skill_backend_mapping.md) — Skill state machine and tactile-confirmed completion
- [OmniHand Asset Validation](../../assets/omnihand_asset_validation.md) — Hardware and firmware inventory
- `vendor/OmniHand-Pro-2025/document/en/API_PYTHON.md` — Full SDK API reference
- `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py` — Bridge implementation
