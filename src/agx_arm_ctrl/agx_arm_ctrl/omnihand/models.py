"""Registry of supported OmniHand hardware models.

Each :class:`HandModel` captures everything that changes between hardware
generations so the rest of the stack can stay model-agnostic:

- ``o10`` — OmniHand 2025 (``AgibotHandO10``, 10 active joints). The original
  integration target. Kept for backward compatibility and the mock backend.
- ``o12_pro`` — OmniHand Pro 2025 (``AgibotHandO12``, 12 active joints). The
  hardware we actually own; this is the migration target.

The ``o12_pro`` joint order, limits, and tactile layout come from the vendor
manual and the ``AgibotHandO12`` Python API. NOTE: the documented joint-angle
limits (degrees) and the vendor demo command vectors disagree at the edges
(e.g. the demo "fist" drives ``thumb_roll`` past the documented range), the same
quirk seen on O10. Treat the limits here as PROVISIONAL command clamps to verify
on hardware, not as ground truth — see the migration proposal §5.5/§12.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


def _rad(degrees: float) -> float:
    return math.radians(degrees)


@dataclass(frozen=True)
class HandModel:
    """Static description of one OmniHand hardware generation."""

    name: str
    # Named active-joint preset file for this model, installed under the package
    # share/config. Each model carries its own gesture vectors because the joint
    # order and count differ (o10: 10 values, o12_pro: 12 values).
    gesture_config_file: str
    joint_suffixes: tuple[str, ...]
    # Active-joint command limits for the RIGHT hand, ordered to match
    # joint_suffixes. The LEFT hand is derived by sign-mirroring per
    # left_pos_direction (-1 flips the joint, +1 keeps it).
    active_joint_min_right: tuple[float, ...]
    active_joint_max_right: tuple[float, ...]
    left_pos_direction: tuple[int, ...]
    # (ROS layout name, vendor EFinger enum member name), tip-to-tip order.
    tactile_fingers: tuple[tuple[str, str], ...]

    def __post_init__(self) -> None:
        n = len(self.joint_suffixes)
        for label, seq in (
            ("active_joint_min_right", self.active_joint_min_right),
            ("active_joint_max_right", self.active_joint_max_right),
            ("left_pos_direction", self.left_pos_direction),
        ):
            if len(seq) != n:
                raise ValueError(
                    f"hand model '{self.name}': {label} has {len(seq)} entries, "
                    f"expected {n} (one per active joint)"
                )

    @property
    def active_joint_count(self) -> int:
        return len(self.joint_suffixes)

    def build_joint_names(self, hand_side: str) -> list[str]:
        prefix = f"{hand_side}_"
        return [f"{prefix}{suffix}" for suffix in self.joint_suffixes]

    def mirror_active_joint_vector(self, values: list[float]) -> list[float]:
        """Mirror a right-hand active-joint vector into the left-hand convention."""
        return [
            direction * float(value)
            for direction, value in zip(self.left_pos_direction, values, strict=True)
        ]

    def joint_limits(self, hand_side: str) -> tuple[list[float], list[float]]:
        """Return (min, max) active-joint command limits for the side."""
        if hand_side == "left":
            mirrored_max = list(self.active_joint_max_right)
            mirrored_min = list(self.active_joint_min_right)
            for index, direction in enumerate(self.left_pos_direction):
                if direction == -1:
                    mirrored_max[index] = -self.active_joint_min_right[index]
                    mirrored_min[index] = -self.active_joint_max_right[index]
            return mirrored_min, mirrored_max
        return list(self.active_joint_min_right), list(self.active_joint_max_right)


# --- OmniHand 2025 (O10) -----------------------------------------------------
# Mirrors the original constants in omnihand_bridge_node.py.
O10 = HandModel(
    name="o10",
    gesture_config_file="omnihand_gestures.yaml",
    joint_suffixes=(
        "thumb_roll_joint",
        "thumb_abad_joint",
        "thumb_mcp_joint",
        "index_abad_joint",
        "index_pip_joint",
        "middle_pip_joint",
        "ring_abad_joint",
        "ring_pip_joint",
        "pinky_abad_joint",
        "pinky_pip_joint",
    ),
    active_joint_min_right=(-0.03, -1.64, 0.0, -0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    active_joint_max_right=(1.12, 0.05, 0.8416, 0.0, 1.48, 1.48, 0.17, 1.48, 0.19, 1.48),
    left_pos_direction=(-1, -1, -1, -1, 1, 1, -1, 1, -1, 1),
    tactile_fingers=(
        ("thumb_tip", "THUMB"),
        ("index_tip", "INDEX"),
        ("middle_tip", "MIDDLE"),
        ("ring_tip", "RING"),
        ("little_tip", "LITTLE"),
    ),
)


# --- OmniHand Pro 2025 (O12) -------------------------------------------------
# Joint order and limits from the OmniHand Pro 2025 manual (§2.4) and the
# AgibotHandO12 Python API. Limits are the RIGHT-hand ranges in radians; only
# thumb_roll and thumb_abad differ between left/right (sign flip), everything
# else is symmetric.
#
# thumb_roll / thumb_abad SIGN FIX (2026-06-29): the original provisional values
# were sign-flipped (thumb_roll [-42,0], thumb_abad [0,+54]). The bridge clamps
# SDK targets to these limits, so positive thumb_roll commands clamped to 0 and the
# thumb never rolled toward the palm (observed on hardware). The official vendor
# URDF (o12_hand_description) and the vendor demo fist preset both use thumb_roll
# [0,+42] and thumb_abad [-54,0]; aligned here so command clamps match the hand.
# The remaining ranges are PROVISIONAL — verify on hardware.
O12_PRO = HandModel(
    name="o12_pro",
    gesture_config_file="omnihand_pro_gestures.yaml",
    joint_suffixes=(
        "thumb_roll_joint",
        "thumb_abad_joint",
        "thumb_mcp_joint",
        "thumb_pip_joint",
        "index_abad_joint",
        "index_mcp_joint",
        "index_pip_joint",
        "middle_abad_joint",
        "middle_mcp_joint",
        "middle_pip_joint",
        "ring_mcp_joint",
        "pinky_mcp_joint",
    ),
    active_joint_min_right=(
        _rad(0.0),    # thumb_roll  (vendor URDF: [0, +42]); see note below
        _rad(-54.0),  # thumb_abad  (vendor URDF: [-54, 0]); see note below
        _rad(-49.0),  # thumb_mcp
        _rad(-74.0),  # thumb_pip
        _rad(-15.0),  # index_abad
        _rad(0.0),    # index_mcp
        _rad(0.0),    # index_pip
        _rad(-15.0),  # middle_abad
        _rad(0.0),    # middle_mcp
        _rad(0.0),    # middle_pip
        _rad(0.0),    # ring_mcp
        _rad(0.0),    # pinky_mcp
    ),
    active_joint_max_right=(
        _rad(42.0),   # thumb_roll  (vendor URDF: [0, +42]); see note below
        _rad(0.0),    # thumb_abad  (vendor URDF: [-54, 0]); see note below
        _rad(0.0),    # thumb_mcp
        _rad(0.0),    # thumb_pip
        _rad(15.0),   # index_abad
        _rad(76.0),   # index_mcp
        _rad(85.0),   # index_pip
        _rad(15.0),   # middle_abad
        _rad(76.0),   # middle_mcp
        _rad(98.0),   # middle_pip
        _rad(79.0),   # ring_mcp
        _rad(79.0),   # pinky_mcp
    ),
    # Only thumb_roll and thumb_abad flip sign between left and right.
    left_pos_direction=(-1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1),
    tactile_fingers=(
        ("thumb_tip", "THUMB"),
        ("index_tip", "INDEX"),
        ("middle_tip", "MIDDLE"),
        ("ring_tip", "RING"),
        ("little_tip", "LITTLE"),
    ),
)


HAND_MODELS: dict[str, HandModel] = {O10.name: O10, O12_PRO.name: O12_PRO}
# o12_pro is the live default: it is the hardware we own, and the O10 SDK vendor
# submodule has been swapped out, so o10 only remains usable via the mock backend.
DEFAULT_HAND_MODEL = O12_PRO.name


def get_hand_model(name: str) -> HandModel:
    try:
        return HAND_MODELS[name]
    except KeyError:
        raise ValueError(
            f"unknown hand_model '{name}'; expected one of {sorted(HAND_MODELS)}"
        ) from None
