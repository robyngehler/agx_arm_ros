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
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


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


# --- Registry-driven model construction --------------------------------------
# The active-joint sets, limits, mirror directions, and tactile maps come from the
# single source of truth, agx_arm_description/config/duo_motion_registry.yaml, so
# this file no longer hand-maintains lists that must match the MoveIt SRDF /
# controllers / _multi_arm_runtime. Only the gesture-preset file (a control-layer
# concern) stays here. O12 limits are stored in the registry as degrees and the
# vendor URDF (o12_hand_description) ranges; O10 as native radians.
_GESTURE_CONFIG_FILE: dict[str, str] = {
    "o10": "omnihand_gestures.yaml",
    "o12_pro": "omnihand_pro_gestures.yaml",
}

_REGISTRY_RELATIVE_PATH = Path("config") / "duo_motion_registry.yaml"


def _find_motion_registry() -> Path:
    """Locate duo_motion_registry.yaml in agx_arm_description (share, then source)."""
    try:
        share_dir = Path(get_package_share_directory("agx_arm_description"))
        candidate = share_dir / _REGISTRY_RELATIVE_PATH
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    source_rel = Path("src") / "agx_arm_sim" / "agx_arm_description" / _REGISTRY_RELATIVE_PATH
    for parent in Path(__file__).resolve().parents:
        candidate = parent / source_rel
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "duo_motion_registry.yaml not found in agx_arm_description (share or source tree)"
    )


def _load_motion_registry() -> dict:
    return yaml.safe_load(_find_motion_registry().read_text(encoding="utf-8")) or {}


def _hand_model_from_registry(name: str, entry: dict) -> HandModel:
    joints = entry.get("active_joints", [])
    unit = str(entry.get("limit_unit", "rad"))
    convert = _rad if unit == "deg" else (lambda value: float(value))
    return HandModel(
        name=name,
        gesture_config_file=_GESTURE_CONFIG_FILE.get(name, ""),
        joint_suffixes=tuple(str(joint["suffix"]) for joint in joints),
        active_joint_min_right=tuple(convert(joint["min"]) for joint in joints),
        active_joint_max_right=tuple(convert(joint["max"]) for joint in joints),
        left_pos_direction=tuple(int(joint["left_dir"]) for joint in joints),
        tactile_fingers=tuple((str(a), str(b)) for a, b in entry.get("tactile_fingers", [])),
    )


_OMNIHAND_REGISTRY = _load_motion_registry().get("omnihand", {})

O10 = _hand_model_from_registry("o10", _OMNIHAND_REGISTRY["o10"])
O12_PRO = _hand_model_from_registry("o12_pro", _OMNIHAND_REGISTRY["o12_pro"])

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
