"""Normalized closure for the AGX parallel gripper, and the one conversion.

Everything above the driver — catalogue actions, the coordinator, the teach
manager — commands a gripper as ``closure`` in [0, 1], where 0.0 is fully open
and 1.0 is fully closed. Physical width in metres stays below, in the FJT bridge
and the driver, so a vendor stroke never reaches the Activity catalogue.

The stroke endpoints and the finger joint names come from the duo motion
registry; agx_arm_ctrl's test_gripper_stroke_agreement asserts they equal the
range and the joints the driver enforces.
"""

from __future__ import annotations

import math

from agx_arm_coordination.motion_registry import load_motion_registry

#: Each finger travels half the opening, and the second mirrors the first, so
#: either one alone determines the width.
FINGER_TO_WIDTH = 2.0
FINGER_SUFFIXES = ("gripper_joint1", "gripper_joint2")

#: Which robot_ids are parallel grippers, and the arm side each rides on.
GRIPPER_SIDES = {"left_gripper": "left", "right_gripper": "right"}

CLOSURE_KEY = "closure"


class ClosureError(ValueError):
    """Raised when a normalized closure or a width is not usable."""


def gripper_stroke() -> tuple[float, float]:
    """``(width_open, width_closed)`` in metres, from the registry."""
    block = load_motion_registry().get("gripper", {}) or {}
    try:
        width_open = float(block["width_open_m"])
        width_closed = float(block["width_closed_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ClosureError(
            "duo_motion_registry.yaml gripper block declares no usable "
            f"width_open_m / width_closed_m: {exc}"
        ) from exc
    if width_open == width_closed:
        raise ClosureError(
            f"gripper stroke is empty (open == closed == {width_open})"
        )
    return width_open, width_closed


def closure_to_width(closure: float) -> float:
    """Width in metres for a normalized closure.

    Rejects a non-finite or out-of-range value rather than saturating it: a
    clamp would turn a corrupt number into the maximum command, and every range
    check downstream would then see a plausible width.
    """
    value = _finite(closure, "closure")
    if not 0.0 <= value <= 1.0:
        raise ClosureError(f"closure {value} outside [0.0, 1.0]")
    width_open, width_closed = gripper_stroke()
    return width_open - value * (width_open - width_closed)


def width_to_closure(width: float) -> float:
    """Normalized closure for a measured width. Not clamped.

    A readback outside the stroke is reported as the value it is, because a
    measurement folded into the command window reads as the window edge
    whatever the hardware did. Use :func:`displayed_closure` for a UI.
    """
    value = _finite(width, "width")
    width_open, width_closed = gripper_stroke()
    return (width_open - value) / (width_open - width_closed)


def displayed_closure(width: float) -> float:
    """:func:`width_to_closure` clamped to [0, 1] for display only."""
    return max(0.0, min(1.0, width_to_closure(width)))


def closure_to_finger_positions(
    joint_names, closure: float
) -> list[float]:
    """Finger positions for a closure, in the order the names are given.

    ``gripper_joint1`` opens positive and ``gripper_joint2`` mirrors it, each
    over half the opening. A name that is neither gets 0.0, which is what the
    driver would read out of it anyway.
    """
    half = closure_to_width(closure) / FINGER_TO_WIDTH
    positions = []
    for name in joint_names:
        if name.endswith("gripper_joint1"):
            positions.append(half)
        elif name.endswith("gripper_joint2"):
            positions.append(-half)
        else:
            positions.append(0.0)
    return positions


def width_from_finger_positions(joint_names, positions) -> float | None:
    """Opening width from whichever finger joint the names carry."""
    for suffix in FINGER_SUFFIXES:
        for index, name in enumerate(joint_names):
            if name.endswith(suffix) and index < len(positions):
                return abs(positions[index]) * FINGER_TO_WIDTH
    return None


def gripper_side(robot_id: str) -> str:
    """Arm side a gripper robot_id rides on (``""`` when it is not a gripper)."""
    return GRIPPER_SIDES.get(robot_id, "")


def _finite(value, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ClosureError(f"{label} '{value}' is not a number") from exc
    if not math.isfinite(number):
        raise ClosureError(f"{label} {value} is not finite")
    return number
