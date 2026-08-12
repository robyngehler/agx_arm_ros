"""What a MIT command must satisfy before it is allowed to reach the arm.

Phase 1A of the V02 refactor. The driver previously checked two things — that
the parallel arrays were the same length and that the message was not empty —
and then forwarded whatever was in them, joint by joint, to the vendor SDK. A
NaN, a joint index the arm does not have, the same joint commanded twice in one
message, or a gain an order of magnitude past the protocol range all reached the
hardware boundary unexamined.

The rule the refactor states is that **SDK clamping is a last protection, not
the input contract**. Clamping silently turns a wrong command into a different
command; the sender never learns it was wrong, and the arm moves somewhere
nobody asked for.

Rejection is atomic per message. A MIT message carries one setpoint per joint
and the firmware keeps executing the last one it received, so admitting six
joints and dropping the seventh would leave the arm in a pose no sender ever
commanded — worse than refusing the message and holding the previous one.

Two tiers, deliberately:

* **rejected** — protocol violations and malformed input. These cannot be a
  legitimate command under any configuration.
* **flagged** — a position outside the *configured joint limit* for that joint.
  The firmware enforces its own limits, and rejecting mid-stream would freeze a
  running impedance loop at its last setpoint. This warns until one hardware
  session establishes that the controller never legitimately crosses a limit;
  the plan then promotes it to a rejection.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class MitProtocolLimits:
    """Value ranges the Nero MIT protocol can actually encode.

    Taken from the pinned vendor driver's ``move_mit`` documentation. Outside
    these, the SDK either raises or the on-wire encoding wraps, so a value out
    here is never a command anyone meant to send.

    **The torque bound depends on the firmware tier**, which was found on
    hardware: the two arms in this unit do not run the same firmware. The
    default-tier driver bounds feed-forward torque per joint (24/16/8 N·m), and
    the 1.11 driver — which the 1.12 driver inherits — bounds every joint at
    16 N·m with a different on-wire encoding (12-bit t_ff, no CRC). Using one
    for the other rejects legitimate commands on joints 5-7 or admits ones the
    SDK will refuse on joints 1-2.
    """

    p_des: Tuple[float, float] = (-12.5, 12.5)
    v_des: Tuple[float, float] = (-45.0, 45.0)
    kp: Tuple[float, float] = (0.0, 500.0)
    kd: Tuple[float, float] = (-5.0, 5.0)
    # Per joint, 1-based. Empty means the tier bounds every joint the same.
    torque_by_joint: Tuple[float, ...] = ()
    torque_uniform: float = 16.0

    def torque_limit(self, joint_index: int) -> float:
        """Feed-forward torque bound for one joint under this firmware tier."""
        if not self.torque_by_joint:
            return self.torque_uniform
        if 1 <= joint_index <= len(self.torque_by_joint):
            return self.torque_by_joint[joint_index - 1]
        # Unknown joint: the caller rejects it anyway, so answer conservatively
        # rather than assume a bound this arm may not have.
        return min(self.torque_by_joint)


NERO_DEFAULT_MIT_LIMITS = MitProtocolLimits(
    torque_by_joint=(24.0, 24.0, 16.0, 16.0, 8.0, 8.0, 8.0),
)
"""Firmware <= 1.10. Feed-forward torque is bounded per joint."""

NERO_V111_MIT_LIMITS = MitProtocolLimits(torque_uniform=16.0)
"""Firmware 1.11 and 1.12 — the 1.12 driver inherits 1.11's ``move_mit``."""

_MIT_LIMITS_BY_TIER = {
    "default": NERO_DEFAULT_MIT_LIMITS,
    "v111": NERO_V111_MIT_LIMITS,
    "v112": NERO_V111_MIT_LIMITS,
}

# Kept as the module default because it matches the driver the SDK builds when
# no firmware tier is given, which is what an unresolved arm actually runs.
NERO_MIT_LIMITS = NERO_DEFAULT_MIT_LIMITS


def mit_limits_for_tier(tier) -> MitProtocolLimits:
    """Return the MIT bounds for a resolved Nero firmware tier.

    Keyed by the tier string rather than the SDK constant so this module needs
    no vendor import and stays testable without a workspace build; the SDK's
    ``NeroFW`` values *are* those strings.
    """
    return _MIT_LIMITS_BY_TIER.get(str(tier), NERO_DEFAULT_MIT_LIMITS)


@dataclass(frozen=True)
class CommandRejection:
    """Why a command was refused, in a form a log line and a test can both use."""

    reason: str
    detail: str


def validate_mit_command(
    joint_index: Sequence[int],
    p_des: Sequence[float],
    v_des: Sequence[float],
    kp: Sequence[float],
    kd: Sequence[float],
    torque: Sequence[float],
    *,
    joint_count: int,
    limits: MitProtocolLimits = NERO_MIT_LIMITS,
) -> Optional[CommandRejection]:
    """Return None when the command may be sent, or why it may not.

    ``joint_index`` is 1-based, matching the vendor protocol.
    """
    arrays = (joint_index, p_des, v_des, kp, kd, torque)
    lengths = {len(array) for array in arrays}
    if len(lengths) > 1:
        return CommandRejection(
            "length_mismatch",
            "MIT arrays have inconsistent lengths: "
            f"joint_index={len(joint_index)} p_des={len(p_des)} "
            f"v_des={len(v_des)} kp={len(kp)} kd={len(kd)} "
            f"torque={len(torque)}",
        )
    if not joint_index:
        return CommandRejection("empty", "MIT command carries no joints")

    seen = set()
    for position, index in enumerate(joint_index):
        if index < 1 or index > joint_count:
            return CommandRejection(
                "unknown_joint",
                f"joint index {index} is outside 1..{joint_count}",
            )
        if index in seen:
            # Last-one-wins would make the command's effect depend on array
            # order, which no sender can reason about.
            return CommandRejection(
                "duplicate_joint",
                f"joint index {index} appears more than once "
                f"(at array position {position})",
            )
        seen.add(index)

    fields = (
        ("p_des", p_des, limits.p_des),
        ("v_des", v_des, limits.v_des),
        ("kp", kp, limits.kp),
        ("kd", kd, limits.kd),
    )
    for name, values, (low, high) in fields:
        for position, value in enumerate(values):
            rejection = _check_value(
                name, value, low, high, joint_index[position]
            )
            if rejection is not None:
                return rejection

    for position, value in enumerate(torque):
        bound = limits.torque_limit(joint_index[position])
        rejection = _check_value(
            "torque", value, -bound, bound, joint_index[position]
        )
        if rejection is not None:
            return rejection

    return None


def _check_value(
    name: str, value: float, low: float, high: float, joint_index: int
) -> Optional[CommandRejection]:
    number = float(value)
    if not math.isfinite(number):
        return CommandRejection(
            "non_finite",
            f"{name} for joint {joint_index} is {number}",
        )
    if number < low or number > high:
        return CommandRejection(
            "out_of_range",
            f"{name}={number:g} for joint {joint_index} is outside "
            f"[{low:g}, {high:g}]",
        )
    return None


def positions_outside_joint_limits(
    joint_index: Sequence[int],
    p_des: Sequence[float],
    joint_names: Sequence[str],
    joint_limits: Dict[str, Sequence[float]],
) -> List[str]:
    """Name the joints commanded past their configured limit.

    Flagged rather than rejected — see the module docstring. Returns an empty
    list when nothing is out of bounds, or when no limits are configured.
    """
    if not joint_limits:
        return []
    outside = []
    for position, index in enumerate(joint_index):
        if index < 1 or index > len(joint_names):
            continue
        name = joint_names[index - 1]
        limit = joint_limits.get(name)
        if not limit or len(limit) != 2:
            continue
        value = float(p_des[position])
        if not math.isfinite(value):
            continue
        low, high = float(limit[0]), float(limit[1])
        if value < low or value > high:
            outside.append(
                f"{name}={value:.4f} outside [{low:.4f}, {high:.4f}]"
            )
    return outside
