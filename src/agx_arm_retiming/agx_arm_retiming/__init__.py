"""Time parameterization for recorded agx_arm trajectories.

Two families of mode, split by what each tool can express:

* ``as_recorded`` and ``smooth`` keep the taught timing and only re-derive the
  derivatives, so a motion whose dynamics were taught deliberately (a dwell, a
  slow pour) survives.
* ``speed_scale`` and ``maximize_speed`` hand the geometric path to MoveIt's
  time-optimal parameterization, which computes its own timing and therefore
  discards anything purely temporal in the recording.
"""

from agx_arm_retiming._totg import retime_path
from agx_arm_retiming.planning import (
    AS_RECORDED,
    MAXIMIZE_SPEED,
    MODES,
    NERO_MAX_VELOCITY,
    SMOOTH,
    SPEED_SCALE,
    RetimedTrajectory,
    RetimingError,
    default_acceleration,
    retime,
)

__all__ = [
    "AS_RECORDED",
    "MAXIMIZE_SPEED",
    "MODES",
    "NERO_MAX_VELOCITY",
    "SMOOTH",
    "SPEED_SCALE",
    "RetimedTrajectory",
    "RetimingError",
    "default_acceleration",
    "retime",
    "retime_path",
]
