"""Time parameterization for recorded agx_arm trajectories.

Two families of mode, split by what each tool can express:

* ``as_recorded`` and ``smooth`` keep the taught timing and fit nothing: the
  recorded samples are replayed at their recorded times, ``smooth`` after a
  moving-average window over the positions. A motion whose dynamics were taught
  deliberately (a dwell, a slow pour) survives, and so does the path.
* ``speed_scale`` and ``maximize_speed`` hand the geometric path to MoveIt's
  time-optimal parameterization, which computes its own timing and therefore
  discards anything purely temporal in the recording.
"""

from agx_arm_retiming._totg import retime_path
from agx_arm_retiming.planning import (
    AS_RECORDED,
    DEFAULT_SMOOTHING_WINDOW_SEC,
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
    "DEFAULT_SMOOTHING_WINDOW_SEC",
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
