"""Time parameterization for recorded agx_arm trajectories.

Two families of mode, split by what each tool can express:

* ``as_recorded`` and ``smooth`` keep the taught duration and pace. Both
  resample the recording onto a uniform grid and filter it there; they differ
  only in filter width. A motion whose dynamics were taught deliberately (a
  dwell, a slow pour) survives.
* ``speed_scale`` and ``maximize_speed`` hand the geometric path to MoveIt's
  time-optimal parameterization, which computes its own timing and therefore
  discards anything purely temporal in the recording.

No mode replays raw samples. A recording's time grid is uneven, and the
controller interpolates linearly between trajectory points, so an uneven knot
becomes a step in commanded velocity
(``docs/sprint_refactor/reference/teach_replay_timebase.md``).
"""

from agx_arm_retiming._totg import retime_path
from agx_arm_retiming.planning import (
    AS_RECORDED,
    DEFAULT_RESAMPLE_DT,
    DEFAULT_SMOOTHING_WINDOW_SEC,
    MAXIMIZE_SPEED,
    MODES,
    NERO_MAX_VELOCITY,
    RECONSTRUCTION_WINDOW_SEC,
    SMOOTH,
    SPEED_SCALE,
    RetimedTrajectory,
    RetimingError,
    default_acceleration,
    retime,
)

__all__ = [
    "AS_RECORDED",
    "DEFAULT_RESAMPLE_DT",
    "DEFAULT_SMOOTHING_WINDOW_SEC",
    "MAXIMIZE_SPEED",
    "MODES",
    "NERO_MAX_VELOCITY",
    "RECONSTRUCTION_WINDOW_SEC",
    "SMOOTH",
    "SPEED_SCALE",
    "RetimedTrajectory",
    "RetimingError",
    "default_acceleration",
    "retime",
    "retime_path",
]
