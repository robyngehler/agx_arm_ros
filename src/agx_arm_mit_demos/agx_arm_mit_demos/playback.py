"""Turn a re-timed recording into the trajectory message the MIT controller takes.

Shared by every replay entry point in this package, so a recording replays the
same way whichever tool dispatched it.
"""

from __future__ import annotations

from typing import Optional

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def duration_msg(seconds: float):
    from builtin_interfaces.msg import Duration

    seconds = max(0.0, float(seconds))
    return Duration(sec=int(seconds), nanosec=int(round((seconds - int(seconds)) * 1e9)))


def retimed_to_joint_trajectory(
    result,
    joint_names,
    *,
    current_positions: Optional[list[float]] = None,
    lead_in_sec: float = 0.0,
    time_scale: float = 1.0,
) -> JointTrajectory:
    """Build the ROS message from a re-timed trajectory.

    Velocities come from the retiming rather than being left empty, which is what
    gives the controller a feedforward instead of pure position chasing.

    ``time_scale`` stretches the taught timing without re-planning the path: 2.0
    replays at half speed. It is not `speed_scale`, which re-parameterizes and
    discards the taught pace.
    """
    if not (time_scale > 0.0):
        raise ValueError("time_scale must be > 0")

    msg = JointTrajectory()
    msg.joint_names = list(joint_names)
    if current_positions is not None and lead_in_sec > 0.0:
        lead_in = JointTrajectoryPoint()
        lead_in.positions = [float(value) for value in current_positions]
        lead_in.velocities = [0.0] * len(current_positions)
        lead_in.time_from_start = duration_msg(0.0)
        msg.points.append(lead_in)
    for time_from_start, positions, velocities in zip(
        result.times, result.positions, result.velocities
    ):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.velocities = [float(value) / time_scale for value in velocities]
        point.time_from_start = duration_msg(time_from_start * time_scale + lead_in_sec)
        msg.points.append(point)
    return msg


__all__ = ["duration_msg", "retimed_to_joint_trajectory"]
