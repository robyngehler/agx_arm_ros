from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import re
from typing import Any, Optional

def _duration_from_seconds(seconds: float):
    from builtin_interfaces.msg import Duration

    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - duration.sec) * 1e9))
    if duration.nanosec >= 1_000_000_000:
        duration.sec += 1
        duration.nanosec -= 1_000_000_000
    return duration


def sanitize_trajectory_name(name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip())
    sanitized = sanitized.strip("._")
    return sanitized or "trajectory"


@dataclass(frozen=True)
class RecordedTrajectoryPoint:
    time_from_start: float
    positions: list[float]
    velocities: list[float]
    efforts: list[float]
    flange_pose: Optional[list[float]] = None


@dataclass(frozen=True)
class RecordedTrajectory:
    name: str
    robot: str
    joint_names: list[str]
    sample_rate_hz: float
    recorded_at: str
    points: list[RecordedTrajectoryPoint]
    metadata: dict[str, Any]

    @property
    def duration(self) -> float:
        if not self.points:
            return 0.0
        return self.points[-1].time_from_start


def trim_trailing_stationary_points(
    points: list[RecordedTrajectoryPoint],
    movement_threshold_rad: float,
) -> tuple[list[RecordedTrajectoryPoint], int]:
    if len(points) < 2:
        return points, len(points) - 1

    last_motion_index = 0
    for index in range(1, len(points)):
        deltas = [
            abs(current - previous)
            for current, previous in zip(points[index].positions, points[index - 1].positions)
        ]
        if max(deltas, default=0.0) >= movement_threshold_rad:
            last_motion_index = index

    return points[: last_motion_index + 1], last_motion_index


def with_finite_difference_velocities(
    times: list[float],
    positions: list[list[float]],
    efforts: list[list[float]],
    flange_poses: Optional[list[Optional[list[float]]]] = None,
) -> list[RecordedTrajectoryPoint]:
    if not times:
        return []

    points: list[RecordedTrajectoryPoint] = []
    joint_count = len(positions[0])
    for index, time_from_start in enumerate(times):
        if index == 0:
            velocities = [0.0] * joint_count
        else:
            dt = max(1e-6, times[index] - times[index - 1])
            velocities = [
                (current - previous) / dt
                for current, previous in zip(positions[index], positions[index - 1])
            ]
        points.append(
            RecordedTrajectoryPoint(
                time_from_start=time_from_start,
                positions=list(positions[index]),
                velocities=velocities,
                efforts=list(efforts[index]),
                flange_pose=None if flange_poses is None else flange_poses[index],
            )
        )
    return points


def save_recorded_trajectory(trajectory: RecordedTrajectory, file_path: str | Path) -> Path:
    path = Path(file_path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = asdict(trajectory)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return path


def load_recorded_trajectory(file_path: str | Path) -> RecordedTrajectory:
    path = Path(file_path).expanduser().resolve()
    payload = json.loads(path.read_text(encoding="utf-8"))
    points = [RecordedTrajectoryPoint(**point) for point in payload["points"]]
    return RecordedTrajectory(
        name=payload["name"],
        robot=payload["robot"],
        joint_names=list(payload["joint_names"]),
        sample_rate_hz=float(payload["sample_rate_hz"]),
        recorded_at=payload["recorded_at"],
        points=points,
        metadata=dict(payload.get("metadata", {})),
    )


def recorded_to_joint_trajectory(trajectory: RecordedTrajectory):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    msg = JointTrajectory()
    msg.joint_names = list(trajectory.joint_names)
    for point in trajectory.points:
        ros_point = JointTrajectoryPoint()
        ros_point.positions = list(point.positions)
        ros_point.velocities = list(point.velocities)
        ros_point.effort = list(point.efforts)
        ros_point.time_from_start = _duration_from_seconds(point.time_from_start)
        msg.points.append(ros_point)
    return msg


def default_recorded_at() -> str:
    return datetime.now(timezone.utc).isoformat()