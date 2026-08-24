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


def deduplicate_recorded_trajectory(
    trajectory: RecordedTrajectory,
    tolerance: float = 0.0,
) -> tuple[RecordedTrajectory, int]:
    """Drop samples that repeat the previous position, and say how many.

    A recorder samples the feedback cache on a fixed clock, so wherever the clock
    runs faster than the arm supplies frames it stores the previous sample again.
    Those rows carry no information and actively harm what reads them: a finite
    difference across a repeat alternates between zero and twice the true value,
    and a spline fitted through them reproduces that as acceleration noise.

    Removing them leaves the sampling uneven, which the trajectory buffer and the
    retiming pipeline both handle -- they interpolate and fit on
    ``time_from_start``. Positions, times, efforts and flange poses of the
    surviving samples are untouched, so the taught path is exactly the one that
    was recorded; only velocities are recomputed, as central differences over
    the new spacing.

    ``sample_rate_hz`` becomes the rate that survived, because that field is what
    a later reader believes about the data.
    """
    points = trajectory.points
    if len(points) < 3:
        return trajectory, 0

    kept = [points[0]]
    for point in points[1:]:
        previous = kept[-1].positions
        if max(abs(a - b) for a, b in zip(point.positions, previous)) > tolerance:
            kept.append(point)
    # The last sample is where the motion ended; dropping it as a repeat would
    # leave the replay short of the taught end pose.
    if kept[-1] is not points[-1]:
        kept.append(points[-1])

    removed = len(points) - len(kept)
    if not removed:
        return trajectory, 0

    joint_count = len(points[0].positions)
    rebuilt: list[RecordedTrajectoryPoint] = []
    for index, point in enumerate(kept):
        if index == 0 or index == len(kept) - 1:
            velocities = [0.0] * joint_count
        else:
            span = kept[index + 1].time_from_start - kept[index - 1].time_from_start
            velocities = (
                [
                    (kept[index + 1].positions[j] - kept[index - 1].positions[j]) / span
                    for j in range(joint_count)
                ]
                if span > 0.0
                else [0.0] * joint_count
            )
        rebuilt.append(
            RecordedTrajectoryPoint(
                time_from_start=point.time_from_start,
                positions=list(point.positions),
                velocities=velocities,
                efforts=list(point.efforts),
                flange_pose=point.flange_pose,
            )
        )

    duration = rebuilt[-1].time_from_start - rebuilt[0].time_from_start
    effective = (len(rebuilt) - 1) / duration if duration > 0.0 else 0.0
    metadata = dict(trajectory.metadata)
    metadata["deduplicated"] = {
        "removed_repeats": removed,
        "original_samples": len(points),
        "original_sample_rate_hz": trajectory.sample_rate_hz,
        "tolerance_rad": tolerance,
    }
    return (
        RecordedTrajectory(
            name=trajectory.name,
            robot=trajectory.robot,
            joint_names=list(trajectory.joint_names),
            sample_rate_hz=round(effective, 3),
            recorded_at=trajectory.recorded_at,
            points=rebuilt,
            metadata=metadata,
        ),
        removed,
    )


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


def smooth_recorded_trajectory(
    trajectory: RecordedTrajectory,
    smoothing_window: int,
) -> RecordedTrajectory:
    """Return a playback-smoothed copy of a recorded trajectory.

    Teach recordings sample a feedback cache at a fixed rate, so 10-30% of
    consecutive points carry byte-identical (stale) positions and the
    finite-difference velocities chatter between ~0 and twice the true value.
    Replayed raw, that staircase makes the arm judder even though MoveIt
    trajectories run smoothly through the same controller.

    Positions are filtered with a zero-phase centered moving average whose
    half-window shrinks symmetrically at the edges (first/last points stay
    exact, no phase lag), and velocities are recomputed as central differences
    of the smoothed positions (endpoints at rest). Times, efforts, flange
    poses, and the stored file are untouched — smoothing is a playback-time
    step so raw recordings stay the ground truth.

    ``smoothing_window`` is the full window in samples (9 at 50 Hz ~ 180 ms,
    ~2.5 Hz cutoff — well above hand-taught motion content). Values <= 1
    return the trajectory unchanged.

    Filters in sample space, so it is a time-domain filter only on a recording
    whose samples are evenly spaced, and it leaves the times untouched. Replay
    goes through ``agx_arm_retiming.retime`` instead, which resamples onto a
    uniform grid first.
    """
    if smoothing_window <= 1 or len(trajectory.points) < 3:
        return trajectory

    points = trajectory.points
    count = len(points)
    joint_count = len(points[0].positions)
    half_window = smoothing_window // 2

    smoothed_positions: list[list[float]] = []
    for index in range(count):
        reach = min(half_window, index, count - 1 - index)
        window_points = points[index - reach: index + reach + 1]
        smoothed_positions.append(
            [
                sum(point.positions[joint] for point in window_points) / len(window_points)
                for joint in range(joint_count)
            ]
        )

    smoothed_points: list[RecordedTrajectoryPoint] = []
    for index in range(count):
        if index == 0 or index == count - 1:
            velocities = [0.0] * joint_count
        else:
            dt = max(1e-6, points[index + 1].time_from_start - points[index - 1].time_from_start)
            velocities = [
                (smoothed_positions[index + 1][joint] - smoothed_positions[index - 1][joint]) / dt
                for joint in range(joint_count)
            ]
        smoothed_points.append(
            RecordedTrajectoryPoint(
                time_from_start=points[index].time_from_start,
                positions=smoothed_positions[index],
                velocities=velocities,
                efforts=list(points[index].efforts),
                flange_pose=points[index].flange_pose,
            )
        )

    metadata = dict(trajectory.metadata)
    metadata["playback_smoothing_window"] = smoothing_window
    return RecordedTrajectory(
        name=trajectory.name,
        robot=trajectory.robot,
        joint_names=list(trajectory.joint_names),
        sample_rate_hz=trajectory.sample_rate_hz,
        recorded_at=trajectory.recorded_at,
        points=smoothed_points,
        metadata=metadata,
    )


def smoothing_window_samples(trajectory: RecordedTrajectory, window_sec: float) -> int:
    """Samples covering ``window_sec`` at this recording's own rate.

    The window belongs in the time domain: the same sample count is a different
    filter at a different recording rate, so a count lets a change to the
    recording rate silently retune the smoothing. Raising teach recordings from
    50 Hz to 100 Hz halved it exactly that way.

    Falls back to the recording's timestamps when it declares no rate, and to 0
    (no smoothing) when neither is usable.
    """
    if window_sec <= 0.0:
        return 0
    rate = float(trajectory.sample_rate_hz or 0.0)
    if rate <= 0.0 and len(trajectory.points) > 1 and trajectory.duration > 0.0:
        rate = (len(trajectory.points) - 1) / trajectory.duration
    if rate <= 0.0:
        return 0
    return max(1, int(round(window_sec * rate)))


def smooth_recorded_trajectory_seconds(
    trajectory: RecordedTrajectory,
    window_sec: float,
) -> RecordedTrajectory:
    """:func:`smooth_recorded_trajectory` with the window given as a duration."""
    return smooth_recorded_trajectory(
        trajectory, smoothing_window_samples(trajectory, window_sec)
    )


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


def recorded_to_joint_trajectory(
    trajectory: RecordedTrajectory,
    *,
    time_scale: float = 1.0,
    current_positions: Optional[list[float]] = None,
    lead_in_sec: float = 0.0,
):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    if time_scale <= 0.0:
        raise ValueError("time_scale must be > 0")
    if lead_in_sec < 0.0:
        raise ValueError("lead_in_sec must be >= 0")

    msg = JointTrajectory()
    msg.joint_names = list(trajectory.joint_names)
    if current_positions is not None and len(current_positions) != len(msg.joint_names):
        raise ValueError(
            f"current_positions length mismatch, expected {len(msg.joint_names)}, got {len(current_positions)}"
        )

    if not trajectory.points:
        return msg

    first_time = float(trajectory.points[0].time_from_start)
    velocity_time_scale = 1.0 / time_scale

    if current_positions is not None and lead_in_sec > 0.0:
        lead_in_point = JointTrajectoryPoint()
        lead_in_point.positions = [float(value) for value in current_positions]
        lead_in_point.velocities = [0.0] * len(current_positions)
        lead_in_point.effort = [0.0] * len(current_positions)
        lead_in_point.time_from_start = _duration_from_seconds(0.0)
        msg.points.append(lead_in_point)

    for point in trajectory.points:
        ros_point = JointTrajectoryPoint()
        ros_point.positions = list(point.positions)
        ros_point.velocities = [float(value) * velocity_time_scale for value in point.velocities]
        ros_point.effort = [0.0] * len(point.positions)
        shifted_time = max(0.0, float(point.time_from_start) - first_time)
        ros_point.time_from_start = _duration_from_seconds((shifted_time * time_scale) + lead_in_sec)
        msg.points.append(ros_point)
    return msg


def default_recorded_at() -> str:
    return datetime.now(timezone.utc).isoformat()