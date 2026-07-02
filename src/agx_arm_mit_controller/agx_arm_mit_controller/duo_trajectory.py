"""Merge per-arm recordings into one synchronized duo (both_arms) trajectory.

This is the linchpin for real dual-arm time-sync (see
``docs/control/teach_and_run.md``): genuine
simultaneous motion needs **one** ``both_arms`` trajectory with a single time
parameterization — two separate per-arm goals to move_group serialize. So both
the teach-side (Option B: store a duo action) and the coordinator (Option A: two
parallel-branch arm actions that must sync are merged at dispatch) build the same
14-dim trajectory through this ROS-free helper.

Each arm is captured on its own clock; this module resamples every arm onto one
common, evenly spaced time grid (linear position interpolation, last-sample hold
past a shorter arm's end) and concatenates the columns in the caller-supplied
joint order. The joint order is passed in (from the motion registry) rather than
hardcoded, so the merged vector always matches ``group_joint_names('both_arms')``
— the one correctness pitfall, since MoveIt's controller split routes columns to
arms purely by joint membership.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

from .trajectory_io import (
    RecordedTrajectory,
    default_recorded_at,
    with_finite_difference_velocities,
)


class DuoMergeError(ValueError):
    """Raised when per-arm recordings cannot be merged into a duo trajectory."""


@dataclass(frozen=True)
class ArmSegment:
    """One arm's recording plus the duo-vector joint names it fills.

    ``joint_names`` are the *target* side-prefixed names for this arm in the
    merged vector (e.g. ``right_arm_joint1..7``); their count must match the
    recording's joint count. Segments are concatenated in the order given, so
    pass them in the registry's ``both_arms`` order (left arm, then right).
    """

    recording: RecordedTrajectory
    joint_names: tuple[str, ...]


def _segment_joint_count(recording: RecordedTrajectory) -> int:
    if recording.points:
        return len(recording.points[0].positions)
    return len(recording.joint_names)


def _sample_times(recording: RecordedTrajectory) -> list[float]:
    return [float(point.time_from_start) for point in recording.points]


def interpolate_columns(
    times: Sequence[float],
    positions: Sequence[Sequence[float]],
    grid: Sequence[float],
) -> list[list[float]]:
    """Linearly interpolate multi-joint ``positions`` (sampled at ``times``) onto ``grid``.

    Queries before the first / after the last sample clamp to the endpoint value
    (last-sample hold), so an arm that finishes early simply holds its final pose
    while a longer arm keeps moving. ``times`` must be non-decreasing.
    """
    if not times:
        raise DuoMergeError("cannot interpolate an empty recording")
    joint_count = len(positions[0])
    out: list[list[float]] = []
    cursor = 0
    last_index = len(times) - 1
    for query in grid:
        if query <= times[0]:
            out.append([float(value) for value in positions[0]])
            continue
        if query >= times[last_index]:
            out.append([float(value) for value in positions[last_index]])
            continue
        # Advance the cursor to the interval [times[cursor], times[cursor + 1]]
        # that brackets `query`. grid is monotonic, so the cursor never rewinds.
        while cursor < last_index and times[cursor + 1] < query:
            cursor += 1
        t0, t1 = times[cursor], times[cursor + 1]
        span = t1 - t0
        alpha = 0.0 if span <= 0.0 else (query - t0) / span
        p0 = positions[cursor]
        p1 = positions[cursor + 1]
        out.append(
            [float(p0[j]) + alpha * (float(p1[j]) - float(p0[j])) for j in range(joint_count)]
        )
    return out


def _default_rate_hz(segments: Sequence[ArmSegment]) -> float:
    rates = [
        float(seg.recording.sample_rate_hz)
        for seg in segments
        if seg.recording.sample_rate_hz and seg.recording.sample_rate_hz > 0.0
    ]
    if rates:
        return max(rates)
    return 50.0


def _build_grid(duration: float, rate_hz: float) -> list[float]:
    if rate_hz <= 0.0:
        raise DuoMergeError("rate_hz must be > 0")
    if duration <= 0.0:
        return [0.0]
    period = 1.0 / rate_hz
    count = int(round(duration / period))
    grid = [index * period for index in range(count + 1)]
    if grid[-1] < duration:
        grid.append(duration)
    return grid


def merge_arm_recordings(
    segments: Sequence[ArmSegment],
    *,
    name: str,
    rate_hz: Optional[float] = None,
    robot: str = "duo",
    metadata: Optional[dict[str, Any]] = None,
) -> RecordedTrajectory:
    """Merge per-arm recordings into one duo trajectory on a shared time grid.

    The merged ``joint_names`` are the segments' target names concatenated in
    order; every frame carries all arms' positions at the same instant. Velocities
    are recomputed by finite difference on the common grid; efforts are zeroed
    (gravity feedforward is recomputed live at playback, never replayed).
    """
    if not segments:
        raise DuoMergeError("no arm segments to merge")

    for index, seg in enumerate(segments):
        joint_count = _segment_joint_count(seg.recording)
        if len(seg.joint_names) != joint_count:
            raise DuoMergeError(
                f"segment {index} ('{seg.recording.name}'): {len(seg.joint_names)} target "
                f"joint names but the recording has {joint_count} joints"
            )
        if not seg.recording.points:
            raise DuoMergeError(
                f"segment {index} ('{seg.recording.name}') has no recorded points"
            )

    duo_joint_names: list[str] = []
    for seg in segments:
        duo_joint_names.extend(seg.joint_names)
    if len(set(duo_joint_names)) != len(duo_joint_names):
        raise DuoMergeError(f"duo joint names contain duplicates: {duo_joint_names}")

    resolved_rate = float(rate_hz) if rate_hz else _default_rate_hz(segments)
    total_duration = max(seg.recording.duration for seg in segments)
    grid = _build_grid(total_duration, resolved_rate)

    resampled: list[list[list[float]]] = [
        interpolate_columns(_sample_times(seg.recording), [p.positions for p in seg.recording.points], grid)
        for seg in segments
    ]

    merged_positions: list[list[float]] = []
    for frame_index in range(len(grid)):
        frame: list[float] = []
        for seg_index in range(len(segments)):
            frame.extend(resampled[seg_index][frame_index])
        merged_positions.append(frame)

    joint_total = len(duo_joint_names)
    points = with_finite_difference_velocities(
        times=list(grid),
        positions=merged_positions,
        efforts=[[0.0] * joint_total for _ in grid],
    )

    payload_metadata: dict[str, Any] = {
        "recording_mode": "duo_merge",
        "merged_from": [seg.recording.name for seg in segments],
        "segment_joint_names": [list(seg.joint_names) for seg in segments],
        "segment_durations_s": [round(seg.recording.duration, 4) for seg in segments],
        "merge_rate_hz": resolved_rate,
        "merged_duration_s": round(total_duration, 4),
    }
    if metadata:
        payload_metadata.update(metadata)

    return RecordedTrajectory(
        name=name,
        robot=robot,
        joint_names=duo_joint_names,
        sample_rate_hz=resolved_rate,
        recorded_at=default_recorded_at(),
        points=points,
        metadata=payload_metadata,
    )


__all__ = ["ArmSegment", "DuoMergeError", "interpolate_columns", "merge_arm_recordings"]
