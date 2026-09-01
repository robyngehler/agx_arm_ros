from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from typing import Sequence


def duration_to_seconds(duration: object) -> float:
    sec = float(getattr(duration, "sec", 0.0))
    nanosec = float(getattr(duration, "nanosec", 0.0))
    return sec + nanosec * 1e-9


@dataclass(frozen=True)
class SampledTrajectoryPoint:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]


@dataclass(frozen=True)
class _TrajectoryPoint:
    time_from_start: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]


class JointTrajectoryBuffer:
    def __init__(self, joint_names: Sequence[str], points: Sequence[_TrajectoryPoint]):
        if not joint_names:
            raise ValueError("joint_names must not be empty")
        if not points:
            raise ValueError("trajectory must contain at least one point")
        self.joint_names = tuple(joint_names)
        self._points = tuple(points)
        # Sampling runs at the control rate plus the action loop's, so it is
        # searched, not scanned: a linear walk measured 0.375 ms per call at the
        # end of a 6111-point replay against 0.059 ms at the start.
        self._times = tuple(point.time_from_start for point in self._points)

    @classmethod
    def from_ros_message(
        cls,
        expected_joint_names: Sequence[str],
        msg: object,
        *,
        allow_joint_reordering: bool = False,
        input_joint_prefix: str = "",
    ) -> "JointTrajectoryBuffer":
        joint_names = tuple(getattr(msg, "joint_names", []))
        if input_joint_prefix:
            # A prefixed bring-up (e.g. the MoveIt duo slice with
            # input_joint_prefix=right_arm_) must also accept trajectories that
            # already carry the raw controller joint names — teach-loop
            # recordings are stored as joint1..7 and replayed over the debug
            # topic against the same controller. Only names that are neither
            # prefixed nor raw are rejected.
            expected_set = set(expected_joint_names)
            unprefixed_joint_names = [
                name for name in joint_names if not name.startswith(input_joint_prefix)
            ]
            if unprefixed_joint_names and not all(name in expected_set for name in joint_names):
                raise ValueError(
                    f"joint_names mismatch, expected all names to start with "
                    f"{input_joint_prefix!r} or match the raw controller joint names, "
                    f"got {list(joint_names)}"
                )
        normalized_joint_names = tuple(
            name[len(input_joint_prefix):]
            if input_joint_prefix and name.startswith(input_joint_prefix)
            else name
            for name in joint_names
        )
        expected = tuple(expected_joint_names)
        if normalized_joint_names == expected:
            reorder_indices = tuple(range(len(expected)))
        else:
            if not allow_joint_reordering:
                raise ValueError(
                    f"joint_names mismatch, expected {list(expected_joint_names)}, got {list(joint_names)}"
                )

            if len(normalized_joint_names) != len(expected):
                raise ValueError(
                    f"joint_names mismatch, expected {list(expected_joint_names)}, got {list(joint_names)}"
                )
            if len(set(normalized_joint_names)) != len(normalized_joint_names):
                raise ValueError("trajectory joint_names contains duplicates")

            joint_index_map = {
                name: index for index, name in enumerate(normalized_joint_names)
            }
            missing_joints = [joint for joint in expected if joint not in joint_index_map]
            unknown_joints = [joint for joint in normalized_joint_names if joint not in expected]
            if missing_joints or unknown_joints:
                raise ValueError(
                    f"joint_names mismatch, expected {list(expected_joint_names)}, got {list(joint_names)}"
                )
            reorder_indices = tuple(joint_index_map[joint] for joint in expected)

        parsed_points: list[_TrajectoryPoint] = []
        last_time = -1.0
        width = len(expected_joint_names)
        for index, point in enumerate(getattr(msg, "points", [])):
            time_from_start = duration_to_seconds(getattr(point, "time_from_start", None))
            if time_from_start < 0.0:
                raise ValueError(f"point {index} has negative time_from_start")
            if time_from_start <= last_time:
                raise ValueError("trajectory point times must be strictly increasing")
            last_time = time_from_start

            positions_raw = tuple(float(value) for value in getattr(point, "positions", []))
            velocities_raw = tuple(float(value) for value in getattr(point, "velocities", []))
            efforts_raw = tuple(float(value) for value in getattr(point, "effort", []))

            if len(positions_raw) != width:
                raise ValueError(
                    f"point {index} positions length mismatch, expected {width}, got {len(positions_raw)}"
                )
            positions = tuple(positions_raw[reorder_index] for reorder_index in reorder_indices)
            velocities = (
                tuple(velocities_raw[reorder_index] for reorder_index in reorder_indices)
                if velocities_raw
                else (0.0,) * width
            )
            efforts = (
                tuple(efforts_raw[reorder_index] for reorder_index in reorder_indices)
                if efforts_raw
                else (0.0,) * width
            )

            if len(velocities) != width:
                raise ValueError(
                    f"point {index} velocities length mismatch, expected {width}, got {len(velocities)}"
                )
            if len(efforts) != width:
                raise ValueError(
                    f"point {index} effort length mismatch, expected {width}, got {len(efforts)}"
                )

            parsed_points.append(
                _TrajectoryPoint(
                    time_from_start=time_from_start,
                    positions=positions,
                    velocities=velocities,
                    efforts=efforts,
                )
            )

        return cls(joint_names=expected, points=parsed_points)

    @property
    def duration(self) -> float:
        return self._points[-1].time_from_start

    @property
    def final_point(self) -> SampledTrajectoryPoint:
        point = self._points[-1]
        return SampledTrajectoryPoint(
            positions=point.positions,
            velocities=point.velocities,
            efforts=point.efforts,
        )

    @property
    def initial_point(self) -> SampledTrajectoryPoint:
        point = self._points[0]
        return SampledTrajectoryPoint(
            positions=point.positions,
            velocities=point.velocities,
            efforts=point.efforts,
        )

    def sample(self, elapsed: float) -> SampledTrajectoryPoint:
        if elapsed <= 0.0:
            first = self._points[0]
            return SampledTrajectoryPoint(first.positions, first.velocities, first.efforts)
        if elapsed >= self.duration:
            return self.final_point

        index = bisect_left(self._times, elapsed, 1)
        if index >= len(self._points):
            return self.final_point
        previous = self._points[index - 1]
        current = self._points[index]

        interval = current.time_from_start - previous.time_from_start
        if interval <= 0.0:
            return SampledTrajectoryPoint(
                current.positions,
                current.velocities,
                current.efforts,
            )
        ratio = (elapsed - previous.time_from_start) / interval
        positions = tuple(
            start + ratio * (end - start)
            for start, end in zip(previous.positions, current.positions)
        )
        velocities = tuple(
            start + ratio * (end - start)
            for start, end in zip(previous.velocities, current.velocities)
        )
        efforts = tuple(
            start + ratio * (end - start)
            for start, end in zip(previous.efforts, current.efforts)
        )
        return SampledTrajectoryPoint(positions, velocities, efforts)