from __future__ import annotations

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

    @classmethod
    def from_ros_message(
        cls,
        expected_joint_names: Sequence[str],
        msg: object,
    ) -> "JointTrajectoryBuffer":
        joint_names = tuple(getattr(msg, "joint_names", []))
        if joint_names != tuple(expected_joint_names):
            raise ValueError(
                f"joint_names mismatch, expected {list(expected_joint_names)}, got {list(joint_names)}"
            )

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

            positions = tuple(float(value) for value in getattr(point, "positions", []))
            velocities_raw = tuple(float(value) for value in getattr(point, "velocities", []))
            efforts_raw = tuple(float(value) for value in getattr(point, "effort", []))

            if len(positions) != width:
                raise ValueError(
                    f"point {index} positions length mismatch, expected {width}, got {len(positions)}"
                )
            velocities = velocities_raw if velocities_raw else (0.0,) * width
            efforts = efforts_raw if efforts_raw else (0.0,) * width

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

        return cls(joint_names=joint_names, points=parsed_points)

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

    def sample(self, elapsed: float) -> SampledTrajectoryPoint:
        if elapsed <= 0.0:
            first = self._points[0]
            return SampledTrajectoryPoint(first.positions, first.velocities, first.efforts)
        if elapsed >= self.duration:
            return self.final_point

        previous = self._points[0]
        for current in self._points[1:]:
            if elapsed <= current.time_from_start:
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
            previous = current

        return self.final_point