"""Arm trajectory planning for the coordinator's performer.

``Trajectory`` actions are dispatched to the existing ``both_arms`` / per-arm
FollowJointTrajectory path (no new arm control package — graph doc §3). The
ROS-free :class:`ArmTrajectoryPlanner` turns a catalogue action + an
:class:`ArmConfig` into a concrete goal (action server + joint names + timed
waypoints); the coordinator node sends it as a ``control_msgs/FollowJointTrajectory``.

Two trajectory sources (graph doc §"Task Composition Design"):

- **anchor-pose endpoints** (``to_pose``): MoveIt-planned transitions between
  named anchor poses. The MVP commands the endpoint joint vector directly and
  lets the controller interpolate from the current pose; collision-aware MoveIt
  planning between anchors is wired in a later slice (documented limitation).
- **recorded waypoints** (``waypoints``): taught Cartesian motions (cap opener,
  pour profile). Replayed as-is. A recorded action with no taught ``waypoints``
  yet raises :class:`NotTaughtError` so the coordinator reports it cleanly.

Pose/waypoint values are placeholders until measured/taught on hardware
(proposal §9 Step 3, hefeweizen_validation_log.md).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from agx_arm_coordination.graph_model import Action


class ArmConfigError(ValueError):
    """Raised for a malformed arm config or an unmappable action."""


class NotTaughtError(RuntimeError):
    """Raised when a recorded trajectory has no taught waypoints yet."""


@dataclass(frozen=True)
class ArmGroup:
    action_server: str
    joint_names: tuple[str, ...]


@dataclass(frozen=True)
class TrajectoryPoint:
    positions: tuple[float, ...]
    time_from_start_sec: float


@dataclass(frozen=True)
class ArmGoal:
    action_id: str
    robot_id: str
    action_server: str
    joint_names: tuple[str, ...]
    points: tuple[TrajectoryPoint, ...]


@dataclass(frozen=True)
class ArmDefaults:
    base_move_time_sec: float = 4.0
    min_move_time_sec: float = 1.0


class ArmConfig:
    """Group joint sets / action servers, named anchor poses, and defaults."""

    def __init__(
        self,
        groups: dict[str, ArmGroup],
        poses: dict[str, tuple[float, ...]],
        defaults: ArmDefaults,
    ) -> None:
        self.groups = groups
        self.poses = poses
        self.defaults = defaults

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ArmConfig":
        data = (data or {}).get("arm_executor", data) or {}
        groups: dict[str, ArmGroup] = {}
        for name, spec in (data.get("groups") or {}).items():
            spec = spec or {}
            joint_names = tuple(str(j) for j in spec.get("joint_names", []))
            if not joint_names:
                # Single source of truth: derive the group's joint names from the
                # motion registry (canonical Nero joints side-prefixed per profile)
                # instead of re-listing them in arm_config.yaml.
                from agx_arm_coordination.motion_registry import group_joint_names
                joint_names = group_joint_names(str(name))
            groups[str(name)] = ArmGroup(
                action_server=str(spec.get("action_server", "")),
                joint_names=joint_names,
            )
        poses = {
            str(name): tuple(float(v) for v in vec)
            for name, vec in (data.get("poses") or {}).items()
        }
        raw_def = data.get("defaults") or {}
        defaults = ArmDefaults(
            base_move_time_sec=float(raw_def.get("base_move_time_sec", 4.0)),
            min_move_time_sec=float(raw_def.get("min_move_time_sec", 1.0)),
        )
        return cls(groups=groups, poses=poses, defaults=defaults)

    @classmethod
    def from_file(cls, path: str | Path) -> "ArmConfig":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls.from_dict(data)


class ArmTrajectoryPlanner:
    def __init__(self, config: ArmConfig) -> None:
        self.config = config

    def _group(self, robot_id: str) -> ArmGroup:
        try:
            return self.config.groups[robot_id]
        except KeyError:
            raise ArmConfigError(
                f"no arm group config for robot_id '{robot_id}'; "
                f"configured: {sorted(self.config.groups)}"
            ) from None

    def _pose_vector(self, pose_names: list[str]) -> tuple[float, ...]:
        out: list[float] = []
        for name in pose_names:
            if name not in self.config.poses:
                raise ArmConfigError(
                    f"unknown anchor pose '{name}'; configured poses: "
                    f"{sorted(self.config.poses)}"
                )
            out.extend(self.config.poses[name])
        return tuple(out)

    def _move_time(self, metadata: dict[str, Any]) -> float:
        scaling = float(metadata.get("velocity_scaling", 1.0) or 1.0)
        scaling = max(scaling, 1e-3)
        return max(
            self.config.defaults.base_move_time_sec / scaling,
            self.config.defaults.min_move_time_sec,
        )

    def plan(self, action: Action) -> ArmGoal:
        group = self._group(action.robot_id)
        metadata = action.metadata
        joint_names = group.joint_names

        if "waypoints" in metadata and metadata.get("waypoints"):
            points = self._plan_recorded(action, joint_names)
        elif metadata.get("source") == "recorded":
            raise NotTaughtError(
                f"recorded trajectory '{action.action_id}' has no taught waypoints yet "
                "(teach on hardware, then add 'waypoints' to its metadata)"
            )
        elif "to_pose" in metadata:
            points = self._plan_anchor_endpoint(action, joint_names)
        else:
            raise ArmConfigError(
                f"action '{action.action_id}' has neither 'to_pose' nor 'waypoints'"
            )

        return ArmGoal(
            action_id=action.action_id,
            robot_id=action.robot_id,
            action_server=group.action_server,
            joint_names=joint_names,
            points=points,
        )

    def _plan_anchor_endpoint(
        self, action: Action, joint_names: tuple[str, ...]
    ) -> tuple[TrajectoryPoint, ...]:
        to_pose = action.metadata["to_pose"]
        pose_names = [to_pose] if isinstance(to_pose, str) else list(to_pose)
        positions = self._pose_vector(pose_names)
        if len(positions) != len(joint_names):
            raise ArmConfigError(
                f"action '{action.action_id}': to_pose {pose_names} expands to "
                f"{len(positions)} joints, group '{action.robot_id}' has "
                f"{len(joint_names)}"
            )
        return (TrajectoryPoint(positions=positions, time_from_start_sec=self._move_time(action.metadata)),)

    def _plan_recorded(
        self, action: Action, joint_names: tuple[str, ...]
    ) -> tuple[TrajectoryPoint, ...]:
        points: list[TrajectoryPoint] = []
        for index, wp in enumerate(action.metadata["waypoints"]):
            positions = tuple(float(v) for v in wp.get("positions", []))
            if len(positions) != len(joint_names):
                raise ArmConfigError(
                    f"action '{action.action_id}' waypoint {index} has "
                    f"{len(positions)} positions, expected {len(joint_names)}"
                )
            points.append(
                TrajectoryPoint(
                    positions=positions,
                    time_from_start_sec=float(wp.get("time_from_start_sec", index + 1)),
                )
            )
        if not points:
            raise NotTaughtError(
                f"recorded trajectory '{action.action_id}' has an empty waypoint list"
            )
        return tuple(points)
