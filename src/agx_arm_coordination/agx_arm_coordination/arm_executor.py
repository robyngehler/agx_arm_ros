"""Arm trajectory planning for the coordinator's performer.

``Trajectory`` actions are dispatched through the **MoveIt multi-arm slice**. The ROS-free
:class:`ArmTrajectoryPlanner` turns a catalogue action + an :class:`ArmConfig`
into a plan object that the coordinator node sends to MoveIt:

- **anchor-pose endpoints** (``to_pose``) -> :class:`MoveGroupPlan`: a joint-space
  goal for the group's MoveIt planning group. The coordinator sends it as a
  ``moveit_msgs/action/MoveGroup`` goal (plan + execute), so the move is
  collision-aware and MoveIt fans it out natively to the per-arm controllers.
- **recorded waypoints** (``waypoints``) -> :class:`RecordedTrajectoryPlan`: a
  taught joint trajectory the coordinator replays through
  ``moveit_msgs/action/ExecuteTrajectory`` (same controller-manager fan-out). A
  recorded action with no taught ``waypoints`` yet raises :class:`NotTaughtError`.

The planning group and the group's joint names come from the motion registry
(single source of truth) — this module never re-declares them. Pose / waypoint
values are placeholders until measured/taught on hardware.
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


class PlanMergeError(ValueError):
    """Raised when per-arm plans cannot be merged into one duo plan."""


@dataclass(frozen=True)
class ArmGroup:
    planning_group: str               # MoveIt planning group (registry moveit_group)
    joint_names: tuple[str, ...]      # registry-derived, side-prefixed


@dataclass(frozen=True)
class TrajectoryPoint:
    positions: tuple[float, ...]
    time_from_start_sec: float


@dataclass(frozen=True)
class MoveGroupPlan:
    """A collision-aware joint-space goal for one MoveIt planning group."""

    action_id: str
    robot_id: str
    planning_group: str
    joint_names: tuple[str, ...]
    target_positions: tuple[float, ...]
    velocity_scaling: float
    acceleration_scaling: float


@dataclass(frozen=True)
class RecordedTrajectoryPlan:
    """A taught joint trajectory replayed through MoveIt's ExecuteTrajectory."""

    action_id: str
    robot_id: str
    planning_group: str
    joint_names: tuple[str, ...]
    points: tuple[TrajectoryPoint, ...]


@dataclass(frozen=True)
class ArmDefaults:
    # Legacy timing knobs (kept for config compatibility); MoveIt times anchor
    # moves itself, so these no longer drive the anchor path.
    base_move_time_sec: float = 4.0
    min_move_time_sec: float = 1.0


_DEFAULT_MOVE_GROUP_ACTION = "/move_action"
_DEFAULT_EXECUTE_TRAJECTORY_ACTION = "/execute_trajectory"


class ArmConfig:
    """Arm groups (registry-derived), named anchor poses, MoveIt action names."""

    def __init__(
        self,
        groups: dict[str, ArmGroup],
        poses: dict[str, tuple[float, ...]],
        defaults: ArmDefaults,
        move_group_action: str = _DEFAULT_MOVE_GROUP_ACTION,
        execute_trajectory_action: str = _DEFAULT_EXECUTE_TRAJECTORY_ACTION,
    ) -> None:
        self.groups = groups
        self.poses = poses
        self.defaults = defaults
        self.move_group_action = move_group_action
        self.execute_trajectory_action = execute_trajectory_action

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ArmConfig":
        data = (data or {}).get("arm_executor", data) or {}

        raw_groups = data.get("groups") or []
        # groups may be a list of names or a mapping keyed by name. The planning
        # group + joint names come from the registry (single source of truth);
        # an explicit per-group planning_group / joint_names override is honoured
        # (used by unit tests so they need no installed registry).
        if isinstance(raw_groups, dict):
            items = [(str(name), spec or {}) for name, spec in raw_groups.items()]
        else:
            items = [(str(name), {}) for name in raw_groups]
        groups: dict[str, ArmGroup] = {}
        for name, spec in items:
            joint_names = tuple(str(j) for j in spec.get("joint_names", []))
            planning_group = str(spec.get("planning_group", ""))
            if not joint_names or not planning_group:
                from agx_arm_coordination.motion_registry import group_joint_names, moveit_group
                joint_names = joint_names or group_joint_names(name)
                planning_group = planning_group or moveit_group(name)
            groups[name] = ArmGroup(planning_group=planning_group, joint_names=joint_names)

        poses = {
            str(name): tuple(float(v) for v in vec)
            for name, vec in (data.get("poses") or {}).items()
        }
        raw_def = data.get("defaults") or {}
        defaults = ArmDefaults(
            base_move_time_sec=float(raw_def.get("base_move_time_sec", 4.0)),
            min_move_time_sec=float(raw_def.get("min_move_time_sec", 1.0)),
        )
        return cls(
            groups=groups,
            poses=poses,
            defaults=defaults,
            move_group_action=str(data.get("move_group_action", _DEFAULT_MOVE_GROUP_ACTION)),
            execute_trajectory_action=str(
                data.get("execute_trajectory_action", _DEFAULT_EXECUTE_TRAJECTORY_ACTION)
            ),
        )

    @classmethod
    def from_file(cls, path: str | Path) -> "ArmConfig":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls.from_dict(data)


def _scaling(metadata: dict[str, Any], key: str) -> float:
    value = float(metadata.get(key, 1.0) or 1.0)
    return min(max(value, 1e-3), 1.0)


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

    def plan(self, action: Action) -> MoveGroupPlan | RecordedTrajectoryPlan:
        group = self._group(action.robot_id)
        metadata = action.metadata

        if "waypoints" in metadata and metadata.get("waypoints"):
            return self._plan_recorded(action, group)
        if metadata.get("source") == "recorded":
            raise NotTaughtError(
                f"recorded trajectory '{action.action_id}' has no taught waypoints yet "
                "(teach on hardware, then add 'waypoints' to its metadata)"
            )
        if "to_pose" in metadata:
            return self._plan_anchor_endpoint(action, group)
        raise ArmConfigError(
            f"action '{action.action_id}' has neither 'to_pose' nor 'waypoints'"
        )

    def _plan_anchor_endpoint(self, action: Action, group: ArmGroup) -> MoveGroupPlan:
        to_pose = action.metadata["to_pose"]
        pose_names = [to_pose] if isinstance(to_pose, str) else list(to_pose)
        positions = self._pose_vector(pose_names)
        if len(positions) != len(group.joint_names):
            raise ArmConfigError(
                f"action '{action.action_id}': to_pose {pose_names} expands to "
                f"{len(positions)} joints, group '{action.robot_id}' has "
                f"{len(group.joint_names)}"
            )
        return MoveGroupPlan(
            action_id=action.action_id,
            robot_id=action.robot_id,
            planning_group=group.planning_group,
            joint_names=group.joint_names,
            target_positions=positions,
            velocity_scaling=_scaling(action.metadata, "velocity_scaling"),
            acceleration_scaling=_scaling(action.metadata, "acceleration_scaling"),
        )

    def _plan_recorded(self, action: Action, group: ArmGroup) -> RecordedTrajectoryPlan:
        points: list[TrajectoryPoint] = []
        for index, wp in enumerate(action.metadata["waypoints"]):
            positions = tuple(float(v) for v in wp.get("positions", []))
            if len(positions) != len(group.joint_names):
                raise ArmConfigError(
                    f"action '{action.action_id}' waypoint {index} has "
                    f"{len(positions)} positions, expected {len(group.joint_names)}"
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
        return RecordedTrajectoryPlan(
            action_id=action.action_id,
            robot_id=action.robot_id,
            planning_group=group.planning_group,
            joint_names=group.joint_names,
            points=tuple(points),
        )


# --- duo dispatch merge --
#
# Two separate goals to the same move_group serialize, so two parallel-branch
# per-arm actions that must run *synchronized* are merged here into ONE duo plan
# for the both_arms group. The duo group's joint order (registry) drives the
# concatenation, so the merged vector always matches what MoveIt splits back to
# the per-arm controllers.


def _interp_columns(
    times: list[float], positions: list[tuple[float, ...]], grid: list[float]
) -> list[list[float]]:
    """Linear per-joint interpolation onto ``grid`` (clamp/hold outside the ends)."""
    if not times:
        raise PlanMergeError("cannot merge a plan with no points")
    width = len(positions[0])
    last = len(times) - 1
    out: list[list[float]] = []
    cursor = 0
    for query in grid:
        if query <= times[0]:
            out.append([float(v) for v in positions[0]])
            continue
        if query >= times[last]:
            out.append([float(v) for v in positions[last]])
            continue
        while cursor < last and times[cursor + 1] < query:
            cursor += 1
        t0, t1 = times[cursor], times[cursor + 1]
        span = t1 - t0
        alpha = 0.0 if span <= 0.0 else (query - t0) / span
        p0, p1 = positions[cursor], positions[cursor + 1]
        out.append([float(p0[j]) + alpha * (float(p1[j]) - float(p0[j])) for j in range(width)])
    return out


def _order_by_group(plans: list, group: ArmGroup):
    """Order per-arm plans so their concatenated joint_names == the duo group's."""
    order = list(group.joint_names)

    def first_index(plan) -> int:
        head = plan.joint_names[0] if plan.joint_names else None
        return order.index(head) if head in order else len(order) + 1

    ordered = sorted(plans, key=first_index)
    concat = tuple(joint for plan in ordered for joint in plan.joint_names)
    if concat != tuple(group.joint_names):
        raise PlanMergeError(
            f"cannot merge: per-arm joints {concat} do not concatenate to duo group "
            f"{tuple(group.joint_names)} (order/coverage mismatch)"
        )
    return ordered


def merge_move_group_plans(plans: list, group: ArmGroup, action_id: str) -> MoveGroupPlan:
    """Merge per-arm anchor goals into one collision-aware both_arms MoveGroup goal."""
    ordered = _order_by_group(plans, group)
    positions = tuple(value for plan in ordered for value in plan.target_positions)
    return MoveGroupPlan(
        action_id=action_id,
        robot_id="both_arms",
        planning_group=group.planning_group,
        joint_names=tuple(group.joint_names),
        target_positions=positions,
        velocity_scaling=min(plan.velocity_scaling for plan in ordered),
        acceleration_scaling=min(plan.acceleration_scaling for plan in ordered),
    )


def merge_recorded_plans(plans: list, group: ArmGroup, action_id: str) -> RecordedTrajectoryPlan:
    """Merge per-arm recorded plans onto one shared timeline (both_arms group).

    Uses the union of both plans' waypoint times as the grid so the taught
    waypoints are preserved; each arm is linearly interpolated onto that grid and
    an arm that ends earlier holds its final pose while the other keeps moving.
    """
    ordered = _order_by_group(plans, group)
    for plan in ordered:
        if not plan.points:
            raise PlanMergeError(f"plan '{plan.action_id}' has no points")
    grid = sorted({point.time_from_start_sec for plan in ordered for point in plan.points})
    columns_per_plan = [
        _interp_columns(
            [point.time_from_start_sec for point in plan.points],
            [point.positions for point in plan.points],
            grid,
        )
        for plan in ordered
    ]
    points: list[TrajectoryPoint] = []
    for frame_index, time_from_start in enumerate(grid):
        merged: list[float] = []
        for columns in columns_per_plan:
            merged.extend(columns[frame_index])
        points.append(
            TrajectoryPoint(positions=tuple(merged), time_from_start_sec=time_from_start)
        )
    return RecordedTrajectoryPlan(
        action_id=action_id,
        robot_id="both_arms",
        planning_group=group.planning_group,
        joint_names=tuple(group.joint_names),
        points=tuple(points),
    )


def merge_arm_plans(plans: list, group: ArmGroup, action_id: str):
    """Merge same-type per-arm plans into one duo plan; raise on a mixed/odd set."""
    if len(plans) < 2:
        raise PlanMergeError("need at least two plans to merge")
    if all(isinstance(plan, MoveGroupPlan) for plan in plans):
        return merge_move_group_plans(plans, group, action_id)
    if all(isinstance(plan, RecordedTrajectoryPlan) for plan in plans):
        return merge_recorded_plans(plans, group, action_id)
    raise PlanMergeError(
        "cannot merge a mix of anchor (MoveGroup) and recorded (ExecuteTrajectory) plans"
    )
