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

import math
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
    #: Feedforward for this point. Empty when the plan carries bare waypoints, in
    #: which case the dispatcher derives it — the MIT controller reads a missing
    #: velocity as a commanded zero and brakes against its own position command.
    velocities: tuple[float, ...] = ()


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
        pose_robot_ids: dict[str, str] | None = None,
    ) -> None:
        self.groups = groups
        self.poses = poses
        # Explicit robot_id per pose (both_arms/left_arm/right_arm). Empty for a
        # legacy bare-list pose, whose side is then inferred from an _L/_R suffix.
        self.pose_robot_ids = pose_robot_ids or {}
        self.defaults = defaults
        self.move_group_action = move_group_action
        self.execute_trajectory_action = execute_trajectory_action

    def pose_robot_id(self, name: str) -> str:
        """Resource a pose belongs to: explicit ``robot_id``, else _L/_R suffix.

        The stored ``robot_id`` is authoritative; the suffix is only a fallback
        for legacy bare-list poses so old configs keep resolving.
        """
        explicit = self.pose_robot_ids.get(name, "")
        if explicit:
            return explicit
        if name.endswith("_L"):
            return "left_arm"
        if name.endswith("_R"):
            return "right_arm"
        return ""

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

        # Each pose is either a bare list (legacy, side from _L/_R suffix) or a
        # mapping {robot_id: <both_arms|left_arm|right_arm>, q: [...]} that stores
        # the resource explicitly (chosen at capture time, so it survives renames
        # and lets both_arms be one 14-DoF entry rather than a paired _L/_R hack).
        poses: dict[str, tuple[float, ...]] = {}
        pose_robot_ids: dict[str, str] = {}
        for name, entry in (data.get("poses") or {}).items():
            name = str(name)
            if isinstance(entry, dict):
                vec = entry.get("q", entry.get("joints", []))
                poses[name] = tuple(float(v) for v in vec)
                pose_robot_ids[name] = str(entry.get("robot_id", ""))
            else:
                poses[name] = tuple(float(v) for v in entry)
                pose_robot_ids[name] = ""
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
            pose_robot_ids=pose_robot_ids,
        )

    @classmethod
    def from_file(cls, path: str | Path) -> "ArmConfig":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls.from_dict(data)


#: How a taught trajectory is turned into commands when the action does not say.
#: `smooth` because a replay has to execute, and 0.3 s because that is the window
#: the hardware session settled on (`docs/sprint_refactor/reference/teach_replay_timebase.md`).
PLAYBACK_DEFAULT_MODE = "smooth"
PLAYBACK_DEFAULT_WINDOW_SEC = 0.3
#: Output grid. The MIT controller runs at 200 Hz and interpolates linearly, so a
#: 5 ms knot lands on a control tick and nothing is interpolated; 10 ms doubled
#: the peak commanded acceleration (11.2 -> 20.3 rad/s²).
PLAYBACK_RESAMPLE_DT = 0.005


#: Keys a ``playback`` block may carry.
PLAYBACK_KEYS = ("mode", "smoothing_window_sec", "speed_scale", "resample_dt")
#: Timing knobs that predate the playback block. They stretch the taught times
#: before the mode ever sees them, so on a recorded action they are deprecated
#: and cannot be combined with an explicit ``playback`` block.
LEGACY_TIMING_KEYS = ("velocity_scaling", "acceleration_scaling")


@dataclass(frozen=True)
class PlaybackSpec:
    """How a recorded action is re-timed for playback.

    Read from the action's ``metadata['playback']``, the same place a grip action
    declares ``payload_update``: an Action-level default, not a property of the
    recording and not of one node in one activity. A run may override it
    activity-wide through the ``PerformActivity`` goal's ``metadata_json``.
    """

    mode: str = PLAYBACK_DEFAULT_MODE
    smoothing_window_sec: float = PLAYBACK_DEFAULT_WINDOW_SEC
    speed_scale: float = 1.0
    resample_dt: float = PLAYBACK_RESAMPLE_DT

    def cache_key(self) -> tuple:
        return (self.mode, self.smoothing_window_sec, self.speed_scale, self.resample_dt)


def playback_spec(metadata: dict[str, Any], action_id: str = "") -> PlaybackSpec:
    """Parse ``metadata['playback']``, defaulting to smooth playback.

    Rejects an unusable request rather than falling back to the default: a replay
    that silently ran under a different mode than the activity asked for is worse
    than one that refused to start.
    """
    from agx_arm_retiming import MODES

    raw = metadata.get("playback") or {}
    if not isinstance(raw, dict):
        raise ArmConfigError(
            f"action '{action_id}': 'playback' must be a mapping, got {type(raw).__name__}"
        )
    unknown = set(raw) - set(PLAYBACK_KEYS)
    if unknown:
        raise ArmConfigError(
            f"action '{action_id}': unknown playback key(s) {sorted(unknown)}; "
            f"expected {', '.join(PLAYBACK_KEYS)}"
        )
    mode = str(raw.get("mode", PLAYBACK_DEFAULT_MODE))
    if mode not in MODES:
        raise ArmConfigError(
            f"action '{action_id}': unknown playback mode '{mode}'; "
            f"expected one of {', '.join(MODES)}"
        )

    def number(key: str, default: float, *, allow_zero: bool = False) -> float:
        try:
            value = float(raw.get(key, default))
        except (TypeError, ValueError):
            raise ArmConfigError(
                f"action '{action_id}': playback {key} must be a number, "
                f"got {raw.get(key)!r}"
            ) from None
        bound = ">= 0" if allow_zero else "> 0"
        if not math.isfinite(value) or value < 0.0 or (value == 0.0 and not allow_zero):
            raise ArmConfigError(
                f"action '{action_id}': playback {key} must be finite and "
                f"{bound}, got {value}"
            )
        return value

    return PlaybackSpec(
        mode=mode,
        # Zero is usable and means "no window beyond the reconstruction floor
        # every timing-preserving mode applies anyway".
        smoothing_window_sec=number(
            "smoothing_window_sec", PLAYBACK_DEFAULT_WINDOW_SEC, allow_zero=True
        ),
        speed_scale=number("speed_scale", 1.0),
        resample_dt=number("resample_dt", PLAYBACK_RESAMPLE_DT),
    )


def _shares_one_prefix(expected: tuple[str, ...], recorded: tuple[str, ...]) -> bool:
    """Report whether each group joint is one shared prefix plus its recorded name.

    One prefix for all of them, in the group's order — so `joint1..7` matches
    `left_arm_joint1..7`, and a duo group's two different prefixes do not match a
    single arm's names repeated.
    """
    if len(expected) != len(recorded) or not recorded:
        return False
    prefixes = set()
    for name, short in zip(expected, recorded):
        if len(name) <= len(short) or not name.endswith(short):
            return False
        prefixes.add(name[: len(name) - len(short)])
    return len(prefixes) == 1


def _scaling(metadata: dict[str, Any], key: str) -> float:
    value = float(metadata.get(key, 1.0) or 1.0)
    return min(max(value, 1e-3), 1.0)


def _recorded_time_scale(
    metadata: dict[str, Any], action_id: str, explicit_playback: bool
) -> float:
    """Stretch recorded waypoint timing by the more conservative metadata scale.

    Deprecated on a recorded action: it scales the taught times before the mode
    sees them, so combined with a `playback` block the timing is scaled twice —
    `tempo_scale: 0.5` under `velocity_scaling: 0.5` runs at a quarter speed and
    neither number says so. `playback` is the single authority, and the pair is
    refused rather than multiplied.
    """
    declared = [
        key for key in LEGACY_TIMING_KEYS
        if key in metadata and float(metadata[key] or 1.0) != 1.0
    ]
    if explicit_playback and declared:
        raise ArmConfigError(
            f"action '{action_id}': {' and '.join(declared)} cannot be combined "
            f"with an explicit 'playback' block — both scale the taught timing, "
            f"so the replay would run at the product of the two. Express the "
            f"speed as playback.speed_scale under mode 'tempo_scale' or "
            f"'speed_scale' and drop {' and '.join(declared)}"
        )
    return 1.0 / min(
        _scaling(metadata, "velocity_scaling"),
        _scaling(metadata, "acceleration_scaling"),
    )


def is_replay(action: Action) -> bool:
    """Whether this action replays a taught path rather than planning a move.

    The same three-way split :meth:`ArmTrajectoryPlanner.plan` makes, exposed so
    a caller can ask before planning. A resume needs it: a replay commands taught
    joint angles from wherever the arm stands, an anchor move plans from the
    current state. An action declared ``recorded`` but not yet taught counts as a
    replay — it is one, it just cannot run yet.
    """
    metadata = action.metadata or {}
    return bool(metadata.get("waypoints")) or metadata.get("source") == "recorded"


class ArmTrajectoryPlanner:
    def __init__(self, config: ArmConfig) -> None:
        self.config = config
        # Retimed command streams, keyed on the action and its playback spec.
        self._retimed: dict[tuple, tuple[TrajectoryPoint, ...]] = {}
        # Run-level playback override, merged over each action's own block. Set
        # per activity run; the catalogue keeps what the action normally does.
        self.playback_override: dict[str, Any] = {}

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

        if metadata.get("waypoints"):
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

    def playback_for(self, action: Action) -> tuple[PlaybackSpec, bool]:
        """Resolve how this action replays, and whether a mode was asked for.

        Three levels, most specific first: the run-level override from the
        activity goal, the action's own block, then the default. The override is
        merged key by key, so a run can change only the tempo and leave the mode
        the catalogue chose.
        """
        merged = dict(action.metadata.get("playback") or {})
        merged.update(self.playback_override)
        return playback_spec({"playback": merged}, action.action_id), bool(merged)

    def _assert_recorded_joints(self, action: Action, group: ArmGroup) -> None:
        """Refuse a recording taught on different joints than the group commands.

        Matching the joint *count* is not enough: a left-arm recording is the
        right shape for the right arm and would replay a mirrored path onto it,
        and a duo recording concatenated in the other side order is the right
        shape for `both_arms`.

        Two spellings are accepted. Names equal to the group's are an exact
        match. Names the group's own carry a single shared prefix over
        (``joint1`` against ``left_arm_joint1``) are the side-prefix convention:
        a single-arm teach recording stores unprefixed joint names, so it names
        the joints and their order while the catalogue names the side. That side
        is then the catalogue's claim and is not checked — emit the sidecar with
        `--joint-prefix` to make it the recording's claim too.
        """
        recorded = action.metadata.get("recording_joint_names")
        if not recorded:
            return
        recorded = tuple(str(name) for name in recorded)
        expected = tuple(group.joint_names)
        if recorded == expected or _shares_one_prefix(expected, recorded):
            return
        raise ArmConfigError(
            f"action '{action.action_id}': recording was taught on "
            f"{list(recorded)}, group '{action.robot_id}' commands "
            f"{list(expected)}; refusing to replay it"
        )

    def _plan_recorded(self, action: Action, group: ArmGroup) -> RecordedTrajectoryPlan:
        self._assert_recorded_joints(action, group)
        spec, explicit = self.playback_for(action)
        time_scale = _recorded_time_scale(action.metadata, action.action_id, explicit)
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
                    time_from_start_sec=(
                        float(wp.get("time_from_start_sec", index + 1)) * time_scale
                    ),
                )
            )
        if not points:
            raise NotTaughtError(
                f"recorded trajectory '{action.action_id}' has an empty waypoint list"
            )
        points = self._retime(action, group, points, spec, time_scale)
        return RecordedTrajectoryPlan(
            action_id=action.action_id,
            robot_id=action.robot_id,
            planning_group=group.planning_group,
            joint_names=group.joint_names,
            points=tuple(points),
        )

    def _retime(self, action: Action, group: ArmGroup, points: list[TrajectoryPoint],
                spec: PlaybackSpec, time_scale: float) -> list[TrajectoryPoint]:
        """Turn taught waypoints into a command stream under the action's mode.

        Cached on everything that determines the output — the action, the spec
        and the legacy time scale applied before it: `speed_scale` searches the
        limit scale and measured 11 s on a 30 s duo recording, which is a stall
        if it happens while the activity is already running.

        Four waypoints are the minimum the retiming needs; below that the taught
        points are dispatched as they are, with velocities derived downstream.
        """
        key = (action.action_id, spec.cache_key(), len(points), time_scale)
        cached = self._retimed.get(key)
        if cached is not None:
            return list(cached)
        if len(points) < 4:
            return points

        from agx_arm_retiming import NERO_MAX_VELOCITY, RetimingError, default_acceleration, retime

        arms = max(1, len(group.joint_names) // len(NERO_MAX_VELOCITY))
        max_velocity = list(NERO_MAX_VELOCITY) * arms
        if len(max_velocity) != len(group.joint_names):
            # A group this planner has no joint limits for: dispatch it as taught
            # rather than inventing a limit for it.
            return points
        try:
            result = retime(
                [point.time_from_start_sec for point in points],
                [list(point.positions) for point in points],
                spec.mode,
                max_velocity=max_velocity,
                max_acceleration=default_acceleration(max_velocity),
                speed_scale=spec.speed_scale,
                smoothing_window_sec=spec.smoothing_window_sec,
                resample_dt=spec.resample_dt,
            )
        except RetimingError as exc:
            raise ArmConfigError(
                f"action '{action.action_id}': cannot replay under mode "
                f"'{spec.mode}': {exc}"
            ) from None

        retimed = [
            TrajectoryPoint(
                positions=tuple(position),
                time_from_start_sec=float(when),
                velocities=tuple(velocity),
            )
            for when, position, velocity in zip(result.times, result.positions, result.velocities)
        ]
        self._retimed[key] = tuple(retimed)
        return retimed


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
