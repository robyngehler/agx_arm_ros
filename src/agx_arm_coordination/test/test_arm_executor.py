"""Unit tests for the ROS-free arm trajectory planner (MoveIt plans)."""

import pytest

from agx_arm_coordination.arm_executor import (
    ArmConfig,
    ArmConfigError,
    ArmGroup,
    ArmTrajectoryPlanner,
    MoveGroupPlan,
    NotTaughtError,
    PlanMergeError,
    RecordedTrajectoryPlan,
    TrajectoryPoint,
    merge_arm_plans,
)
from agx_arm_coordination.graph_model import Action


# Explicit planning_group + joint_names so the unit tests need no installed registry.
CONFIG = ArmConfig.from_dict({
    "arm_executor": {
        "move_group_action": "/move_action",
        "execute_trajectory_action": "/execute_trajectory",
        "groups": {
            "both_arms": {"planning_group": "both_arms", "joint_names": ["l1", "l2", "r1", "r2"]},
            "right_arm": {"planning_group": "right_arm", "joint_names": ["r1", "r2"]},
        },
        "poses": {
            "A_L": [0.1, 0.2], "A_R": [0.3, 0.4],
            "B_L": [1.1, 1.2], "B_R": [1.3, 1.4],
        },
    }
})
PLANNER = ArmTrajectoryPlanner(CONFIG)


def _traj(robot_id, metadata):
    return Action(action_id="a", actiontype_id="Trajectory", robot_id=robot_id, metadata=metadata)


def test_anchor_endpoint_builds_movegroup_plan_in_order():
    plan = PLANNER.plan(_traj("both_arms", {"to_pose": ["B_L", "B_R"], "velocity_scaling": 1.0}))
    assert isinstance(plan, MoveGroupPlan)
    assert plan.planning_group == "both_arms"
    assert plan.joint_names == ("l1", "l2", "r1", "r2")
    assert plan.target_positions == (1.1, 1.2, 1.3, 1.4)


def test_scaling_factors_are_clamped_to_unit_interval():
    plan = PLANNER.plan(_traj("both_arms", {
        "to_pose": ["A_L", "A_R"], "velocity_scaling": 0.1, "acceleration_scaling": 5.0,
    }))
    assert plan.velocity_scaling == pytest.approx(0.1)
    assert plan.acceleration_scaling == 1.0  # clamped from 5.0


def test_recorded_without_waypoints_raises_not_taught():
    with pytest.raises(NotTaughtError):
        PLANNER.plan(_traj("both_arms", {"source": "recorded"}))


def test_recorded_with_waypoints_builds_execute_trajectory_plan():
    plan = PLANNER.plan(_traj("right_arm", {
        "waypoints": [
            {"positions": [0.0, 0.0], "time_from_start_sec": 1.0},
            {"positions": [0.5, 0.5], "time_from_start_sec": 2.0},
        ]
    }))
    assert isinstance(plan, RecordedTrajectoryPlan)
    assert plan.planning_group == "right_arm"
    assert len(plan.points) == 2
    assert plan.points[1].positions == (0.5, 0.5)


def test_recorded_waypoints_honor_scaling_metadata_by_stretching_time():
    plan = PLANNER.plan(_traj("right_arm", {
        "waypoints": [
            {"positions": [0.0, 0.0], "time_from_start_sec": 1.0},
            {"positions": [0.5, 0.5], "time_from_start_sec": 2.0},
        ],
        "velocity_scaling": 0.25,
        "acceleration_scaling": 0.5,
    }))

    assert isinstance(plan, RecordedTrajectoryPlan)
    assert [point.time_from_start_sec for point in plan.points] == pytest.approx([4.0, 8.0])


def test_pose_length_mismatch_raises():
    with pytest.raises(ArmConfigError):
        # to_pose expands to 2 joints for a 4-joint group
        PLANNER.plan(_traj("both_arms", {"to_pose": ["A_L"]}))


def test_unknown_group_raises():
    with pytest.raises(ArmConfigError):
        PLANNER.plan(_traj("left_arm", {"to_pose": ["A_L"]}))


def test_neither_to_pose_nor_waypoints_raises():
    with pytest.raises(ArmConfigError):
        PLANNER.plan(_traj("both_arms", {"velocity_scaling": 0.1}))


# --- dispatch merge (Case 4): two synced per-arm plans -> one both_arms plan ---

_BOTH = ArmGroup(planning_group="both_arms", joint_names=("l1", "l2", "r1", "r2"))


def _mg(robot_id, joints, positions):
    return MoveGroupPlan(
        action_id=robot_id, robot_id=robot_id, planning_group=robot_id,
        joint_names=joints, target_positions=positions,
        velocity_scaling=1.0, acceleration_scaling=1.0,
    )


def _rec(robot_id, joints, frames):
    return RecordedTrajectoryPlan(
        action_id=robot_id, robot_id=robot_id, planning_group=robot_id, joint_names=joints,
        points=tuple(TrajectoryPoint(positions=p, time_from_start_sec=t) for t, p in frames),
    )


def test_merge_move_group_orders_by_group_regardless_of_input_order():
    left = _mg("left_arm", ("l1", "l2"), (0.1, 0.2))
    right = _mg("right_arm", ("r1", "r2"), (0.3, 0.4))
    merged = merge_arm_plans([right, left], _BOTH, "sync")  # right first on purpose
    assert isinstance(merged, MoveGroupPlan)
    assert merged.robot_id == "both_arms"
    assert merged.joint_names == ("l1", "l2", "r1", "r2")
    assert merged.target_positions == (0.1, 0.2, 0.3, 0.4)


def test_merge_recorded_uses_union_timeline_and_holds_short_arm():
    left = _rec("left_arm", ("l1", "l2"), [(0.0, (0.0, 0.0)), (1.0, (1.0, 1.0))])
    right = _rec("right_arm", ("r1", "r2"), [(0.0, (0.0, 0.0)), (2.0, (2.0, 2.0))])
    merged = merge_arm_plans([left, right], _BOTH, "sync")
    assert isinstance(merged, RecordedTrajectoryPlan)
    assert merged.joint_names == ("l1", "l2", "r1", "r2")
    times = [p.time_from_start_sec for p in merged.points]
    assert times == [0.0, 1.0, 2.0]  # union of both arms' waypoint times
    # at t=2 the left arm holds its final (1,1); right reached (2,2)
    assert merged.points[-1].positions == pytest.approx((1.0, 1.0, 2.0, 2.0))
    # at t=1 right is halfway (1,1)
    assert merged.points[1].positions == pytest.approx((1.0, 1.0, 1.0, 1.0))


def test_merge_rejects_mixed_plan_types():
    left = _mg("left_arm", ("l1", "l2"), (0.1, 0.2))
    right = _rec("right_arm", ("r1", "r2"), [(0.0, (0.0, 0.0))])
    with pytest.raises(PlanMergeError):
        merge_arm_plans([left, right], _BOTH, "sync")


def test_merge_rejects_coverage_mismatch():
    left = _mg("left_arm", ("l1", "l2"), (0.1, 0.2))
    dup = _mg("left_arm", ("l1", "l2"), (0.3, 0.4))
    with pytest.raises(PlanMergeError):
        merge_arm_plans([left, dup], _BOTH, "sync")
