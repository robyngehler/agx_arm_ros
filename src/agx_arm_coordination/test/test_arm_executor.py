"""Unit tests for the ROS-free arm trajectory planner."""

import pytest

from agx_arm_coordination.arm_executor import (
    ArmConfig,
    ArmConfigError,
    ArmTrajectoryPlanner,
    NotTaughtError,
)
from agx_arm_coordination.graph_model import Action


CONFIG = ArmConfig.from_dict({
    "arm_executor": {
        "groups": {
            "both_arms": {
                "action_server": "both_arms_controller/follow_joint_trajectory",
                "joint_names": ["l1", "l2", "r1", "r2"],
            },
            "right_arm": {
                "action_server": "right_arm_controller/follow_joint_trajectory",
                "joint_names": ["r1", "r2"],
            },
        },
        "defaults": {"base_move_time_sec": 4.0, "min_move_time_sec": 1.5},
        "poses": {
            "A_L": [0.1, 0.2], "A_R": [0.3, 0.4],
            "B_L": [1.1, 1.2], "B_R": [1.3, 1.4],
        },
    }
})
PLANNER = ArmTrajectoryPlanner(CONFIG)


def _traj(robot_id, metadata):
    return Action(action_id="a", actiontype_id="Trajectory", robot_id=robot_id, metadata=metadata)


def test_anchor_endpoint_concatenates_pose_vectors_in_order():
    goal = PLANNER.plan(_traj("both_arms", {"to_pose": ["B_L", "B_R"], "velocity_scaling": 1.0}))
    assert goal.joint_names == ("l1", "l2", "r1", "r2")
    assert len(goal.points) == 1
    assert goal.points[0].positions == (1.1, 1.2, 1.3, 1.4)
    assert goal.action_server == "both_arms_controller/follow_joint_trajectory"


def test_velocity_scaling_slows_move_time():
    fast = PLANNER.plan(_traj("both_arms", {"to_pose": ["A_L", "A_R"], "velocity_scaling": 1.0}))
    slow = PLANNER.plan(_traj("both_arms", {"to_pose": ["A_L", "A_R"], "velocity_scaling": 0.1}))
    assert slow.points[0].time_from_start_sec > fast.points[0].time_from_start_sec


def test_move_time_floor_is_respected():
    goal = PLANNER.plan(_traj("both_arms", {"to_pose": ["A_L", "A_R"], "velocity_scaling": 100.0}))
    assert goal.points[0].time_from_start_sec == 1.5


def test_recorded_without_waypoints_raises_not_taught():
    with pytest.raises(NotTaughtError):
        PLANNER.plan(_traj("both_arms", {"source": "recorded"}))


def test_recorded_with_waypoints_is_planned():
    action = _traj("right_arm", {
        "waypoints": [
            {"positions": [0.0, 0.0], "time_from_start_sec": 1.0},
            {"positions": [0.5, 0.5], "time_from_start_sec": 2.0},
        ]
    })
    goal = PLANNER.plan(action)
    assert len(goal.points) == 2
    assert goal.points[1].positions == (0.5, 0.5)


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
