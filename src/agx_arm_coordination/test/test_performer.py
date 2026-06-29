"""Unit tests for the ROS-free performer routing."""

import pytest

from agx_arm_coordination.graph_model import Action
from agx_arm_coordination.performer import KIND_ARM, KIND_HAND, RoutingError, route


def test_gripper_left_hand_routes_to_hand_left():
    decision = route(Action("a", "Gripper", "left_hand"))
    assert decision.kind == KIND_HAND
    assert decision.side == "left"


def test_gripper_right_hand_routes_to_hand_right():
    decision = route(Action("a", "Gripper", "right_hand"))
    assert decision.kind == KIND_HAND
    assert decision.side == "right"


def test_trajectory_both_arms_routes_to_arm():
    decision = route(Action("a", "Trajectory", "both_arms"))
    assert decision.kind == KIND_ARM
    assert decision.robot_id == "both_arms"


def test_trajectory_per_arm_routes_to_arm():
    assert route(Action("a", "Trajectory", "left_arm")).kind == KIND_ARM
    assert route(Action("a", "Trajectory", "right_arm")).kind == KIND_ARM


def test_mismatched_actiontype_and_robot_raises():
    # Gripper on an arm, or Trajectory on a hand, has no route.
    with pytest.raises(RoutingError):
        route(Action("a", "Gripper", "both_arms"))
    with pytest.raises(RoutingError):
        route(Action("a", "Trajectory", "left_hand"))
