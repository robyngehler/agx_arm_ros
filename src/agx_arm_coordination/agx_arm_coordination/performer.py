"""Performer routing — which executor runs a given catalogue action.

Coordinator-internal: the coordinator
calls :func:`route` to decide whether an action is a hand skill (dispatched as a
``PerformAction`` goal to the matching ``omnihand_skill_controller``) or an arm
trajectory (dispatched to the MoveIt multi-arm slice via MoveGroup /
ExecuteTrajectory). The routing decision itself is ROS-free so it can be
unit-tested; the coordinator node turns the decision into the actual ROS goal.
"""

from __future__ import annotations

from dataclasses import dataclass

from agx_arm_coordination.graph_model import (
    ACTIONTYPE_GRIPPER,
    ACTIONTYPE_TRAJECTORY,
    Action,
)
from agx_arm_coordination.gripper_closure import gripper_side

KIND_HAND = "hand"
KIND_ARM = "arm"
KIND_GRIPPER = "gripper"

_HAND_ROBOTS = ("left_hand", "right_hand")
_ARM_ROBOTS = ("both_arms", "left_arm", "right_arm")


class RoutingError(ValueError):
    """Raised when an action cannot be routed to any executor."""


@dataclass(frozen=True)
class RoutingDecision:
    kind: str          # KIND_HAND | KIND_ARM | KIND_GRIPPER
    robot_id: str
    side: str = ""     # "left"/"right" for hands and grippers; "" for arms


def route(action: Action) -> RoutingDecision:
    """Map a catalogue action to its executor.

    ``Gripper`` + ``{left,right}_hand`` -> hand skill controller;
    ``Gripper`` + ``{left,right}_gripper`` -> the parallel gripper's own
    FollowJointTrajectory server, never the hand skill controller: a five-finger
    hand ending on tactile contact and a two-jaw gripper closing to a width do
    not share a command shape;
    ``Trajectory`` + ``{both_arms,left_arm,right_arm}`` -> arm FJT executor.
    Anything else is a routing error (caught by the coordinator as a structured
    failure rather than a crash).
    """
    if action.actiontype_id == ACTIONTYPE_GRIPPER and action.robot_id in _HAND_ROBOTS:
        side = "left" if action.robot_id == "left_hand" else "right"
        return RoutingDecision(kind=KIND_HAND, robot_id=action.robot_id, side=side)
    gripper_side_name = gripper_side(action.robot_id)
    if action.actiontype_id == ACTIONTYPE_GRIPPER and gripper_side_name:
        return RoutingDecision(
            kind=KIND_GRIPPER, robot_id=action.robot_id, side=gripper_side_name
        )
    if action.actiontype_id == ACTIONTYPE_TRAJECTORY and action.robot_id in _ARM_ROBOTS:
        return RoutingDecision(kind=KIND_ARM, robot_id=action.robot_id)
    raise RoutingError(
        f"action '{action.action_id}' "
        f"({action.actiontype_id}/{action.robot_id}) has no executor route"
    )
