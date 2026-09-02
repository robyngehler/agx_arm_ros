"""The coordinator's parallel-gripper path: what it sends, and what it refuses.

A gripper action goes to the side's FollowJointTrajectory server — the same
surface MoveIt and the teach manager use — and never to the hand skill
controller or to a bare command topic. The node needs ROS to construct, so these
build a bare instance via ``__new__`` and drive the dispatch with a stub client.
"""

import pytest

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory

from agx_arm_coordination.coordinator_node import (
    CoordinatorNode,
    DispatchError,
    _GripperChild,
)
from agx_arm_coordination.graph_model import Action, GraphError
from agx_arm_coordination.gripper_closure import gripper_stroke
from agx_arm_coordination.performer import route


class _Logger:
    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


class _StubClient:
    def __init__(self, available=True):
        self.available = available
        self.sent = []

    def wait_for_server(self, timeout_sec=None):
        return self.available

    def send_goal_async(self, goal):
        self.sent.append(goal)
        return None


def _coord(available=True):
    node = CoordinatorNode.__new__(CoordinatorNode)
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node.goal_accept_timeout = 0.1
    node.gripper_action_template = (
        "/{side}_arm/gripper_controller/follow_joint_trajectory"
    )
    node.gripper_joint_template = "{side}_arm_gripper_joint"
    node._gripper_clients = {
        "left": _StubClient(available), "right": _StubClient(available)
    }
    return node


def _action(closure=0.5, robot_id="right_gripper"):
    return Action(
        action_id="grip",
        actiontype_id="Gripper",
        robot_id=robot_id,
        metadata={"target": {"closure": closure}},
    )


def test_a_closure_becomes_a_trajectory_goal_on_the_side_server():
    node = _coord()
    action = _action(closure=1.0)
    node._dispatch_gripper(7, action, route(action))

    goal = node._gripper_clients["right"].sent[0]
    assert goal.trajectory.joint_names == [
        "right_arm_gripper_joint1", "right_arm_gripper_joint2"
    ]
    _, width_closed = gripper_stroke()
    assert goal.trajectory.points[0].positions == pytest.approx(
        [width_closed / 2.0, -width_closed / 2.0]
    )
    assert not node._gripper_clients["left"].sent


def test_the_left_gripper_goes_to_the_left_server():
    node = _coord()
    action = _action(robot_id="left_gripper")
    node._dispatch_gripper(1, action, route(action))
    assert node._gripper_clients["left"].sent
    assert not node._gripper_clients["right"].sent


def test_a_missing_gripper_server_fails_the_dispatch_rather_than_the_activity():
    node = _coord(available=False)
    action = _action()
    with pytest.raises(DispatchError):
        node._dispatch_gripper(1, action, route(action))


def test_a_hand_action_never_reaches_the_gripper_path():
    hand = Action(
        action_id="open", actiontype_id="Gripper", robot_id="right_hand",
        metadata={"skill_name": "open_hand"},
    )
    assert route(hand).kind == "hand"


def test_a_trajectory_on_a_gripper_is_refused_at_load():
    """Earlier than routing: there is no path that takes a trajectory on two jaws."""
    with pytest.raises(GraphError, match="executes only Gripper"):
        Action(
            action_id="x", actiontype_id="Trajectory", robot_id="right_gripper",
            metadata={},
        )


# --- how the child reads a result -------------------------------------------

class _Wrapper:
    def __init__(self, status, error_code, error_string=""):
        self.status = status
        self.result = FollowJointTrajectory.Result()
        self.result.error_code = error_code
        self.result.error_string = error_string


def _child_result(wrapper):
    child = _GripperChild(1, "grip")
    child._interpret_result(wrapper)
    return child


def test_a_succeeded_goal_completes_the_action():
    child = _child_result(_Wrapper(
        GoalStatus.STATUS_SUCCEEDED, FollowJointTrajectory.Result.SUCCESSFUL,
        "settled at 0.0300 m",
    ))
    assert child.success


def test_a_canceled_goal_is_a_failure_not_a_completion():
    """It comes back with a SUCCESSFUL error_code; the status is what decides."""
    child = _child_result(_Wrapper(
        GoalStatus.STATUS_CANCELED, FollowJointTrajectory.Result.SUCCESSFUL,
        "canceled: cancel requested during travel",
    ))
    assert not child.success
    assert "canceled" in child.message


def test_a_tolerance_violation_fails_and_carries_the_reason():
    child = _child_result(_Wrapper(
        GoalStatus.STATUS_SUCCEEDED,
        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
        "gripper goal failed (no_progress): width still 0.0500 m",
    ))
    assert not child.success
    assert "no_progress" in child.message
