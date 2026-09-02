"""Teach manager gripper mode: normalized in, normalized out, FJT in between.

The operator types a closure in [0, 1] and reads one back; metres never appear.
The command goes to the same FollowJointTrajectory server MoveIt and the
coordinator use, never to a bare command topic. The node needs ROS to construct,
so these build a bare instance via ``__new__`` with stub clients.
"""
from types import SimpleNamespace

import pytest

from agx_arm_coordination.gripper_closure import gripper_stroke

from agx_arm_mit_demos.teach_manager import TeachManagerNode


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warn(self, msg):
        self.lines.append(("warn", msg))

    def error(self, msg):
        self.lines.append(("error", msg))


class _StubClient:
    _action_name = "/right_arm/gripper_controller/follow_joint_trajectory"

    def __init__(self, available=True):
        self.available = available
        self.sent = []

    def wait_for_server(self, timeout_sec=None):
        return self.available

    def server_is_ready(self):
        return self.available

    def send_goal_async(self, goal):
        self.sent.append(goal)
        raise _Sent()


class _Sent(Exception):
    """Stops the test before the ROS spin; the goal is what is under test."""


def _manager(width=0.05, available=True):
    node = TeachManagerNode.__new__(TeachManagerNode)
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node.args = SimpleNamespace(service_timeout=0.1, gripper_timeout_sec=0.1)
    arm = SimpleNamespace(label="right_arm", side_prefix="right_arm_")
    node.arms = [arm]
    node.gripper_clients = {"right_arm": _StubClient(available)}
    node.gripper_status_by_arm = {
        "right_arm": None if width is None else SimpleNamespace(width=width)
    }
    node.gripper_status_monotonic = {"right_arm": 0.0}
    node.gripper_status_topics = {"right_arm": "/right_arm/feedback/gripper_status"}
    node.gripper_selected_index = 0
    return node


def _sent_goal(node, closure):
    with pytest.raises(_Sent):
        node.command_gripper_closure(closure)
    return node.gripper_clients["right_arm"].sent[0]


@pytest.mark.parametrize("closure", [0.0, 0.5, 1.0, 0.63])
def test_a_closure_becomes_prefixed_finger_positions(closure):
    node = _manager()
    goal = _sent_goal(node, closure)
    assert goal.trajectory.joint_names == [
        "right_arm_gripper_joint1", "right_arm_gripper_joint2"
    ]
    width_open, width_closed = gripper_stroke()
    expected = (width_open - closure * (width_open - width_closed)) / 2.0
    assert goal.trajectory.points[0].positions == pytest.approx(
        [expected, -expected]
    )


@pytest.mark.parametrize("bad", [-0.1, 1.5, float("nan"), float("inf")])
def test_an_out_of_range_closure_commands_nothing(bad):
    node = _manager()
    node.command_gripper_closure(bad)
    assert not node.gripper_clients["right_arm"].sent
    assert any(level == "warn" for level, _ in node._logger.lines)


def test_a_missing_server_commands_nothing_and_says_so():
    node = _manager(available=False)
    node.command_gripper_closure(0.5)
    assert not node.gripper_clients["right_arm"].sent
    assert any(level == "error" for level, _ in node._logger.lines)


# --- readback ---------------------------------------------------------------

def test_the_readback_is_shown_as_closure_not_metres():
    width_open, width_closed = gripper_stroke()
    node = _manager(width=width_open)
    assert node._gripper_closure_for("right_arm") == pytest.approx(0.0)
    node = _manager(width=width_closed)
    assert node._gripper_closure_for("right_arm") == pytest.approx(1.0)
    node = _manager(width=(width_open + width_closed) / 2.0)
    assert node._gripper_closure_for("right_arm") == pytest.approx(0.5)


def test_a_measurement_past_the_stroke_only_clamps_the_display():
    width_open, _ = gripper_stroke()
    node = _manager(width=width_open + 0.004)
    assert node._gripper_closure_for("right_arm") == 0.0


def test_without_a_readback_the_closure_is_unknown_rather_than_zero():
    node = _manager(width=None)
    assert node._gripper_closure_for("right_arm") is None


# --- discovery --------------------------------------------------------------

def test_an_arm_with_neither_status_nor_server_carries_no_gripper():
    node = _manager(width=None, available=False)
    assert node._grippers_present() == []
    assert node.selected_gripper_arm() is None


def test_a_status_sample_is_enough_to_count_as_present():
    node = _manager(width=0.05, available=False)
    assert [a.label for a in node._grippers_present()] == ["right_arm"]
