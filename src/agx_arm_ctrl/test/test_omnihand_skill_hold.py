"""The internal grasp hold watches; it must not command.

The skill controller and the FollowJointTrajectory bridge both reach the same
hand, and the bridge keeps exactly one pending target. While the hold republished
its grasp pose at the control rate, a hold tick landing during a trajectory could
replace that trajectory's target — and the action would then read the hold's
verification as its own delivery, reporting the hand arrived somewhere it never
went.

Nothing covered this path before: `_hold_tick`, `hold_internal` and
`GRASP_HOLDING` appeared in no test, which is how a 20 Hz command stream stayed
invisible. These tests pin the behaviour that replaced it — the hold observes
contact and can escalate, and issues no commands at all.
"""

from __future__ import annotations

import pytest
import rclpy

from agx_arm_ctrl.omnihand_skill_controller_node import OmniHandSkillController


class _Recorder:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)

    def __len__(self) -> int:
        return len(self.messages)


@pytest.fixture()
def controller():
    rclpy.init(args=["--ros-args", "-p", "omnihand_type:=right"])
    node = OmniHandSkillController()
    node.command_pub = _Recorder()
    node.event_pub = _Recorder()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _enter_hold(node, *, score: float = 1.0, sensors=("thumb",), on_loss="warn") -> None:
    with node._lock:
        node._holding = True
        node._hold_target = [0.4] * len(node.joint_names)
        node._hold_confirmed_score = score
        node._hold_sensors = list(sensors)
        node._hold_on_contact_loss = on_loss
        node._hold_warned = False


def test_a_hold_tick_issues_no_command(controller):
    _enter_hold(controller)

    for _ in range(50):
        controller._hold_tick()

    assert len(controller.command_pub) == 0


def test_a_tick_without_a_hold_does_nothing(controller):
    for _ in range(10):
        controller._hold_tick()

    assert len(controller.command_pub) == 0
    assert len(controller.event_pub) == 0


def test_the_hold_still_escalates_on_lost_contact(controller):
    """Removing the command stream must not remove the monitoring with it."""
    _enter_hold(controller, score=1.0, sensors=("thumb",), on_loss="abort_activity")
    # Contact reads as zero: no tactile has arrived, which is well below the
    # critical fraction of the confirmed score.
    controller._hold_tick()

    kinds = [getattr(msg, "event_type", "") for msg in controller.event_pub.messages]
    assert "contact_lost" in kinds
    assert len(controller.command_pub) == 0


def test_the_monitor_rate_is_not_the_control_rate(controller):
    """Contact is what the hold looks at, and tactile publishes at ~1 Hz.

    Ticking at the control rate re-read the same sample twenty times over.
    """
    monitor_rate = float(controller.get_parameter("hold_monitor_rate_hz").value)
    assert monitor_rate < controller.defaults.control_rate_hz
