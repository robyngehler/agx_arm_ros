"""The terminal state this controller leaves behind, and how a stopped goal ends.

Two properties, both safety rather than feature:

- the firmware executes the last MIT command it received indefinitely, so the
  setpoint left behind on shutdown is what the arm does from then on. Every rung
  of the ladder holds the current pose; none of them may be a kp=0 command, which
  carries no stiffness and sags under gravity. Where feedback cannot place the
  arm there is no MIT hold to build, and the pose hold belongs to the MOVE-J rung
  below — so this node commands nothing rather than something weaker.
- a goal stopped by something other than its own action client is still
  EXECUTING as far as rcl is concerned, and ``canceled()`` is not a legal
  transition out of that state.
"""

import time

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from agx_arm_mit_controller.mit_controller_node import NeroMitControllerNode


JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


def _joint_state(positions: list[float]) -> JointState:
    msg = JointState()
    msg.name = list(JOINTS)
    msg.position = positions
    msg.velocity = [0.0] * len(positions)
    return msg


class _RecordingPublisher:
    def __init__(self):
        self.commands = []

    def publish(self, msg):
        self.commands.append(msg)


class _StoppedGoalHandle:
    def __init__(self, cancel_requested: bool):
        self.is_cancel_requested = cancel_requested
        self.ended = ""

    def canceled(self):
        self.ended = "canceled"

    def abort(self):
        self.ended = "aborted"


def _node():
    node = NeroMitControllerNode()
    node.move_mit_pub = _RecordingPublisher()
    node._feedback_callback(_joint_state([0.4] * 7))
    return node


# --- the terminal setpoint ---------------------------------------------------

def test_shutdown_hold_is_stiff_and_carries_the_gravity_feedforward():
    rclpy.init()
    node = _node()
    try:
        assert node._publish_stiff_hold_command()
        cmd = node.move_mit_pub.commands[-1]

        assert list(cmd.p_des) == [0.4] * 7, "the hold is at the measured pose"
        assert list(cmd.v_des) == [0.0] * 7
        assert all(value > 0.0 for value in cmd.kp), (
            "a kp=0 terminal setpoint has no stiffness and sags under gravity"
        )
        assert list(cmd.kd) == [float(v) for v in node.kd]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_shutdown_commands_nothing_when_feedback_cannot_place_the_arm():
    """No pose, no MIT hold — and no weaker command in its place.

    A hold at a synthesised pose would be a wrong hold, and a damped stop would
    be a sag. The pose hold is the driver's MOVE-J from here down.
    """
    rclpy.init()
    node = _node()
    try:
        node.last_feedback_monotonic = time.monotonic() - 1000.0
        assert not node._has_fresh_feedback()

        assert not node._publish_stiff_hold_command()
        assert node.move_mit_pub.commands == []
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# --- terminal goal transition ------------------------------------------------

def test_externally_stopped_goal_aborts_instead_of_raising():
    """``cancel_trajectory`` and the duo e-stop leave the goal EXECUTING.

    ``canceled()`` is legal only out of CANCELING. Calling it here raised out of
    the execute callback, and rclpy reported the goal aborted anyway — minus
    everything the callback still had to do.
    """
    handle = _StoppedGoalHandle(cancel_requested=False)
    NeroMitControllerNode._end_goal_stopped(handle)
    assert handle.ended == "aborted"


def test_client_cancelled_goal_still_ends_as_canceled():
    handle = _StoppedGoalHandle(cancel_requested=True)
    NeroMitControllerNode._end_goal_stopped(handle)
    assert handle.ended == "canceled"


# --- the prohibition ---------------------------------------------------------

def test_stale_feedback_publishes_no_mit_command_at_all():
    """The dead-man is an escalation, not a weaker command.

    A kp=0 zero-torque command needs no pose, which is why it used to be
    streamed here — and it has no stiffness, so it traded a runaway for a sag.
    There is no MIT command that holds without a pose, so this rung publishes
    nothing and asks the driver for MOVE-J at the current pose instead.
    """
    rclpy.init()
    node = _node()
    try:
        node.require_device_authority = False
        node._set_enabled(True)
        node.move_mit_pub.commands.clear()

        node.last_feedback_monotonic = time.monotonic() - 1000.0
        assert not node._has_fresh_feedback()
        node._control_loop()

        assert node.move_mit_pub.commands == [], (
            "a MIT command was published with no pose to hold"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_the_damped_stop_is_not_available_anywhere():
    """Removed, not merely unused: an escalation step that exists gets called."""
    for name in ("_publish_damped_stop_command", "STALE_STOP_TORQUE_RAMP_S"):
        assert not hasattr(NeroMitControllerNode, name), (
            f"{name} is still reachable on the controller"
        )


def test_freedrive_is_refused_without_a_gravity_model():
    """The only other kp=0 command is gated on the thing that stops it sagging.

    kp=0 *with* gravity is a mode, not a stop: the model carries the arm's
    weight, which is what makes it back-drivable rather than limp. Without a
    model the same command is a sag, so freedrive must be unreachable there —
    that gate is what keeps the prohibition true, not the mode's intent.
    """
    rclpy.init()
    node = _node()
    try:
        assert not node.gravity_compensation_enabled, "harness assumption"

        request = SetBool.Request()
        request.data = True
        response = node._freedrive_callback(request, SetBool.Response())

        assert not response.success
        assert "gravity" in response.message
        assert not node.freedrive_active
        assert node.move_mit_pub.commands == []
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_freedrive_carries_the_gravity_feedforward_when_it_is_allowed():
    rclpy.init()
    node = _node()
    try:
        node._compute_feedforward = lambda reference: [1.5] * 7
        node._publish_freedrive_command()
        cmd = node.move_mit_pub.commands[-1]

        assert list(cmd.kp) == [0.0] * 7
        assert all(abs(value) > 0.0 for value in cmd.torque), (
            "freedrive without gravity feedforward is a sag, not a mode"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
