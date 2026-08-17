"""The enable path on an arm whose feedback has not started yet.

Three facts used to be one. A transport exists and can carry a command; the
firmware is pushing feedback; the joints answered the last enable request. The
enable service derived the first from the second, so on an arm that boots mute
it refused with "Agx_arm is not connected" — while a CAN session was open and
bound (measured on hardware 2026-08-17). The command that could have restored
the feedback was gated on the feedback it produces.

Splitting them costs one thing and buys another. What it must NOT do is let a
command that was merely *sent* be reported as a state that was *reached*: these
tests pin both halves, because relaxing the gate without keeping the evidence
rule would turn a deadlock into a lie.
"""

import threading

import pytest
from std_srvs.srv import SetBool

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode
from agx_arm_ctrl.sdk_worker import SdkWorker


_WORKERS: list[SdkWorker] = []


@pytest.fixture(autouse=True)
def _shut_down_workers():
    yield
    while _WORKERS:
        _WORKERS.pop().shutdown()


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warns = []
        self.errors = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warns.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class _Reporting:
    ENABLE = 1
    DISABLE = 0
    INVALID = 255


class _ModeMsg:
    class Enums:
        CanActiveMsgReporting = _Reporting

    def __init__(self):
        self.enable_can_push = _Reporting.INVALID
        self.move_mode = 0
        self.ctrl_mode = 1


class _SilentArm:
    """Accepts commands, answers no readback — the mute-arm shape.

    ``readback`` None means the low-speed feedback frame never arrived, which is
    a different fact from a frame that arrived and disagreed.
    """

    def __init__(self, *, readback=None):
        self.readback = readback
        self.commands = []
        self.push_frames = []
        self.threads = set()
        self._msg_mode = _ModeMsg()

    def _record(self):
        self.threads.add(threading.current_thread().name)

    def enable(self):
        self._record()
        self.commands.append("enable")
        return True

    def disable(self):
        self._record()
        self.commands.append("disable")
        return True

    def get_joint_enable_status(self, joint_index):
        assert joint_index == 255
        self._record()
        if self.readback is None:
            raise RuntimeError("no feedback frame yet")
        return self.readback

    def _set_mode(self):
        self._record()
        self.push_frames.append(self._msg_mode.enable_can_push)


def _node(arm, *, transport=True, enable_flag=False, verified=False) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.agx_arm = arm
    node.is_nero = True
    node.enable_flag = enable_flag
    node._enable_verified = verified
    node._transport_connected = transport
    node.control_ready = True
    node._control_ready_logged = True
    node.enable_timeout = 0.2
    node.feedback_timeout = 1.0
    node._hand_window_push_silenced = False
    node._last_good_feedback_monotonic = 0.0
    node._last_feedback_advance_monotonic = 0.0
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    node._sdk = SdkWorker("arm_test")
    _WORKERS.append(node._sdk)
    return node


def _enable(node, data: bool):
    request = SetBool.Request()
    request.data = data
    return node._enable_callback(request, SetBool.Response())


def test_missing_feedback_does_not_suppress_the_enable_command():
    """The deadlock, pinned: no feedback, and the command still goes out."""
    arm = _SilentArm(readback=None)
    node = _node(arm)

    response = _enable(node, True)

    assert arm.commands == ["enable"], "the enable request must reach the arm"
    assert response.success is False, "a sent command is not a verified state"
    assert "unknown" in response.message


def test_the_enable_path_asserts_the_feedback_push_it_needs_to_verify():
    """Verification needs feedback, so the path asks for feedback."""
    arm = _SilentArm(readback=None)
    node = _node(arm)

    _enable(node, True)

    assert _Reporting.ENABLE in arm.push_frames


def test_no_transport_is_refused_as_a_transport_problem():
    """The one honest reason to refuse: there is no session to command on."""
    arm = _SilentArm(readback=True)
    node = _node(arm, transport=False)

    response = _enable(node, True)

    assert response.success is False
    assert "no CAN transport" in response.message
    assert arm.commands == []


def test_a_verified_readback_is_what_reports_success():
    arm = _SilentArm(readback=True)
    node = _node(arm)

    response = _enable(node, True)

    assert response.success is True
    assert node.enable_flag is True
    assert node._enable_verified is True


def test_an_unverified_flag_does_not_short_circuit_the_bootstrap():
    """enable_flag alone is not evidence, so it may not skip the command.

    A mute arm whose flag was never verified would otherwise answer "already
    enabled" and never send the frame that could wake it.
    """
    arm = _SilentArm(readback=True)
    node = _node(arm, enable_flag=True, verified=False)

    response = _enable(node, True)

    assert arm.commands == ["enable"]
    assert response.success is True


def test_a_verified_flag_still_short_circuits():
    arm = _SilentArm(readback=True)
    node = _node(arm, enable_flag=True, verified=True)

    response = _enable(node, True)

    assert arm.commands == []
    assert response.success is True
    assert response.message == "Agx_arm already enabled"


def test_disable_is_attempted_without_feedback_and_closes_the_gate_first():
    """The dangerous direction: missing feedback is when stopping matters most.

    Motion admission drops the moment the request goes out, whatever the
    readback later says — but the result is still not reported as verified.
    """
    arm = _SilentArm(readback=None)
    node = _node(arm, enable_flag=True, verified=True)

    response = _enable(node, False)

    assert arm.commands == ["disable"]
    assert node.control_ready is False, "no new motion while the state is unknown"
    assert response.success is False
    assert "unknown" in response.message


def test_a_contradicted_enable_is_not_reported_as_unknown():
    """Evidence against is evidence: it must not be blurred into "no answer"."""
    arm = _SilentArm(readback=False)
    node = _node(arm)

    response = _enable(node, True)

    assert response.success is False
    assert "contradicts" in response.message
    assert node.enable_flag is False


def test_verification_is_unavailable_not_contradicted_without_a_readback():
    arm = _SilentArm(readback=None)
    node = _node(arm)

    assert node._verify_enable(True, timeout=0.1) == node.ENABLE_UNAVAILABLE
    assert node._enable_verified is False


def test_every_bootstrap_and_enable_call_reaches_the_sdk_from_the_worker():
    """§7.1 ownership: one thread touches the session, and it is the worker's.

    The enable readback used to poll ``get_joint_enable_status`` straight off
    the calling thread while the worker drove the arm — the single-owner rule
    held for the command half and not for the half that verifies it.
    """
    arm = _SilentArm(readback=True)
    node = _node(arm)

    _enable(node, True)
    node._ensure_feedback_push_enabled("ownership check", force=True)

    assert arm.threads == {node._sdk.thread_name}, (
        f"SDK reached from {arm.threads - {node._sdk.thread_name}} "
        "outside the session owner"
    )
