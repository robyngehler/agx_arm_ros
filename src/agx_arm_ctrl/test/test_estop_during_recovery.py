"""An emergency stop during recovery must not compete for a dying session.

Recovery owns the SDK session exclusively and is tearing the link down. Every
stage of the normal stop ladder would issue a call over that session, and none
of them could be verified — verification reads the same dead link. So the stop
latches locally, requests the unit stop, and says plainly that a new hardware
stop cannot be confirmed in this window.
"""

import threading

from std_srvs.srv import Trigger

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode
from agx_arm_ctrl.device_authority import DeviceAuthority, DeviceState, UnitSafety


class _Logger:
    def __init__(self):
        self.errors = []

    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, msg, *_a, **_k):
        self.errors.append(str(msg))


class _CountingArm:
    """Fails the test loudly if the stop path touches it."""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        def record(*_a, **_k):
            self.calls.append(name)
            raise AssertionError(f"stop path called the SDK during recovery: {name}")
        return record


def _node(recovering=True):
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    node.arm_type = "nero"
    node.device_id = "arm_right"
    node.agx_arm = _CountingArm()
    node._recovery_in_progress = recovering
    node._recovery_lock = threading.Lock()
    node._recovery_started_monotonic = 0.0
    node._estop_latched = False
    node._unit_safety = UnitSafety("arm_right", writer=False)
    node._authority = DeviceAuthority("arm_right", node._unit_safety)
    node._authority.go_standby("connected")
    node._authority.rearm(verified=True, detail="test")
    node.unit_stop_requests = []
    node._request_unit_stop = node.unit_stop_requests.append
    return node


def test_no_sdk_call_is_issued_while_recovery_owns_the_session():
    node = _node()
    node._emergency_stop_callback(None, Trigger.Response())
    assert node.agx_arm.calls == []


def test_the_device_is_latched_and_refuses_motion_anyway():
    node = _node()
    node._emergency_stop_callback(None, Trigger.Response())

    assert node._estop_latched is True
    assert node._authority.state is DeviceState.FAULTED
    assert not node._authority.snapshot().motion_ready


def test_the_unit_stop_is_still_requested():
    """Local protective action does not depend on the writer, and neither does
    telling the unit that a new safety era began."""
    node = _node()
    node._emergency_stop_callback(None, Trigger.Response())
    assert node.unit_stop_requests == ["emergency stop requested"]


def test_it_reports_that_a_new_stop_cannot_be_confirmed():
    node = _node()
    response = node._emergency_stop_callback(None, Trigger.Response())

    assert response.success is False, "an unverifiable stop must not report success"
    assert "unverifiable" in response.message
    assert "RECOVERING" in response.message
    assert "physical emergency stop" in response.message
    assert node.logger.errors, "this has to be loud"


class _LadderReached(Exception):
    """Marker: execution got past the recovery-window early return."""


def test_the_normal_path_is_untouched_when_recovery_is_not_running():
    """The early return must be specific to the recovery window.

    Rather than stand up the whole stop ladder, this trips a marker at its
    first step: reaching it at all is the property under test.
    """
    node = _node(recovering=False)

    def _first_step_of_the_ladder(_reason):
        raise _LadderReached

    node._restore_feedback_push = _first_step_of_the_ladder

    try:
        node._emergency_stop_callback(None, Trigger.Response())
    except _LadderReached:
        return
    raise AssertionError("the normal stop ladder did not run")
