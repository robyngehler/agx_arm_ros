"""Unit tests for the post-recovery fault lockout.

Guards the shared-CAN step-and-settle plan (Phase 1 item 6 / Phase 2 item 3):
after a bus recovery the driver must refuse new motion until an operator/
supervisor explicitly clears the fault, instead of silently re-arming control on
the next healthy tick. The node connects to hardware in __init__, so tests build
a bare instance via __new__ and drive the lockout state machine directly.
"""

from std_srvs.srv import Trigger

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode
from agx_arm_ctrl.device_authority import DeviceAuthority, UnitSafety


class _FakeLogger:
    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


def _node(require_ack=True):
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _FakeLogger()
    node.require_fault_ack = require_ack
    node._fault_lockout = False
    node._fault_lockout_logged = False
    node.control_ready = True
    node._control_ready_logged = True
    node._hand_window_active = False
    node._last_good_feedback_monotonic = 0.0
    node._recovery_in_progress = False
    node.enable_flag = True
    # The lockout drives the published device authority, so a bare node needs
    # one. No publisher is attached: transitions are recorded, not published.
    node._unit_safety = UnitSafety()
    node._authority = DeviceAuthority("arm_left", node._unit_safety)
    return node


def test_enter_fault_lockout_latches_and_drops_control():
    node = _node()
    node._enter_fault_lockout("bus recovery")
    assert node._fault_lockout is True
    assert node.control_ready is False


def test_check_can_control_refused_during_lockout():
    node = _node()
    node._fault_lockout = True
    assert node._check_can_control() is False


def test_lockout_is_skipped_when_ack_not_required():
    node = _node(require_ack=False)
    node._enter_fault_lockout("bus recovery")
    assert node._fault_lockout is False
    assert node.control_ready is True


def test_clear_fault_lockout_releases_motion():
    node = _node()
    node._fault_lockout = True
    resp = node._clear_fault_lockout_callback(None, Trigger.Response())
    assert resp.success is True
    assert node._fault_lockout is False
    # After clearing, _check_can_control is no longer blocked by the lockout.
    node.control_ready = True
    node._check_arm_ready = lambda: True
    node.enable_flag = True
    node.is_switch_seamlessly = True
    assert node._check_can_control() is True
