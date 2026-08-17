"""L1 tests for enable verification and Nero firmware-tier resolution.

Both are Phase 1A items. They share one shape with the e-stop and the CAN
recovery defects already fixed in this sprint: an action that cannot verify its
own effect used to report the effect anyway.
"""

import pytest
from pyAgxArm import NeroFW

from agx_arm_ctrl.agx_arm_ctrl_single_node import (
    AgxArmRosNode,
    resolve_nero_firmware,
)
from agx_arm_ctrl.sdk_worker import SdkWorker


_WORKERS: list[SdkWorker] = []


@pytest.fixture(autouse=True)
def _shut_down_workers():
    """Every worker a test starts is joined again, so none leak into the next."""
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


class _EnableArm:
    """An arm whose readback can lag, or disagree with the command outright."""

    def __init__(self, *, enabled=False, readback_after=0, readback_stuck=False):
        self.enabled = enabled
        self.readback_after = readback_after
        self.readback_stuck = readback_stuck
        self.reads = 0
        self.commands = []

    def enable(self):
        self.commands.append("enable")
        if not self.readback_stuck:
            self.enabled = True
        return True

    def disable(self):
        self.commands.append("disable")
        if not self.readback_stuck:
            self.enabled = False
        return True

    def get_joint_enable_status(self, joint_index):
        assert joint_index == 255
        self.reads += 1
        if self.reads <= self.readback_after:
            # The low-speed feedback frame still predates the command.
            return not self.enabled
        return self.enabled


def _node(arm, *, enable_flag=False) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.agx_arm = arm
    node.enable_flag = enable_flag
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    # Enable/disable reaches the SDK through the worker like every other
    # steady-state call, so the unit under test needs a real one.
    node._sdk = SdkWorker("arm_test")
    # An enable REQUEST needs a transport and nothing else — no feedback, no
    # prior enable state. That separation is what lets a mute arm be woken.
    node._transport_connected = True
    node._enable_verified = False
    node.feedback_timeout = 2.0
    _WORKERS.append(node._sdk)
    return node


def test_enable_confirmed_by_the_readback_is_reported_as_enabled():
    arm = _EnableArm()
    node = _node(arm)

    assert node._enable_arm(True, timeout=0.5) is True
    assert node.enable_flag is True


def test_an_enable_the_readback_contradicts_is_not_reported_as_success():
    arm = _EnableArm(readback_stuck=True)
    node = _node(arm)

    assert node._enable_arm(True, timeout=0.1) is False
    assert node.enable_flag is False
    assert node.logger.errors, "a contradicted enable must be logged as an error"


def test_a_failed_disable_leaves_the_arm_marked_enabled():
    """The dangerous direction: the rest of the node must not think it is off."""
    arm = _EnableArm(enabled=True, readback_stuck=True)
    node = _node(arm, enable_flag=True)

    assert node._enable_arm(False, timeout=0.1) is False
    assert node.enable_flag is True


def test_a_lagging_readback_is_given_the_remaining_budget():
    arm = _EnableArm(readback_after=2)
    node = _node(arm)

    assert node._enable_arm(True, timeout=1.0) is True
    assert arm.reads > 1
    assert node.enable_flag is True


def test_firmware_tiers_map_to_the_driver_that_speaks_them():
    assert resolve_nero_firmware("1.09")[0] == NeroFW.DEFAULT
    assert resolve_nero_firmware("1.10")[0] == NeroFW.DEFAULT
    assert resolve_nero_firmware("1.11")[0] == NeroFW.V111
    assert resolve_nero_firmware("1.12")[0] == NeroFW.V112
    assert resolve_nero_firmware("1.13")[0] == NeroFW.V112
    assert resolve_nero_firmware("2.00")[0] == NeroFW.V112


def test_a_1_12_arm_is_no_longer_driven_with_the_1_11_protocol():
    """The defect this fixes: there was no V112 branch at all."""
    tier, explanation = resolve_nero_firmware("1.12")
    assert tier == NeroFW.V112
    assert "V112" in explanation


def test_versions_are_compared_as_numbers_not_as_strings():
    """As strings, '1.9' >= '1.11' — which would claim a newer protocol tier."""
    assert "1.9" >= "1.11"
    assert resolve_nero_firmware("1.9")[0] == NeroFW.DEFAULT


def test_an_unparseable_version_falls_back_and_says_so():
    tier, explanation = resolve_nero_firmware("unknown")
    assert tier == NeroFW.DEFAULT
    assert "not parseable" in explanation
