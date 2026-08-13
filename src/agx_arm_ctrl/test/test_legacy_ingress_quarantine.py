"""The direct arm-motion topics must not move the arm unauthenticated.

`control/move_j`, `move_p`, `move_l`, `move_c`, `move_js` and the shared
`control/joint_states` follow path predate the authority contract. They carry no
commander, no device or unit generation and no sequence, so a command on them
cannot be shown to be current, and its sender cannot be shown to be entitled to
move this arm — which is what admission exists to establish.

Effector control is deliberately out of scope here: the gripper and hand are
separate devices with their own contract, and the arm's quarantine would be the
wrong boundary for them.
"""

import time

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class _Logger:
    def __init__(self):
        self.warns = []

    def info(self, *_a, **_k):
        pass

    def warn(self, msg, *_a, **_k):
        self.warns.append(str(msg))

    def error(self, *_a, **_k):
        pass


def _node(allow=False):
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.allow_legacy_motion_ingress = allow
    node._command_rejections = {}
    node._last_rejection_log_monotonic = {}
    node._rejection_log_period_s = 0.0
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    return node


def test_the_legacy_paths_are_refused_by_default():
    node = _node()
    for path in ("control/move_j", "control/move_p", "control/move_l",
                 "control/move_c", "control/move_js",
                 "control/joint_states arm follow"):
        assert node._legacy_ingress_allowed(path) is False


def test_the_refusal_names_the_path_and_says_how_to_proceed():
    node = _node()
    node._legacy_ingress_allowed("control/move_j")
    message = node.logger.warns[-1]
    assert "control/move_j" in message
    assert "no commander" in message
    assert "allow_legacy_motion_ingress" in message


def test_refusals_are_counted_per_path():
    node = _node()
    node._legacy_ingress_allowed("control/move_j")
    node._legacy_ingress_allowed("control/move_j")
    node._legacy_ingress_allowed("control/move_p")

    assert node._command_rejections[("control/move_j", "legacy_ingress")] == 2
    assert node._command_rejections[("control/move_p", "legacy_ingress")] == 1


def test_the_log_is_rate_limited_because_these_arrive_at_the_control_rate():
    node = _node()
    node._rejection_log_period_s = 60.0
    node._last_rejection_log_monotonic = {}
    for _ in range(50):
        node._legacy_ingress_allowed("control/move_j")

    assert len(node.logger.warns) == 1
    assert node._command_rejections[("control/move_j", "legacy_ingress")] == 50


def test_a_development_profile_can_still_use_them():
    node = _node(allow=True)
    assert node._legacy_ingress_allowed("control/move_j") is True
    assert node.logger.warns == []


def test_allowing_them_records_nothing_because_nothing_was_refused():
    node = _node(allow=True)
    for _ in range(10):
        node._legacy_ingress_allowed("control/move_js")
    assert node._command_rejections == {}


def test_the_first_refusal_is_logged_immediately():
    """A quarantined arm that says nothing is indistinguishable from a dead one."""
    node = _node()
    node._rejection_log_period_s = 60.0
    node._last_rejection_log_monotonic = {}
    start = time.monotonic()
    node._legacy_ingress_allowed("control/move_j")
    assert node.logger.warns, "the first refusal must not be swallowed"
    assert time.monotonic() - start < 1.0
