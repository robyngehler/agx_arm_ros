"""Unit tests for surfacing silently-dropped arm commands.

Guards plan section 1.3.2 / Phase 1 item 2: the forked SDK exposes a monotonic
send-error count; the driver logs a rising count while feedback looks healthy
instead of letting silent TX loss pass unnoticed. Older/mock backends without
the counter API must be a no-op.
"""

import time

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class _RecordingLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, msg, *_a, **_k):
        self.warnings.append(msg)

    def error(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


class _CountingArm:
    def __init__(self, count):
        self._count = count

    def get_send_error_count(self):
        return self._count

    def get_last_send_error(self):
        return "no buffer space available [105]"


def _node(arm):
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node.agx_arm = arm
    node._last_send_error_count = 0
    node._last_tx_loss_log = 0.0
    return node


def test_rising_send_error_count_is_logged():
    node = _node(_CountingArm(3))
    node._surface_silent_tx_loss()
    assert len(node._logger.warnings) == 1
    assert "silent TX loss" in node._logger.warnings[0]
    assert node._last_send_error_count == 3


def test_steady_send_error_count_is_not_logged():
    node = _node(_CountingArm(3))
    node._last_send_error_count = 3
    node._surface_silent_tx_loss()
    assert node._logger.warnings == []


def test_repeat_within_window_is_rate_limited():
    arm = _CountingArm(1)
    node = _node(arm)
    node._surface_silent_tx_loss()          # count 0 -> 1, logs
    arm._count = 2
    node._last_tx_loss_log = time.monotonic()  # inside the 5 s window
    node._surface_silent_tx_loss()          # count rises but is rate-limited
    assert len(node._logger.warnings) == 1
    assert node._last_send_error_count == 2  # still tracked


def test_backend_without_counter_api_is_noop():
    node = _node(object())  # no get_send_error_count
    node._surface_silent_tx_loss()  # must not raise
    assert node._logger.warnings == []


def test_contract_check_warns_when_counter_api_absent():
    # #6: the safety signal must not degrade silently — a stale pin / missing
    # fork is surfaced loudly at startup.
    node = _node(object())  # no get_send_error_count
    node._check_tx_observability_contract()
    assert any("UNAVAILABLE" in w for w in node._logger.warnings)


def test_contract_check_is_quiet_when_counter_api_present():
    node = _node(_CountingArm(0))  # fork present
    node._check_tx_observability_contract()
    assert node._logger.warnings == []
