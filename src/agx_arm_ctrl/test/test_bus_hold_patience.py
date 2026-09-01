"""The held-bus classifier that keeps a watchdog hold out of the fault path.

The external CAN watchdog terminates the bus, commands its own MOVE-J hold and
gives the bus back. Recovery cannot succeed against it, and the fault lockout it
latches afterwards refuses every command on the healthy bus that follows — the
arm publishes fresh feedback while nothing it is sent executes.

These tests pin the signature that separates a held bus from a broken one, and
the two ways the wait ends: the bus comes back, or the patience runs out.
"""

import time
import types

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class _Link:
    """Only the attributes the classifier reads, with the counters faked."""

    can_port = "fake_can"

    def __init__(self, rx, tx, up=True, patience=2.0):
        self.rx = rx
        self.tx = tx
        self.up = up
        self.bus_hold_patience_s = patience
        self.logs = []
        self._link_counter_files = {}
        self._last_link_rx_packets = None
        self._last_link_tx_packets = None
        self._bus_held_since_monotonic = None
        self._bus_held_logged = False

    def _link_counter(self, name):
        return self.rx if name == "rx_packets" else self.tx

    def _link_is_up(self):
        return self.up

    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda m: self.logs.append(("info", m)),
            warn=lambda m: self.logs.append(("warn", m)),
            error=lambda m: self.logs.append(("error", m)),
        )


def _defer(link):
    return AgxArmRosNode._bus_hold_defers_recovery(link)


def _baseline(link):
    """Two calls, so the classifier has a previous sample to compare against."""
    _defer(link)
    link.rx += 10
    link.tx += 10
    _defer(link)


def test_a_single_sample_is_not_evidence():
    assert _defer(_Link(100, 100)) is False


def test_an_advancing_rx_is_a_healthy_bus():
    link = _Link(100, 100)
    _baseline(link)
    link.rx += 10
    link.tx += 10
    assert _defer(link) is False


def test_rx_stopped_while_tx_is_accepted_reads_as_held():
    link = _Link(100, 100)
    _baseline(link)
    link.tx += 10  # rx frozen, tx still going out, link up
    assert _defer(link) is True


def test_the_held_bus_is_announced_once_not_every_cycle():
    link = _Link(100, 100)
    _baseline(link)
    for _ in range(5):
        link.tx += 10
        _defer(link)
    assert sum(1 for level, _ in link.logs if level == "warn") == 1


def test_patience_runs_out_and_hands_the_stall_to_the_fault_path():
    link = _Link(100, 100, patience=2.0)
    _baseline(link)
    link.tx += 10
    assert _defer(link) is True
    link._bus_held_since_monotonic = time.monotonic() - 5.0
    link.tx += 10
    assert _defer(link) is False


def test_returning_rx_ends_the_wait_and_clears_the_state():
    link = _Link(100, 100)
    _baseline(link)
    link.tx += 10
    assert _defer(link) is True
    link.rx += 1
    link.tx += 10
    assert _defer(link) is False
    assert link._bus_held_since_monotonic is None
    assert any("RX resumed" in message for _, message in link.logs)


def test_a_downed_link_is_a_fault_not_a_hold():
    link = _Link(100, 100, up=False)
    _baseline(link)
    link.tx += 10
    assert _defer(link) is False


def test_a_controller_that_stopped_accepting_tx_is_a_fault_not_a_hold():
    """Bus-off drops TX. Without that half of the signature this is a fault."""
    link = _Link(100, 100)
    _baseline(link)
    assert _defer(link) is False
