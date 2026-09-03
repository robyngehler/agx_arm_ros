"""The held-bus classifier that keeps a watchdog hold out of the fault path.

The external CAN watchdog terminates the bus, commands its own MOVE-J hold and
gives the bus back. Recovery cannot succeed against it, and the fault lockout it
latches afterwards refuses every command on the healthy bus that follows — the
arm publishes fresh feedback while nothing it is sent executes.

These tests pin the signature that separates a held bus from a broken one, the
clock the silence is measured against, and the two ways the wait ends.

The transmit half of the signature is the controller's transmit error counter,
not ``tx_packets``: an unacknowledged ONE-SHOT frame is never counted, so that
counter freezes exactly when the hold begins
(``docs/sprint_refactor/reference/bus_hold_tx_evidence.md``).
"""

import types

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class _Link:
    """Only the attributes the classifier reads, with counters and clock faked."""

    can_port = "fake_can"
    bus_hold_min_tec = 8

    def __init__(self, rx=100, tx=100, up=True, patience=2.0, min_silence=0.25,
                 tec=0):
        self.rx = rx
        self.tx = tx
        self.up = up
        self.tec = tec
        self.now = 1000.0
        self.bus_hold_patience_s = patience
        self.bus_hold_min_silence_s = min_silence
        self.logs = []
        self._link_counter_files = {}
        self._last_link_rx_packets = None
        self._last_link_tx_packets = None
        self._last_rx_advance_monotonic = None
        self._bus_held_since_monotonic = None
        self._bus_held_logged = False

    def _link_counter(self, name):
        return self.rx if name == "rx_packets" else self.tx

    def _link_is_up(self):
        return self.up

    def _link_tx_error_counter(self):
        """``None`` stands for a netlink read that did not happen."""
        return self.tec

    def _transmits_are_unacknowledged(self, previous_tx, tx):
        return AgxArmRosNode._transmits_are_unacknowledged(self, previous_tx, tx)

    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda m: self.logs.append(("info", m)),
            warn=lambda m: self.logs.append(("warn", m)),
            error=lambda m: self.logs.append(("error", m)),
        )


def _defer(link):
    """Call the classifier with the link's own fake clock."""
    import agx_arm_ctrl.agx_arm_ctrl_single_node as module

    real = module.time.monotonic
    module.time.monotonic = lambda: link.now
    try:
        return AgxArmRosNode._bus_hold_defers_recovery(link)
    finally:
        module.time.monotonic = real


def _tick(link, dt, rx_frames=2, tx_frames=14, tec=None):
    """Advance the fake clock and the counters by one publish-loop cycle."""
    link.now += dt
    link.rx += rx_frames
    link.tx += tx_frames
    if tec is not None:
        link.tec = tec


def _silence(link, cycles, *, tx_frames=0, tec=128):
    """Hold RX quiet for `cycles` publish cycles and return the last verdict."""
    verdict = False
    for _ in range(cycles):
        _tick(link, 0.005, rx_frames=0, tx_frames=tx_frames, tec=tec)
        verdict = _defer(link)
    return verdict


def test_healthy_streaming_never_reads_as_a_hold():
    """The publish loop is faster than the arm's update spacing.

    Sampled per call, "no new frame since last time" is true constantly during
    perfectly healthy streaming — which is why the silence has a clock.
    """
    link = _Link()
    for step in range(200):
        # 5 ms publish cycles, a complete arm update only every other one.
        _tick(link, 0.005, rx_frames=2 if step % 2 else 0)
        assert _defer(link) is False
    assert link._bus_held_since_monotonic is None


def test_silence_shorter_than_the_minimum_is_not_a_hold():
    link = _Link(min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 20) is False        # 0.10 s of silence


def test_silence_past_the_minimum_with_unacked_transmits_reads_as_held():
    link = _Link(min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80) is True         # 0.40 s of silence


def test_frozen_tx_packets_do_not_prevent_the_hold():
    """The measured signature of a watchdog stop.

    ``tx_packets`` stops advancing the instant the peer stops acknowledging,
    because an unacknowledged ONE-SHOT frame is abandoned and never counted.
    The host was still transmitting at ~1320/s throughout.
    """
    link = _Link(min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80, tx_frames=0, tec=128) is True


def test_a_silent_bus_with_no_transmit_errors_is_a_fault_not_a_hold():
    """Nothing says the peer is merely quiet, so the ordinary fault path runs."""
    link = _Link(min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80, tec=0) is False


def test_a_transmit_error_count_below_the_threshold_is_not_a_hold():
    """One stray error is not a peer that stopped answering."""
    link = _Link(min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80, tec=4) is False


def test_the_hold_survives_the_command_stream_being_gated_off():
    """TX stops because the gate stops it, and that must not end the hold."""
    link = _Link(min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80) is True
    # Gated: no frames at all, and no transmit-side signal left to read.
    for _ in range(40):
        _tick(link, 0.005, rx_frames=0, tx_frames=0, tec=0)
        assert _defer(link) is True


def test_the_held_bus_is_announced_once_not_every_cycle():
    link = _Link(min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    _silence(link, 120)
    assert sum(1 for level, _ in link.logs if level == "warn") == 1


def test_patience_runs_out_and_hands_the_stall_to_the_fault_path():
    link = _Link(patience=1.0, min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80) is True
    link.now += 2.0                          # past the 1 s patience
    assert _defer(link) is False


def test_returning_rx_ends_the_wait_and_clears_the_state():
    link = _Link(min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80) is True
    _tick(link, 0.005, rx_frames=2)
    assert _defer(link) is False
    assert link._bus_held_since_monotonic is None
    assert any("RX resumed" in message for _, message in link.logs)


def test_a_downed_link_is_a_fault_not_a_hold():
    link = _Link(up=False, min_silence=0.25)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80) is False


def test_without_a_netlink_read_the_packet_edge_still_holds():
    """No `ip` available: fall back to the ``tx_packets`` edge.

    Weaker — it only reports while frames are still being acknowledged — but it
    under-detects rather than over-detects.
    """
    link = _Link(min_silence=0.25, tec=None)
    _tick(link, 0.005)
    _defer(link)
    assert _silence(link, 80, tx_frames=14, tec=None) is True
