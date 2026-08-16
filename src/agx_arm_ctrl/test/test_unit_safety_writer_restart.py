"""L1 tests for unit-safety semantics across a writer restart.

The writer holds its generation counter in memory and starts from zero, so an
epoch only means something inside one run of that process. Before the
incarnation model, an observer that had reached generation 5 silently discarded
everything a restarted writer published until it climbed back past 5 — the unit
could not be told that a new safety era had begun, and the symptom was silence
rather than an error.
"""

from agx_arm_ctrl.device_authority import (
    DeviceAuthority,
    DeviceState,
    UnitSafety,
    UnitSafetySnapshot,
)


def _writer(name="unit_safety", incarnation="run-a", started_ns=1_000):
    return UnitSafety(name, incarnation=incarnation, started_ns=started_ns)


def _observer(device_id="arm_left"):
    """An observer plus a device that follows it, as a driver process wires it."""
    safety = UnitSafety(device_id, writer=False)
    device = DeviceAuthority(device_id, safety)
    device.go_standby("connected")
    return safety, device


def test_first_observation_is_adopted_and_does_not_fail_closed():
    """A cold boot must not demand an operator rearm before anything can move."""
    safety, device = _observer()
    writer = _writer()

    assert safety.observe(writer.snapshot())
    assert not safety.stopped
    assert device.state is DeviceState.STANDBY
    assert safety.incarnation_changes == 0


def test_a_restarted_writer_is_accepted_rather_than_discarded_as_stale():
    """The defect this model exists to remove: low epochs read as stale."""
    safety, device = _observer()
    writer = _writer()
    safety.observe(writer.snapshot())

    for i in range(3):
        writer.stop(f"stop {i}")
        safety.observe(writer.snapshot())
        writer.rearm(f"rearm {i}")
        safety.observe(writer.snapshot())
    assert safety.epoch == 6
    assert not safety.stopped

    # The writer restarts: fresh incarnation, generation back to zero.
    restarted = _writer(incarnation="run-b", started_ns=2_000)
    assert restarted.epoch == 0

    assert safety.observe(restarted.snapshot()), (
        "a generation from the restarted writer was discarded as stale; the unit "
        "cannot be told that a new safety era began"
    )
    assert safety.incarnation_changes == 1


def test_a_writer_restart_holds_the_unit_stopped_rather_than_re_enabling_motion():
    """The restarted writer says 'not stopped'; it cannot vouch for that."""
    safety, device = _observer()
    writer = _writer()
    safety.observe(writer.snapshot())
    assert device.state is DeviceState.STANDBY

    restarted = _writer(incarnation="run-b", started_ns=2_000)
    assert not restarted.stopped  # a fresh writer believes all is well

    safety.observe(restarted.snapshot())
    assert safety.stopped, "a writer restart silently re-enabled the unit"
    assert device.state is DeviceState.STOPPED
    assert "restarted" in safety.snapshot().reason


def test_an_explicit_rearm_after_a_restart_restores_operation():
    """Fail-closed is only correct if there is a deterministic way out."""
    safety, device = _observer()
    writer = _writer()
    safety.observe(writer.snapshot())

    restarted = _writer(incarnation="run-b", started_ns=2_000)
    safety.observe(restarted.snapshot())
    assert device.state is DeviceState.STOPPED

    # The operator rearms. The writer allocates unconditionally, precisely
    # because it does not believe a stop is in force.
    safety.observe(restarted.rearm("operator rearm"))
    assert not safety.stopped
    assert device.state is DeviceState.STANDBY


def test_a_straggler_from_the_dead_writer_cannot_walk_the_unit_back():
    """The old writer's last message may arrive after the new one is followed."""
    safety, device = _observer()
    old = _writer(incarnation="run-a", started_ns=1_000)
    safety.observe(old.snapshot())

    restarted = _writer(incarnation="run-b", started_ns=2_000)
    safety.observe(restarted.snapshot())
    safety.observe(restarted.rearm("operator rearm"))
    assert not safety.stopped

    # A high-epoch message from the writer that has since died.
    old.stop("stop from the previous era")
    for _ in range(9):
        old.rearm("noise")
        old.stop("noise")
    assert old.epoch > safety.epoch

    assert not safety.observe(old.snapshot()), (
        "a message from a superseded writer incarnation was adopted"
    )
    assert not safety.stopped
    assert device.state is DeviceState.STANDBY


def test_observe_preserves_the_writer_that_actually_allocated_the_generation():
    """An observer reported its own id as the allocator, hiding the real source."""
    safety = UnitSafety("arm_left", writer=False)
    writer = _writer("unit_safety")

    safety.observe(writer.snapshot())

    assert safety.snapshot().writer_id == "unit_safety", (
        "the observer reported itself as the allocator of a generation it adopted"
    )
    assert safety.snapshot().incarnation == "run-a"


def test_two_live_writers_are_still_reported_as_a_contradiction():
    """The incarnation model must not swallow the two-writer symptom."""
    safety = UnitSafety("arm_left", writer=False)
    a = _writer("writer_a", incarnation="run-a", started_ns=1_000)
    safety.observe(a.snapshot())

    a.stop("stop from a")
    safety.observe(a.snapshot())
    assert safety.stopped

    # Same incarnation and epoch, opposite meaning, different allocator.
    contradiction = UnitSafetySnapshot(
        epoch=safety.epoch,
        stopped=False,
        reason="rearmed by b",
        writer_id="writer_b",
        incarnation="run-a",
        started_ns=1_000,
    )
    assert not safety.observe(contradiction)
    assert safety.conflicts == 1
    assert safety.stopped, "the safer reading did not win"
