"""L1 tests for the per-device authority and the two epochs.

These encode the Phase 1A rules that are easy to lose in later edits: motion is
admitted in exactly one state, acknowledging a fault is not the same as arming
the device, and a device epoch stays private to its device.
"""

from agx_arm_ctrl.device_authority import (
    DeviceAuthority,
    DeviceState,
    Reject,
    UnitSafety,
)


def _armed(device_id="arm_left", owner="mit", safety=None):
    """Return an authority already claimed and rearmed, plus its unit safety."""
    safety = safety or UnitSafety()
    authority = DeviceAuthority(device_id, safety)
    authority.go_standby("connected")
    assert authority.claim(owner).accepted
    assert authority.rearm(verified=True, detail="feedback advancing").accepted
    return authority, safety


def test_motion_is_admitted_only_when_ready():
    authority, _ = _armed()
    stamp = authority.stamp("mit", sequence=1)
    assert authority.admit(stamp).accepted

    authority.enter_recovering("bus down")
    verdict = authority.admit(authority.stamp("mit", sequence=2))
    assert not verdict.accepted
    assert verdict.reason is Reject.NOT_READY
    assert "recovering" in verdict.detail


def test_a_stamp_from_before_an_interruption_is_rejected_after_it():
    authority, _ = _armed()
    in_flight = authority.stamp("mit", sequence=7)

    authority.enter_recovering("bus down")
    authority.go_standby("bus back")
    assert authority.rearm(verified=True, detail="feedback advancing").accepted

    verdict = authority.admit(in_flight)
    assert not verdict.accepted
    assert verdict.reason is Reject.STALE_DEVICE_EPOCH


def test_sequence_must_advance_within_an_epoch():
    authority, _ = _armed()
    assert authority.admit(authority.stamp("mit", sequence=4)).accepted
    replayed = authority.admit(authority.stamp("mit", sequence=4))
    assert not replayed.accepted
    assert replayed.reason is Reject.STALE_SEQUENCE
    assert authority.admit(authority.stamp("mit", sequence=5)).accepted


def test_sequence_watermark_resets_with_the_epoch():
    authority, _ = _armed()
    assert authority.admit(authority.stamp("mit", sequence=900)).accepted

    authority.enter_faulted("comm error")
    assert authority.acknowledge_fault("operator").accepted
    assert authority.rearm(verified=True, detail="feedback advancing").accepted

    # A fresh epoch starts a fresh sequence; a low number is not stale here.
    assert authority.admit(authority.stamp("mit", sequence=0)).accepted


def test_only_one_commander_at_a_time():
    safety = UnitSafety()
    authority = DeviceAuthority("arm_left", safety)
    authority.go_standby("connected")

    assert authority.claim("coordinator").accepted
    second = authority.claim("teach_gui")
    assert not second.accepted
    assert second.reason is Reject.NOT_OWNER

    assert not authority.release("teach_gui").accepted
    assert authority.release("coordinator").accepted
    assert authority.claim("teach_gui").accepted


def test_a_command_from_a_previous_owner_is_rejected():
    authority, _ = _armed(owner="coordinator")
    stamp = authority.stamp("coordinator", sequence=1)

    authority.revoke("operator took over")
    assert authority.claim("teach_gui").accepted

    verdict = authority.admit(stamp)
    assert not verdict.accepted
    # The ownership change bumped the epoch, so the stamp is stale on both
    # counts; whichever fires first, it must not be admitted.
    assert verdict.reason in (Reject.NOT_OWNER, Reject.STALE_DEVICE_EPOCH)


def test_acknowledging_a_fault_does_not_arm_the_device():
    authority, _ = _armed()
    authority.enter_faulted("CAN send failed")

    assert authority.acknowledge_fault("operator").accepted
    assert authority.state is DeviceState.STANDBY
    assert not authority.motion_ready

    verdict = authority.admit(authority.stamp("mit", sequence=1))
    assert not verdict.accepted
    assert verdict.reason is Reject.NOT_READY


def test_rearm_without_evidence_is_refused():
    """The 0E lesson: a bus that came back on its own is not a verified rearm."""
    authority, _ = _armed()
    authority.enter_faulted("CAN send failed")
    assert authority.acknowledge_fault("operator").accepted

    refused = authority.rearm(verified=False, detail="no feedback checked")
    assert not refused.accepted
    assert authority.state is DeviceState.STANDBY

    assert authority.rearm(verified=True, detail="feedback advancing").accepted
    assert authority.motion_ready


def test_a_latched_fault_must_be_acknowledged_before_rearming():
    authority, _ = _armed()
    authority.enter_faulted("CAN send failed")

    refused = authority.rearm(verified=True, detail="feedback advancing")
    assert not refused.accepted
    assert "acknowledge" in refused.detail
    assert authority.state is DeviceState.FAULTED


def test_unit_stop_halts_every_device_and_a_unit_rearm_does_not_arm_them():
    safety = UnitSafety()
    arm, _ = _armed("arm_left", safety=safety)
    hand, _ = _armed("hand_left", owner="skills", safety=safety)

    safety.stop("emergency stop")
    assert arm.state is DeviceState.STOPPED
    assert hand.state is DeviceState.STOPPED

    safety.rearm("estop released")
    assert arm.state is DeviceState.STANDBY
    assert hand.state is DeviceState.STANDBY
    assert not arm.motion_ready
    assert not hand.motion_ready


def test_an_arm_recovery_does_not_invalidate_the_same_side_hand():
    """The reason the epoch is per device and not per side.

    Arm and hand on one side sit on separate buses (C1). A left-arm recovery
    must not abort a left-hand grasp that never shared that bus.
    """
    safety = UnitSafety()
    arm, _ = _armed("arm_left", safety=safety)
    hand, _ = _armed("hand_left", owner="skills", safety=safety)
    hand_stamp = hand.stamp("skills", sequence=1)

    arm.enter_recovering("can_nero_left down")
    arm.go_standby("can_nero_left back")
    assert arm.rearm(verified=True, detail="feedback advancing").accepted

    assert hand.motion_ready
    assert hand.admit(hand_stamp).accepted


def test_a_unit_stop_does_invalidate_an_in_flight_hand_command():
    """The case that *should* be global still is."""
    safety = UnitSafety()
    hand, _ = _armed("hand_left", owner="skills", safety=safety)
    stamp = hand.stamp("skills", sequence=1)

    safety.stop("emergency stop")
    safety.rearm("estop released")
    assert hand.rearm(verified=True, detail="feedback advancing").accepted

    verdict = hand.admit(stamp)
    assert not verdict.accepted
    assert verdict.reason is Reject.STALE_UNIT_EPOCH


def test_unit_safety_observed_from_another_process_only_moves_forward():
    safety = UnitSafety()
    device = DeviceAuthority("arm_right", safety)
    device.go_standby("connected")

    live = UnitSafety()
    live.stop("emergency stop")
    assert safety.observe(live.snapshot())
    assert device.state is DeviceState.STOPPED

    # A stale snapshot from before the stop must not clear it.
    assert not safety.observe(safety.snapshot())
    assert device.state is DeviceState.STOPPED
    assert safety.stopped


def test_authority_changes_are_published_as_they_happen():
    """MIT aborts on authority loss, so the loss has to be pushed, not polled."""
    seen = []
    safety = UnitSafety()
    device = DeviceAuthority("arm_left", safety, on_change=seen.append)
    device.go_standby("connected")
    device.claim("mit")
    device.rearm(verified=True, detail="feedback advancing")
    del seen[:]

    safety.stop("emergency stop")
    assert seen, "an emergency stop must reach the consumer without a poll"
    assert seen[-1].state is DeviceState.STOPPED
    assert not seen[-1].motion_ready
    assert seen[-1].unit_stopped


def test_a_device_constructed_during_a_unit_stop_starts_stopped():
    safety = UnitSafety()
    safety.stop("emergency stop")
    late = DeviceAuthority("hand_right", safety)
    assert late.state is DeviceState.STOPPED
    assert not late.motion_ready


def test_rearm_is_refused_while_the_unit_stop_is_active():
    safety = UnitSafety()
    device = DeviceAuthority("arm_left", safety)
    device.go_standby("connected")
    safety.stop("emergency stop")

    refused = device.rearm(verified=True, detail="feedback advancing")
    assert not refused.accepted
    assert "unit" in refused.detail


# --- readiness is not permission ---------------------------------------------

def test_a_ready_but_unowned_device_is_not_commandable():
    """The gap the rename exists to close.

    `motion_ready` says the hardware is ready. It says nothing about whether
    *you* may command it, and a consumer that reads it as permission would
    stream commands that admission refuses on every single one.
    """
    safety = UnitSafety()
    device = DeviceAuthority("arm_left", safety)
    device.go_standby("connected")
    assert device.rearm(verified=True, detail="feedback advancing").accepted

    assert device.motion_ready is True
    verdict = device.may_command("mit")
    assert not verdict.accepted
    assert verdict.reason is Reject.NO_OWNER


def test_may_command_agrees_with_admit_on_the_same_state():
    """One rule, two callers: the check must not drift between them."""
    safety = UnitSafety()
    device = DeviceAuthority("arm_left", safety)
    device.go_standby("connected")

    for owner in ("mit", "teach_gui"):
        for claimed in (None, "mit"):
            device.revoke("test reset")
            if claimed:
                device.claim(claimed)
            device.rearm(verified=True, detail="test")
            permission = device.may_command(owner)
            admitted = device.admit(device.stamp(owner, sequence=1))
            assert permission.accepted == admitted.accepted, (
                f"may_command and admit disagree for owner={owner!r} "
                f"claimed={claimed!r}"
            )
            if not permission.accepted:
                assert permission.reason is admitted.reason


def test_may_command_refuses_a_device_that_is_not_ready():
    authority, _ = _armed(owner="mit")
    assert authority.may_command("mit").accepted

    authority.enter_recovering("bus down")
    verdict = authority.may_command("mit")
    assert not verdict.accepted
    assert verdict.reason is Reject.NOT_READY


def test_may_command_refuses_a_different_commander():
    authority, _ = _armed(owner="coordinator")
    verdict = authority.may_command("teach_gui")
    assert not verdict.accepted
    assert verdict.reason is Reject.NOT_OWNER


# --- unit safety needs exactly one writer ------------------------------------

def test_an_observer_may_not_allocate_a_generation():
    """A second allocator is how one epoch ends up meaning two things."""
    observer = UnitSafety("arm_left", writer=False)
    assert observer.is_writer is False

    for attempt in (lambda: observer.stop("local estop"),
                    lambda: observer.rearm("local clear")):
        try:
            attempt()
        except RuntimeError as exc:
            assert "not the unit-safety writer" in str(exc)
        else:
            raise AssertionError("an observer minted its own generation")


def test_an_observer_still_adopts_what_the_writer_publishes():
    writer = UnitSafety("supervisor")
    observer = UnitSafety("arm_left", writer=False)

    assert observer.observe(writer.stop("emergency stop"))
    assert observer.stopped
    assert observer.observe(writer.rearm("released"))
    assert not observer.stopped


def test_two_writers_minting_the_same_generation_is_counted_not_merged():
    """The concrete symptom of more than one writer, made visible."""
    left = UnitSafety("arm_left")
    right = UnitSafety("arm_right")

    left.stop("left estop")          # left is at epoch 1, stopped
    right.rearm("right says fine")   # right is at epoch 1, NOT stopped

    assert not left.observe(right.snapshot())
    assert left.conflicts == 1
    # The safer reading survives: a contradiction never clears a live stop.
    assert left.stopped


def test_a_contradicting_stop_wins_over_a_local_rearm():
    left = UnitSafety("arm_left")
    right = UnitSafety("arm_right")

    left.rearm("left says fine")
    right.stop("right estop")

    assert left.observe(right.snapshot())
    assert left.conflicts == 1
    assert left.stopped


def test_a_duplicate_from_the_same_writer_is_not_a_conflict():
    """Republished latched state must not look like a second allocator."""
    writer = UnitSafety("supervisor")
    observer = UnitSafety("arm_left", writer=False)
    observer.observe(writer.stop("emergency stop"))

    assert not observer.observe(writer.snapshot())
    assert observer.conflicts == 0
    assert observer.stopped
