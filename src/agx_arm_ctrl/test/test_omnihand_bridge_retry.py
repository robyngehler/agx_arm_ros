"""Unit tests for the bridge command verify-and-retry layer (mock backend)."""

from __future__ import annotations

import pytest
import rclpy
from sensor_msgs.msg import JointState

from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode


def _cycle(node) -> None:
    """One acquisition plus one publication.

    They used to be a single timer callback. Acquisition now runs on its own
    thread so no SDK call sits on the ROS executor, so a test that wants both
    halves asks for both — and a test about publication alone simply does not
    call the acquiring half.
    """
    node._acquire_once()
    node._publication_tick()


@pytest.fixture()
def bridge_node():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "command_retry_period_s:=0.05",
            "-p", "joint_read_rate:=1000.0",
            # No graph to look the owner up in: an in-process node is not a
            # running commander, and the liveness watchdog would revoke it.
            "-p", "owner_liveness_grace_s:=0.0",
        ]
    )
    node = OmniHandBridgeNode()
    # The bridge is fail-closed: an unclaimed hand executes nothing, and a hand
    # that has not yet reported itself connected is not READY. These tests are
    # about the retry layer, so they bring the device to commandable the way the
    # runtime does — one feedback tick, then a claim by the reactive primitive.
    _cycle(node)
    assert node._authority.claim("reactive:test_owner").accepted
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _await_send(node, timeout: float = 2.0) -> None:
    """Block until the worker has finished the submitted command."""
    pending = node.pending_command
    call = pending.get("call") if pending else None
    if call is None:
        return
    try:
        call.result(timeout)
    except Exception:
        pass


def _command_msg(node: OmniHandBridgeNode, value: float = 0.3) -> JointState:
    msg = JointState()
    msg.name = list(node.joint_names[:2])
    msg.position = [value, value]
    return msg


def test_command_verifies_and_clears_pending(bridge_node):
    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node.pending_command is not None
    assert bridge_node.pending_command["attempts"] == 1

    # mock backend applies targets instantly; a fresh readback verifies them
    _cycle(bridge_node)
    bridge_node._command_retry_tick()

    assert bridge_node.pending_command is None


def test_arm_only_joint_state_is_ignored(bridge_node):
    msg = JointState()
    msg.name = ["joint1", "joint2"]
    msg.position = [0.1, 0.2]

    bridge_node._joint_states_command_callback(msg)

    assert bridge_node.pending_command is None


def test_failed_sends_retry_until_attempts_exhausted(bridge_node):
    def rejecting_apply(target_map, control_mode):
        raise RuntimeError("bus congested")

    bridge_node.backend.apply_joint_targets = rejecting_apply

    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node.pending_command is not None

    for expected_attempts in range(2, bridge_node.command_retry_max_attempts + 1):
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        # An attempt is only spent once a readback could have judged the last
        # send, so the poll has to run for the retry loop to advance.
        _cycle(bridge_node)
        bridge_node._command_retry_tick()
        if bridge_node.pending_command is not None:
            assert bridge_node.pending_command["attempts"] == expected_attempts

    # exhausted: one more tick gives up instead of re-sending forever
    if bridge_node.pending_command is not None:
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        _cycle(bridge_node)
        bridge_node._command_retry_tick()
    assert bridge_node.pending_command is None
    assert bridge_node._command_delivery_failed is True


def test_attempt_is_not_spent_without_a_readback_opportunity(bridge_node):
    """The attempt budget is spent on evidence, not on the clock.

    Regression for the hardware failure of 2026-07-24: one 请求超时 put the
    backend into fault backoff (one probe every fault_poll_interval_s) while
    the retry timer kept firing every command_retry_period_s. All 8 attempts
    burned in 2.4 s with at most one readback in between, so the target was
    declared lost inside the very hand window opened to deliver it.
    """
    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node.pending_command["attempts"] == 1

    # No acquisition: no readback has landed since the send. Ageing the
    # send has to age the last readback with it, or the priming readback the
    # fixture needed to reach READY would drift to the wrong side of it and
    # count as evidence this test is asserting does not exist.
    for _ in range(20):
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        bridge_node.last_joint_read_monotonic -= 10.0
        bridge_node._command_retry_tick()

    assert bridge_node.pending_command is not None
    assert bridge_node.pending_command["attempts"] == 1
    assert bridge_node._command_delivery_failed is False


def test_pending_command_keeps_the_probe_at_retry_cadence_under_fault_backoff(
    bridge_node,
):
    """Fault backoff must not starve the readback that ends the retries.

    Backing off is right for idle polling during an error storm, but a pending
    command is already re-sending — the probe is the only thing that can
    confirm it and STOP that traffic.
    """
    bridge_node.fault_poll_interval_s = 2.0
    bridge_node._fault_backoff_active = True
    bridge_node.joint_read_min_interval_s = 0.05
    bridge_node.command_retry_period_s = 0.3

    bridge_node.pending_command = None
    assert bridge_node._effective_read_interval() == 2.0

    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node._effective_read_interval() == 0.3


def test_unverified_target_retries_then_gives_up(bridge_node):
    """Fingers blocked by contact: the command lands, the hand never arrives."""

    def blocked_apply(target_map, control_mode):
        # Accepted by the device, but the pose does not change — the mock's own
        # apply would move `positions` to the target, which is the one thing this
        # test needs not to happen.
        return len(target_map)

    bridge_node.backend.apply_joint_targets = blocked_apply
    bridge_node.backend.positions = [99.0] * len(bridge_node.joint_names)

    bridge_node._joint_states_command_callback(_command_msg(bridge_node))

    attempts_seen = []
    while bridge_node.pending_command is not None:
        _await_send(bridge_node)
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        _cycle(bridge_node)
        attempts_seen.append(bridge_node.pending_command["attempts"])
        bridge_node._command_retry_tick()

    assert max(attempts_seen) == bridge_node.command_retry_max_attempts
    assert bridge_node._command_delivery_failed is True


def test_joint_read_rate_sets_the_acquisition_interval(bridge_node):
    """The rate is honoured by pacing the loop, not by gating each cycle.

    It used to be a gate inside the tick, because acquisition rode on the publish
    timer and had to decide per tick whether it was due. That rounding is what
    made an intended 20 Hz measure 15.4 Hz. The loop now sleeps the interval, so
    a cycle IS a read and the interval is what the loop asks for.
    """
    bridge_node.joint_read_min_interval_s = 3600.0
    bridge_node._fault_backoff_active = False
    bridge_node.pending_command = None

    assert bridge_node._effective_read_interval() == 3600.0


def test_no_joint_state_published_before_first_successful_readback(bridge_node):
    published = []

    class FaultedBackend:
        communication_fault = True

        def read_joint_state(self):
            return [0.0] * len(bridge_node.joint_names)

        def read_status(self):
            return bridge_node.backend.read_status()

        def read_tactile(self):
            return bridge_node.backend.read_tactile()

    original_backend = bridge_node.backend
    faulted = FaultedBackend()
    faulted.read_status = original_backend.read_status
    faulted.read_tactile = original_backend.read_tactile
    bridge_node.backend = faulted
    bridge_node.last_good_joint_read_monotonic = 0.0
    bridge_node.hand_joint_states_pub.publish = published.append

    _cycle(bridge_node)

    assert published == []


# --- a slow joint is not a stuck joint ---------------------------------------


def _creeping_backend(node, target: float, step: float):
    """Make the readback crawl toward the target instead of arriving at once.

    A thumb travels further than a finger and takes longer than the retry budget
    spent on the clock. This is that joint, in the small.
    """
    names = list(node.joint_names[:2])
    reached = {name: 0.0 for name in names}

    def creeping_apply(target_map, control_mode):
        for name in names:
            if name in target_map:
                gap = target - reached[name]
                reached[name] += step if gap > step else gap
        node.backend.positions = [
            reached.get(name, 0.0) for name in node.joint_names
        ]
        return len(target_map)

    node.backend.apply_joint_targets = creeping_apply
    return reached


def test_a_joint_still_closing_in_does_not_spend_the_budget(bridge_node):
    """The budget catches a command the hand never received, not a slow joint."""
    bridge_node._creep = _creeping_backend(bridge_node, target=0.3, step=0.02)
    bridge_node._joint_states_command_callback(_command_msg(bridge_node, 0.3))

    # 0.3 rad at 0.02 per cycle needs ~10 cycles to come inside the 0.10 rad
    # tolerance — more than the 8-attempt budget, which is the point.
    cycles = 0
    for _ in range(20):
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        _cycle(bridge_node)
        bridge_node._command_retry_tick()
        cycles += 1
        if bridge_node.pending_command is None:
            break
        # Never runs away with the budget while the gap keeps shrinking.
        assert bridge_node.pending_command["attempts"] <= 2
    assert cycles > bridge_node.command_retry_max_attempts, (
        "the creep arrived inside the old budget, so this proves nothing"
    )

    assert bridge_node.pending_command is None, "a closing joint was given up on"


def test_a_stalled_joint_still_exhausts_the_budget(bridge_node):
    """No progress, no reprieve: this is what the budget exists for."""
    def stalled_apply(target_map, control_mode):
        return len(target_map)

    bridge_node.backend.apply_joint_targets = stalled_apply
    bridge_node._joint_states_command_callback(_command_msg(bridge_node, 0.9))

    for _ in range(bridge_node.command_retry_max_attempts + 4):
        if bridge_node.pending_command is None:
            break
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        _cycle(bridge_node)
        bridge_node._command_retry_tick()

    assert bridge_node.pending_command is None
    assert bridge_node._command_delivery_failed


def test_a_settled_command_reports_the_attempts_it_spent(bridge_node):
    """Reporting 0 made every give-up read 'after 0 attempts' downstream."""
    def stalled_apply(target_map, control_mode):
        return len(target_map)

    bridge_node.backend.apply_joint_targets = stalled_apply
    bridge_node._joint_states_command_callback(_command_msg(bridge_node, 0.9))

    for _ in range(bridge_node.command_retry_max_attempts + 4):
        if bridge_node.pending_command is None:
            break
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        _cycle(bridge_node)
        bridge_node._command_retry_tick()

    assert bridge_node._last_command_attempts >= bridge_node.command_retry_max_attempts


def test_the_wall_clock_deadline_bounds_a_creep_that_never_arrives(bridge_node):
    """Progress alone must not extend the wait for ever."""
    bridge_node.command_verify_timeout_s = 0.0
    _creeping_backend(bridge_node, target=0.3, step=0.001)
    bridge_node._joint_states_command_callback(_command_msg(bridge_node, 0.3))
    bridge_node.pending_command["deadline"] = 0.0

    bridge_node.pending_command["last_send_monotonic"] -= 10.0
    _cycle(bridge_node)
    bridge_node._command_retry_tick()

    assert bridge_node.pending_command is None
    assert bridge_node._command_delivery_failed
