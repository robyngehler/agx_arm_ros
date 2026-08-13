"""L1 tests for the arm driver's published device authority.

The driver's readiness lived in four separate booleans — `enable_flag`,
`control_ready`, `_fault_lockout`, `_hand_window_active` — and the only thing a
controller could subscribe to was `feedback/hand_window_active`, which says
"the shared bus is busy" and nothing about faults, stops, or ownership.

These tests hold the mapping from those gates to the one published state, and
hold the epoch semantics that a derived state cannot provide by itself.
"""

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode, derive_device_id
from agx_arm_ctrl.device_authority import (
    DeviceAuthority,
    DeviceState,
    UnitSafety,
    UnitSafetySnapshot,
)


class _FakeLogger:
    def __init__(self):
        self.errors = []

    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, msg, *_a, **_k):
        self.errors.append(str(msg))


def _node(device_id="arm_left"):
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    node.require_fault_ack = True
    node._fault_lockout = False
    node._estop_latched = False
    node._fault_lockout_logged = False
    node._recovery_in_progress = False
    node._hand_window_active = False
    node.control_ready = True
    node._control_ready_logged = True
    node.enable_flag = True
    node._last_good_feedback_monotonic = 0.0
    node._publish_fault_lockout = lambda: None
    # Mirrors the real node: a device observes generations, it does not mint
    # them. Tests needing a stop feed one in as the writer would.
    node._unit_safety = UnitSafety(device_id, writer=False)
    node.unit_stop_requests = []
    node._request_unit_stop = node.unit_stop_requests.append
    node._authority = DeviceAuthority(device_id, node._unit_safety)
    return node


def _writer_says(node, *, epoch, stopped, reason):
    """Feed a generation in as the single unit-safety writer would."""
    node._unit_safety.observe(
        UnitSafetySnapshot(
            epoch=epoch, stopped=stopped, reason=reason, writer_id="unit_safety"
        )
    )


def _ready_node():
    node = _node()
    node._sync_authority("test")
    assert node._authority.state is DeviceState.READY
    return node


# --- identity ----------------------------------------------------------------

def test_the_device_id_follows_the_deployed_can_interface_names():
    assert derive_device_id("can_nero_left") == "arm_left"
    assert derive_device_id("can_nero_right") == "arm_right"


def test_an_unrecognised_interface_still_produces_a_usable_id():
    assert derive_device_id("can0") == "arm_can0"
    assert derive_device_id("") == "arm_unknown"


# --- gates map to one state --------------------------------------------------

def test_all_gates_open_means_the_device_motion_ready():
    node = _ready_node()
    assert node._authority.snapshot().motion_ready


def test_an_arm_that_is_not_enabled_does_not_accept_motion():
    node = _ready_node()
    node.enable_flag = False
    node._sync_authority("test")

    snapshot = node._authority.snapshot()
    assert snapshot.state is DeviceState.STANDBY
    assert not snapshot.motion_ready
    assert "not enabled" in snapshot.reason


def test_an_open_hand_window_parks_the_arm_rather_than_faulting_it():
    """Degraded-mode quiescence is not a fault, and must not read as one."""
    node = _ready_node()
    node._hand_window_active = True
    node._sync_authority("test")

    snapshot = node._authority.snapshot()
    assert snapshot.state is DeviceState.STANDBY
    assert "hand window" in snapshot.reason


def test_waiting_for_feedback_is_standby_not_ready():
    node = _ready_node()
    node.control_ready = False
    node._sync_authority("test")
    assert node._authority.state is DeviceState.STANDBY


def test_a_running_recovery_is_reported_as_recovering():
    node = _ready_node()
    node._recovery_in_progress = True
    node._sync_authority("test")
    assert node._authority.state is DeviceState.RECOVERING


# --- faults ------------------------------------------------------------------

def test_a_fault_lockout_faults_the_device_immediately():
    """Not on the next publish tick: a controller must stop streaming now."""
    node = _ready_node()
    node._enter_fault_lockout("bus recovery")
    assert node._authority.state is DeviceState.FAULTED
    assert not node._authority.snapshot().motion_ready


def test_clearing_the_lockout_acknowledges_but_does_not_arm_in_one_step():
    node = _ready_node()
    node._enter_fault_lockout("bus recovery")
    node._fault_lockout = False
    node.control_ready = False

    node._sync_authority("test")
    assert node._authority.state is DeviceState.STANDBY

    node.control_ready = True
    node._sync_authority("test")
    assert node._authority.state is DeviceState.READY


# --- epochs ------------------------------------------------------------------

def test_a_fault_and_recovery_cycle_invalidates_commands_issued_before_it():
    node = _ready_node()
    before = node._authority.device_epoch

    node._enter_fault_lockout("bus recovery")
    node._fault_lockout = False
    node._sync_authority("test")

    assert node._authority.device_epoch > before


def test_a_state_change_that_does_not_cross_ready_keeps_the_epoch():
    node = _ready_node()
    node.enable_flag = False
    node._sync_authority("test")
    parked = node._authority.device_epoch

    node._recovery_in_progress = True
    node._sync_authority("test")

    assert node._authority.device_epoch == parked


def test_repeated_syncs_in_a_steady_state_do_not_churn_the_epoch():
    """It runs every publish cycle at 200 Hz; it must be idempotent."""
    node = _ready_node()
    epoch = node._authority.device_epoch
    for _ in range(50):
        node._sync_authority("publish loop")
    assert node._authority.device_epoch == epoch


# --- unit safety -------------------------------------------------------------

def test_a_unit_stop_outranks_every_local_gate():
    node = _ready_node()
    _writer_says(node, epoch=1, stopped=True, reason="emergency stop")

    node._fault_lockout = True
    node._sync_authority("test")
    assert node._authority.state is DeviceState.STOPPED

    node._fault_lockout = False
    node._recovery_in_progress = True
    node._sync_authority("test")
    assert node._authority.state is DeviceState.STOPPED


def test_a_unit_stop_is_published_without_waiting_for_a_sync():
    seen = []
    node = _ready_node()
    node._authority.set_on_change(seen.append)
    del seen[:]

    _writer_says(node, epoch=1, stopped=True, reason="emergency stop")

    assert seen, "an emergency stop must reach subscribers without a poll"
    assert seen[-1].state is DeviceState.STOPPED
    assert seen[-1].unit_stopped
    assert not seen[-1].motion_ready


def test_clearing_a_device_latch_does_not_release_the_unit_stop():
    """A device releasing a unit-wide generation is the second-allocator bug."""
    from std_srvs.srv import Trigger

    node = _ready_node()
    _writer_says(node, epoch=1, stopped=True, reason="emergency stop")
    node.control_ready = False

    response = node._clear_fault_lockout_callback(None, Trigger.Response())

    assert node._unit_safety.stopped, "a device must not clear a unit stop"
    assert "unit_safety/rearm" in response.message
    assert node._authority.state is DeviceState.STOPPED


def test_attaching_a_listener_hands_it_the_current_state():
    """A late subscriber must not wait for the next transition."""
    node = _ready_node()
    seen = []
    node._authority.set_on_change(seen.append)

    assert len(seen) == 1
    assert seen[0].state is DeviceState.READY
    assert seen[0].device_id == "arm_left"


# --- what the services tell the caller ---------------------------------------

def test_clearing_says_what_it_did_and_what_it_could_not():
    """A verified e-stop leaves a unit stop the device is not allowed to clear.

    Saying only "no fault lockout was active" would be true and useless: the
    caller still has a stopped unit and no idea what holds it.
    """
    from std_srvs.srv import Trigger

    node = _ready_node()
    _writer_says(node, epoch=1, stopped=True, reason="emergency stop")

    response = node._clear_fault_lockout_callback(None, Trigger.Response())
    assert "unit_safety/rearm" in response.message

    _writer_says(node, epoch=2, stopped=False, reason="operator rearm")
    node._fault_lockout = True
    response = node._clear_fault_lockout_callback(None, Trigger.Response())
    assert response.message == "fault lockout cleared"

    response = node._clear_fault_lockout_callback(None, Trigger.Response())
    assert "nothing to clear" in response.message


def test_an_emergency_stop_faults_this_device_without_waiting_for_the_writer():
    """Safety local, bookkeeping global.

    The device must stop itself with no other process alive. Only the unit-wide
    statement that a new safety era began needs the writer, and that is
    requested, never waited on.
    """
    node = _ready_node()
    node._authority.enter_faulted("emergency stop requested")
    node._request_unit_stop("emergency stop requested")

    assert node._authority.state is DeviceState.FAULTED
    assert not node._authority.snapshot().motion_ready
    assert node.unit_stop_requests == ["emergency stop requested"]
    # The generation did not move: that is not this device's to allocate.
    assert node._unit_safety.snapshot().epoch == 0


def test_a_device_cannot_mint_its_own_unit_generation():
    node = _ready_node()
    try:
        node._unit_safety.stop("local estop")
    except RuntimeError as exc:
        assert "not the unit-safety writer" in str(exc)
    else:
        raise AssertionError("a device allocated a unit-safety generation")


def test_an_emergency_stop_latch_survives_the_derived_sync():
    """Found on hardware 2026-08-13.

    `_sync_authority` runs every publish cycle and derives the state from the
    gates. A fault set directly by the e-stop was erased on the next tick, so
    the arm went back to accepting motion seconds after a verified stop. The
    latch is what the derived mapping has to respect.
    """
    node = _ready_node()
    node._estop_latched = True

    for _ in range(50):
        node._sync_authority("publish loop")

    assert node._authority.state is DeviceState.FAULTED
    assert not node._authority.snapshot().motion_ready


def test_only_an_operator_clears_the_emergency_stop_latch():
    from std_srvs.srv import Trigger

    node = _ready_node()
    node._estop_latched = True
    node._sync_authority("publish loop")
    assert not node._authority.snapshot().motion_ready

    response = node._clear_fault_lockout_callback(None, Trigger.Response())
    assert "emergency stop latch cleared" in response.message

    node._sync_authority("publish loop")
    assert node._authority.snapshot().motion_ready


def test_the_hand_window_reason_is_specific_enough_to_replace_the_boolean():
    """What the retired `feedback/hand_window_active` used to tell a controller.

    That topic said only "the shared bus is busy". The authority says the same
    thing plus which device, which generation, and why — so a controller no
    longer needs a second subscription to learn it, and the MIT controller's
    hand-window gate could be removed rather than duplicated.
    """
    node = _ready_node()
    node._hand_window_active = True
    node._sync_authority("publish loop")

    snapshot = node._authority.snapshot()
    assert not snapshot.motion_ready
    assert "hand window" in snapshot.reason
    assert snapshot.device_id == "arm_left"
    assert snapshot.device_epoch > 0
