"""L1 tests for the arm driver's published device authority.

The driver's readiness lived in four separate booleans — `enable_flag`,
`control_ready`, `_fault_lockout`, `_hand_window_active` — and the only thing a
controller could subscribe to was `feedback/hand_window_active`, which says
"the shared bus is busy" and nothing about faults, stops, or ownership.

These tests hold the mapping from those gates to the one published state, and
hold the epoch semantics that a derived state cannot provide by itself.
"""

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode, derive_device_id
from agx_arm_ctrl.device_authority import DeviceAuthority, DeviceState, UnitSafety


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
    node._fault_lockout_logged = False
    node._recovery_in_progress = False
    node._hand_window_active = False
    node.control_ready = True
    node._control_ready_logged = True
    node.enable_flag = True
    node._last_good_feedback_monotonic = 0.0
    node._publish_fault_lockout = lambda: None
    node._unit_safety = UnitSafety()
    node._authority = DeviceAuthority(device_id, node._unit_safety)
    return node


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

def test_all_gates_open_means_the_device_accepts_motion():
    node = _ready_node()
    assert node._authority.snapshot().accepts_motion


def test_an_arm_that_is_not_enabled_does_not_accept_motion():
    node = _ready_node()
    node.enable_flag = False
    node._sync_authority("test")

    snapshot = node._authority.snapshot()
    assert snapshot.state is DeviceState.STANDBY
    assert not snapshot.accepts_motion
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
    assert not node._authority.snapshot().accepts_motion


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
    node._unit_safety.stop("emergency stop")

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

    node._unit_safety.stop("emergency stop")

    assert seen, "an emergency stop must reach subscribers without a poll"
    assert seen[-1].state is DeviceState.STOPPED
    assert seen[-1].unit_stopped
    assert not seen[-1].accepts_motion


def test_clearing_the_fault_lockout_releases_the_unit_stop_but_arms_nothing():
    from std_srvs.srv import Trigger

    node = _ready_node()
    node._unit_safety.stop("emergency stop")
    node.control_ready = False

    node._clear_fault_lockout_callback(None, Trigger.Response())

    assert not node._unit_safety.stopped
    assert node._authority.state is DeviceState.STANDBY
    assert not node._authority.snapshot().accepts_motion


def test_attaching_a_listener_hands_it_the_current_state():
    """A late subscriber must not wait for the next transition."""
    node = _ready_node()
    seen = []
    node._authority.set_on_change(seen.append)

    assert len(seen) == 1
    assert seen[0].state is DeviceState.READY
    assert seen[0].device_id == "arm_left"


# --- what the services tell the caller ---------------------------------------

def test_clearing_reports_every_latch_it_released():
    """Found on hardware: a verified e-stop leaves a unit stop and no lockout.

    Reporting only the lockout answered "no fault lockout was active" to a call
    that had just rearmed the unit.
    """
    from std_srvs.srv import Trigger

    node = _ready_node()
    node._unit_safety.stop("emergency stop")

    response = node._clear_fault_lockout_callback(None, Trigger.Response())
    assert "unit safety stop released" in response.message

    node._fault_lockout = True
    response = node._clear_fault_lockout_callback(None, Trigger.Response())
    assert response.message == "fault lockout cleared"

    response = node._clear_fault_lockout_callback(None, Trigger.Response())
    assert "nothing to clear" in response.message
