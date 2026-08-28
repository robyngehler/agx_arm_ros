"""L1 tests for what a MIT command must satisfy to reach the arm.

The driver used to check array lengths and non-emptiness and then forward
whatever was in the arrays. Each test below is a command that reached the vendor
SDK before this guard existed.
"""

import math
import threading
import time

import pytest

from agx_arm_ctrl.command_validation import (
    NERO_DEFAULT_MIT_LIMITS,
    NERO_V111_MIT_LIMITS,
    mit_limits_for_tier,
    positions_outside_joint_limits,
    validate_mit_command,
)


def _command(
    joint_index=(1, 2, 3),
    p_des=(0.0, 0.0, 0.0),
    v_des=(0.0, 0.0, 0.0),
    kp=(10.0, 10.0, 10.0),
    kd=(0.8, 0.8, 0.8),
    torque=(0.0, 0.0, 0.0),
    joint_count=7,
    limits=NERO_DEFAULT_MIT_LIMITS,
):
    return validate_mit_command(
        joint_index, p_des, v_des, kp, kd, torque,
        joint_count=joint_count, limits=limits,
    )


def test_a_well_formed_command_is_admitted():
    assert _command() is None


def test_a_full_seven_joint_command_is_admitted():
    assert _command(
        joint_index=tuple(range(1, 8)),
        p_des=(0.1,) * 7,
        v_des=(0.0,) * 7,
        kp=(30.0,) * 7,
        kd=(1.0,) * 7,
        torque=(0.5,) * 7,
    ) is None


def test_inconsistent_array_lengths_are_refused():
    rejection = _command(joint_index=(1, 2), p_des=(0.0, 0.0, 0.0))
    assert rejection is not None
    assert rejection.reason == "length_mismatch"


def test_an_empty_command_is_refused():
    rejection = _command(
        joint_index=(), p_des=(), v_des=(), kp=(), kd=(), torque=()
    )
    assert rejection is not None
    assert rejection.reason == "empty"


@pytest.mark.parametrize("bad_index", [0, -1, 8, 255])
def test_a_joint_the_arm_does_not_have_is_refused(bad_index):
    rejection = _command(joint_index=(1, 2, bad_index))
    assert rejection is not None
    assert rejection.reason == "unknown_joint"
    assert str(bad_index) in rejection.detail


def test_the_same_joint_commanded_twice_is_refused():
    """Last-one-wins would make the effect depend on array order."""
    rejection = _command(joint_index=(1, 2, 2))
    assert rejection is not None
    assert rejection.reason == "duplicate_joint"


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_a_non_finite_value_is_refused(bad):
    assert _command(p_des=(0.0, bad, 0.0)).reason == "non_finite"
    assert _command(v_des=(0.0, bad, 0.0)).reason == "non_finite"
    assert _command(kp=(10.0, bad, 10.0)).reason == "non_finite"
    assert _command(kd=(0.8, bad, 0.8)).reason == "non_finite"
    assert _command(torque=(0.0, bad, 0.0)).reason == "non_finite"


def test_a_position_the_protocol_cannot_encode_is_refused():
    assert _command(p_des=(0.0, 13.0, 0.0)).reason == "out_of_range"
    assert _command(p_des=(0.0, -13.0, 0.0)).reason == "out_of_range"
    assert _command(p_des=(0.0, 12.5, 0.0)) is None


def test_a_velocity_the_protocol_cannot_encode_is_refused():
    assert _command(v_des=(0.0, 46.0, 0.0)).reason == "out_of_range"
    assert _command(v_des=(0.0, 45.0, 0.0)) is None


def test_gains_outside_the_protocol_range_are_refused():
    assert _command(kp=(10.0, 501.0, 10.0)).reason == "out_of_range"
    assert _command(kp=(10.0, -0.1, 10.0)).reason == "out_of_range"
    assert _command(kd=(0.8, 5.1, 0.8)).reason == "out_of_range"
    assert _command(kd=(0.8, -5.1, 0.8)).reason == "out_of_range"


def test_the_default_tier_bounds_torque_per_joint():
    """Firmware <= 1.10: joints 1-2 take 24 Nm, 3-4 take 16, 5-7 take 8."""
    assert _command(joint_index=(1,), p_des=(0.0,), v_des=(0.0,),
                    kp=(10.0,), kd=(0.8,), torque=(23.0,)) is None
    assert _command(joint_index=(3,), p_des=(0.0,), v_des=(0.0,),
                    kp=(10.0,), kd=(0.8,), torque=(23.0,)).reason == "out_of_range"
    assert _command(joint_index=(3,), p_des=(0.0,), v_des=(0.0,),
                    kp=(10.0,), kd=(0.8,), torque=(15.0,)) is None
    assert _command(joint_index=(6,), p_des=(0.0,), v_des=(0.0,),
                    kp=(10.0,), kd=(0.8,), torque=(15.0,)).reason == "out_of_range"


def test_the_rejection_detail_names_the_joint_and_the_bound():
    rejection = _command(kp=(10.0, 900.0, 10.0))
    assert "kp" in rejection.detail
    assert "joint 2" in rejection.detail
    assert "500" in rejection.detail


# --- joint limits are flagged, not refused -----------------------------------

JOINT_NAMES = [f"joint{n}" for n in range(1, 8)]
JOINT_LIMITS = {name: [-1.5, 1.5] for name in JOINT_NAMES}


def test_a_position_past_the_configured_limit_is_flagged():
    outside = positions_outside_joint_limits(
        (1, 2, 3), (0.0, 2.0, 0.0), JOINT_NAMES, JOINT_LIMITS
    )
    assert len(outside) == 1
    assert "joint2" in outside[0]


def test_a_position_inside_the_configured_limit_is_not_flagged():
    assert positions_outside_joint_limits(
        (1, 2, 3), (0.0, 1.4, -1.4), JOINT_NAMES, JOINT_LIMITS
    ) == []


def test_flagging_is_silent_when_no_limits_are_configured():
    assert positions_outside_joint_limits(
        (1, 2), (99.0, -99.0), JOINT_NAMES, {}
    ) == []


def test_flagging_never_raises_on_input_the_validator_would_have_refused():
    """It runs after validation, but must not be a second failure mode."""
    assert positions_outside_joint_limits(
        (1, 99), (0.0, 0.0), JOINT_NAMES, JOINT_LIMITS
    ) == []
    assert positions_outside_joint_limits(
        (1,), (math.nan,), JOINT_NAMES, JOINT_LIMITS
    ) == []


# --- the driver actually refuses ---------------------------------------------

from types import SimpleNamespace  # noqa: E402

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode  # noqa: E402
from agx_arm_ctrl.sdk_worker import SdkWorker as _SdkWorker  # noqa: E402
from agx_arm_msgs.msg import MoveMITMsg  # noqa: E402


def _drain(node):
    """Wait for everything submitted before this call to have run.

    The normal lane is FIFO, so a call that settles after ours proves ours is
    finished. Cheaper and less fragile than sleeping, and it fails loudly if the
    worker is wedged instead of silently asserting on an empty list.
    """
    node._sdk.call("drain", lambda: None, timeout=2.0)


class _FakeLogger:
    def __init__(self):
        self.warns = []
        self.errors = []

    def info(self, *_a, **_k):
        pass

    def warn(self, msg, *_a, **_k):
        self.warns.append(str(msg))

    def error(self, msg, *_a, **_k):
        self.errors.append(str(msg))


class _CountingArm:
    """Records every MIT frame that made it to the vendor SDK, and from where.

    The thread name is recorded because "which thread wrote to the SDK" is the
    property under test, not an implementation detail: it is how the Phase 1A
    exit gate is read on hardware too, off the per-thread call counter.
    """

    def __init__(self):
        self.sent = []
        self.sent_from = []
        # The mode bracket the setpoint cycle opens and must always close.
        self.bracket = []
        self.on_send = None
        self.on_move_j = None

    def move_j(self, q):
        if self.on_move_j is not None:
            self.on_move_j(q)

    def set_motion_mode(self, _mode):
        pass

    def set_auto_set_motion_mode_enabled(self, enabled):
        self.bracket.append("close" if enabled else "open")

    def move_mit(self, **kwargs):
        self.sent.append(kwargs)
        self.sent_from.append(threading.current_thread().name)
        if self.on_send is not None:
            self.on_send(kwargs)


def _mit_node(arm):
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.agx_arm = arm
    # The setpoint is written by the session owner now, so the stub needs a
    # real worker: what these tests assert about `arm.sent` is what came out
    # the far end of the queue, not what the callback thread did.
    node._sdk = _SdkWorker("arm_test")
    node._authority = SimpleNamespace(device_epoch=0)
    node.arm_joint_count = 7
    node.arm_joint_names = list(JOINT_NAMES)
    node.arm_joint_limits = dict(JOINT_LIMITS)
    node.mit_limits = NERO_DEFAULT_MIT_LIMITS
    node._command_rejections = {}
    node._estop_latched = False
    # These exercise the *payload* contract. Admission — commander,
    # generations, sequence — has its own tests against the authority.
    node.require_command_stamp = False
    node._last_rejection_log_monotonic = {}
    node._rejection_log_period_s = 0.0
    node._current_motion_mode = "mit"
    node.is_mit_mode = True
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    node._check_can_control = lambda: True
    return node


def _msg(joint_index, p_des, **overrides):
    msg = MoveMITMsg()
    msg.joint_index = list(joint_index)
    msg.p_des = list(p_des)
    count = len(joint_index)
    msg.v_des = list(overrides.get("v_des", [0.0] * count))
    msg.kp = list(overrides.get("kp", [10.0] * count))
    msg.kd = list(overrides.get("kd", [0.8] * count))
    msg.torque = list(overrides.get("torque", [0.0] * count))
    return msg


def test_a_valid_command_reaches_the_sdk():
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 2, 3), (0.1, 0.2, 0.3)))
    _drain(node)
    assert len(arm.sent) == 3


def test_the_setpoint_is_written_by_the_session_owner():
    """Not by the thread the command arrived on — that is the Phase 1A gate.

    The callback returning before the frames exist is the point: the check that
    used to make this test pass without a drain was reading the effect of a
    synchronous SDK call from the subscription thread.
    """
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 2, 3), (0.1, 0.2, 0.3)))
    _drain(node)
    assert {t for t in arm.sent_from} == {node._sdk.thread_name}


def test_a_nan_setpoint_never_reaches_the_sdk():
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 2, 3), (0.1, math.nan, 0.3)))
    _drain(node)
    assert arm.sent == []
    assert node.logger.errors


def test_the_whole_message_is_refused_not_the_bad_joint():
    """Six good joints and one bad one would leave a pose nobody commanded."""
    arm = _CountingArm()
    node = _mit_node(arm)
    positions = [0.1] * 7
    positions[6] = 99.0
    node._move_mit_callback(_msg(tuple(range(1, 8)), positions))
    _drain(node)
    assert arm.sent == []


def test_rejections_are_counted_per_reason():
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 1), (0.1, 0.2)))
    node._move_mit_callback(_msg((1, 1), (0.1, 0.2)))
    node._move_mit_callback(_msg((1, 2), (math.inf, 0.2)))
    _drain(node)

    assert node._command_rejections[("move_mit", "duplicate_joint")] == 2
    assert node._command_rejections[("move_mit", "non_finite")] == 1
    assert arm.sent == []


def test_a_position_past_a_joint_limit_is_warned_but_still_sent():
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 2), (0.0, 2.0)))
    _drain(node)

    assert len(arm.sent) == 2, "a joint-limit excursion must not freeze the loop"
    assert node.logger.warns
    assert "joint2" in node.logger.warns[-1]


def test_a_setpoint_issued_under_a_superseded_epoch_is_refused_not_sent():
    """A recovery bumps the epoch; work stamped before it must not arrive after.

    The refusal is reported because ``submit`` settles this case on the spot —
    it is one of the outcomes that *establish* the command never ran, unlike a
    wait that merely expired.
    """
    arm = _CountingArm()
    node = _mit_node(arm)
    node._sdk.set_epoch(4)
    node._authority = SimpleNamespace(device_epoch=3)

    node._move_mit_callback(_msg((1, 2, 3), (0.1, 0.2, 0.3)))
    _drain(node)

    assert arm.sent == []
    assert node._command_rejections[("move_mit", "dropped")] == 1


def test_only_the_newest_queued_setpoint_is_sent():
    """A stalled worker must not work through a backlog of stale poses."""
    arm = _CountingArm()
    node = _mit_node(arm)
    release = threading.Event()
    node._sdk.submit("block", lambda: release.wait(2.0))

    node._move_mit_callback(_msg((1,), (0.1,)))
    node._move_mit_callback(_msg((1,), (0.2,)))
    node._move_mit_callback(_msg((1,), (0.3,)))
    release.set()
    _drain(node)

    assert [frame["p_des"] for frame in arm.sent] == [0.3]
    assert node._sdk.dropped_replaced == 2


def test_a_setpoint_is_one_cycle_of_seven_preemptible_frames():
    """Not one task, and not seven independent submissions.

    One task made a seven-frame setpoint non-preemptible and measured 21 ms
    worst case on hardware — the whole stop budget in one queue entry. Seven
    submissions would let the next setpoint interleave with this one. The cycle
    is one queue entry that yields to the safety lane between frames.
    """
    arm = _CountingArm()
    node = _mit_node(arm)
    stops = []
    node._sdk.submit_safety("emergency_stop", lambda: stops.append("early"))

    node._move_mit_callback(_msg(tuple(range(1, 8)), (0.1,) * 7))
    _drain(node)

    assert [frame["joint_index"] for frame in arm.sent] == list(range(1, 8))
    assert arm.bracket == ["open", "close"], "the mode bracket must close"
    # The stop was queued before the setpoint, so strict lane order alone
    # covers it; what this pins is that the bracket is part of the cycle.
    assert stops == ["early"]


def test_a_stop_reaches_the_arm_between_two_joint_frames():
    """The preemption point the cycle exists to create."""
    arm = _CountingArm()
    node = _mit_node(arm)
    order = []
    reached = threading.Event()
    queued = threading.Event()

    def on_frame(kwargs):
        order.append(f"joint{kwargs['joint_index']}")
        if kwargs["joint_index"] == 3:
            reached.set()
            queued.wait(2.0)

    arm.on_send = on_frame

    def submit_stop():
        reached.wait(2.0)
        node._sdk.submit_safety("emergency_stop", lambda: order.append("STOP"))
        queued.set()

    stopper = threading.Thread(target=submit_stop)
    stopper.start()
    node._move_mit_callback(_msg(tuple(range(1, 8)), (0.1,) * 7))
    _drain(node)
    stopper.join(2.0)

    assert order.index("STOP") == order.index("joint3") + 1
    assert order[-1] == "joint7", "the cycle still finishes after the stop ran"


# --- the firmware tier decides the torque bound ------------------------------
#
# Found on hardware 2026-08-12: the two arms of this unit do not run the same
# firmware (right 1.06 -> default tier, left 1.11 -> v111). The tiers bound
# feed-forward torque differently, so one table for both is wrong in both
# directions.

def _torque(joint, value, limits):
    return validate_mit_command(
        (joint,), (0.0,), (0.0,), (10.0,), (0.8,), (value,),
        joint_count=7, limits=limits,
    )


def test_the_v111_tier_bounds_every_joint_at_the_same_torque():
    for joint in range(1, 8):
        assert _torque(joint, 15.9, NERO_V111_MIT_LIMITS) is None
        assert _torque(joint, 16.1, NERO_V111_MIT_LIMITS).reason == "out_of_range"


def test_the_default_table_would_refuse_valid_v111_commands():
    """The dangerous direction: a legitimate command frozen out mid-stream."""
    assert _torque(6, 12.0, NERO_V111_MIT_LIMITS) is None
    assert _torque(6, 12.0, NERO_DEFAULT_MIT_LIMITS).reason == "out_of_range"


def test_the_v111_table_would_admit_commands_the_default_tier_refuses():
    assert _torque(1, 20.0, NERO_DEFAULT_MIT_LIMITS) is None
    assert _torque(1, 20.0, NERO_V111_MIT_LIMITS).reason == "out_of_range"


def test_the_tier_lookup_covers_every_tier_the_sdk_ships():
    assert mit_limits_for_tier("default") is NERO_DEFAULT_MIT_LIMITS
    assert mit_limits_for_tier("v111") is NERO_V111_MIT_LIMITS
    # 1.12 inherits 1.11's move_mit, so it inherits its bounds.
    assert mit_limits_for_tier("v112") is NERO_V111_MIT_LIMITS


def test_an_unknown_tier_falls_back_to_the_driver_the_sdk_would_build():
    assert mit_limits_for_tier("v999") is NERO_DEFAULT_MIT_LIMITS
    assert mit_limits_for_tier(None) is NERO_DEFAULT_MIT_LIMITS


def test_an_emergency_stop_overtakes_the_queued_setpoints():
    """What the safety lane is for, at the level the driver uses it.

    The stop's own frames are queued behind three setpoints that are already
    waiting. Lane order, not queue position, decides what reaches the arm first.
    """
    arm = _CountingArm()
    node = _mit_node(arm)
    node.feedback_timeout = 2.0
    release = threading.Event()
    blocking = threading.Event()

    def blocker():
        blocking.set()
        release.wait(2.0)

    # Wait until the worker is provably inside the blocking call. Merely
    # queueing it is not enough: it sits on the lowest lane, so a setpoint
    # would overtake it and run before anything else was queued at all.
    node._sdk.submit("block", blocker)
    assert blocking.wait(2.0), "worker never picked up the blocking call"

    for _ in range(3):
        node._move_mit_callback(_msg((1, 2, 3), (0.1, 0.2, 0.3)))

    order = []
    arm.on_send = lambda kwargs: order.append("setpoint")
    arm.on_move_j = lambda _q: order.append("hold")

    # The stop's own work is the MOVE-J hold. There is deliberately no kp=0 MIT
    # command to send here any more: it would end the setpoint without stiffness,
    # and a sagging arm is not a stop.
    stopper = threading.Thread(
        target=lambda: node._sdk_safety("move_j", lambda: arm.move_j([0.0] * 7))
    )
    stopper.start()
    time.sleep(0.05)  # let the hold reach the queue behind the setpoints
    release.set()
    stopper.join(3.0)
    _drain(node)

    assert order[0] == "hold", f"a setpoint went out before the stop: {order[:9]}"
