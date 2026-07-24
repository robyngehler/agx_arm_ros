"""Repo-owned control of the Nero firmware's active CAN feedback push.

The arm's feedback push (Nero -> host, low CAN IDs, one frame set per joint) is
what saturates a shared side bus: measured on hardware the bus carries ~2150
frames/s while the arm merely *holds*, and gating the MIT command stream leaves
that rate unchanged. A hand window can only free the bus for the OmniHand's
high-ID, low-priority CANFD frames by silencing that push.

The stock SDK never exposes the push on its own — it only flips it as a side
effect of a mode switch:

* ``set_normal_mode()``   -> push ENABLE  (this is what ``prepare_hand_window``
  used to call, i.e. it actively turned the flood back on)
* ``set_leader_mode()``   -> push DISABLE, plus zero-force drag
* ``set_follower_mode()`` -> push DISABLE, plus follower linkage

Leader mode is not an option for a hand window: the firmware has no gravity
model for this mounting pose and none for the end-effector payload, so the arm
sags instead of holding. What the window needs is the mode the arm is already
in after ``agx_arm_ctrl`` enables it — CAN control, statically holding the
commanded pose — with only the push turned off.

``vendor/pyAgxArm`` is a pinned submodule and is not developed in from this
repository, so the push-only sequence is reproduced here. It is exactly the
first half of the SDK's own ``set_leader_mode``/``set_follower_mode`` (mode
frame 0x151 with byte 6 = enable/disable and ``move_mode`` = "no change"),
without the trailing linkage-config frame that would change the control mode.
"""

# Mode frame byte 1: 255 is outside the valid MOVE-mode range and is what the
# SDK itself writes when a mode frame must not change the movement mode.
_MOVE_MODE_NO_CHANGE = 255

UNSUPPORTED_MESSAGE = (
    "installed pyAgxArm exposes no cached mode message (_msg_mode/_set_mode); "
    "cannot silence the arm feedback push"
)


def supports_can_push(arm) -> bool:
    """Report whether this driver exposes the mode message the push bit rides on."""
    msg_mode = getattr(arm, "_msg_mode", None)
    return (
        msg_mode is not None
        and getattr(arm, "_set_mode", None) is not None
        and getattr(msg_mode, "Enums", None) is not None
        and hasattr(msg_mode.Enums, "CanActiveMsgReporting")
    )


FALLBACK_MIT_MOVE_MODES = frozenset({0x04, 0x06})


def mit_move_mode_codes(arm) -> frozenset:
    """MOVE-mode code(s) that mean "firmware is running the host-fed MIT loop".

    The encoding is firmware-dependent — 0x04 on Nero < v111, 0x06 from v111 —
    and the two are NOT interchangeable: 0x04 is unassigned on v111+, so a
    hardcoded set would misread a healthy hold there as MIT. The active driver
    already carries the mapping its own firmware understands, so ask it, and
    fall back to both codes only when that lookup is unavailable.
    """
    try:
        return frozenset({int(arm._msg_mode.Enums.MotionMode.MIT)})
    except Exception:
        return FALLBACK_MIT_MOVE_MODES


def set_can_push(arm, enabled: bool) -> None:
    """Turn the arm's CAN feedback push on or off, keeping the control mode.

    Only the push bit is asserted: ``move_mode`` is sent as "no change" and the
    cached ``ctrl_mode`` (normally CAN control) is re-asserted as-is, so the arm
    keeps holding the pose it was commanded to hold.

    Raises ``AttributeError`` when the installed SDK has no mode message.
    Callers must always pair ``False`` with a later ``True``: while the push is
    off the host receives no new feedback frames at all.
    """
    if not supports_can_push(arm):
        raise AttributeError(UNSUPPORTED_MESSAGE)
    msg_mode = arm._msg_mode
    reporting = msg_mode.Enums.CanActiveMsgReporting
    msg_mode.enable_can_push = reporting.ENABLE if enabled else reporting.DISABLE
    previous_move_mode = msg_mode.move_mode
    msg_mode.move_mode = _MOVE_MODE_NO_CHANGE
    try:
        arm._set_mode()
    finally:
        # Leave the cached message neutral again: INVALID means "this frame does
        # not touch the push", so later mode frames (motion-mode switches during
        # normal streaming) do not re-toggle it as a side effect.
        msg_mode.enable_can_push = reporting.INVALID
        msg_mode.move_mode = previous_move_mode
