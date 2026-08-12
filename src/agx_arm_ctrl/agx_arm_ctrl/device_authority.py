"""Per-device authority and the two epochs that invalidate commands.

Phase 1A of the V02 refactor. The proposal's "side hardware authority" is one
grain too coarse: since each device has its own CAN bus (constraint C1), a side
is no longer a resource. An arm and the hand on the same side have separate
buses, separate failure modes and separate recovery paths, so they get separate
authorities and separate epochs. A per-side epoch would let a left-arm recovery
invalidate an in-flight left *hand* grasp on a bus the hand does not share —
reintroducing in software exactly the coupling the rewiring removed.

Two epochs, because they answer different questions:

* ``device_epoch`` — bumped when *this* device's ownership, arming or transport
  changes, so a command issued before the change is rejected after it;
* ``unit_safety_epoch`` — bumped for what genuinely invalidates every device at
  once: an emergency stop or a unit fault.

Deliberately free of ROS and of the vendor SDK, so the rules are testable at L1
without a workspace build or hardware.

Two rules here encode findings from the Phase 0 baseline rather than taste:

1. **Motion is accepted in exactly one state.** Anything else — offline,
   recovering, faulted, stopped, or armed-but-not-rearmed — rejects with a
   structured reason instead of a bare boolean.
2. **Acknowledging a fault never rearms the device.** The 0E fault test reported
   "recovery succeeded" for a CAN bus that had come back on its own; clearing
   the latch and asserting the device is fit to move are separate acts, and the
   second one demands evidence it was given.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, List, Optional


class DeviceState(Enum):
    """Where one device stands with respect to accepting motion."""

    OFFLINE = "offline"
    """No usable session with the device — not connected, or the link is gone."""

    STANDBY = "standby"
    """Session exists, motion not accepted. Where an acknowledged fault lands."""

    READY = "ready"
    """The only state in which motion commands are admitted."""

    RECOVERING = "recovering"
    """Transport recovery in progress; the outcome is not yet known."""

    FAULTED = "faulted"
    """A fault is latched. Needs acknowledgement, and then a verified rearm."""

    STOPPED = "stopped"
    """Halted by unit safety. Cleared only by a unit rearm, then a device rearm."""


class Reject(Enum):
    """Why a command was refused at the hardware boundary."""

    NOT_READY = "not_ready"
    NOT_OWNER = "not_owner"
    NO_OWNER = "no_owner"
    STALE_DEVICE_EPOCH = "stale_device_epoch"
    UNKNOWN_DEVICE_EPOCH = "unknown_device_epoch"
    STALE_UNIT_EPOCH = "stale_unit_epoch"
    UNKNOWN_UNIT_EPOCH = "unknown_unit_epoch"
    STALE_SEQUENCE = "stale_sequence"


@dataclass(frozen=True)
class Verdict:
    """Outcome of an authority decision, with a reason when it is a refusal."""

    accepted: bool
    reason: Optional[Reject] = None
    detail: str = ""

    def __bool__(self) -> bool:
        """Allow ``if verdict:`` at call sites that only care about the outcome."""
        return self.accepted


ACCEPTED = Verdict(accepted=True)


@dataclass(frozen=True)
class CommandStamp:
    """The identity a command must carry to be admitted to a device.

    Mirrors the fields being added to ``MoveMITMsg`` and to the consolidated
    hand command: who is commanding, under which device and unit epoch, and
    where the command sits in that owner's sequence.
    """

    owner_id: str
    device_epoch: int
    unit_safety_epoch: int
    sequence: int


@dataclass(frozen=True)
class UnitSafetySnapshot:
    """Unit-wide safety state as published and as observed by other processes."""

    epoch: int
    stopped: bool
    reason: str


@dataclass(frozen=True)
class AuthoritySnapshot:
    """One device's authoritative state — the payload other nodes consume.

    MIT consumes this instead of the ``feedback/hand_window_active`` boolean and
    aborts when ``accepts_motion`` goes false or the epoch moves under it.
    """

    device_id: str
    state: DeviceState
    device_epoch: int
    unit_safety_epoch: int
    unit_stopped: bool
    owner_id: str
    reason: str

    @property
    def accepts_motion(self) -> bool:
        """True only while the device admits motion commands."""
        return self.state is DeviceState.READY


class UnitSafety:
    """The unit-wide safety epoch, shared by every device on the unit.

    Lives in each process and is kept in step across processes by publishing
    :meth:`snapshot` and feeding what arrives into :meth:`observe`. Epochs only
    ever move forward, so a late or reordered message cannot walk the unit back
    into a state it has already left.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._epoch = 0
        self._stopped = False
        self._reason = "init"
        self._listeners: List[Callable[[UnitSafetySnapshot], None]] = []

    def snapshot(self) -> UnitSafetySnapshot:
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> UnitSafetySnapshot:
        return UnitSafetySnapshot(
            epoch=self._epoch, stopped=self._stopped, reason=self._reason
        )

    def add_listener(
        self, listener: Callable[[UnitSafetySnapshot], None]
    ) -> UnitSafetySnapshot:
        """Register a listener and return the state it should start from.

        Listeners are notified outside the lock, so a listener may take its own
        lock without ordering against this one.
        """
        with self._lock:
            self._listeners.append(listener)
            return self._snapshot_locked()

    def stop(self, reason: str) -> UnitSafetySnapshot:
        """Latch a unit-wide stop and invalidate every in-flight command."""
        return self._advance(stopped=True, reason=reason)

    def rearm(self, reason: str) -> UnitSafetySnapshot:
        """Clear the unit stop. Devices land in STANDBY, not in READY."""
        return self._advance(stopped=False, reason=reason)

    def observe(self, snapshot: UnitSafetySnapshot) -> bool:
        """Adopt unit safety state seen from another process.

        Returns True when the snapshot moved this process forward. An older or
        equal epoch is ignored rather than applied, so out-of-order delivery
        cannot resurrect a cleared stop or drop a live one.
        """
        with self._lock:
            if snapshot.epoch <= self._epoch:
                return False
            self._epoch = snapshot.epoch
            self._stopped = snapshot.stopped
            self._reason = snapshot.reason
            current = self._snapshot_locked()
            listeners = list(self._listeners)
        self._notify(listeners, current)
        return True

    def _advance(self, *, stopped: bool, reason: str) -> UnitSafetySnapshot:
        with self._lock:
            self._epoch += 1
            self._stopped = stopped
            self._reason = reason
            current = self._snapshot_locked()
            listeners = list(self._listeners)
        self._notify(listeners, current)
        return current

    @staticmethod
    def _notify(
        listeners: List[Callable[[UnitSafetySnapshot], None]],
        snapshot: UnitSafetySnapshot,
    ) -> None:
        for listener in listeners:
            listener(snapshot)

    @property
    def epoch(self) -> int:
        with self._lock:
            return self._epoch

    @property
    def stopped(self) -> bool:
        with self._lock:
            return self._stopped


class DeviceAuthority:
    """Authority over one device: one arm, or one hand's transport session.

    The hand authorities are *transport* authorities: what they own is the
    vendor SDK session and the CAN transport for one hand, not the semantics of
    a grasp, which stays with the skill controller.

    The unit epoch is cached locally and refreshed by a listener, so this object
    never reaches into :class:`UnitSafety` while holding its own lock.
    """

    def __init__(
        self,
        device_id: str,
        safety: UnitSafety,
        on_change: Optional[Callable[[AuthoritySnapshot], None]] = None,
    ) -> None:
        self.device_id = device_id
        self._safety = safety
        self._on_change = on_change
        self._lock = threading.RLock()

        self._state = DeviceState.OFFLINE
        self._device_epoch = 0
        self._owner_id = ""
        self._reason = "init"
        self._last_sequence = -1
        # Seeded before registering: the listener can fire on another thread
        # the instant registration returns.
        self._unit_epoch = 0
        self._unit_stopped = False

        self._on_unit_safety(safety.add_listener(self._on_unit_safety))

    # -- observation ----------------------------------------------------

    def set_on_change(
        self, listener: Optional[Callable[[AuthoritySnapshot], None]]
    ) -> None:
        """Attach the change listener and immediately hand it current state.

        Exists because the publisher usually outlives its transport: a ROS node
        builds its authority before its publishers, and a listener that only
        saw *future* transitions would leave the first subscriber waiting for a
        state change that may never come.
        """
        with self._lock:
            self._on_change = listener
            snapshot = self._snapshot_locked()
        if listener is not None:
            listener(snapshot)

    def snapshot(self) -> AuthoritySnapshot:
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> AuthoritySnapshot:
        return AuthoritySnapshot(
            device_id=self.device_id,
            state=self._state,
            device_epoch=self._device_epoch,
            unit_safety_epoch=self._unit_epoch,
            unit_stopped=self._unit_stopped,
            owner_id=self._owner_id,
            reason=self._reason,
        )

    @property
    def state(self) -> DeviceState:
        with self._lock:
            return self._state

    @property
    def device_epoch(self) -> int:
        with self._lock:
            return self._device_epoch

    @property
    def owner_id(self) -> str:
        with self._lock:
            return self._owner_id

    @property
    def accepts_motion(self) -> bool:
        with self._lock:
            return self._state is DeviceState.READY

    # -- ownership ------------------------------------------------------

    def claim(self, owner_id: str) -> Verdict:
        """Take command of the device. One commander at a time, by design."""
        if not owner_id:
            return Verdict(False, Reject.NO_OWNER, "claim needs a non-empty owner_id")
        with self._lock:
            if self._owner_id and self._owner_id != owner_id:
                return Verdict(
                    False,
                    Reject.NOT_OWNER,
                    f"{self.device_id} is held by '{self._owner_id}'",
                )
            if self._owner_id == owner_id:
                return ACCEPTED
            self._set_owner_locked(owner_id, f"claimed by {owner_id}")
            return ACCEPTED

    def release(self, owner_id: str) -> Verdict:
        """Give up command. Only the current owner may release."""
        with self._lock:
            if not self._owner_id:
                return ACCEPTED
            if self._owner_id != owner_id:
                return Verdict(
                    False,
                    Reject.NOT_OWNER,
                    f"'{owner_id}' cannot release a device held by "
                    f"'{self._owner_id}'",
                )
            self._set_owner_locked("", f"released by {owner_id}")
            return ACCEPTED

    def revoke(self, reason: str) -> None:
        """Take the device away from its owner — safety and operator override."""
        with self._lock:
            if not self._owner_id:
                return
            self._set_owner_locked("", f"revoked: {reason}")

    def _set_owner_locked(self, owner_id: str, reason: str) -> None:
        # An ownership change invalidates whatever the previous owner had in
        # flight, so it bumps the epoch just as a fault or a rearm does.
        self._owner_id = owner_id
        self._bump_locked(reason)

    # -- lifecycle ------------------------------------------------------

    def go_offline(self, reason: str) -> None:
        """The session or link is gone."""
        self._transition(DeviceState.OFFLINE, reason)

    def go_standby(self, reason: str) -> None:
        """A session exists but the device is not armed for motion."""
        self._transition(DeviceState.STANDBY, reason)

    def enter_recovering(self, reason: str) -> None:
        """Transport recovery has started; the outcome is not yet known."""
        self._transition(DeviceState.RECOVERING, reason)

    def enter_faulted(self, reason: str) -> None:
        """Latch a device fault. Survives until acknowledged."""
        self._transition(DeviceState.FAULTED, reason)

    def acknowledge_fault(self, reason: str) -> Verdict:
        """Clear the fault latch without arming the device.

        Deliberately separate from :meth:`rearm`. Acknowledging says the fault
        has been seen; it says nothing about whether the device can move.
        """
        with self._lock:
            if self._state is not DeviceState.FAULTED:
                return Verdict(
                    False,
                    Reject.NOT_READY,
                    f"no latched fault to acknowledge (state={self._state.value})",
                )
            self._transition_locked(
                DeviceState.STANDBY, f"fault acknowledged, not rearmed: {reason}"
            )
            return ACCEPTED

    def rearm(self, *, verified: bool, detail: str) -> Verdict:
        """Return the device to READY — but only against positive evidence.

        ``verified`` must be the result of an actual check that the device is
        answering and fit to move, not the absence of a complaint. The 0E fault
        test is the reason this argument exists: recovery was reported as
        succeeded for a bus that had merely come back by itself.
        """
        with self._lock:
            if self._unit_stopped:
                return Verdict(
                    False,
                    Reject.NOT_READY,
                    "unit safety stop is active; rearm the unit first",
                )
            if self._state is DeviceState.FAULTED:
                return Verdict(
                    False,
                    Reject.NOT_READY,
                    "fault is still latched; acknowledge it before rearming",
                )
            if not verified:
                self._transition_locked(
                    DeviceState.STANDBY, f"rearm refused, unverified: {detail}"
                )
                return Verdict(
                    False,
                    Reject.NOT_READY,
                    f"rearm needs verified evidence: {detail}",
                )
            self._transition_locked(DeviceState.READY, f"rearmed: {detail}")
            return ACCEPTED

    def _transition(self, new_state: DeviceState, reason: str) -> None:
        with self._lock:
            self._transition_locked(new_state, reason)

    def _transition_locked(self, new_state: DeviceState, reason: str) -> None:
        old_state = self._state
        self._state = new_state
        self._reason = reason
        # Crossing the READY boundary in either direction invalidates every
        # command stamped on the other side of it.
        if (old_state is DeviceState.READY) != (new_state is DeviceState.READY):
            self._bump_locked(reason)
        elif old_state is not new_state:
            self._publish_locked()

    def _bump_locked(self, reason: str) -> None:
        self._device_epoch += 1
        self._last_sequence = -1
        self._reason = reason
        self._publish_locked()

    def _publish_locked(self) -> None:
        if self._on_change is not None:
            self._on_change(self._snapshot_locked())

    # -- unit safety ----------------------------------------------------

    def _on_unit_safety(self, unit: UnitSafetySnapshot) -> None:
        with self._lock:
            if unit.epoch < self._unit_epoch:
                return
            self._unit_epoch = unit.epoch
            self._unit_stopped = unit.stopped
            if unit.stopped:
                self._transition_locked(
                    DeviceState.STOPPED, f"unit stop: {unit.reason}"
                )
            elif self._state is DeviceState.STOPPED:
                # A cleared unit stop does not arm anything. The device still
                # needs its own verified rearm.
                self._transition_locked(
                    DeviceState.STANDBY, f"unit rearmed: {unit.reason}"
                )
            else:
                self._publish_locked()

    def stamp(self, owner_id: str, sequence: int) -> CommandStamp:
        """Build the stamp a command must carry to reach this device now."""
        with self._lock:
            return CommandStamp(
                owner_id=owner_id,
                device_epoch=self._device_epoch,
                unit_safety_epoch=self._unit_epoch,
                sequence=sequence,
            )

    # -- admission ------------------------------------------------------

    def admit(self, stamp: CommandStamp) -> Verdict:
        """Decide whether one command may reach the hardware, and record it.

        Accepting advances the per-epoch sequence watermark, so this both
        decides and commits; it is not a query.
        """
        with self._lock:
            if self._state is not DeviceState.READY:
                return Verdict(
                    False,
                    Reject.NOT_READY,
                    f"{self.device_id} is {self._state.value}: {self._reason}",
                )
            if not self._owner_id:
                return Verdict(
                    False, Reject.NO_OWNER, f"{self.device_id} has no commander"
                )
            if stamp.owner_id != self._owner_id:
                return Verdict(
                    False,
                    Reject.NOT_OWNER,
                    f"'{stamp.owner_id}' is not the commander "
                    f"('{self._owner_id}')",
                )
            if stamp.unit_safety_epoch != self._unit_epoch:
                stale = stamp.unit_safety_epoch < self._unit_epoch
                return Verdict(
                    False,
                    Reject.STALE_UNIT_EPOCH if stale else Reject.UNKNOWN_UNIT_EPOCH,
                    f"unit epoch {stamp.unit_safety_epoch} != {self._unit_epoch}",
                )
            if stamp.device_epoch != self._device_epoch:
                stale = stamp.device_epoch < self._device_epoch
                return Verdict(
                    False,
                    (
                        Reject.STALE_DEVICE_EPOCH
                        if stale
                        else Reject.UNKNOWN_DEVICE_EPOCH
                    ),
                    f"device epoch {stamp.device_epoch} != {self._device_epoch}",
                )
            if stamp.sequence <= self._last_sequence:
                return Verdict(
                    False,
                    Reject.STALE_SEQUENCE,
                    f"sequence {stamp.sequence} <= {self._last_sequence}",
                )
            self._last_sequence = stamp.sequence
            return ACCEPTED


def with_sequence(stamp: CommandStamp, sequence: int) -> CommandStamp:
    """Return the same stamp at a new sequence number."""
    return replace(stamp, sequence=sequence)
