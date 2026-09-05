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
* ``unit_safety_epoch`` — bumped on every unit-safety transition, the stop and
  the rearm alike, since a device rearmed under the old generation must not be
  treated as current.

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
import time
from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, List, Optional

REARM_HINT = (
    "clear it with 'ros2 service call /unit_safety/rearm std_srvs/srv/Trigger {}'"
)

# Three missed heartbeats of the writer node (2.0 s each). Past this an observer
# stops treating the incarnation it follows as alive, which is what lets it adopt
# a writer that started earlier but is the only one still publishing.
DEFAULT_STALE_WRITER_AFTER_S = 6.0


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
    """Unit-wide safety state as published and as observed by other processes.

    ``writer_id`` names the process that allocated the generation. Without it,
    two writers can mint the same epoch with opposite meanings — one publishing
    "5, stopped", another "5, rearmed" — and a receiver that only compares
    numbers cannot tell that it is looking at a contradiction rather than a
    duplicate.

    ``incarnation`` and ``started_ns`` identify one *run* of the writer.
    Generations are only comparable inside a single incarnation: a restarted
    writer counts from zero again, so an epoch alone cannot order two messages
    that came from different runs. ``started_ns`` orders the incarnations
    themselves, so a straggler from a superseded writer cannot be mistaken for
    news — while that writer is still being heard from. Start time orders two
    live writers; whether one is still alive is a question only silence answers.
    """

    epoch: int
    stopped: bool
    reason: str
    writer_id: str = ""
    incarnation: str = ""
    started_ns: int = 0

    @property
    def order_key(self) -> tuple:
        """Total order over published states: incarnation first, then epoch."""
        return (self.started_ns, self.epoch)


@dataclass(frozen=True)
class AuthoritySnapshot:
    """One device's authoritative state — the payload other nodes consume.

    MIT consumes this instead of the ``feedback/hand_window_active`` boolean and
    aborts when ``motion_ready`` goes false or the epoch moves under it.
    """

    device_id: str
    state: DeviceState
    device_epoch: int
    unit_safety_epoch: int
    unit_stopped: bool
    owner_id: str
    reason: str

    @property
    def motion_ready(self) -> bool:
        """True when the hardware is ready — not that *you* may command it.

        Permission needs ownership and current epochs too; see
        :meth:`DeviceAuthority.may_command`. Keeping the two apart is what stops
        a consumer being told yes and then having every command refused.
        """
        return self.state is DeviceState.READY


class UnitSafety:
    """The unit-wide safety epoch, shared by every device on the unit.

    Lives in each process and is kept in step across processes by publishing
    :meth:`snapshot` and feeding what arrives into :meth:`observe`. Epochs only
    ever move forward, so a late or reordered message cannot walk the unit back
    into a state it has already left.

    **Exactly one process on a unit may be the writer.** A second writer is not
    redundancy: both allocate from their own counter, so the unit ends up with
    one generation number carrying two meanings — "5, stopped" from one and
    "5, rearmed" from the other — and no receiver can order them.

    That writer exists as a running process:
    :mod:`agx_arm_ctrl.unit_safety_node`. Device processes construct this with
    ``writer=False`` and are observers — they adopt what the writer publishes and
    refuse to mint generations. A device that needs a unit stop asks the writer
    for one over ``RequestUnitStop``.

    Being an observer does not make a device dependent on the writer to stop
    itself. Local device-stop remains unilateral and independent, so losing the
    writer cannot prevent a device from stopping; what needs the writer is only
    the *unit-wide* statement that a new safety era has begun.

    :meth:`observe` still counts contradictions rather than ignoring them,
    because a second writer can be introduced by misconfiguration and the symptom
    has to stay visible if it is.

    **Writer restart is ordered per incarnation, not per epoch.** Generations
    live in memory and start at 0, so a restarted writer republishes epochs its
    observers have already passed; comparing epoch numbers alone would drop
    everything it says until it climbs back above the highest number seen.

    Each writer run carries an ``incarnation`` and its start time. Ordering
    applies within an incarnation; a new one is adopted outright and
    fail-closed, since a restart is no evidence of safety. A straggler from the
    previous incarnation is rejected rather than allowed to un-stop the unit.

    **A writer counts as superseded only while the one that replaced it is still
    publishing.** Start time orders two live writers; it cannot establish that
    the younger one is still there. Ordering on start time alone leaves an
    observer following a dead incarnation and deaf to the live writer, so its
    rearm is dropped and no running process can clear the stop — measured on the
    unit 2026-09-05, both arms held STOPPED with the only live writer ignored.
    Silence past ``stale_writer_after_s`` therefore reopens adoption, still
    fail-closed.
    """

    def __init__(
        self,
        writer_id: str = "",
        *,
        writer: bool = True,
        incarnation: str = "",
        started_ns: int = 0,
        stale_writer_after_s: float = DEFAULT_STALE_WRITER_AFTER_S,
    ) -> None:
        """Create a unit-safety view.

        ``writer=False`` makes this a pure observer: it adopts what the
        authoritative writer publishes and refuses to mint generations of its
        own. Exactly one process on a unit may be the writer — see the class
        docstring for why a second one is not merely redundant.

        ``incarnation`` and ``started_ns`` identify this run of the writer and
        travel with every generation it allocates. An observer leaves them empty
        and takes them from whatever it adopts.

        ``stale_writer_after_s`` is how long an observer keeps treating the
        incarnation it follows as alive without hearing from it. It must exceed
        the writer's heartbeat period, or an ordinary gap between heartbeats
        reads as a dead writer.
        """
        self._lock = threading.Lock()
        self._writer_id = writer_id
        self._writer = writer
        self._incarnation = incarnation
        self._started_ns = started_ns
        self._stale_writer_after_s = float(stale_writer_after_s)
        self._last_followed_seen: Optional[float] = None
        self._pending_reports: List[str] = []
        # Who allocated the generation currently held. For a writer that is
        # itself; for an observer it is whoever published what it adopted, which
        # is not the same thing and must not be reported as if it were.
        self._source_writer_id = writer_id if writer else ""
        self._seen_any = writer
        self._epoch = 0
        self._stopped = False
        self._reason = "init"
        self.conflicts = 0
        self.incarnation_changes = 0
        # Messages from a writer this observer has already moved past. One or two
        # are a dead writer's last words; a steady stream is a second writer that
        # is still running.
        self.stragglers = 0
        self._listeners: List[Callable[[UnitSafetySnapshot], None]] = []

    def snapshot(self) -> UnitSafetySnapshot:
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> UnitSafetySnapshot:
        return UnitSafetySnapshot(
            epoch=self._epoch,
            stopped=self._stopped,
            reason=self._reason,
            writer_id=self._source_writer_id,
            incarnation=self._incarnation,
            started_ns=self._started_ns,
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

    @property
    def is_writer(self) -> bool:
        """Whether this view may allocate unit-safety generations."""
        with self._lock:
            return self._writer

    def observe(
        self, snapshot: UnitSafetySnapshot, *, now: Optional[float] = None
    ) -> bool:
        """Adopt unit safety state seen from another process.

        Returns True when the snapshot moved this process forward. ``now`` is a
        monotonic reading, injected by tests.

        Ordering is per incarnation. Inside one run of the writer an older or
        equal epoch is ignored, so out-of-order delivery cannot resurrect a
        cleared stop or drop a live one. Across runs the epoch means nothing —
        a restarted writer counts from zero — so incarnations are ordered by
        when the writer started, and a **new incarnation fails closed**: the
        unit is held stopped until that writer explicitly rearms it.

        That is deliberately not "reconstruct the old counter". An observer
        cannot know what happened while the writer was down, and guessing that
        nothing did is the one answer that can silently re-enable motion.

        An older incarnation is refused only while the one being followed is
        still heard from. Silence past ``stale_writer_after_s`` reopens
        adoption, fail-closed, so the unit cannot end up following a writer that
        no longer exists while ignoring the one that does.
        """
        now = time.monotonic() if now is None else now
        with self._lock:
            same_incarnation = snapshot.incarnation == self._incarnation
            if same_incarnation:
                self._last_followed_seen = now

            adopted_new_incarnation = not same_incarnation and self._seen_any
            if adopted_new_incarnation:
                # A different run of the writer. An older start time is a
                # straggler from a writer that has since died — adopting it
                # would walk the unit back into a safety era that has ended —
                # but only while the incarnation being followed is still
                # publishing. If that one has gone silent, the older writer is
                # the only one left, and refusing it strands the unit stopped
                # with nothing alive able to rearm it.
                followed_is_live = (
                    self._last_followed_seen is not None
                    and now - self._last_followed_seen < self._stale_writer_after_s
                )
                superseded = snapshot.started_ns < self._started_ns
                if superseded and followed_is_live:
                    self.stragglers += 1
                    self._report_straggler_locked(snapshot)
                    return False
                self.incarnation_changes += 1
                previous_incarnation = self._incarnation
                self._incarnation = snapshot.incarnation
                self._started_ns = snapshot.started_ns
                self._source_writer_id = snapshot.writer_id
                self._epoch = snapshot.epoch
                self._stopped = True
                self._last_followed_seen = now
                if superseded:
                    self._reason = (
                        f"unit-safety writer '{snapshot.writer_id}' incarnation "
                        f"'{snapshot.incarnation}' adopted after incarnation "
                        f"'{previous_incarnation}' went silent; holding the stop "
                        "until an explicit rearm"
                    )
                else:
                    self._reason = (
                        f"unit-safety writer restarted (incarnation "
                        f"'{snapshot.incarnation}' from '{snapshot.writer_id}'); "
                        "holding the stop until an explicit rearm"
                    )
                self._pending_reports.append(
                    f"unit safety writer changed ({self.incarnation_changes} so "
                    f"far): {self._reason}. This device is holding a stop because "
                    f"the new writer cannot vouch for what happened while the "
                    f"previous one was reachable; {REARM_HINT}."
                )
            else:
                contradiction = (
                    same_incarnation
                    and snapshot.epoch == self._epoch
                    and snapshot.stopped != self._stopped
                    and snapshot.writer_id != self._source_writer_id
                )
                if contradiction:
                    # One generation, two meanings, two allocators. No ordering
                    # resolves this — it is the concrete symptom of more than one
                    # writer, so it is counted, and the safer reading wins.
                    self.conflicts += 1
                    self._pending_reports.append(
                        f"unit safety CONTRADICTION seen ({self.conflicts} so "
                        f"far): generation {snapshot.epoch} from "
                        f"'{snapshot.writer_id}' contradicts the same generation "
                        f"from '{self._source_writer_id}'. More than one process "
                        "is allocating generations; find the second writer."
                    )
                    if not snapshot.stopped:
                        return False
                    self._stopped = True
                    self._reason = (
                        f"conflicting unit-safety generation {snapshot.epoch} "
                        f"from '{snapshot.writer_id}'; holding the stop"
                    )
                elif self._seen_any and snapshot.epoch <= self._epoch:
                    return False
                else:
                    # First observation, or a forward step inside this
                    # incarnation.
                    self._incarnation = snapshot.incarnation
                    self._started_ns = snapshot.started_ns
                    self._source_writer_id = snapshot.writer_id
                    self._epoch = snapshot.epoch
                    self._stopped = snapshot.stopped
                    self._reason = snapshot.reason
                    self._last_followed_seen = now
            self._seen_any = True
            current = self._snapshot_locked()
            listeners = list(self._listeners)
        self._notify(listeners, current)
        return True

    def _report_straggler_locked(self, snapshot: UnitSafetySnapshot) -> None:
        """Queue a report about a writer this observer has already moved past.

        A dead writer leaves a message or two behind; a second *running* writer
        produces one every heartbeat, forever. The count is what separates them,
        so the first report waits for the third straggler and repeats sparsely
        after that.
        """
        if self.stragglers != 3 and self.stragglers % 60 != 0:
            return
        self._pending_reports.append(
            f"a superseded unit-safety writer is still publishing "
            f"({self.stragglers} messages from incarnation "
            f"'{snapshot.incarnation}', writer '{snapshot.writer_id}'): more "
            "than one unit_safety process is alive on this unit. Its "
            "generations are being refused; stop the leftover process."
        )

    def drain_reports(self) -> List[str]:
        """Take the reports queued since the last call, once each.

        A counter says what has happened since process start, so logging on a
        non-zero counter reprints one event for the life of the process — a
        single writer change produced 582 identical error lines in one session.
        Reports are queued where the transition happens and consumed here.
        """
        with self._lock:
            reports = self._pending_reports
            self._pending_reports = []
        return reports

    def _advance(self, *, stopped: bool, reason: str) -> UnitSafetySnapshot:
        with self._lock:
            if not self._writer:
                # An observer minting its own generation is exactly how two
                # processes end up with the same epoch meaning opposite things.
                raise RuntimeError(
                    f"'{self._writer_id or 'observer'}' is not the unit-safety "
                    "writer and may not allocate a generation; request the stop "
                    "from the writer instead"
                )
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
    def motion_ready(self) -> bool:
        """Hardware readiness only. See :meth:`may_command` for permission."""
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

    def may_command(self, owner_id: str) -> Verdict:
        """Would a correctly stamped command from ``owner_id`` be admitted?

        The same checks :meth:`admit` makes, minus the sequence — which is a
        property of the command, not of the permission. This exists so a
        consumer can ask the question it actually cares about ("may I stream?")
        instead of inferring it from ``motion_ready``, which answers only "is
        the hardware ready?". The two differ whenever the device is unowned or
        owned by somebody else, and inferring one from the other is how a
        controller ends up streaming commands that are all refused.
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
            if owner_id != self._owner_id:
                return Verdict(
                    False,
                    Reject.NOT_OWNER,
                    f"'{owner_id}' is not the commander ('{self._owner_id}')",
                )
            return ACCEPTED

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
