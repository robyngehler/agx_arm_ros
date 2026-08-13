"""One authoritative answer to "may another activity start on this unit?".

Phase 1C of the V02 refactor, and deliberately small: this is the exclusivity
guard, not the event-driven coordinator rewrite that follows in Phase 3.

It is pulled ahead of the parallel-operation work on purpose. Each device now
has its own CAN bus, so a later phase lets same-side arm and hand motion run at
the same time — which multiplies the ways two activities can interleave. The
rule that only one activity owns the unit has to exist *before* that
parallelism, not after it.

What it replaces: the coordinator accepted every goal unconditionally
(``goal_callback=lambda _req: GoalResponse.ACCEPT``) and tracked a running
activity in a plain boolean that nothing consulted before dispatching. Two
overlapping goals would both have been executed, against the same arms.

The state is two values and a reason:

.. code-block:: text

    READY      -> accept one activity
    EXECUTING  -> reject every further goal, with a structured reason
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class UnitActivityState(Enum):
    """Whether the unit is free to take an activity."""

    READY = "ready"
    EXECUTING = "executing"


class RejectReason(Enum):
    """Why an activity was not admitted."""

    UNIT_BUSY = "unit_busy"
    UNIT_STOPPING = "unit_stopping"
    UNIT_STOPPED = "unit_stopped"
    UNIT_SAFETY_UNKNOWN = "unit_safety_unknown"


@dataclass(frozen=True)
class Admission:
    """The decision, and — when refused — a reason a caller can act on."""

    accepted: bool
    reason: Optional[RejectReason] = None
    detail: str = ""

    def __bool__(self) -> bool:
        """Allow ``if admission:`` where only the outcome matters."""
        return self.accepted


ADMITTED = Admission(accepted=True)


class UnitActivity:
    """Tracks the one activity a unit may be running, and refuses the rest.

    :meth:`can_accept` is the cheap non-mutating check for the action server's
    goal callback. :meth:`try_claim` is the authoritative one: it decides and
    takes the slot in a single step, so two goals that both passed the goal
    callback cannot both start executing.
    """

    def __init__(
        self,
        *,
        require_unit_safety: bool = True,
        unit_safety_timeout_s: float = 6.0,
        clock=time.monotonic,
    ) -> None:
        """Guard admission for one unit.

        ``require_unit_safety`` is fail-closed by default: a unit whose safety
        generation cannot be established is a unit that must not be given new
        work. Losing the writer mid-run never stops what is already authorised
        — that is the point of the split — but starting something new commits
        the unit to motion it could not afterwards invalidate.
        """
        self._lock = threading.Lock()
        self._activity_id = ""
        self._stopping = False
        self._stop_reason = ""
        self.rejected_count = 0

        self._require_unit_safety = require_unit_safety
        self._unit_safety_timeout_s = unit_safety_timeout_s
        self._clock = clock
        self._unit_safety_epoch = -1
        self._unit_stopped = False
        self._unit_safety_reason = ""
        self._unit_safety_seen_at = None

    @property
    def state(self) -> UnitActivityState:
        with self._lock:
            return (
                UnitActivityState.EXECUTING
                if self._activity_id
                else UnitActivityState.READY
            )

    @property
    def activity_id(self) -> str:
        """The running activity, or an empty string when the unit is ready."""
        with self._lock:
            return self._activity_id

    @property
    def is_running(self) -> bool:
        with self._lock:
            return bool(self._activity_id)

    def can_accept(self, activity_id: str) -> Admission:
        """Report whether an activity would be admitted, without taking the slot."""
        with self._lock:
            return self._admission_locked(activity_id)

    def try_claim(self, activity_id: str) -> Admission:
        """Take the unit for one activity, or refuse with a reason."""
        with self._lock:
            admission = self._admission_locked(activity_id)
            if admission.accepted:
                self._activity_id = activity_id
            else:
                self.rejected_count += 1
            return admission

    def observe_unit_safety(
        self, *, epoch: int, stopped: bool, reason: str = ""
    ) -> None:
        """Record the writer's current generation, and that it is still alive.

        Called for every message including the writer's heartbeat, because the
        timestamp is the liveness signal: the latched value survives the writer,
        so a stale value and a live one are otherwise indistinguishable.
        """
        with self._lock:
            self._unit_safety_epoch = int(epoch)
            self._unit_stopped = bool(stopped)
            self._unit_safety_reason = reason
            self._unit_safety_seen_at = self._clock()

    @property
    def unit_safety_known(self) -> bool:
        """Whether the unit's safety generation is currently established."""
        with self._lock:
            return self._unit_safety_known_locked()

    def _unit_safety_known_locked(self) -> bool:
        if self._unit_safety_seen_at is None:
            return False
        return (
            self._clock() - self._unit_safety_seen_at
        ) <= self._unit_safety_timeout_s

    def _admission_locked(self, activity_id: str) -> Admission:
        if self._stopping:
            return Admission(
                False,
                RejectReason.UNIT_STOPPING,
                f"coordinator is stopping ({self._stop_reason})",
            )
        if self._unit_stopped and self._unit_safety_known_locked():
            return Admission(
                False,
                RejectReason.UNIT_STOPPED,
                f"unit safety stop is in force at generation "
                f"{self._unit_safety_epoch} ({self._unit_safety_reason}); "
                f"'{activity_id}' was not started",
            )
        if self._require_unit_safety and not self._unit_safety_known_locked():
            seen = (
                "none has ever arrived"
                if self._unit_safety_seen_at is None
                else f"the last was {self._clock() - self._unit_safety_seen_at:.1f}s ago"
            )
            return Admission(
                False,
                RejectReason.UNIT_SAFETY_UNKNOWN,
                f"unit safety state is not established ({seen}); "
                f"'{activity_id}' was not started. A running activity is "
                "unaffected — this refuses new work only",
            )
        if self._activity_id:
            return Admission(
                False,
                RejectReason.UNIT_BUSY,
                f"activity '{self._activity_id}' is already running; "
                f"'{activity_id}' was not started",
            )
        return ADMITTED

    def release(self, activity_id: str) -> None:
        """Give the unit back. Releasing an activity that is not the current one
        is ignored, so a late unwind cannot free a slot someone else holds.
        """
        with self._lock:
            if self._activity_id == activity_id:
                self._activity_id = ""

    def begin_stop(self, reason: str) -> bool:
        """Refuse further activities and report whether one is still running.

        The caller uses the return value to decide whether it may release the
        process immediately or has to let the running activity unwind first.
        """
        with self._lock:
            if not self._stopping:
                self._stopping = True
                self._stop_reason = reason
            return bool(self._activity_id)

    @property
    def stopping(self) -> bool:
        with self._lock:
            return self._stopping

    @property
    def stop_reason(self) -> str:
        with self._lock:
            return self._stop_reason
