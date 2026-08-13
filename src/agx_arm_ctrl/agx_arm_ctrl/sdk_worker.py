"""One owner of a device's SDK session at any instant.

Phase 1A of the V02 refactor. Today the arm driver reaches the SDK from the
200 Hz publish thread, from ROS subscription callbacks and from service handlers
at the same time. That is a time-of-check/time-of-use race against the hardware:
a service handler can read a mode, and a callback can change it before the
handler acts on what it read. The Phase 0 measurement showed the same shape as a
CPU problem — a ``prepare_hand_window`` service callback blocked long enough to
cause a publish-loop overrun, because both threads contend for the same GIL and
the same SDK.

Serializing onto one thread per device fixes the race and makes the cost
visible: with every call on one named thread, :class:`RuntimeMetrics` attributes
all SDK traffic to that thread, and any call that still bypasses the worker
shows up under a different thread name in the same report. That is the Phase 1A
exit gate, and it is why the worker counts calls itself.

Two lanes, not more:

* ``SAFETY`` — emergency stop and its immediate support. Drained before
  anything else, never coalesced, never dropped for a stale epoch.
* ``NORMAL`` — everything else, strictly FIFO.

A third "reads first" lane was considered and rejected: reordering a read ahead
of a queued write breaks the causality the callers rely on. Freshness for
streaming setpoints is handled by ``replace_key`` instead, which is ordering-
preserving because it replaces a command that has not started yet.

**Exactly one SDK owner at any instant, and one SDK call per task.** The
safety lane overtakes queued work; nothing preempts work already running. Measured on hardware
(`docs/sprint_refactor/reference/sdk_latency_budget.md`), individual SDK calls
are almost all sub-millisecond — the worst on the hot path is ``move_mit`` at
3.32 ms — so a single call is not what threatens the stop path. The driver's
*composite* operations are: ``_enable_arm`` is a `while not enable()` loop
bounded only by a 5 s timeout, and it is 48 calls of at most 1.15 ms each.
Submitted as one task that becomes a 5 s block on an emergency stop; submitted
as one task per iteration, the safety lane interleaves. Callers keep their loops
and submit each iteration.

The budget that follows: an emergency stop reaches the SDK within 20 ms while
normal work runs. ``sdk_queue_wait`` and ``sdk.<call>`` are timed separately so
a violation says which of the two caused it.

**Recovery does not run here.** A measured `disconnect` blocks for a second in
one call, which no task granularity shortens, so destructive recovery owns the
SDK session exclusively instead of queueing behind — and while it does, this
worker owns nothing. The invariant is one owner at a time, not one thread for
everything; the earlier wording "no SDK call outside the worker" is superseded.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from enum import Enum
from typing import Any, Callable, Deque, Dict, Optional


class Lane(Enum):
    """Dispatch priority. Safety work overtakes queued motion, nothing else does."""

    SAFETY = "safety"
    NORMAL = "normal"


class CallOutcome(Enum):
    """How a submitted call ended."""

    PENDING = "pending"
    DONE = "done"
    FAILED = "failed"
    DROPPED = "dropped"
    REJECTED = "rejected"


class CallNotExecuted(RuntimeError):
    """The call is **guaranteed** not to have reached the hardware.

    Only raised for outcomes that establish that: dropped for a stale epoch,
    superseded, or rejected. Never for a waiter running out of patience.
    """

    def __init__(self, call: "Call", detail: str) -> None:
        super().__init__(detail)
        self.call = call
        self.detail = detail


class CallOutcomeUnknown(RuntimeError):
    """The wait expired. The call may be executing, or may still execute.

    Distinct from :class:`CallNotExecuted` because the difference is the whole
    point for a side-effecting SDK write: a caller that treats a timeout as "not
    sent" and resends has just commanded the hardware twice. A timeout is the
    waiter's experience, not a fact about the call.
    """

    def __init__(self, call: "Call", detail: str) -> None:
        super().__init__(detail)
        self.call = call
        self.detail = detail


class Call:
    """A unit of SDK work and its outcome.

    Callers that need the answer wait on it; callers that only need the command
    delivered can drop the handle. Either way the outcome is recorded, so a
    dropped or rejected command is never mistaken for a delivered one.
    """

    __slots__ = (
        "name",
        "lane",
        "epoch",
        "replace_key",
        "outcome",
        "value",
        "error",
        "detail",
        "queued_at",
        "_fn",
        "_done",
    )

    def __init__(
        self,
        name: str,
        fn: Callable[[], Any],
        lane: Lane,
        epoch: Optional[int],
        replace_key: Optional[str],
    ) -> None:
        self.name = name
        self.lane = lane
        self.epoch = epoch
        self.replace_key = replace_key
        self.outcome = CallOutcome.PENDING
        self.value: Any = None
        self.error: Optional[BaseException] = None
        self.detail = ""
        self.queued_at = time.monotonic()
        self._fn = fn
        self._done = threading.Event()

    @property
    def executed(self) -> bool:
        """True when the call actually reached the SDK, successfully or not."""
        return self.outcome in (CallOutcome.DONE, CallOutcome.FAILED)

    def wait(self, timeout: Optional[float] = None) -> bool:
        """Block until the call settles. False on timeout."""
        return self._done.wait(timeout)

    def result(self, timeout: Optional[float] = None) -> Any:
        """Return the call's value, or raise what stopped it.

        Three outcomes, deliberately not two:

        * the SDK's own exception when the call ran and failed;
        * :class:`CallNotExecuted` when it is *guaranteed* not to have run;
        * :class:`CallOutcomeUnknown` when the wait expired, because the call
          may be executing at that moment or may still execute. Collapsing that
          into "not executed" is what would let a caller resend a command the
          hardware is already acting on.
        """
        if not self._done.wait(timeout):
            raise CallOutcomeUnknown(
                self,
                f"{self.name}: still pending after {timeout}s — it may be "
                "executing or may execute later; do not assume it was not sent",
            )
        if self.outcome is CallOutcome.DONE:
            return self.value
        if self.outcome is CallOutcome.FAILED:
            assert self.error is not None
            raise self.error
        raise CallNotExecuted(self, f"{self.name}: {self.outcome.value} — {self.detail}")

    def _settle(
        self,
        outcome: CallOutcome,
        *,
        value: Any = None,
        error: Optional[BaseException] = None,
        detail: str = "",
    ) -> None:
        self.outcome = outcome
        self.value = value
        self.error = error
        self.detail = detail
        self._done.set()

    def _run(self) -> None:
        try:
            value = self._fn()
        except BaseException as exc:  # noqa: B902 - the SDK raises anything
            self._settle(CallOutcome.FAILED, error=exc, detail=repr(exc))
        else:
            self._settle(CallOutcome.DONE, value=value)


class SdkWorker:
    """Serializes all SDK access for one device onto one named thread.

    ``epoch`` on a submission is the device epoch it was stamped under. When the
    epoch advances — an ownership transition, a recovery, a rearm — queued work
    from the previous epoch is dropped rather than delivered late to hardware
    that has since changed hands.
    """

    def __init__(
        self,
        device_id: str,
        *,
        max_queued: int = 64,
        metrics: Any = None,
        logger: Any = None,
    ) -> None:
        self.device_id = device_id
        self.max_queued = max_queued
        self._metrics = metrics
        self._logger = logger

        self._lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self._safety: Deque[Call] = deque()
        self._normal: Deque[Call] = deque()
        self._by_key: Dict[str, Call] = {}
        self._epoch: Optional[int] = None
        self._running = True

        self.dropped_stale = 0
        self.dropped_replaced = 0
        self.rejected_full = 0

        # The thread name is evidence, not decoration: RuntimeMetrics keys SDK
        # calls by thread, so "one thread per arm" is checked by reading it.
        self._thread = threading.Thread(
            target=self._run, name=f"sdk-{device_id}", daemon=True
        )
        self._thread.start()

    # -- submission -----------------------------------------------------

    def submit(
        self,
        name: str,
        fn: Callable[[], Any],
        *,
        lane: Lane = Lane.NORMAL,
        epoch: Optional[int] = None,
        replace_key: Optional[str] = None,
    ) -> Call:
        """Queue one SDK call and return its handle.

        ``replace_key`` supersedes an identically-keyed call that has not
        started yet — the streaming-setpoint case, where the newest setpoint is
        the only one worth sending.
        """
        call = Call(name, fn, lane, epoch, replace_key)

        # Re-entrant submission from the worker itself would deadlock if it
        # waited. Execution is already serialized on this thread, so run it
        # inline; ordering against the queue is the caller's problem and lane
        # priority does not apply to work that is already running.
        if threading.current_thread() is self._thread:
            self._execute(call)
            return call

        with self._lock:
            if not self._running:
                call._settle(
                    CallOutcome.REJECTED, detail=f"{self.device_id} worker is stopped"
                )
                return call
            if lane is Lane.NORMAL and self._is_stale_locked(epoch):
                self.dropped_stale += 1
                call._settle(
                    CallOutcome.DROPPED,
                    detail=f"epoch {epoch} is behind {self._epoch}",
                )
                return call
            if lane is Lane.NORMAL and len(self._normal) >= self.max_queued:
                self.rejected_full += 1
                call._settle(
                    CallOutcome.REJECTED,
                    detail=(
                        f"{self.device_id} queue full "
                        f"({len(self._normal)}/{self.max_queued})"
                    ),
                )
                return call
            if replace_key is not None and lane is Lane.NORMAL:
                self._replace_locked(replace_key)
                self._by_key[replace_key] = call
            if lane is Lane.SAFETY:
                self._safety.append(call)
            else:
                self._normal.append(call)
            self._wake.notify()
        return call

    def submit_safety(self, name: str, fn: Callable[[], Any]) -> Call:
        """Queue a call on the safety lane, ahead of everything queued."""
        return self.submit(name, fn, lane=Lane.SAFETY)

    def call(
        self,
        name: str,
        fn: Callable[[], Any],
        *,
        timeout: float = 1.0,
        lane: Lane = Lane.NORMAL,
        epoch: Optional[int] = None,
    ) -> Any:
        """Submit and wait. Convenience for the read paths that need the value."""
        return self.submit(name, fn, lane=lane, epoch=epoch).result(timeout)

    def cancel_if_pending(self, call: Call) -> bool:
        """Cancel a call **only** while it is still queued.

        Returns True when it is now guaranteed not to run. Returns False when it
        has started, finished, or already settled — there is no way to unsend an
        SDK call, and pretending otherwise is how a caller ends up believing the
        hardware was left untouched.

        This is the honest counterpart to a wait timeout: a timeout says the
        outcome is unknown, and this is the one operation that can make it
        known, by removing the call before it ever reaches the SDK.
        """
        with self._lock:
            if call.outcome is not CallOutcome.PENDING:
                return False
            for queue in (self._safety, self._normal):
                try:
                    queue.remove(call)
                except ValueError:
                    continue
                if call.replace_key is not None:
                    if self._by_key.get(call.replace_key) is call:
                        del self._by_key[call.replace_key]
                break
            else:
                # Pending but not in a queue: the worker has already taken it.
                return False
        call._settle(CallOutcome.REJECTED, detail="cancelled while queued")
        return True

    # -- epoch ----------------------------------------------------------

    def set_epoch(self, epoch: int) -> int:
        """Advance the worker's epoch and drop queued work from before it.

        Returns how many queued calls were dropped. Safety-lane work is never
        dropped: an emergency stop is still the right thing to send to hardware
        whose ownership just changed.
        """
        dropped = []
        with self._lock:
            if self._epoch is not None and epoch < self._epoch:
                return 0
            self._epoch = epoch
            keep: Deque[Call] = deque()
            while self._normal:
                call = self._normal.popleft()
                if call.epoch is not None and call.epoch < epoch:
                    dropped.append(call)
                else:
                    keep.append(call)
            self._normal = keep
            for call in dropped:
                if call.replace_key is not None:
                    self._by_key.pop(call.replace_key, None)
            self.dropped_stale += len(dropped)
        for call in dropped:
            call._settle(
                CallOutcome.DROPPED, detail=f"epoch {call.epoch} is behind {epoch}"
            )
        return len(dropped)

    def _is_stale_locked(self, epoch: Optional[int]) -> bool:
        return epoch is not None and self._epoch is not None and epoch < self._epoch

    def _replace_locked(self, replace_key: str) -> None:
        previous = self._by_key.pop(replace_key, None)
        if previous is None or previous.outcome is not CallOutcome.PENDING:
            return
        try:
            self._normal.remove(previous)
        except ValueError:
            return
        self.dropped_replaced += 1
        previous._settle(
            CallOutcome.DROPPED, detail=f"superseded by a newer {replace_key}"
        )

    # -- execution ------------------------------------------------------

    def _run(self) -> None:
        while True:
            stale: Optional[Call] = None
            stale_detail = ""
            with self._lock:
                while self._running and not self._safety and not self._normal:
                    self._wake.wait()
                if not self._running and not self._safety:
                    return
                if self._safety:
                    call = self._safety.popleft()
                else:
                    call = self._normal.popleft()
                    if call.replace_key is not None:
                        if self._by_key.get(call.replace_key) is call:
                            del self._by_key[call.replace_key]
                    if self._is_stale_locked(call.epoch):
                        # The epoch advanced while this sat in the queue.
                        self.dropped_stale += 1
                        stale_detail = f"epoch {call.epoch} is behind {self._epoch}"
                        stale, call = call, None
            if stale is not None:
                # Settled outside the lock, so a waiter woken by it cannot run
                # while the queue is held.
                stale._settle(CallOutcome.DROPPED, detail=stale_detail)
                continue
            self._execute(call)

    def _execute(self, call: Call) -> None:
        if self._metrics is not None:
            self._metrics.record_duration(
                "sdk_queue_wait", time.monotonic() - call.queued_at
            )
            self._metrics.record_sdk_call(call.name)
            with self._metrics.time_block(f"sdk.{call.name}"):
                call._run()
        else:
            call._run()
        if call.outcome is CallOutcome.FAILED and self._logger is not None:
            self._logger.error(f"SDK call {call.name} failed: {call.detail}")

    # -- shutdown -------------------------------------------------------

    def shutdown(self, timeout: float = 2.0) -> None:
        """Stop the worker, dropping whatever was still queued."""
        with self._lock:
            self._running = False
            pending = list(self._normal)
            self._normal.clear()
            self._by_key.clear()
            self._wake.notify_all()
        for call in pending:
            call._settle(
                CallOutcome.REJECTED, detail=f"{self.device_id} worker shut down"
            )
        self._thread.join(timeout)

    @property
    def queue_depth(self) -> int:
        with self._lock:
            return len(self._safety) + len(self._normal)

    @property
    def thread_name(self) -> str:
        return self._thread.name
