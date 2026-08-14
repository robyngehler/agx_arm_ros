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

Four lanes, in strict priority: ``SAFETY``, ``CONTROL``, ``ACQUISITION``,
``DIAGNOSTIC`` (see :class:`Lane`). Two were not enough. With everything that is
not a stop sharing one queue, the 200 Hz acquisition loop had to wait behind
every setpoint and lost half its rate on hardware — the feedback a control loop
depends on was competing with status reads on equal terms.

A "reads first" lane was considered and rejected: reordering a read ahead of a
queued write breaks the causality the callers rely on. Freshness for streaming
setpoints is handled by ``replace_key`` instead, which is ordering-preserving
because it replaces a command that has not started yet.

**Exactly one SDK owner at any instant.** The safety lane overtakes queued work;
nothing preempts a call already running, so the unit of work decides how long a
stop can be delayed.

That unit is neither "one call" nor "one command". Measured on hardware
(`docs/sprint_refactor/reference/sdk_latency_budget.md`): a MIT setpoint
submitted as a single task costs 6.4 ms mean and 21 ms worst case — the entire
stop budget spent inside one queue entry. Submitted as seven independent calls
instead, two setpoints interleave and the arm holds half of each. So a command
that is several transmits but one instruction is submitted as a *cycle*
(:meth:`SdkWorker.submit_cycle`): one entry for queueing, superseding and the
epoch check, executed one step at a time with the safety lane drained between
steps.

Retry loops are still not cycles. ``_enable_arm`` is a `while not enable()` loop
bounded only by a 5 s timeout; it stays on its calling thread and submits each
iteration, because a cycle runs to completion once started and a 5 s cycle would
reintroduce exactly the block this structure exists to remove.

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

try:  # instrumentation only; the worker must run without it
    from agx_arm_ctrl.runtime_metrics import name_os_thread as _name_os_thread
except Exception:  # pragma: no cover
    def _name_os_thread(_name):
        return False


class Lane(Enum):
    """Dispatch priority, strictly in this order.

    Four rather than two, because "everything that is not a stop" turned out to
    contain work with very different claims on the device. Measured on hardware
    with one lane for all of it: the acquisition loop lost half its rate to the
    command stream, because a 200 Hz read had to queue behind every setpoint.

    * ``SAFETY`` — emergency stop and its immediate support. Never dropped for a
      stale epoch, never coalesced, never refused for a full queue.
    * ``CONTROL`` — active control transmits. The setpoints that are currently
      moving the device outrank reading it.
    * ``ACQUISITION`` — the feedback the control loop and the watchdog depend on.
    * ``DIAGNOSTIC`` — status and one-off reads. Nothing waits on these to keep
      a device safe or moving.

    Strict priority, so a saturated ``CONTROL`` lane can starve the two below it.
    That is deliberate — a device that cannot be commanded is worse than one
    whose diagnostics are late — but it is a property to watch, not a free lunch,
    and the ordering is only correct while control traffic is bounded.
    """

    SAFETY = "safety"
    CONTROL = "control"
    ACQUISITION = "acquisition"
    DIAGNOSTIC = "diagnostic"


#: Dispatch order. The worker drains each lane fully before looking at the next.
LANE_ORDER = (Lane.SAFETY, Lane.CONTROL, Lane.ACQUISITION, Lane.DIAGNOSTIC)


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
        "steps",
        "always",
        "_fn",
        "_done",
    )

    def __init__(
        self,
        name: str,
        fn: Optional[Callable[[], Any]],
        lane: Lane,
        epoch: Optional[int],
        replace_key: Optional[str],
        steps: Optional[list] = None,
        always: Optional[tuple] = None,
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
        # A cycle: several SDK calls that are one logical command. Queued,
        # superseded and epoch-checked as a unit, but executed one step at a
        # time so the safety lane runs between them. ``always`` closes whatever
        # the first step opened, and runs even when a step raises.
        self.steps = steps
        self.always = always
        self._fn = fn
        self._done = threading.Event()

    @property
    def is_cycle(self) -> bool:
        """True when this is several SDK calls executed as one command."""
        return self.steps is not None

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
        self._queues: Dict[Lane, Deque[Call]] = {
            lane: deque() for lane in LANE_ORDER
        }
        self._by_key: Dict[str, Call] = {}
        self._epoch: Optional[int] = None
        self._running = True
        # Quiesced: the worker stops dequeuing but keeps accepting
        # submissions, so a caller does not have to know that ownership has
        # moved. Work queued meanwhile is subject to the usual epoch rules,
        # which is what stops it arriving stale after recovery.
        self._quiesced = False
        self._executing = False
        self._idle = threading.Condition(self._lock)

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
        lane: Lane = Lane.DIAGNOSTIC,
        epoch: Optional[int] = None,
        replace_key: Optional[str] = None,
    ) -> Call:
        """Queue one SDK call and return its handle.

        ``replace_key`` supersedes an identically-keyed call that has not
        started yet — the streaming-setpoint case, where the newest setpoint is
        the only one worth sending.

        The default lane is the lowest one on purpose: work nobody classified
        must not overtake the control stream on the strength of an omission.
        """
        return self._enqueue(Call(name, fn, lane, epoch, replace_key))

    def submit_cycle(
        self,
        name: str,
        steps: list,
        *,
        lane: Lane = Lane.CONTROL,
        epoch: Optional[int] = None,
        replace_key: Optional[str] = None,
        always: Optional[tuple] = None,
    ) -> Call:
        """Queue several SDK calls as one logical command.

        For a command that is several transmits but one instruction to the
        device — a MIT setpoint is seven joint frames inside a mode bracket.
        Submitting it as a single call made the whole thing non-preemptible, and
        it was measured at 6.4 ms mean and 21 ms worst case, which is the entire
        emergency-stop budget spent inside one queue entry. Submitting each
        frame separately would instead let two setpoints interleave and leave
        the arm holding half of each.

        A cycle is both: **one** entry for queueing, superseding and the epoch
        check, executed **one step at a time** with the safety lane drained in
        between. ``steps`` is a list of ``(name, fn)``; ``always`` is one more
        such pair, run at the end whether or not a step raised, for closing what
        the first step opened.
        """
        return self._enqueue(
            Call(name, None, lane, epoch, replace_key, steps=list(steps), always=always)
        )

    def _enqueue(self, call: Call) -> Call:
        lane = call.lane

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
            # The safety lane is exempt from every refusal below: a stop is
            # still the right thing to send to a device whose epoch moved on,
            # and dropping it for a full queue would be the worst possible time.
            if lane is not Lane.SAFETY:
                if self._is_stale_locked(call.epoch):
                    self.dropped_stale += 1
                    call._settle(
                        CallOutcome.DROPPED,
                        detail=f"epoch {call.epoch} is behind {self._epoch}",
                    )
                    return call
                queue = self._queues[lane]
                if len(queue) >= self.max_queued:
                    self.rejected_full += 1
                    call._settle(
                        CallOutcome.REJECTED,
                        detail=(
                            f"{self.device_id} {lane.value} queue full "
                            f"({len(queue)}/{self.max_queued})"
                        ),
                    )
                    return call
                if call.replace_key is not None:
                    self._replace_locked(call.replace_key)
                    self._by_key[call.replace_key] = call
            self._queues[lane].append(call)
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
        lane: Lane = Lane.DIAGNOSTIC,
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
            for queue in self._queues.values():
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
            for lane in LANE_ORDER:
                if lane is Lane.SAFETY:
                    continue
                keep: Deque[Call] = deque()
                while self._queues[lane]:
                    call = self._queues[lane].popleft()
                    if call.epoch is not None and call.epoch < epoch:
                        dropped.append(call)
                    else:
                        keep.append(call)
                self._queues[lane] = keep
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
            self._queues[previous.lane].remove(previous)
        except ValueError:
            # Already taken by the worker. A cycle that has started is not
            # superseded half-way: it finishes, and the newer one follows.
            return
        self.dropped_replaced += 1
        previous._settle(
            CallOutcome.DROPPED, detail=f"superseded by a newer {replace_key}"
        )

    # -- ownership ------------------------------------------------------

    def quiesce(self, timeout: float = 5.0) -> bool:
        """Stop dequeuing and wait for the in-flight call to finish.

        Returns True once this worker is guaranteed to be touching nothing, so
        another owner — recovery — may take the SDK session. False means a call
        is still running and ownership must NOT be transferred: two owners on
        one session is the race the worker exists to remove, and taking it back
        by force would only move the race.

        Submissions keep being accepted while quiesced. A caller should not
        have to know that ownership moved, and anything queued here is still
        subject to the epoch rules, which is what stops it arriving stale on the
        far side of a recovery.
        """
        with self._lock:
            self._quiesced = True
            self._wake.notify_all()
            deadline = time.monotonic() + timeout
            while self._executing:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._idle.wait(remaining)
            return True

    def resume(self) -> None:
        """Take the SDK session back and start dequeuing again."""
        with self._lock:
            self._quiesced = False
            self._wake.notify_all()

    @property
    def quiesced(self) -> bool:
        with self._lock:
            return self._quiesced

    # -- execution ------------------------------------------------------

    def _has_work_locked(self) -> bool:
        return any(self._queues[lane] for lane in LANE_ORDER)

    def _next_call_locked(self) -> Optional[Call]:
        """Pop the highest-priority queued call. Strict lane order."""
        for lane in LANE_ORDER:
            queue = self._queues[lane]
            if queue:
                return queue.popleft()
        return None

    def _run(self) -> None:
        _name_os_thread(self._thread.name)
        while True:
            stale: Optional[Call] = None
            stale_detail = ""
            with self._lock:
                while self._running and (
                    self._quiesced or not self._has_work_locked()
                ):
                    self._wake.wait()
                if not self._running and not self._queues[Lane.SAFETY]:
                    return
                if self._quiesced and self._running:
                    # Woken spuriously while quiesced. Shutdown deliberately
                    # falls through instead: a stop still queued on the safety
                    # lane must drain even if ownership was handed away.
                    continue
                self._executing = True
                call = self._next_call_locked()
                if call.lane is not Lane.SAFETY:
                    if call.replace_key is not None:
                        if self._by_key.get(call.replace_key) is call:
                            del self._by_key[call.replace_key]
                    if self._is_stale_locked(call.epoch):
                        # The epoch advanced while this sat in the queue.
                        self.dropped_stale += 1
                        stale_detail = f"epoch {call.epoch} is behind {self._epoch}"
                        stale, call = call, None
                        self._executing = False
                        self._idle.notify_all()
            if stale is not None:
                # Settled outside the lock, so a waiter woken by it cannot run
                # while the queue is held.
                stale._settle(CallOutcome.DROPPED, detail=stale_detail)
                continue
            try:
                self._execute(call)
            finally:
                # Marks the end of "this worker is touching the SDK". quiesce()
                # waits on exactly this, because ownership may only move while
                # no call is in flight.
                with self._lock:
                    self._executing = False
                    self._idle.notify_all()

    def _take_safety_locked(self) -> Optional[Call]:
        queue = self._queues[Lane.SAFETY]
        return queue.popleft() if queue else None

    def _drain_safety(self) -> None:
        """Run whatever is waiting on the safety lane, right now.

        Called between the steps of a cycle. This is what makes the
        non-preemptible unit one SDK call instead of one whole command: a stop
        that arrives while a seven-frame setpoint is going out no longer waits
        for the last frame.
        """
        while True:
            with self._lock:
                call = self._take_safety_locked()
            if call is None:
                return
            self._execute(call)

    def _execute(self, call: Call) -> None:
        if self._metrics is not None:
            # Per lane, not one aggregate. The lanes have opposite expectations:
            # a diagnostic read waiting behind the control stream is the design
            # working, while the safety lane's wait *is* the emergency-stop
            # budget. Averaging them together produced a number that could not
            # answer either question.
            self._metrics.record_duration(
                f"sdk_queue_wait.{call.lane.value}", time.monotonic() - call.queued_at
            )
        if call.is_cycle:
            self._execute_cycle(call)
        elif self._metrics is not None:
            self._metrics.record_sdk_call(call.name)
            with self._metrics.time_block(f"sdk.{call.name}"):
                call._run()
        else:
            call._run()
        if call.outcome is CallOutcome.FAILED and self._logger is not None:
            self._logger.error(f"SDK call {call.name} failed: {call.detail}")

    @staticmethod
    def _run_step(name: str, fn: Callable[[], Any]) -> None:
        """Run one step of a cycle.

        Deliberately not instrumented. ``MeasuredSdk`` already counts and times
        every real SDK call by its own name, and a step named after the call it
        makes would be recorded twice — which is what a first hardware run
        showed as 1400 ``move_mit`` per second for 700 frames. The step is a
        preemption boundary; the cycle as a whole is the timed unit.
        """
        fn()

    def _execute_cycle(self, call: Call) -> None:
        """Run a cycle step by step, letting the safety lane in between.

        The cycle is still atomic against other work on its own lane — no second
        setpoint interleaves with this one — because it was dequeued as one
        entry. Only the safety lane is allowed in, which is the whole point.
        """
        started = time.monotonic()
        error: Optional[BaseException] = None
        try:
            for step_name, step_fn in call.steps:
                self._run_step(step_name, step_fn)
                self._drain_safety()
        except BaseException as exc:  # noqa: B902 - the SDK raises anything
            error = exc
        finally:
            if call.always is not None:
                always_name, always_fn = call.always
                try:
                    self._run_step(always_name, always_fn)
                except BaseException as exc:  # noqa: B902
                    # Closing failed. Report the original cause if there was
                    # one: it is what explains the state the device is in.
                    if error is None:
                        error = exc
        if self._metrics is not None:
            self._metrics.record_duration(
                f"sdk.{call.name}", time.monotonic() - started
            )
        if error is not None:
            call._settle(CallOutcome.FAILED, error=error, detail=repr(error))
        else:
            call._settle(CallOutcome.DONE)

    # -- shutdown -------------------------------------------------------

    def shutdown(self, timeout: float = 2.0) -> None:
        """Stop the worker, dropping whatever was still queued."""
        with self._lock:
            self._running = False
            pending = []
            for lane in LANE_ORDER:
                if lane is Lane.SAFETY:
                    # Left queued on purpose: the run loop drains the safety
                    # lane on the way out, so a stop is not stranded by shutdown.
                    continue
                pending.extend(self._queues[lane])
                self._queues[lane].clear()
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
            return sum(len(self._queues[lane]) for lane in LANE_ORDER)

    @property
    def thread_name(self) -> str:
        return self._thread.name
