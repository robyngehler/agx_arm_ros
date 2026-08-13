"""L1 tests for the serialized per-device SDK worker.

The Phase 1A exit gate is **exactly one SDK owner at any instant**: steady-state
commands and reads through this worker, and destructive recovery owning the SDK
session exclusively while the device is recovering. These tests hold the worker
to the steady-state half — one thread, safety ahead of queued motion, and work
from a superseded epoch never reaching the hardware.

Note what this worker does **not** give: priority is non-preemptive once a call
has started, and a measured 1 s blocking `disconnect` is why recovery is not
routed through here at all.
"""

import threading
import time

import pytest

from agx_arm_ctrl.runtime_metrics import RuntimeMetrics
from agx_arm_ctrl.sdk_worker import (
    CallNotExecuted,
    CallOutcome,
    CallOutcomeUnknown,
    SdkWorker,
)


@pytest.fixture
def worker():
    worker = SdkWorker("arm_left")
    yield worker
    worker.shutdown()


def _occupy(worker):
    """Block the worker on one long call and return the gate that frees it.

    Without this the queue tests race the worker: it may drain a submission
    before the next one is made, and then queue order proves nothing.
    """
    started = threading.Event()
    gate = threading.Event()

    def blocker():
        started.set()
        gate.wait(5.0)

    worker.submit("blocking_read", blocker)
    assert started.wait(2.0), "worker never picked up the blocking call"
    return gate


def test_every_call_runs_on_the_one_worker_thread(worker):
    threads = set()

    def record():
        threads.add(threading.current_thread().name)
        return "ok"

    for _ in range(5):
        assert worker.call("get_arm_status", record, timeout=1.0) == "ok"

    assert threads == {worker.thread_name}
    assert worker.thread_name == "sdk-arm_left"


def test_the_sdk_exception_reaches_the_caller(worker):
    def boom():
        raise OSError(105, "No buffer space available")

    with pytest.raises(OSError):
        worker.call("joint_ctrl", boom, timeout=1.0)


def test_a_call_that_never_ran_is_not_a_call_that_failed(worker):
    """A dropped command must never look like a delivered one."""
    worker.set_epoch(5)
    call = worker.submit("joint_ctrl", lambda: "sent", epoch=4)

    assert call.outcome is CallOutcome.DROPPED
    assert not call.executed
    with pytest.raises(CallNotExecuted):
        call.result(timeout=0.1)


def test_safety_work_overtakes_queued_motion(worker):
    gate = _occupy(worker)
    order = []

    for index in range(4):
        worker.submit(f"joint_ctrl_{index}", lambda i=index: order.append(f"motion{i}"))
    safety = worker.submit_safety("emergency_stop", lambda: order.append("estop"))

    gate.set()
    assert safety.wait(2.0)
    assert order[0] == "estop", f"safety ran behind queued motion: {order}"


def test_an_epoch_bump_drops_queued_motion_before_it_reaches_the_sdk(worker):
    gate = _occupy(worker)
    sent = []

    stale = [
        worker.submit("joint_ctrl", lambda i=i: sent.append(i), epoch=1)
        for i in range(3)
    ]

    assert worker.set_epoch(2) == 3
    gate.set()
    fresh = worker.submit("joint_ctrl", lambda: sent.append("fresh"), epoch=2)
    assert fresh.wait(2.0)

    assert sent == ["fresh"]
    assert all(call.outcome is CallOutcome.DROPPED for call in stale)


def test_an_epoch_bump_does_not_drop_safety_work(worker):
    """Ownership changing hands is no reason not to stop the hardware."""
    gate = _occupy(worker)
    ran = []

    estop = worker.submit_safety("emergency_stop", lambda: ran.append("estop"))
    worker.set_epoch(9)
    gate.set()

    assert estop.wait(2.0)
    assert ran == ["estop"]


def test_a_stale_setpoint_is_replaced_rather_than_queued(worker):
    gate = _occupy(worker)
    sent = []

    superseded = worker.submit(
        "mit_setpoint", lambda: sent.append(1), replace_key="mit_setpoint"
    )
    latest = worker.submit(
        "mit_setpoint", lambda: sent.append(2), replace_key="mit_setpoint"
    )
    gate.set()

    assert latest.wait(2.0)
    assert superseded.outcome is CallOutcome.DROPPED
    assert sent == [2]
    assert worker.dropped_replaced == 1


def test_a_full_queue_rejects_the_new_call_and_keeps_the_accepted_ones():
    worker = SdkWorker("arm_left", max_queued=3)
    try:
        gate = _occupy(worker)
        sent = []

        accepted = [
            worker.submit("joint_ctrl", lambda i=i: sent.append(i)) for i in range(3)
        ]
        overflow = worker.submit("joint_ctrl", lambda: sent.append("overflow"))

        assert overflow.outcome is CallOutcome.REJECTED
        assert worker.rejected_full == 1

        gate.set()
        assert accepted[-1].wait(2.0)
        assert sent == [0, 1, 2], "an accepted call was silently discarded"
    finally:
        worker.shutdown()


def test_shutdown_rejects_what_it_could_not_send():
    worker = SdkWorker("arm_left")
    gate = _occupy(worker)
    pending = worker.submit("joint_ctrl", lambda: "sent")

    worker.shutdown(timeout=0.1)
    gate.set()

    assert pending.outcome is CallOutcome.REJECTED
    assert "shut down" in pending.detail

    after = worker.submit("joint_ctrl", lambda: "sent")
    assert after.outcome is CallOutcome.REJECTED


def test_a_call_submitted_from_inside_a_call_does_not_deadlock(worker):
    def outer():
        return worker.call("inner_read", lambda: "inner", timeout=1.0)

    assert worker.call("outer_read", outer, timeout=2.0) == "inner"


def test_the_metrics_report_attributes_every_call_to_one_thread():
    metrics = RuntimeMetrics(enabled=True, report_period_s=0.0)
    worker = SdkWorker("arm_left", metrics=metrics)
    try:
        for _ in range(3):
            worker.call("get_motor_states", lambda: 0.0, timeout=1.0)
        report = metrics.report()
    finally:
        worker.shutdown()

    assert "from 1 thread(s): sdk-arm_left" in report
    assert "get_motor_states[sdk-arm_left]: 3" in report


def test_calls_stay_in_submission_order(worker):
    gate = _occupy(worker)
    order = []

    for index in range(10):
        worker.submit(f"call_{index}", lambda i=index: order.append(i))
    last = worker.submit("call_last", lambda: order.append("last"))
    gate.set()

    assert last.wait(2.0)
    assert order == list(range(10)) + ["last"]


def test_queue_depth_drains(worker):
    for _ in range(5):
        worker.submit("noop", lambda: None)
    deadline = time.monotonic() + 2.0
    while worker.queue_depth and time.monotonic() < deadline:
        time.sleep(0.01)
    assert worker.queue_depth == 0


# --- timeout is not a statement about the hardware ---------------------------

def test_a_wait_timeout_does_not_claim_the_call_was_not_executed(worker):
    """The distinction that matters for a side-effecting write.

    A caller that reads a timeout as "not sent" and resends has commanded the
    hardware twice. A timeout is the waiter's experience, not a fact about the
    call.
    """
    gate = _occupy(worker)
    queued = worker.submit("joint_ctrl", lambda: "sent")
    try:
        with pytest.raises(CallOutcomeUnknown) as excinfo:
            queued.result(timeout=0.05)
        assert "may execute later" in str(excinfo.value)
        assert not isinstance(excinfo.value, CallNotExecuted)
    finally:
        gate.set()
        queued.wait(2.0)
    # And it did in fact execute, which is exactly why the claim would be wrong.
    assert queued.outcome is CallOutcome.DONE


def test_a_dropped_call_still_reports_guaranteed_non_execution(worker):
    worker.set_epoch(5)
    call = worker.submit("joint_ctrl", lambda: "sent", epoch=4)
    with pytest.raises(CallNotExecuted):
        call.result(timeout=0.1)


def test_cancelling_a_queued_call_guarantees_it_will_not_run(worker):
    gate = _occupy(worker)
    sent = []
    call = worker.submit("joint_ctrl", lambda: sent.append("ran"))

    assert worker.cancel_if_pending(call) is True
    gate.set()
    assert call.wait(2.0)

    assert sent == []
    assert call.outcome is CallOutcome.REJECTED
    with pytest.raises(CallNotExecuted):
        call.result(timeout=0.1)


def test_cancelling_a_finished_call_reports_that_it_could_not(worker):
    call = worker.submit("joint_ctrl", lambda: "sent")
    assert call.wait(2.0)
    assert worker.cancel_if_pending(call) is False


def test_cancelling_a_call_already_taken_by_the_worker_reports_false(worker):
    """There is no way to unsend an SDK call, and this must not pretend."""
    started = threading.Event()
    release = threading.Event()

    def slow():
        started.set()
        release.wait(3.0)

    call = worker.submit("blocking_write", slow)
    assert started.wait(2.0)
    try:
        assert worker.cancel_if_pending(call) is False
    finally:
        release.set()
        call.wait(3.0)
    assert call.outcome is CallOutcome.DONE
