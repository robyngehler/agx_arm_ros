"""Unit tests for the Phase 0 baseline counters.

The counters exist to produce the refactor's before/after evidence, so the two
properties that matter are: they cost nothing while disabled, and the SDK-call
attribution really does separate threads — "every SDK call comes from one
worker" is the Phase 1 exit criterion, and this is what will be read to check it.

Test level: **L1**.
"""

import threading

from agx_arm_ctrl.runtime_metrics import RuntimeMetrics


def test_disabled_metrics_record_nothing():
    metrics = RuntimeMetrics(enabled=False)
    metrics.record_sdk_call("get_joint_angles")
    metrics.record_duration("publish_batch", 0.5)
    with metrics.time_block("publish_batch"):
        pass
    assert metrics.report() == ""


def test_duration_summary_reports_count_and_extremes():
    metrics = RuntimeMetrics(enabled=True, report_period_s=0.0)
    for seconds in (0.001, 0.010, 0.004):
        metrics.record_duration("publish_batch", seconds)

    report = metrics.report()
    assert "publish_batch" in report
    assert "n=3" in report
    assert "max=10.00ms" in report
    assert "min=1.00ms" in report


def test_sdk_calls_are_attributed_per_thread():
    """The evidence for 'the SDK has no exclusive caller'.

    Two threads reaching the same call must show up as two entries, because a
    single-worker claim is exactly what this has to be able to contradict.
    """
    metrics = RuntimeMetrics(enabled=True, report_period_s=0.0)

    def worker():
        for _ in range(3):
            metrics.record_sdk_call("get_motor_states")

    other = threading.Thread(target=worker, name="publish-thread")
    other.start()
    other.join()
    metrics.record_sdk_call("get_motor_states")

    report = metrics.report()
    assert "from 2 thread(s)" in report
    assert "publish-thread" in report
    assert "sdk calls: 4" in report


def test_report_resets_the_window():
    metrics = RuntimeMetrics(enabled=True, report_period_s=0.0)
    metrics.record_sdk_call("get_joint_angles")
    assert metrics.report() != ""
    assert metrics.report() == "", "a drained window must not repeat itself"


def test_time_block_measures_the_block():
    metrics = RuntimeMetrics(enabled=True, report_period_s=0.0)
    with metrics.time_block("motor_state_reads"):
        sum(range(10000))
    report = metrics.report()
    assert "motor_state_reads" in report
    assert "n=1" in report
