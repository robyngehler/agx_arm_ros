"""Low-overhead runtime counters for the Phase 0 baseline.

The refactor's performance claims need a before/after, and the numbers that
matter are not visible from outside the process: how long a loop iteration
actually took, how many blocking SDK calls one cycle made, and which thread made
them. `docs/sprint_refactor/reference/critical_cpu_paths.md` names the suspects;
this is how they get counted.

Deliberately plain, per constraint C6:

* in-process counters plus a periodic log line — **no ROS topic**, because
  publishing metrics from the node under measurement loads the thing being
  measured, and this stack is short on CPU precisely when the numbers matter;
* no allocation per sample beyond a float, and no lock on the hot path except a
  short one around the aggregate;
* disabled by default, so an unmeasured deployment pays nothing.

Everything here is descriptive. It changes no control behaviour, which is what
makes it safe to leave in place across the migration.
"""

from __future__ import annotations

import threading
import time
from collections import defaultdict


class _Stat:
    """Running count/sum/min/max for one named duration."""

    __slots__ = ("count", "total_s", "min_s", "max_s")

    def __init__(self) -> None:
        self.count = 0
        self.total_s = 0.0
        self.min_s = float("inf")
        self.max_s = 0.0

    def add(self, seconds: float) -> None:
        self.count += 1
        self.total_s += seconds
        if seconds < self.min_s:
            self.min_s = seconds
        if seconds > self.max_s:
            self.max_s = seconds

    def summary(self) -> str:
        if not self.count:
            return "n=0"
        mean_ms = (self.total_s / self.count) * 1e3
        return (
            f"n={self.count} mean={mean_ms:.2f}ms "
            f"min={self.min_s * 1e3:.2f}ms max={self.max_s * 1e3:.2f}ms"
        )


class RuntimeMetrics:
    """Counters for loop timing and SDK call attribution.

    Thread-safe because the arm node's SDK is reached from the publish thread,
    ROS callbacks and service handlers at once — which is the very problem
    Phase 1 fixes, and until then the attribution is the evidence for it.
    """

    def __init__(self, enabled: bool = False, report_period_s: float = 10.0) -> None:
        self.enabled = enabled
        self.report_period_s = report_period_s
        self._lock = threading.Lock()
        self._durations: dict[str, _Stat] = defaultdict(_Stat)
        # SDK calls counted per (call, thread name): one caller on one thread is
        # the Phase 1 exit criterion, so the shape of this map is the evidence.
        self._sdk_calls: dict[tuple, int] = defaultdict(int)
        self._last_report = time.monotonic()
        self._window_started = self._last_report

    def record_duration(self, name: str, seconds: float) -> None:
        if not self.enabled:
            return
        with self._lock:
            self._durations[name].add(seconds)

    def record_sdk_call(self, name: str) -> None:
        """Count one blocking vendor-SDK call from the calling thread."""
        if not self.enabled:
            return
        key = (name, threading.current_thread().name)
        with self._lock:
            self._sdk_calls[key] += 1

    def time_block(self, name: str) -> "_Timer":
        """Context manager timing a block; a no-op while disabled."""
        return _Timer(self, name)

    def due(self) -> bool:
        return (
            self.enabled
            and (time.monotonic() - self._last_report) >= self.report_period_s
        )

    def report(self) -> str:
        """Render the window and reset it. Empty string when there is nothing."""
        with self._lock:
            now = time.monotonic()
            window_s = max(now - self._window_started, 1e-9)
            durations = self._durations
            sdk_calls = self._sdk_calls
            self._durations = defaultdict(_Stat)
            self._sdk_calls = defaultdict(int)
            self._last_report = now
            self._window_started = now

        if not durations and not sdk_calls:
            return ""

        parts = [f"runtime metrics over {window_s:.1f}s"]
        for name in sorted(durations):
            parts.append(f"  {name}: {durations[name].summary()}")

        if sdk_calls:
            total = sum(sdk_calls.values())
            threads = {thread for _name, thread in sdk_calls}
            parts.append(
                f"  sdk calls: {total} ({total / window_s:.0f}/s) "
                f"from {len(threads)} thread(s): {', '.join(sorted(threads))}"
            )
            for (name, thread), count in sorted(
                sdk_calls.items(), key=lambda item: -item[1]
            ):
                parts.append(
                    f"    {name}[{thread}]: {count} ({count / window_s:.0f}/s)"
                )
        return "\n".join(parts)


class _Timer:
    __slots__ = ("_metrics", "_name", "_start")

    def __init__(self, metrics: RuntimeMetrics, name: str) -> None:
        self._metrics = metrics
        self._name = name
        self._start = 0.0

    def __enter__(self) -> "_Timer":
        if self._metrics.enabled:
            self._start = time.perf_counter()
        return self

    def __exit__(self, *_exc) -> bool:
        if self._metrics.enabled:
            self._metrics.record_duration(
                self._name, time.perf_counter() - self._start
            )
        return False
