"""Cancellation is bounded, and authority is released the same way every time.

Two defects this pins. Cancelling children fired the cancel goals and cleared
the bookkeeping in the same breath, so cleanup reported itself finished the
instant it was requested — an activity could publish "aborted" while an arm was
still executing, with nothing recording that the two disagreed. And the hand
windows were reopened only on the paths someone remembered; an unexpected
exception left one open, which keeps the arm's MIT gate shut so the *next*
activity finds an arm that silently refuses to move.
"""

from __future__ import annotations

import threading

from agx_arm_coordination.coordinator_node import CoordinatorNode, _Child


class _RecordingLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg, *_a, **_k):
        self.messages.append(("info", str(msg)))

    def warn(self, msg, *_a, **_k):
        self.messages.append(("warn", str(msg)))

    def error(self, msg, *_a, **_k):
        self.messages.append(("error", str(msg)))

    def errors(self):
        return [m for lvl, m in self.messages if lvl == "error"]


class _StubbornChild(_Child):
    """A child that acknowledges the cancel but never reports done."""

    def __init__(self, action_no, action_id):
        super().__init__(action_no, action_id)
        self.cancelled = False

    def poll(self):
        return

    def request_cancel(self):
        self.cancelled = True


class _CompliantChild(_Child):
    """A child that finishes as soon as it is polled after a cancel."""

    def __init__(self, action_no, action_id):
        super().__init__(action_no, action_id)
        self.cancelled = False

    def poll(self):
        if self.cancelled and not self.done:
            self.mark(False, "canceled")

    def request_cancel(self):
        self.cancelled = True


def _coord(cleanup_timeout=0.2):
    node = CoordinatorNode.__new__(CoordinatorNode)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node._progress = threading.Event()
    node.cleanup_timeout = cleanup_timeout
    node._events = []
    node._event = lambda kind, **kw: node._events.append((kind, kw))
    return node


def test_cancellation_waits_for_children_to_confirm():
    node = _coord()
    a, b = _CompliantChild(10, "a"), _CompliantChild(20, "b")
    running = {10: a, 20: b}

    node._cancel_children(running)

    assert a.cancelled and b.cancelled
    assert a.done and b.done, "cleanup returned before the children had stopped"
    assert running == {}
    assert not node._logger.errors(), node._logger.errors()


def test_a_child_that_never_stops_is_named_rather_than_silently_dropped():
    node = _coord(cleanup_timeout=0.15)
    stuck = _StubbornChild(10, "wont_stop")
    running = {10: stuck}

    node._cancel_children(running)

    assert stuck.cancelled, "the cancel was never requested"
    assert running == {}, "the unit must be released even when a child hangs"
    errors = node._logger.errors()
    assert any("cleanup deadline" in m and "wont_stop" in m for m in errors), errors
    assert any(
        kind == "failed" and kw.get("state") == "cleanup_timeout"
        for kind, kw in node._events
    ), node._events


def test_a_merged_child_is_cancelled_once_not_once_per_action():
    """A merged dual-arm goal appears under both action numbers."""
    node = _coord()
    merged = _CompliantChild(10, "left+right")
    merged.action_nos = [10, 20]
    running = {10: merged, 20: merged}

    cancels = []
    original = merged.request_cancel

    def counting_cancel():
        cancels.append(1)
        original()

    merged.request_cancel = counting_cancel

    node._cancel_children(running)

    assert len(cancels) == 1, f"merged child cancelled {len(cancels)} times"
    assert merged.done


def test_a_completing_child_wakes_the_activity_loop():
    """The loop waits on an event rather than sweeping at a fixed rate."""
    child = _Child(10, "a")
    woken = threading.Event()
    child.set_notify(woken.set)

    class _Future:
        def __init__(self):
            self._cb = None

        def add_done_callback(self, cb):
            self._cb = cb

        def resolve(self):
            self._cb(self)

    future = _Future()
    child.attach_goal_future(future)
    assert not woken.is_set()

    future.resolve()

    assert woken.is_set(), "a resolving goal future did not wake the activity loop"
