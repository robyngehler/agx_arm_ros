"""Unit tests for the coordinator's arm<->hand window handoff wiring.

Guards the shared-CAN step-and-settle plan section 3 end-to-end contract: before
a hand action runs on a shared side bus the coordinator must quiesce the arm via
prepare_hand_window and reopen it afterwards via resume_arm_control, and every
exit path (success, abort, cancel) must reopen any window it left open. The node
needs ROS to construct, so tests build a bare instance via ``__new__`` and drive
the handoff state machine with a stubbed trigger call.
"""

import pytest

from agx_arm_coordination.coordinator_node import CoordinatorNode, DispatchError


class _RecordingLogger:
    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


def _coord(trigger_result=(True, "ok")):
    node = CoordinatorNode.__new__(CoordinatorNode)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node.handoff_enabled = True
    node._open_hand_windows = set()
    node._prepare_clients = {"left": object(), "right": object()}
    node._resume_clients = {"left": object(), "right": object()}
    node._trigger_calls = []

    def _fake_trigger(client, label):
        node._trigger_calls.append(label)
        return trigger_result

    node._call_trigger_sync = _fake_trigger
    return node


def test_open_hand_window_marks_side_open_on_success():
    node = _coord((True, "held"))
    node._open_hand_window("right")
    assert "right" in node._open_hand_windows
    assert any("prepare_hand_window[right]" in c for c in node._trigger_calls)


def test_open_hand_window_raises_and_leaves_side_closed_on_failure():
    node = _coord((False, "hold not verified"))
    with pytest.raises(DispatchError):
        node._open_hand_window("right")
    assert "right" not in node._open_hand_windows


def test_open_hand_window_is_noop_when_disabled():
    node = _coord()
    node.handoff_enabled = False
    node._open_hand_window("right")
    assert node._open_hand_windows == set()
    assert node._trigger_calls == []


def test_resume_reopens_and_clears_the_side():
    node = _coord((True, "resumed"))
    node._open_hand_windows.add("right")
    node._resume_hand_window("right")
    assert "right" not in node._open_hand_windows
    assert any("resume_arm_control[right]" in c for c in node._trigger_calls)


def test_resume_all_reopens_every_open_side():
    node = _coord((True, "resumed"))
    node._open_hand_windows.update({"left", "right"})
    node._resume_all_hand_windows()
    assert node._open_hand_windows == set()


def test_resume_unknown_side_is_noop():
    node = _coord()
    node._resume_hand_window("right")  # never opened
    assert node._trigger_calls == []


def test_failed_resume_still_clears_side_so_it_is_not_retried_forever():
    node = _coord((False, "feedback stale"))
    node._open_hand_windows.add("right")
    node._resume_hand_window("right")
    assert "right" not in node._open_hand_windows
