"""The payload transition's position in the activity loop.

The invariant: no downstream arm action may start until the payload transition
its predecessor declared has succeeded. Tested against the real scheduler and the
real ``_execute_activity`` loop over the shipped ``tea_pour_left_v1`` graph, with
dispatch and the payload service stubbed — so the ordering under test is the
coordinator's own, not a re-implementation of it.

The gravity effect of the swap on a live MIT controller is not covered here; that
is the L3 static payload check.
"""

from pathlib import Path

import pytest

from agx_arm_coordination.coordinator_node import CoordinatorNode, _Child, _HandChild
from agx_arm_coordination.graph_loader import ActivityCatalogue
from agx_arm_coordination.graph_model import ROBOT_UNITS_DEDICATED

_CONFIG = Path(__file__).resolve().parents[1] / "config"

GRIP = 70            # left_hand_grip_handle    -> payload attach
POST_GRIP = 80       # left_arm_to_teapot_post_grip (the lift)
RELEASE = 150        # left_hand_release_handle -> payload detach
WITHDRAW = 160       # left_arm_teapot_handle_release


@pytest.fixture(autouse=True)
def _rclpy_context():
    """The activity loop runs under ``while rclpy.ok()``; without a context it exits at once."""
    import rclpy

    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


class _RecordingLogger:
    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


class _GoalHandle:
    is_cancel_requested = False

    def __init__(self):
        self.request = type("R", (), {"activity_id": "tea_pour_left_v1"})()
        self.terminal = ""

    def abort(self):
        self.terminal = "abort"

    def succeed(self):
        self.terminal = "succeed"

    def canceled(self):
        self.terminal = "canceled"

    def publish_feedback(self, _feedback):
        pass


def _tea_catalogue() -> ActivityCatalogue:
    """The shipped catalogue, loaded the way the coordinator loads it."""
    return ActivityCatalogue.from_config_dir(_CONFIG, ROBOT_UNITS_DEDICATED)


def _coord(payload_results=None):
    """A bare coordinator whose children complete instantly and in dispatch order."""
    import threading

    node = CoordinatorNode.__new__(CoordinatorNode)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node.catalogue = _tea_catalogue()
    node.robot_units = ROBOT_UNITS_DEDICATED
    node.handoff_enabled = False
    node._open_hand_windows = set()
    node._payload_clients = {"left": object(), "right": object()}
    node._progress = threading.Event()
    node.watchdog_period = 0.0
    node._stop_requested = False
    node._stop_reason = ""
    node._stop_lock = threading.Lock()
    node.event_pub = None
    node.arm_dry_run = False

    node.trace: list[tuple[str, object]] = []
    node._payload_results = dict(payload_results or {})

    def _event(*_a, **_k):
        pass

    node._event = _event
    node._publish_feedback = lambda *a, **k: None

    def _dispatch_units(batch, activity_id):
        del activity_id
        children = []
        for item in batch:
            action = node.catalogue.get_action_detail(item.action_id)
            child = (
                _HandChild(item.action_no, item.action_id)
                if action.actiontype_id == "Gripper"
                else _Child(item.action_no, item.action_id)
            )
            child.side = "left"
            node.trace.append(("dispatch", item.action_no))
            child.mark(True, "stub")
            children.append(child)
        return children

    node._dispatch_units = _dispatch_units

    def _fake_setbool(client, value, label):
        del client
        node.trace.append(("payload", value))
        return node._payload_results.get(label, (True, "ok"))

    node._call_setbool_sync = _fake_setbool
    node._resume_hand_window = lambda side: None
    node._resume_all_hand_windows = lambda: None
    node._stop_running = lambda running, reason: None
    node._cancel_children = lambda running: None
    return node


def _run(node) -> tuple[object, object]:
    handle = _GoalHandle()
    return handle, node._execute_activity(handle)


def _order(trace, *entries):
    """Indices of the given trace entries, asserting each occurs exactly once."""
    indices = []
    for entry in entries:
        matches = [i for i, item in enumerate(trace) if item == entry]
        assert len(matches) == 1, f"{entry} occurred {len(matches)} times in {trace}"
        indices.append(matches[0])
    return indices


def test_the_graph_under_test_declares_the_two_transitions():
    catalogue = _tea_catalogue()
    assert catalogue.get_action_detail("left_hand_grip_handle").payload_update == "attach"
    assert catalogue.get_action_detail("left_hand_release_handle").payload_update == "detach"


def test_attach_lands_between_the_grip_and_the_lift():
    node = _coord()
    _, result = _run(node)

    assert result.success, result.message
    grip, attach, lift = _order(
        node.trace, ("dispatch", GRIP), ("payload", True), ("dispatch", POST_GRIP)
    )
    assert grip < attach < lift


def test_detach_lands_between_the_release_and_the_withdraw():
    node = _coord()
    _, result = _run(node)

    assert result.success, result.message
    release, detach, withdraw = _order(
        node.trace, ("dispatch", RELEASE), ("payload", False), ("dispatch", WITHDRAW)
    )
    assert release < detach < withdraw


def test_the_payload_service_is_called_exactly_twice_in_a_full_run():
    node = _coord()
    _, result = _run(node)

    assert result.success, result.message
    assert [value for kind, value in node.trace if kind == "payload"] == [True, False]


def test_a_failed_attach_stops_the_activity_before_the_lift():
    node = _coord({"payload_attach[left]": (False, "no payload gravity model")})
    handle, result = _run(node)

    assert not result.success
    assert "payload update failed" in result.message
    assert result.failed_action_id == "left_hand_grip_handle"
    assert handle.terminal == "abort"
    # The grip node never counts as completed, so the lift is never dispatched.
    assert ("dispatch", POST_GRIP) not in node.trace
    assert result.completed_nodes == 6  # nodes 10..60; 70 failed


def test_a_failed_detach_stops_the_activity_before_the_withdraw():
    node = _coord({"payload_detach[left]": (False, "gravity compensation is not active")})
    _, result = _run(node)

    assert not result.success
    assert result.failed_action_id == "left_hand_release_handle"
    assert ("dispatch", WITHDRAW) not in node.trace


@pytest.mark.parametrize("failing", ["payload_attach[left]", "payload_detach[left]"])
def test_a_failed_transition_never_reports_the_activity_successful(failing):
    node = _coord({failing: (False, "unavailable")})
    handle, result = _run(node)

    assert not result.success
    assert handle.terminal == "abort"
