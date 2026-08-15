"""Every SDK call for a hand goes through one worker, and off the executor.

The calls were already serialized before this, but only because the bridge spins
single-threaded. That was an accident: one edit to a MultiThreadedExecutor would
have ended it silently, and both sibling nodes in this package already use one.
It also bought nothing, because a blocking read sat on the executor thread. A
17 ms status read stopped the node answering its own claim service, which was
observed on hardware as a service that "did not answer".
"""

from __future__ import annotations

import threading
import time

import pytest
import rclpy
from std_srvs.srv import Trigger

from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode
from agx_arm_ctrl.sdk_worker import Lane
from agx_arm_msgs.srv import ClaimDevice


@pytest.fixture()
def bridge_node():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "joint_read_rate:=20.0",
            "-p", "owner_liveness_grace_s:=0.0",
        ]
    )
    node = OmniHandBridgeNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_the_bridge_owns_one_sdk_worker(bridge_node):
    assert bridge_node._sdk.device_id == bridge_node._authority.device_id


def test_acquisition_runs_off_the_executor(bridge_node):
    """The thread that reads is not the thread that serves ROS."""
    seen: list[str] = []
    original = bridge_node.backend.read_joint_state

    def recording_read():
        seen.append(threading.current_thread().name)
        return original()

    bridge_node.backend.read_joint_state = recording_read
    bridge_node._acquire_once()

    assert seen, "no read happened"
    # The worker executes it, never the caller and never the executor.
    assert all(name.startswith("sdk-") for name in seen), seen


def test_a_slow_diagnostic_read_does_not_block_a_claim(bridge_node):
    """The property the whole slice exists for.

    A long SDK read used to sit on the executor thread, so the node could not
    answer its own claim service while it ran. Now the read is on the worker and
    the service handler never touches the SDK, so the claim is answered while the
    read is still in flight.
    """
    started = threading.Event()
    release = threading.Event()

    def slow_read():
        started.set()
        release.wait(timeout=5.0)
        return []

    # Occupy the worker with a long diagnostic call, as a status read would.
    bridge_node._sdk.submit("slow_status", slow_read, lane=Lane.DIAGNOSTIC)
    assert started.wait(timeout=2.0), "the worker never started the slow read"

    try:
        answered_at = time.monotonic()
        request = ClaimDevice.Request()
        request.owner_id = "reactive:test_owner"
        request.claim = True
        response = bridge_node._claim_device_callback(request, ClaimDevice.Response())
        elapsed = time.monotonic() - answered_at

        assert response.accepted
        assert elapsed < 0.5, f"the claim waited {elapsed:.2f}s on the SDK"
    finally:
        release.set()


def test_a_stop_does_not_wait_for_the_worker_either(bridge_node):
    """A stop is queued on the safety lane and acknowledged at once."""
    release = threading.Event()
    started = threading.Event()

    def slow_read():
        started.set()
        release.wait(timeout=5.0)
        return []

    bridge_node._sdk.submit("slow_status", slow_read, lane=Lane.DIAGNOSTIC)
    assert started.wait(timeout=2.0)

    try:
        began = time.monotonic()
        response = bridge_node._stop_callback(Trigger.Request(), Trigger.Response())
        elapsed = time.monotonic() - began

        assert response.success
        assert elapsed < 0.5, f"the stop service waited {elapsed:.2f}s"
    finally:
        release.set()


def test_a_stop_is_queued_ahead_of_pending_control_work(bridge_node):
    """Lane priority: the stop overtakes control work that has not started."""
    order: list[str] = []
    release = threading.Event()
    started = threading.Event()

    def blocker():
        started.set()
        release.wait(timeout=5.0)

    bridge_node._sdk.submit("blocker", blocker, lane=Lane.DIAGNOSTIC)
    assert started.wait(timeout=2.0)

    bridge_node._sdk.submit("control", lambda: order.append("control"), lane=Lane.CONTROL)
    bridge_node._sdk.submit_safety("safety", lambda: order.append("safety"))

    release.set()
    deadline = time.monotonic() + 3.0
    while len(order) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)

    assert order == ["safety", "control"], order
