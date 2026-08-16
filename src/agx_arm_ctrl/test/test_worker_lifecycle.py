"""A node that owns an SdkWorker must not outlive it.

Worker threads are daemons, so a process exit disposes of them and hides a
missing shutdown entirely. Nothing that keeps the process alive is so forgiving:
a test session, a repeated bringup, or a composed node accumulates threads that
each still hold a device's SDK session — which is the opposite of the
one-owner-at-a-time invariant the worker exists to provide.
"""

from __future__ import annotations

import threading

import pytest
import rclpy

from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode


def _sdk_threads() -> list[threading.Thread]:
    """Live worker threads, which name themselves after the device they serve.

    Counted as objects, not as names: every bridge for the same hand names its
    worker identically, so a set of names cannot tell one live worker from four
    and would report a leak of four threads as clean.
    """
    return [t for t in threading.enumerate() if t.name.startswith("sdk-")]


def _acquisition_threads() -> list[threading.Thread]:
    return [t for t in threading.enumerate() if t.name.startswith("hand-acq")]


@pytest.fixture()
def ros():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "joint_read_rate:=20.0",
            "-p", "owner_liveness_grace_s:=0.0",
        ]
    )
    yield
    rclpy.shutdown()


def test_destroying_the_bridge_stops_its_worker(ros):
    before = len(_sdk_threads())
    node = OmniHandBridgeNode()
    assert node._sdk.is_alive
    assert len(_sdk_threads()) > before, "the worker thread did not start"

    node.destroy_node()

    assert not node._sdk.is_alive, "the SDK worker outlived the node"
    assert len(_sdk_threads()) <= before, "a worker thread leaked"
    assert _acquisition_threads() == [], "the acquisition thread outlived the node"


def test_repeated_bringup_does_not_accumulate_worker_threads(ros):
    """The shape a composed process or a long test session actually takes."""
    before = len(_sdk_threads())

    for _ in range(4):
        node = OmniHandBridgeNode()
        node.destroy_node()

    leaked = len(_sdk_threads()) - before
    assert leaked == 0, f"{leaked} worker thread(s) leaked across four bringups"


def test_shutdown_is_idempotent(ros):
    """Teardown arrives by more than one path; twice must not raise or hang."""
    node = OmniHandBridgeNode()

    node.shutdown()
    node.shutdown()
    node.destroy_node()

    assert not node._sdk.is_alive
