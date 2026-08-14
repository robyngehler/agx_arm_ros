"""The claim service must survive a refusal.

A refused claim is the normal case the whole exclusivity design rests on, and it
was the one path never taken: `rclpy` caches a logger's severity per call site
and raises if it changes, so a single site logging INFO on success and WARN on
refusal threw the first time a second commander asked for a hand — out of a
service callback, killing the bridge. On hardware that looked like the service
"not answering", which sent the investigation after the client.
"""

from __future__ import annotations

import pytest
import rclpy

from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode
from agx_arm_msgs.srv import ClaimDevice


@pytest.fixture()
def bridge_node():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "owner_liveness_grace_s:=0.0",
        ]
    )
    node = OmniHandBridgeNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _call(node, owner: str, claim: bool):
    request = ClaimDevice.Request()
    request.owner_id = owner
    request.claim = claim
    return node._claim_device_callback(request, ClaimDevice.Response())


def test_a_refused_claim_answers_instead_of_raising(bridge_node):
    first = _call(bridge_node, "reactive:owner_a", True)
    assert first.accepted

    # Same call site, opposite outcome — this is what used to kill the node.
    second = _call(bridge_node, "trajectory:owner_b", True)

    assert not second.accepted
    assert second.reason
    assert second.message


def test_repeated_refusals_keep_answering(bridge_node):
    assert _call(bridge_node, "reactive:owner_a", True).accepted

    for _ in range(5):
        assert not _call(bridge_node, "trajectory:owner_b", True).accepted


def test_a_release_by_a_non_owner_is_refused_not_fatal(bridge_node):
    assert _call(bridge_node, "reactive:owner_a", True).accepted

    response = _call(bridge_node, "trajectory:owner_b", False)

    assert not response.accepted
    assert bridge_node._authority.snapshot().owner_id == "reactive:owner_a"


def test_the_claim_answer_names_the_device_not_the_sdk_index(bridge_node):
    """`self.device_id` is also the vendor SDK's numeric id further down."""
    response = _call(bridge_node, "reactive:owner_a", True)

    assert "hand_" in response.message
