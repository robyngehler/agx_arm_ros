"""Normalized closure: the mapping, what it refuses, and how a gripper routes.

The catalogue and the teach manager speak closure in [0, 1]; metres live below
the FJT bridge. These cover the conversion both ways and the resource model that
makes a parallel gripper a robot in its own right rather than a hand.
"""
import math

import pytest

from agx_arm_coordination.graph_model import (
    Action,
    GraphError,
    ROBOT_UNITS_DEDICATED,
    ROBOT_UNITS_SHARED,
    conflicts,
)
from agx_arm_coordination.gripper_closure import (
    ClosureError,
    FINGER_SUFFIXES,
    FINGER_TO_WIDTH,
    closure_to_finger_positions,
    closure_to_width,
    displayed_closure,
    gripper_stroke,
    width_from_finger_positions,
    width_to_closure,
)
from agx_arm_coordination.performer import KIND_GRIPPER, KIND_HAND, route


def _gripper_action(action_id="grip", closure=0.5, **metadata):
    return Action(
        action_id=action_id,
        actiontype_id="Gripper",
        robot_id="right_gripper",
        metadata={"target": {"closure": closure}, **metadata},
    )


# --- the mapping ------------------------------------------------------------

def test_the_endpoints_are_the_declared_stroke():
    width_open, width_closed = gripper_stroke()
    assert closure_to_width(0.0) == pytest.approx(width_open)
    assert closure_to_width(1.0) == pytest.approx(width_closed)


def test_half_closure_is_the_midpoint():
    width_open, width_closed = gripper_stroke()
    assert closure_to_width(0.5) == pytest.approx((width_open + width_closed) / 2.0)


def test_the_mapping_round_trips():
    for closure in (0.0, 0.13, 0.5, 0.87, 1.0):
        assert width_to_closure(closure_to_width(closure)) == pytest.approx(closure)


@pytest.mark.parametrize("bad", [-0.01, 1.01, 2.0, -5.0])
def test_a_closure_outside_the_range_is_refused(bad):
    with pytest.raises(ClosureError):
        closure_to_width(bad)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_a_non_finite_closure_is_refused_rather_than_saturated(bad):
    """Never clamped: a clamp turns a corrupt number into the maximum command."""
    with pytest.raises(ClosureError):
        closure_to_width(bad)


def test_a_readback_outside_the_stroke_reports_what_it_read():
    """A measurement is not folded into the command window; only the display is."""
    width_open, _ = gripper_stroke()
    assert width_to_closure(width_open + 0.005) < 0.0
    assert displayed_closure(width_open + 0.005) == 0.0


def test_fingers_mirror_and_each_covers_half_the_opening():
    names = ["right_arm_gripper_joint1", "right_arm_gripper_joint2"]
    positions = closure_to_finger_positions(names, 0.0)
    width_open, _ = gripper_stroke()
    assert positions[0] == pytest.approx(width_open / 2.0)
    assert positions[1] == pytest.approx(-width_open / 2.0)
    assert width_from_finger_positions(names, positions) == pytest.approx(width_open)


def test_a_joint_that_is_not_a_finger_gets_nothing():
    positions = closure_to_finger_positions(["right_arm_joint1"], 0.5)
    assert positions == [0.0]


# --- the catalogue ----------------------------------------------------------

def test_a_gripper_action_needs_a_closure():
    with pytest.raises(GraphError):
        Action(
            action_id="grip", actiontype_id="Gripper",
            robot_id="right_gripper", metadata={},
        )


def test_a_malformed_closure_is_refused_at_load_not_at_dispatch():
    with pytest.raises(GraphError):
        _gripper_action(closure=1.5)


def test_any_closure_in_range_works_without_a_named_preset():
    assert _gripper_action(closure=0.63).closure == pytest.approx(0.63)


def test_a_payload_change_stays_declared_rather_than_inferred():
    """A closure is a width, not a statement about holding something."""
    assert _gripper_action(closure=0.95).payload_update == ""
    assert _gripper_action(closure=0.95, payload_update="attach").payload_update == "attach"


# --- routing and resources --------------------------------------------------

def test_a_gripper_routes_to_its_own_trajectory_server_not_the_hand_controller():
    decision = route(_gripper_action())
    assert decision.kind == KIND_GRIPPER
    assert decision.side == "right"
    assert decision.kind != KIND_HAND


@pytest.mark.parametrize("units", [ROBOT_UNITS_SHARED, ROBOT_UNITS_DEDICATED])
def test_a_gripper_is_serialized_against_its_own_arm_on_both_topologies(units):
    """It rides the arm's bus and the arm's SDK session, on either wiring."""
    assert conflicts("right_gripper", "right_arm", units)
    assert conflicts("right_gripper", "both_arms", units)
    assert conflicts("right_gripper", "right_gripper", units)


@pytest.mark.parametrize("units", [ROBOT_UNITS_SHARED, ROBOT_UNITS_DEDICATED])
def test_the_two_grippers_do_not_conflict_with_each_other(units):
    assert not conflicts("right_gripper", "left_gripper", units)


def test_a_gripper_is_not_a_hand():
    assert not conflicts("right_gripper", "right_hand", ROBOT_UNITS_DEDICATED)
    with pytest.raises(GraphError):
        # A hand action carries a skill_name, not a closure, and vice versa.
        Action(
            action_id="grip", actiontype_id="Gripper",
            robot_id="right_gripper", metadata={"skill_name": "open_hand"},
        )


def test_the_finger_joints_are_the_ones_the_registry_declares():
    """The mapping is written twice — here and in agx_arm_ctrl's FJT bridge.

    The packages cannot import one another, so both are pinned to the registry
    instead of to each other; agx_arm_ctrl's test_gripper_stroke_agreement is
    the other half of this check.
    """
    from agx_arm_coordination.motion_registry import load_motion_registry

    gripper = load_motion_registry().get("gripper", {})
    assert list(FINGER_SUFFIXES) == list(gripper["canonical_joints"])
    assert FINGER_TO_WIDTH == 2.0
