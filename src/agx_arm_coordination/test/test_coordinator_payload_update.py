"""Unit tests for the action-level payload transition.

Two rules are under test. A payload change is declared by the action, never
inferred from the hand preset — ``pre_grip_handle`` and ``release_handle`` map to
the same ``can_pre_grip`` shape and only one of them means the teapot is gone.
And a declared transition is applied before the node counts as completed, so the
next arm action cannot be admitted under the wrong gravity model.

The node needs ROS to construct, so tests build a bare instance via ``__new__``
and stub the service call.
"""

import pytest

from agx_arm_coordination.coordinator_node import CoordinatorNode, _HandChild
from agx_arm_coordination.graph_model import Action, GraphError


class _RecordingLogger:
    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


class _Catalogue:
    def __init__(self, actions):
        self._actions = actions

    def get_action_detail(self, action_id):
        return self._actions[action_id]


def _action(action_id, **metadata):
    return Action(
        action_id=action_id,
        actiontype_id="Gripper",
        robot_id="left_hand",
        metadata={"skill_name": action_id, **metadata},
    )


_TEA_ACTIONS = {
    "left_hand_pre_grip_handle": _action("left_hand_pre_grip_handle"),
    "left_hand_grip_handle": _action("left_hand_grip_handle", payload_update="attach"),
    "left_hand_release_handle": _action(
        "left_hand_release_handle", payload_update="detach"
    ),
}


def _coord(setbool_result=(True, "ok"), actions=None):
    node = CoordinatorNode.__new__(CoordinatorNode)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node.catalogue = _Catalogue(_TEA_ACTIONS if actions is None else actions)
    node._payload_clients = {"left": object(), "right": object()}
    node.payload_calls = []

    def _fake_setbool(client, value, label):
        node.payload_calls.append((label, value))
        return setbool_result

    node._call_setbool_sync = _fake_setbool
    return node


def _child(action_id, side="left"):
    child = _HandChild(70, action_id)
    child.side = side
    return child


# --- metadata semantics ------------------------------------------------------

def test_pre_grip_makes_no_payload_call():
    node = _coord()
    ok, _ = node._apply_payload_update(_child("left_hand_pre_grip_handle"))
    assert ok
    assert node.payload_calls == []


def test_grip_attaches_the_payload():
    node = _coord()
    ok, _ = node._apply_payload_update(_child("left_hand_grip_handle"))
    assert ok
    assert node.payload_calls == [("payload_attach[left]", True)]


def test_release_detaches_the_payload():
    node = _coord()
    ok, _ = node._apply_payload_update(_child("left_hand_release_handle"))
    assert ok
    assert node.payload_calls == [("payload_detach[left]", False)]


def test_the_shared_pre_grip_preset_does_not_decide_payload_state():
    """pre_grip and release both run can_pre_grip; only release detaches."""
    node = _coord()
    node._apply_payload_update(_child("left_hand_pre_grip_handle"))
    node._apply_payload_update(_child("left_hand_release_handle"))
    assert node.payload_calls == [("payload_detach[left]", False)]


def test_the_transition_goes_to_the_arm_on_the_hands_own_side():
    node = _coord()
    node._apply_payload_update(_child("left_hand_grip_handle", side="right"))
    assert node.payload_calls == [("payload_attach[right]", True)]


# --- failure semantics -------------------------------------------------------

def test_a_failed_payload_service_fails_the_action():
    node = _coord((False, "no payload gravity model is configured"))
    ok, msg = node._apply_payload_update(_child("left_hand_grip_handle"))
    assert not ok
    assert "no payload gravity model" in msg


def test_a_child_without_a_side_falls_back_to_the_actions_robot_id():
    node = _coord()
    ok, _ = node._apply_payload_update(_child("left_hand_grip_handle", side=""))
    assert ok
    assert node.payload_calls == [("payload_attach[left]", True)]


def test_an_action_naming_no_single_arm_is_refused_rather_than_resolved():
    both = Action(
        action_id="both_arms_lift",
        actiontype_id="Trajectory",
        robot_id="both_arms",
        metadata={"payload_update": "attach"},
    )
    node = _coord(actions={"both_arms_lift": both})
    ok, msg = node._apply_payload_update(_child("both_arms_lift", side=""))
    assert not ok
    assert "no single arm side" in msg
    assert node.payload_calls == []


def test_an_unknown_action_id_changes_nothing():
    node = _coord()

    class _Missing:
        def get_action_detail(self, action_id):
            raise KeyError(action_id)

    node.catalogue = _Missing()
    ok, _ = node._apply_payload_update(_child("left_hand_grip_handle"))
    assert ok
    assert node.payload_calls == []


# --- catalogue validation ----------------------------------------------------

def test_a_mistyped_payload_update_is_rejected_at_load():
    with pytest.raises(GraphError) as exc:
        _action("left_hand_grip_handle", payload_update="atach")
    assert "payload_update" in str(exc.value)


def test_an_action_without_the_field_reports_no_transition():
    assert _action("left_hand_rest_fist").payload_update == ""


def test_the_shipped_tea_pour_catalogue_declares_exactly_two_transitions():
    """The demo's own catalogue, not a fixture: attach on grip, detach on release."""
    from pathlib import Path

    import yaml

    from agx_arm_coordination.graph_model import parse_catalogue

    fragment = (
        Path(__file__).resolve().parents[1]
        / "config" / "catalogue.d" / "tea_pour_left_v1.yaml"
    )
    actions = parse_catalogue(yaml.safe_load(fragment.read_text(encoding="utf-8")))
    transitions = {
        action_id: action.payload_update
        for action_id, action in actions.items()
        if action.payload_update
    }
    assert transitions == {
        "left_hand_grip_handle": "attach",
        "left_hand_release_handle": "detach",
    }
