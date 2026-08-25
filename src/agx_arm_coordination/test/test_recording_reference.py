"""Unit tests for referencing a taught recording instead of inlining waypoints.

A catalogue that carried the dense recording would be unreadable, and decimating
it to fit is exactly what a replay cannot undo afterwards.
"""

import json

import pytest

from agx_arm_coordination.graph_loader import (
    load_recording_waypoints,
    resolve_recordings,
)
from agx_arm_coordination.graph_model import Action


def _lean(path, points=5):
    path.write_text(json.dumps({
        "joint_names": ["j1", "j2"],
        "times": [round(i * 0.01, 4) for i in range(points)],
        "positions": [[i * 0.1, i * 0.2] for i in range(points)],
    }), encoding="utf-8")
    return path


def _full(path, points=5):
    path.write_text(json.dumps({
        "name": "x", "robot": "nero", "joint_names": ["j1", "j2"],
        "sample_rate_hz": 100.0, "recorded_at": "", "metadata": {},
        "points": [
            {"time_from_start": round(i * 0.01, 4), "positions": [i * 0.1, i * 0.2],
             "velocities": [0.0, 0.0], "efforts": [0.0, 0.0], "flange_pose": None}
            for i in range(points)
        ],
    }), encoding="utf-8")
    return path


def _action(metadata):
    return Action(
        action_id="left_arm_pour", actiontype_id="Trajectory",
        robot_id="left_arm", metadata=metadata,
    )


def test_a_lean_sidecar_loads_as_waypoints(tmp_path):
    waypoints = load_recording_waypoints(_lean(tmp_path / "r.json"))
    assert len(waypoints) == 5
    assert waypoints[0] == {"positions": [0.0, 0.0], "time_from_start_sec": 0.0}
    assert waypoints[-1]["time_from_start_sec"] == pytest.approx(0.04)


def test_a_full_teach_recording_can_be_referenced_directly(tmp_path):
    """So a file can be pointed at straight from the teach library."""
    lean = load_recording_waypoints(_lean(tmp_path / "lean.json"))
    full = load_recording_waypoints(_full(tmp_path / "full.json"))
    assert lean == full


def test_a_reference_is_resolved_relative_to_the_config_dir(tmp_path):
    (tmp_path / "recordings").mkdir()
    _lean(tmp_path / "recordings" / "pour.json")
    actions = {"left_arm_pour": _action(
        {"source": "recorded", "recording": "recordings/pour.json"}
    )}
    resolve_recordings(actions, tmp_path)
    assert len(actions["left_arm_pour"].metadata["waypoints"]) == 5


def test_a_missing_recording_stops_the_coordinator_coming_up(tmp_path):
    """Fail at load, not on an activity that is already running."""
    actions = {"left_arm_pour": _action({"source": "recorded", "recording": "nope.json"})}
    with pytest.raises(ValueError, match="does not exist"):
        resolve_recordings(actions, tmp_path)


def test_a_malformed_recording_is_refused_with_the_action_named(tmp_path):
    (tmp_path / "bad.json").write_text("{not json", encoding="utf-8")
    actions = {"left_arm_pour": _action({"source": "recorded", "recording": "bad.json"})}
    with pytest.raises(ValueError, match="left_arm_pour.*not valid JSON"):
        resolve_recordings(actions, tmp_path)


def test_mismatched_times_and_positions_are_refused(tmp_path):
    path = tmp_path / "r.json"
    path.write_text(json.dumps({"times": [0.0, 0.1], "positions": [[0.0]]}), encoding="utf-8")
    with pytest.raises(ValueError, match="2 times against 1 position rows"):
        load_recording_waypoints(path)


def test_declaring_both_a_reference_and_inline_waypoints_is_refused(tmp_path):
    _lean(tmp_path / "r.json")
    actions = {"left_arm_pour": _action({
        "source": "recorded", "recording": "r.json",
        "waypoints": [{"positions": [0.0, 0.0], "time_from_start_sec": 0.0}],
    })}
    with pytest.raises(ValueError, match="not both"):
        resolve_recordings(actions, tmp_path)


def test_an_action_without_a_reference_is_left_alone(tmp_path):
    inline = [{"positions": [0.0, 0.0], "time_from_start_sec": 0.0}]
    actions = {"left_arm_pour": _action({"source": "recorded", "waypoints": inline})}
    resolve_recordings(actions, tmp_path)
    assert actions["left_arm_pour"].metadata["waypoints"] == inline


# --- what the sidecar has to prove before an activity starts ----------------


def test_the_taught_joint_names_travel_with_the_waypoints(tmp_path):
    """The planner compares them against the group it is about to command; only
    it knows which group that is."""
    _lean(tmp_path / "r.json")
    actions = {"left_arm_pour": _action({"source": "recorded", "recording": "r.json"})}
    resolve_recordings(actions, tmp_path)
    assert actions["left_arm_pour"].metadata["recording_joint_names"] == ["j1", "j2"]


@pytest.mark.parametrize("payload, message", [
    ({"joint_names": ["j1", "j2"], "times": [0.0, 0.1],
      "positions": [[0.0, 0.0], [0.1]]}, "sample 1 has 1 positions"),
    ({"joint_names": ["j1", "j2", "j3"], "times": [0.0, 0.1],
      "positions": [[0.0, 0.0], [0.1, 0.1]]}, "against 3 declared joint names"),
    ({"times": [0.0, 0.1, 0.05], "positions": [[0.0], [0.1], [0.2]]},
     "must strictly increase"),
    ({"times": [0.0, 0.1, 0.1], "positions": [[0.0], [0.1], [0.2]]},
     "must strictly increase"),
])
def test_a_sidecar_that_cannot_be_replayed_is_refused_at_load(tmp_path, payload, message):
    path = tmp_path / "r.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_recording_waypoints(path)


@pytest.mark.parametrize("bad", ["NaN", "Infinity"])
def test_non_finite_values_are_refused(tmp_path, bad):
    """`json.loads` accepts NaN and Infinity, and a clamp downstream would map
    either onto a limit — a plausible-looking maximum command."""
    path = tmp_path / "r.json"
    path.write_text(
        '{"times": [0.0, 0.1, %s], "positions": [[0.0], [0.1], [0.2]]}' % bad,
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="non-finite time"):
        load_recording_waypoints(path)

    path.write_text(
        '{"times": [0.0, 0.1, 0.2], "positions": [[0.0], [%s], [0.2]]}' % bad,
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="non-finite position"):
        load_recording_waypoints(path)


# --- static playback configuration ------------------------------------------


def test_a_malformed_playback_block_fails_at_catalogue_load():
    """Prewarming would catch it too, but only once someone runs an activity
    that happens to use the action."""
    from agx_arm_coordination.graph_loader import validate_playback

    actions = {"left_arm_pour": _action({"source": "recorded", "playback": {"mode": "warp"}})}
    with pytest.raises(ValueError, match="unknown playback mode"):
        validate_playback(actions)


def test_a_valid_playback_block_passes_catalogue_load():
    from agx_arm_coordination.graph_loader import validate_playback

    validate_playback({"left_arm_pour": _action(
        {"source": "recorded", "playback": {"mode": "tempo_scale", "speed_scale": 0.6}}
    )})
