"""Unit tests for how a recorded action selects its playback mode.

The mode is a property of this step in this activity, so it is declared per
action like ``payload_update`` is, and can be overridden for one run.
"""

import pytest

from agx_arm_coordination.arm_executor import (
    ArmConfig,
    ArmConfigError,
    ArmTrajectoryPlanner,
    PLAYBACK_DEFAULT_MODE,
    PLAYBACK_DEFAULT_WINDOW_SEC,
    PlaybackSpec,
    playback_spec,
)
from agx_arm_coordination.graph_model import Action


def _config():
    return ArmConfig.from_dict({
        "arm_executor": {
            "groups": {
                "left_arm": {
                    "planning_group": "left_arm",
                    "joint_names": [f"left_arm_joint{i}" for i in range(1, 8)],
                },
            },
            "poses": {},
        }
    })


def _taught_action(playback=None, points=40):
    import math

    waypoints = []
    for index in range(points):
        phase = index / max(points - 1, 1)
        waypoints.append({
            "positions": [0.4 * math.sin(2 * math.pi * phase) + 0.05 * j for j in range(7)],
            "time_from_start_sec": round(phase * 4.0, 4),
        })
    metadata = {"source": "recorded", "waypoints": waypoints}
    if playback is not None:
        metadata["playback"] = playback
    return Action(
        action_id="left_arm_pour",
        actiontype_id="Trajectory",
        robot_id="left_arm",
        metadata=metadata,
    )


def test_the_default_is_smooth_with_the_hardware_session_window():
    spec = playback_spec({})
    assert spec == PlaybackSpec()
    assert spec.mode == PLAYBACK_DEFAULT_MODE == "smooth"
    assert spec.smoothing_window_sec == PLAYBACK_DEFAULT_WINDOW_SEC == 0.3


def test_an_action_can_name_its_own_mode():
    spec = playback_spec({"playback": {"mode": "tempo_scale", "speed_scale": 0.6}})
    assert spec.mode == "tempo_scale"
    assert spec.speed_scale == 0.6
    # Unstated keys keep the default.
    assert spec.smoothing_window_sec == PLAYBACK_DEFAULT_WINDOW_SEC


@pytest.mark.parametrize("block, message", [
    ({"mode": "warp"}, "unknown playback mode"),
    ({"speed_scale": 0.0}, "must be finite"),
    ({"speed_scale": -1.0}, "must be finite"),
    ({"speed_scale": float("nan")}, "must be finite"),
    ({"speed_scale": float("inf")}, "must be finite"),
    ({"speed_scale": "fast"}, "must be a number"),
    ({"resample_dt": 0.0}, "must be finite"),
    ({"resample_dt": float("inf")}, "must be finite"),
    ({"smoothing_window_sec": -0.1}, "must be finite"),
    ({"smoothing_window_sec": float("nan")}, "must be finite"),
    ({"tempo": 2.0}, "unknown playback key"),
])
def test_an_unusable_playback_request_is_refused_not_silently_defaulted(block, message):
    """A replay that ran under a different mode than the activity asked for is
    worse than one that refused to start."""
    with pytest.raises(ArmConfigError, match=message):
        playback_spec({"playback": block}, "left_arm_pour")


def test_a_zero_smoothing_window_is_usable():
    """Zero is a request, not a malformed value: it means no window beyond the
    reconstruction floor every timing-preserving mode applies anyway."""
    assert playback_spec({"playback": {"smoothing_window_sec": 0.0}}).smoothing_window_sec == 0.0


def test_playback_must_be_a_mapping():
    with pytest.raises(ArmConfigError, match="must be a mapping"):
        playback_spec({"playback": ["smooth"]}, "left_arm_pour")


def test_a_recorded_plan_is_retimed_and_carries_velocities():
    planner = ArmTrajectoryPlanner(_config())
    plan = planner.plan(_taught_action())

    # Retimed onto the controller's own grid, so it is much denser than taught.
    assert len(plan.points) > 40
    gaps = [
        b.time_from_start_sec - a.time_from_start_sec
        for a, b in zip(plan.points, plan.points[1:])
    ]
    assert max(gaps[:-1]) - min(gaps[:-1]) < 1e-9
    assert all(len(p.velocities) == 7 for p in plan.points)
    assert any(abs(v) > 1e-6 for p in plan.points for v in p.velocities)


def test_tempo_scale_changes_the_duration_and_smooth_does_not():
    planner = ArmTrajectoryPlanner(_config())
    base = planner.plan(_taught_action())
    slow = planner.plan(_taught_action({"mode": "tempo_scale", "speed_scale": 0.5}))
    assert slow.points[-1].time_from_start_sec == pytest.approx(
        2.0 * base.points[-1].time_from_start_sec, rel=0.02
    )


def test_a_run_level_override_beats_the_action_and_is_not_permanent():
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action({"mode": "smooth"})
    taught = planner.plan(action).points[-1].time_from_start_sec

    planner.playback_override = {"mode": "tempo_scale", "speed_scale": 0.5}
    assert planner.plan(action).points[-1].time_from_start_sec == pytest.approx(
        2.0 * taught, rel=0.02
    )

    planner.playback_override = {}
    assert planner.plan(action).points[-1].time_from_start_sec == pytest.approx(taught)


def test_a_second_plan_of_the_same_action_is_served_from_cache():
    """Retiming costs up to 11 s under speed_scale; a dispatch must not pay it."""
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action()
    first = planner.plan(action)
    second = planner.plan(action)
    assert [p.positions for p in first.points] == [p.positions for p in second.points]
    assert len(planner._retimed) == 1


def test_a_trajectory_too_short_to_retime_is_dispatched_as_taught():
    planner = ArmTrajectoryPlanner(_config())
    plan = planner.plan(_taught_action(points=3))
    assert len(plan.points) == 3
    assert all(p.velocities == () for p in plan.points)


def test_a_run_level_override_is_parsed_from_the_activity_goal():
    from agx_arm_coordination.coordinator_node import _playback_override

    assert _playback_override("") == {}
    assert _playback_override("   ") == {}
    assert _playback_override('{"playback": {"mode": "as_recorded"}}') == {"mode": "as_recorded"}
    # Other run-time overrides stay possible; this parser only claims its key.
    assert _playback_override('{"something_else": 1}') == {}


@pytest.mark.parametrize("payload, message", [
    ("not json", "not a JSON object"),
    ("[1, 2]", "expected a JSON object"),
    ('{"playback": "smooth"}', "must be an object"),
])
def test_an_unusable_activity_override_is_refused(payload, message):
    from agx_arm_coordination.coordinator_node import _playback_override

    with pytest.raises(ValueError, match=message):
        _playback_override(payload)


# --- playback against the legacy timing knobs -------------------------------


def test_legacy_scaling_cannot_be_combined_with_an_explicit_playback_block():
    """Both scale the taught timing, so the replay would run at the product:
    tempo_scale 0.5 under velocity_scaling 0.5 is a quarter speed and neither
    number says so."""
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action({"mode": "tempo_scale", "speed_scale": 0.5})
    action.metadata["velocity_scaling"] = 0.5

    with pytest.raises(ArmConfigError, match="cannot be combined"):
        planner.plan(action)


def test_a_run_level_override_also_makes_playback_the_authority():
    """The conflict is the same whichever level asked for the mode."""
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action()
    action.metadata["acceleration_scaling"] = 0.5
    # No playback block anywhere: the deprecated knob still stretches the times.
    assert planner.plan(action).points[-1].time_from_start_sec > 7.0

    planner.playback_override = {"mode": "smooth"}
    with pytest.raises(ArmConfigError, match="acceleration_scaling"):
        planner.plan(action)


def test_a_legacy_action_without_a_playback_block_keeps_working():
    """Deprecated, not removed: catalogue entries migrate over time."""
    planner = ArmTrajectoryPlanner(_config())
    plain = planner.plan(_taught_action()).points[-1].time_from_start_sec
    action = _taught_action()
    action.metadata["velocity_scaling"] = 0.5
    assert planner.plan(action).points[-1].time_from_start_sec == pytest.approx(
        2.0 * plain, rel=0.02
    )


def test_a_scaling_of_one_is_not_a_conflict():
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action({"mode": "smooth"})
    action.metadata["velocity_scaling"] = 1.0
    assert planner.plan(action).points


# --- the recording's own joints ---------------------------------------------


def test_a_recording_taught_on_other_joints_is_refused():
    """A left-arm recording is the right shape for the right arm and would
    replay a mirrored path onto it; the count alone cannot see that."""
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action()
    action.metadata["recording_joint_names"] = [
        f"right_arm_joint{i}" for i in range(1, 8)
    ]
    with pytest.raises(ArmConfigError, match="was taught on"):
        planner.plan(action)


def test_a_recording_taught_on_the_same_joints_in_another_order_is_refused():
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action()
    names = [f"left_arm_joint{i}" for i in range(1, 8)]
    action.metadata["recording_joint_names"] = names[::-1]
    with pytest.raises(ArmConfigError, match="was taught on"):
        planner.plan(action)


def test_matching_joint_names_replay_normally():
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action()
    action.metadata["recording_joint_names"] = [
        f"left_arm_joint{i}" for i in range(1, 8)
    ]
    assert planner.plan(action).points


def test_an_unprefixed_single_arm_recording_replays():
    """A single-arm teach recording stores `joint1..7`: it names the joints and
    their order, and the catalogue names the side."""
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action()
    action.metadata["recording_joint_names"] = [f"joint{i}" for i in range(1, 8)]
    assert planner.plan(action).points


def test_the_prefix_has_to_be_the_same_one_for_every_joint():
    """Otherwise a duo recording's two sides would pass as one arm's."""
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action()
    action.metadata["recording_joint_names"] = (
        ["joint1", "joint2", "joint3"] + [f"arm_joint{i}" for i in range(4, 8)]
    )
    with pytest.raises(ArmConfigError, match="was taught on"):
        planner.plan(action)


def test_an_unprefixed_recording_in_the_wrong_order_is_still_refused():
    planner = ArmTrajectoryPlanner(_config())
    action = _taught_action()
    action.metadata["recording_joint_names"] = [f"joint{i}" for i in range(1, 8)][::-1]
    with pytest.raises(ArmConfigError, match="was taught on"):
        planner.plan(action)


# --- preplanning and cancellation -------------------------------------------


class _FakeGraph:
    def __init__(self, action_ids):
        self.nodes = {
            index: type("N", (), {"action_id": action_id})()
            for index, action_id in enumerate(action_ids)
        }


def _node_with(actions, planner, stop_requested=False):
    """A bare CoordinatorNode carrying only what _prewarm_arm_actions touches."""
    import threading

    from agx_arm_coordination.coordinator_node import CoordinatorNode

    node = CoordinatorNode.__new__(CoordinatorNode)
    node._stop_lock = threading.Lock()
    node._stop_requested = stop_requested
    node.arm_planner = planner
    node.arm_dry_run = False
    node.catalogue = type("C", (), {"actions": {a.action_id: a for a in actions}})()
    node.get_logger = lambda: type("L", (), {
        "info": staticmethod(lambda *a, **k: None),
        "error": staticmethod(lambda *a, **k: None),
    })()
    return node


def _named(action_id, playback=None):
    import dataclasses

    return dataclasses.replace(_taught_action(playback), action_id=action_id)


def test_every_arm_action_is_planned_before_the_first_one_moves():
    planner = ArmTrajectoryPlanner(_config())
    actions = [_named("a"), _named("b")]
    node = _node_with(actions, planner)

    problems, interrupted = node._prewarm_arm_actions(_FakeGraph(["a", "b"]), "demo", None)
    assert (problems, interrupted) == ([], False)
    assert len(planner._retimed) == 2


def test_a_refusal_is_reported_against_the_action_that_refused():
    planner = ArmTrajectoryPlanner(_config())
    actions = [_named("a"), _named("b", {"mode": "tempo_scale", "speed_scale": 40.0})]
    node = _node_with(actions, planner)

    problems, interrupted = node._prewarm_arm_actions(_FakeGraph(["a", "b"]), "demo", None)
    assert interrupted is False
    assert len(problems) == 1 and problems[0].startswith("b: ")


def test_a_stop_during_preplanning_abandons_the_rest():
    """A geometric retiming is seconds per action, so a stop must not wait out
    the whole graph before anything can unwind."""
    planner = ArmTrajectoryPlanner(_config())
    node = _node_with([_named("a"), _named("b")], planner, stop_requested=True)

    problems, interrupted = node._prewarm_arm_actions(_FakeGraph(["a", "b"]), "demo", None)
    assert interrupted is True
    assert problems == []
    assert planner._retimed == {}


def test_a_cancel_between_actions_stops_the_next_one_being_planned():
    planner = ArmTrajectoryPlanner(_config())
    node = _node_with([_named("a"), _named("b")], planner)

    class _CancelAfterFirst:
        def __init__(self):
            self._checks = 0

        @property
        def is_cancel_requested(self):
            self._checks += 1
            return self._checks > 1

    problems, interrupted = node._prewarm_arm_actions(
        _FakeGraph(["a", "b"]), "demo", _CancelAfterFirst()
    )
    assert interrupted is True
    # The one already planned is kept; the cache is what the next run reuses.
    assert len(planner._retimed) == 1


def test_an_anchor_naming_a_pose_that_is_gone_fails_before_the_first_move():
    """Validation checks action ids, edges and resources, never a `to_pose`.

    An anchor pointing at a pose that was renamed or re-captured out of
    arm_config.yaml would otherwise surface at dispatch, mid-sequence, possibly
    with the payload attached.
    """
    import dataclasses

    planner = ArmTrajectoryPlanner(_config())
    anchor = dataclasses.replace(
        _taught_action(),
        action_id="to_gone",
        metadata={"source": "moveit_planned", "to_pose": "Was_Recaptured_L"},
    )
    node = _node_with([_named("a"), anchor], planner)

    problems, interrupted = node._prewarm_arm_actions(
        _FakeGraph(["a", "to_gone"]), "demo", None
    )
    assert interrupted is False
    assert len(problems) == 1
    assert problems[0].startswith("to_gone: ") and "unknown anchor pose" in problems[0]


def test_a_not_yet_taught_action_still_dry_runs():
    """A dry run reports it at dispatch and carries on, so planning ahead must
    not be the stricter of the two."""
    import dataclasses

    planner = ArmTrajectoryPlanner(_config())
    untaught = dataclasses.replace(
        _taught_action(), action_id="untaught", metadata={"source": "recorded"}
    )
    node = _node_with([untaught], planner)
    node.arm_dry_run = True

    assert node._prewarm_arm_actions(_FakeGraph(["untaught"]), "demo", None) == ([], False)

    node.arm_dry_run = False
    problems, _ = node._prewarm_arm_actions(_FakeGraph(["untaught"]), "demo", None)
    assert len(problems) == 1 and problems[0].startswith("untaught: ")
