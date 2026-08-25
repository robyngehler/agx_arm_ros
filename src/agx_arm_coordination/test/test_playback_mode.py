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
    ({"resample_dt": 0.0}, "must be finite"),
    ({"tempo": 2.0}, "unknown playback key"),
])
def test_an_unusable_playback_request_is_refused_not_silently_defaulted(block, message):
    """A replay that ran under a different mode than the activity asked for is
    worse than one that refused to start."""
    with pytest.raises(ArmConfigError, match=message):
        playback_spec({"playback": block}, "left_arm_pour")


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
