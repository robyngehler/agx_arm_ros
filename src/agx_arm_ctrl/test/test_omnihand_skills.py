"""Unit tests for the ROS-free OmniHand skill logic (agx_arm_ctrl.omnihand.skills)."""

from agx_arm_ctrl.omnihand.skills import (
    AGG_MAX,
    AGG_MEAN,
    AGG_MIN,
    MOTION_CLOSE_UNTIL_CONTACT,
    MOTION_OPEN,
    aggregate_contact,
    contact_score,
    matched_finger_values,
    parse_skill_catalogue,
    parse_tactile,
    step_toward,
    within_tolerance,
)


# --- skill catalogue ---------------------------------------------------------

def test_fallback_catalogue_has_mvp_skills():
    cat = parse_skill_catalogue(None)
    assert cat.resolve("open_hand").motion == MOTION_OPEN
    assert cat.resolve("open_hand").target_preset == "zero"
    grasp = cat.resolve("grasp_glass_until_contact")
    assert grasp.motion == MOTION_CLOSE_UNTIL_CONTACT
    assert grasp.target_preset == "fist_vendor_demo"
    assert cat.resolve("stop_hand").target_preset is None


def test_catalogue_parses_yaml_dict_and_defaults():
    data = {
        "omnihand_skills": {
            "open_hand": {"motion": "open", "target_preset": "flat"},
            "grasp": {"motion": "close_until_contact", "target_preset": "fist"},
        },
        "defaults": {"contact_aggregation": "min", "close_step_rad": 0.02},
    }
    cat = parse_skill_catalogue(data)
    assert cat.resolve("open_hand").target_preset == "flat"
    assert cat.defaults.contact_aggregation == "min"
    assert cat.defaults.close_step_rad == 0.02


def test_unknown_skill_raises():
    cat = parse_skill_catalogue(None)
    try:
        cat.resolve("nope")
    except ValueError as exc:
        assert "nope" in str(exc)
    else:  # pragma: no cover
        raise AssertionError("expected ValueError for unknown skill")


# --- tactile parsing ---------------------------------------------------------

def test_parse_tactile_splits_blocks_and_picks_normal_force():
    # 3 fingers, block = [online_state, normal_force, tangent_force]
    layout = "thumb_tip,index_tip,middle_tip"
    values = [1.0, 0.4, 0.05, 1.0, 0.6, 0.1, 1.0, 0.0, 0.0]
    reading = parse_tactile(layout, values, normal_force_offset=1)
    assert reading.per_finger_normal["thumb_tip"] == 0.4
    assert reading.per_finger_normal["index_tip"] == 0.6
    assert reading.per_finger_normal["middle_tip"] == 0.0


def test_parse_tactile_flat_array_has_no_attribution():
    reading = parse_tactile("flat_array", [0.0] * 32)
    assert reading.per_finger_normal == {}


def test_matched_finger_values_uses_aliases():
    layout = "thumb_tip,index_tip,middle_tip,ring_tip,little_tip"
    values = []
    for normal in (0.4, 0.6, 0.5, 0.3, 0.1):
        values += [1.0, normal, 0.0]
    reading = parse_tactile(layout, values, normal_force_offset=1)
    # "pinky" should alias to little_tip; "ring" -> ring_tip.
    vals = matched_finger_values(reading, ["thumb", "ring", "pinky"])
    assert sorted(vals) == [0.1, 0.3, 0.4]


def test_contact_score_aggregations():
    layout = "thumb_tip,index_tip,middle_tip"
    values = [1.0, 0.2, 0.0, 1.0, 0.6, 0.0, 1.0, 0.4, 0.0]
    reading = parse_tactile(layout, values, normal_force_offset=1)
    sensors = ["thumb", "index", "middle"]
    assert contact_score(reading, sensors, AGG_MAX) == 0.6
    assert contact_score(reading, sensors, AGG_MIN) == 0.2
    assert abs(contact_score(reading, sensors, AGG_MEAN) - 0.4) < 1e-9


def test_aggregate_contact_empty_is_zero():
    assert aggregate_contact([], AGG_MEAN) == 0.0


# --- motion helpers ----------------------------------------------------------

def test_step_toward_is_bounded():
    out = step_toward([0.0, 0.0], [1.0, -1.0], max_step=0.1)
    assert out == [0.1, -0.1]


def test_step_toward_reaches_when_close():
    out = step_toward([0.95, -0.95], [1.0, -1.0], max_step=0.1)
    assert abs(out[0] - 1.0) < 1e-9
    assert abs(out[1] + 1.0) < 1e-9


def test_within_tolerance():
    assert within_tolerance([0.0, 0.0], [0.03, -0.03], 0.05)
    assert not within_tolerance([0.0, 0.0], [0.1, 0.0], 0.05)
