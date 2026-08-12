"""A non-finite control value must never become a finite maximum command.

`clamp(value, limit)` is `max(-limit, min(limit, value))`, and for NaN that
returns `limit`. A corrupt gravity solve or trajectory point therefore left the
controller as *full torque* — and reached the driver as a plausible number that
no boundary check could recognise as wrong.
"""

import math

from agx_arm_mit_controller.mit_controller_node import clamp, first_non_finite


def test_clamp_turns_non_finite_into_the_maximum_command():
    """The behaviour the guard exists to prevent, pinned so it cannot surprise."""
    assert clamp(float("nan"), 8.0) == 8.0
    assert clamp(float("inf"), 8.0) == 8.0
    assert clamp(float("-inf"), 8.0) == -8.0


def test_finite_values_pass_the_check():
    assert first_non_finite((("kp", [1.0, 2.0]), ("kd", [0.1]))) is None
    assert first_non_finite((("empty", []),)) is None


def test_the_first_bad_value_is_named_with_its_field_and_index():
    detail = first_non_finite((("kp", [1.0, 2.0]), ("torque", [0.0, math.nan])))
    assert detail == "torque[1]=nan"


def test_infinities_are_caught_in_both_directions():
    assert first_non_finite((("t", [math.inf],),)) == "t[0]=inf"
    assert first_non_finite((("t", [-math.inf],),)) == "t[0]=-inf"


def test_a_value_that_is_not_a_number_at_all_is_caught():
    detail = first_non_finite((("kp", [1.0, None]),))
    assert detail is not None
    assert "not a number" in detail


def test_the_check_reports_the_earliest_field_in_order():
    detail = first_non_finite((("a", [math.nan]), ("b", [math.nan])))
    assert detail.startswith("a[0]")
