"""A readback is a measurement, not a command: it is reported, never saturated."""

from types import SimpleNamespace

import pytest

from agx_arm_ctrl.omnihand.sdk_o12_pro import O12ProSdkBackend


def _backend(raw, *, window=(-0.5, 0.5)):
    """A stand-in self carrying only what the readback path touches."""
    low, high = window
    names = [f"left_j{i}" for i in range(len(raw))]
    stub = SimpleNamespace(
        joint_names=names,
        positions=[0.0] * len(raw),
        _active_joint_min=[low] * len(raw),
        _active_joint_max=[high] * len(raw),
        _window_warning="",
        _last_out_of_window=[],
        communication_fault=False,
        connected=True,
        status_text="",
        hand=SimpleNamespace(get_all_active_joint_angles=lambda: list(raw)),
    )
    for name in ("_clamp", "_report_out_of_window", "read_joint_state",
                 "_set_fault", "_clear_fault"):
        setattr(stub, name, getattr(O12ProSdkBackend, name).__get__(stub))
    return stub


def test_a_reading_inside_the_window_is_returned_as_measured():
    backend = _backend([0.1, -0.2, 0.0])
    assert backend.read_joint_state() == pytest.approx([0.1, -0.2, 0.0])
    assert backend._window_warning == ""


def test_a_reading_outside_the_window_is_not_pinned_to_the_edge():
    """Pinned, a joint reports the edge whatever it did, and no target matches it."""
    backend = _backend([0.9, -0.9, 0.0])
    assert backend.read_joint_state() == pytest.approx([0.9, -0.9, 0.0])


def test_an_out_of_window_reading_names_the_joint():
    backend = _backend([0.9, 0.0, 0.0])
    backend.read_joint_state()
    assert "left_j0" in backend._window_warning
    assert "outside" in backend._window_warning


def test_the_same_out_of_window_reading_is_not_repeated():
    backend = _backend([0.9, 0.0, 0.0])
    backend.read_joint_state()
    first = backend._window_warning
    backend._window_warning = ""
    backend.read_joint_state()
    assert backend._window_warning == "", "an unchanged reading re-reported"
    assert "left_j0" in first


def test_a_command_is_still_saturated():
    """Only the readback stopped being clamped."""
    backend = _backend([0.0, 0.0, 0.0])
    assert backend._clamp([9.0, -9.0, 0.2]) == pytest.approx([0.5, -0.5, 0.2])


def test_the_zero_window_edge_case_that_produced_negative_zero():
    """left thumb_abad's window is [-0.0, +0.94]; clamping pinned it at -0.0."""
    backend = _backend([-0.31], window=(-0.0, 0.94))
    assert backend.read_joint_state() == pytest.approx([-0.31])
    assert backend._clamp([-0.31]) == pytest.approx([-0.0])
