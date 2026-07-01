import pytest

from agx_arm_mit_controller.duo_trajectory import (
    ArmSegment,
    DuoMergeError,
    interpolate_columns,
    merge_arm_recordings,
)
from agx_arm_mit_controller.trajectory_io import RecordedTrajectory, RecordedTrajectoryPoint


def _recording(name, joint_names, frames, rate_hz=10.0):
    """frames: list of (t, [positions...])."""
    points = [
        RecordedTrajectoryPoint(
            time_from_start=t,
            positions=list(pos),
            velocities=[0.0] * len(pos),
            efforts=[0.0] * len(pos),
        )
        for t, pos in frames
    ]
    return RecordedTrajectory(
        name=name,
        robot="nero",
        joint_names=list(joint_names),
        sample_rate_hz=rate_hz,
        recorded_at="2026-01-01T00:00:00",
        points=points,
        metadata={},
    )


def test_interpolate_columns_linear_midpoint():
    grid = [0.0, 0.5, 1.0]
    out = interpolate_columns([0.0, 1.0], [[0.0, 10.0], [2.0, 20.0]], grid)
    assert out[0] == [0.0, 10.0]
    assert out[1] == pytest.approx([1.0, 15.0])
    assert out[2] == [2.0, 20.0]


def test_interpolate_columns_clamps_past_end():
    # query beyond the last sample holds the final value (short-arm hold)
    out = interpolate_columns([0.0, 1.0], [[0.0], [5.0]], [2.0, 3.0])
    assert out == [[5.0], [5.0]]


def test_merge_concatenates_in_segment_order():
    left = _recording("left", ["joint1", "joint2"], [(0.0, [0.0, 0.0]), (1.0, [1.0, 2.0])])
    right = _recording("right", ["joint1", "joint2"], [(0.0, [0.0, 0.0]), (1.0, [3.0, 4.0])])
    merged = merge_arm_recordings(
        [
            ArmSegment(left, ("left_arm_joint1", "left_arm_joint2")),
            ArmSegment(right, ("right_arm_joint1", "right_arm_joint2")),
        ],
        name="duo",
        rate_hz=1.0,
    )
    assert merged.joint_names == [
        "left_arm_joint1",
        "left_arm_joint2",
        "right_arm_joint1",
        "right_arm_joint2",
    ]
    # last frame carries both arms at their final poses, in order
    assert merged.points[-1].positions == pytest.approx([1.0, 2.0, 3.0, 4.0])
    # every frame is 14-dim-style: sum of both segment widths
    assert all(len(p.positions) == 4 for p in merged.points)


def test_merge_holds_shorter_arm_at_final_pose():
    short = _recording("short", ["joint1"], [(0.0, [0.0]), (1.0, [1.0])])
    long = _recording("long", ["joint1"], [(0.0, [0.0]), (2.0, [2.0])])
    merged = merge_arm_recordings(
        [ArmSegment(short, ("left_arm_joint1",)), ArmSegment(long, ("right_arm_joint1",))],
        name="duo",
        rate_hz=1.0,
    )
    # grid spans the longer arm's duration (0,1,2); short arm holds 1.0 after t=1
    assert merged.duration == pytest.approx(2.0)
    positions = [p.positions for p in merged.points]
    assert positions[0] == pytest.approx([0.0, 0.0])
    assert positions[1] == pytest.approx([1.0, 1.0])
    assert positions[2] == pytest.approx([1.0, 2.0])  # short held at 1.0, long reached 2.0


def test_merge_rejects_joint_count_mismatch():
    rec = _recording("r", ["joint1", "joint2"], [(0.0, [0.0, 0.0])])
    with pytest.raises(DuoMergeError):
        merge_arm_recordings([ArmSegment(rec, ("only_one",))], name="duo", rate_hz=1.0)


def test_merge_rejects_duplicate_duo_joint_names():
    a = _recording("a", ["joint1"], [(0.0, [0.0]), (1.0, [1.0])])
    b = _recording("b", ["joint1"], [(0.0, [0.0]), (1.0, [1.0])])
    with pytest.raises(DuoMergeError):
        merge_arm_recordings(
            [ArmSegment(a, ("dup",)), ArmSegment(b, ("dup",))],
            name="duo",
            rate_hz=1.0,
        )


def test_merge_requires_segments():
    with pytest.raises(DuoMergeError):
        merge_arm_recordings([], name="duo")
