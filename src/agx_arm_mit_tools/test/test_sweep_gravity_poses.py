"""Pose-set construction and refusal, without the SDK or an arm."""

import pytest

from agx_arm_mit_tools.sweep_gravity_poses import (
	JOINT_COUNT,
	build_sweep,
	check_reachable,
	joint_limits,
	load_poses,
)

LIMITS = [(-2.6, 2.6)] * JOINT_COUNT
START = [0.0] * JOINT_COUNT


URDF = """<?xml version="1.0"?>
<robot name="t">
  <link name="base_link"/>
  {joints}
</robot>
"""


def _urdf(tmp_path, prefix=""):
	joints = "".join(
		f'<joint name="{prefix}joint{i}" type="revolute">'
		f'<parent link="base_link"/><child link="l{i}"/>'
		f'<limit lower="-2.6" upper="2.6" effort="1" velocity="1"/></joint>'
		for i in range(1, JOINT_COUNT + 1)
	)
	path = tmp_path / "t.urdf"
	path.write_text(URDF.format(joints=joints), encoding="utf-8")
	return path


def test_limits_are_read_for_every_joint(tmp_path):
	assert joint_limits(_urdf(tmp_path)) == LIMITS


def test_prefixed_duo_joints_are_found_too(tmp_path):
	assert joint_limits(_urdf(tmp_path, "right_arm_")) == LIMITS


def test_a_urdf_missing_a_joint_is_refused(tmp_path):
	path = tmp_path / "short.urdf"
	path.write_text(URDF.format(joints=""), encoding="utf-8")
	with pytest.raises(SystemExit, match="joint1"):
		joint_limits(path)


def test_a_bare_list_pose_file_loads(tmp_path):
	path = tmp_path / "p.yaml"
	path.write_text("- [0, 0, 0, 0, 0, 0, 0]\n- [0.1, 0, 0, 0, 0, 0, 0]\n", encoding="utf-8")
	poses = load_poses(path)
	assert [name for name, _ in poses] == ["pose1", "pose2"]
	assert poses[1][1][0] == pytest.approx(0.1)


def test_the_arm_config_poses_mapping_loads(tmp_path):
	path = tmp_path / "arm_config.yaml"
	path.write_text(
		"poses:\n"
		"  Can_Pre_Grip_R: {robot_id: right_arm, q: [0.1, 0, 0, 0, 0, 0, 0]}\n",
		encoding="utf-8",
	)
	poses = load_poses(path)
	assert poses == [("Can_Pre_Grip_R", [0.1, 0, 0, 0, 0, 0, 0])]


def test_a_duo_pose_is_refused_rather_than_halved(tmp_path):
	"""14 values fit no single arm; taking half would command the other side's angles."""
	path = tmp_path / "duo.yaml"
	path.write_text(
		"poses:\n  Wave_Both: {robot_id: both_arms, q: " + str([0.0] * 14) + "}\n",
		encoding="utf-8",
	)
	with pytest.raises(SystemExit, match="14 values"):
		load_poses(path)


def test_the_sweep_moves_one_joint_at_a_time():
	poses = build_sweep(START, LIMITS, [2, 3], steps=5, fraction=0.5)
	for name, pose in poses:
		if name in ("start",) or name.endswith("_back"):
			continue
		moved = [i for i, value in enumerate(pose) if value != START[i]]
		assert len(moved) <= 1, f"{name} moved {moved}"


def test_the_sweep_spans_enough_for_the_fitter():
	poses = build_sweep(START, LIMITS, [2], steps=5, fraction=0.5)
	values = [pose[1] for _, pose in poses]
	# 50% of a 5.2 rad travel, centred on the start pose.
	assert max(values) - min(values) == pytest.approx(2.6, abs=1e-6)


def test_the_sweep_stays_inside_the_limits():
	tight = list(LIMITS)
	tight[1] = (-0.2, 0.2)
	poses = build_sweep(START, tight, [2], steps=5, fraction=1.0)
	assert all(-0.2 <= pose[1] <= 0.2 for _, pose in poses)


def test_a_joint_with_no_room_is_skipped_not_commanded():
	tight = list(LIMITS)
	tight[1] = (-0.01, 0.01)
	poses = build_sweep(START, tight, [2], steps=5, fraction=1.0)
	assert [name for name, _ in poses] == ["start"]


def test_a_pose_outside_the_limits_is_refused():
	bad = [("far", [3.0] + [0.0] * 6)]
	with pytest.raises(SystemExit, match="outside"):
		check_reachable(bad, LIMITS, START, max_step=5.0)


def test_a_pose_that_would_swing_too_far_is_refused():
	bad = [("swing", [2.0] + [0.0] * 6)]
	with pytest.raises(SystemExit, match="jumps"):
		check_reachable(bad, LIMITS, START, max_step=1.2)


def test_the_step_is_measured_against_the_previous_pose():
	"""Two poses each within one step of their predecessor are fine together."""
	walk = [("a", [1.0] + [0.0] * 6), ("b", [2.0] + [0.0] * 6)]
	check_reachable(walk, LIMITS, START, max_step=1.2)


def test_a_generated_sweep_passes_its_own_check():
	poses = build_sweep(START, LIMITS, [2, 3, 4, 6], steps=5, fraction=0.5)
	check_reachable(poses, LIMITS, START, max_step=1.2)


def test_no_single_move_exceeds_one_grid_interval():
	"""The traverse is what keeps the default sweep inside the default guard."""
	poses = build_sweep(START, LIMITS, [2], steps=5, fraction=0.5)
	values = [pose[1] for _, pose in poses]
	interval = 2.6 / 4
	steps = [abs(b - a) for a, b in zip(values, values[1:])]
	assert max(steps) <= interval + 1e-9, f"largest move {max(steps):.4f} > {interval:.4f}"


def test_the_traverse_visits_both_ends():
	poses = build_sweep(START, LIMITS, [2], steps=5, fraction=0.5)
	values = [pose[1] for _, pose in poses]
	assert min(values) == pytest.approx(-1.3)
	assert max(values) == pytest.approx(1.3)


def test_it_ends_where_it_started():
	poses = build_sweep(START, LIMITS, [2, 3], steps=5, fraction=0.5)
	assert poses[-1][1] == START


def test_the_other_arms_poses_are_skipped_not_refused(tmp_path):
	"""arm_config mixes per-arm and 14-value both_arms entries."""
	path = tmp_path / "arm_config.yaml"
	path.write_text(
		"arm_executor:\n"
		"  poses:\n"
		"    Pick_R: {robot_id: right_arm, q: [0.1, 0, 0, 0, 0, 0, 0]}\n"
		"    Pick_L: {robot_id: left_arm, q: [0.2, 0, 0, 0, 0, 0, 0]}\n"
		"    Wave_Both: {robot_id: both_arms, q: " + str([0.0] * 14) + "}\n",
		encoding="utf-8",
	)
	assert load_poses(path, "right") == [("Pick_R", [0.1, 0, 0, 0, 0, 0, 0])]
	assert load_poses(path, "left") == [("Pick_L", [0.2, 0, 0, 0, 0, 0, 0])]


def test_a_file_with_no_pose_for_this_side_is_refused(tmp_path):
	path = tmp_path / "left_only.yaml"
	path.write_text(
		"poses:\n  Pick_L: {robot_id: left_arm, q: [0.2, 0, 0, 0, 0, 0, 0]}\n",
		encoding="utf-8",
	)
	with pytest.raises(SystemExit, match="no pose for the right arm"):
		load_poses(path, "right")
