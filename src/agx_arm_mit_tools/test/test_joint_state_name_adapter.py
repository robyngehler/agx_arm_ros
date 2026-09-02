from agx_arm_mit_tools.joint_state_name_adapter import adapt_joint_name, adapt_joint_names


def test_adapt_joint_name_prepends_prefix_once():
	assert adapt_joint_name("joint1", "right_arm_", "prepend") == "right_arm_joint1"
	assert adapt_joint_name("right_arm_joint1", "right_arm_", "prepend") == "right_arm_joint1"
	assert adapt_joint_name("right_thumb_roll_joint", "right_arm_", "prepend") == "right_thumb_roll_joint"


def test_adapt_joint_name_strips_prefix_when_present():
	assert adapt_joint_name("right_arm_joint1", "right_arm_", "strip") == "joint1"
	assert adapt_joint_name("joint1", "right_arm_", "strip") == "joint1"


def test_adapt_joint_name_prefixes_gripper_fingers():
	"""The gripper's fingers live in the arm's name space, the hand's do not.

	Left unprefixed, they reach move_group as gripper_joint1/2 while the model
	holds <arm_prefix>gripper_joint1/2 — the joint is reported both missing and
	unknown at the publish rate.
	"""
	assert adapt_joint_name("gripper_joint1", "right_arm_", "prepend") == "right_arm_gripper_joint1"
	assert adapt_joint_name("gripper_joint2", "right_arm_", "prepend") == "right_arm_gripper_joint2"
	assert adapt_joint_name("right_arm_gripper_joint1", "right_arm_", "prepend") == "right_arm_gripper_joint1"
	assert adapt_joint_name("gripper_joint1", "right_arm_", "strip") == "gripper_joint1"
	assert adapt_joint_name("right_arm_gripper_joint1", "right_arm_", "strip") == "gripper_joint1"


def test_adapt_joint_name_passes_through_synthetic_gripper_width():
	"""`gripper` is the driver's opening-width joint and has no URDF counterpart
	at any prefix, so prefixing it would only move the error."""
	assert adapt_joint_name("gripper", "right_arm_", "prepend") == "gripper"


def test_adapt_joint_names_preserves_order():
	assert adapt_joint_names(["joint1", "right_thumb_roll_joint", "joint2"], "right_arm_", "prepend") == [
		"right_arm_joint1",
		"right_thumb_roll_joint",
		"right_arm_joint2",
	]