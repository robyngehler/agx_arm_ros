from agx_arm_mit_tools.joint_state_name_adapter import adapt_joint_name, adapt_joint_names


def test_adapt_joint_name_prepends_prefix_once():
	assert adapt_joint_name("joint1", "right_arm_", "prepend") == "right_arm_joint1"
	assert adapt_joint_name("right_arm_joint1", "right_arm_", "prepend") == "right_arm_joint1"
	assert adapt_joint_name("right_thumb_roll_joint", "right_arm_", "prepend") == "right_thumb_roll_joint"


def test_adapt_joint_name_strips_prefix_when_present():
	assert adapt_joint_name("right_arm_joint1", "right_arm_", "strip") == "joint1"
	assert adapt_joint_name("joint1", "right_arm_", "strip") == "joint1"


def test_adapt_joint_names_preserves_order():
	assert adapt_joint_names(["joint1", "right_thumb_roll_joint", "joint2"], "right_arm_", "prepend") == [
		"right_arm_joint1",
		"right_thumb_roll_joint",
		"right_arm_joint2",
	]