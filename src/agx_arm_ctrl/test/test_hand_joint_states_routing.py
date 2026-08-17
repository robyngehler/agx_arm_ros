"""Where each arm driver looks for its hand's joint states.

The driver folds them into the combined ``feedback/joint_states``, which is the
only place move_group learns the hand's 24 joints from. Point it at a topic
nobody publishes and move_group holds a partial state forever — it plans
nothing, and says so once per second rather than failing.
"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

RELATIVE = "feedback/omnihand/joint_states"


def _launch_module():
    """Load the launch file by path; its name carries a dot, so import cannot."""
    path = (
        Path(get_package_share_directory("agx_arm_ctrl"))
        / "launch" / "start_agx_arm_moveit.launch.py"
    )
    spec = importlib.util.spec_from_file_location("_moveit_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _topic(bridge_here: str, effector: str, side: str) -> str:
    return _launch_module()._hand_joint_states_topic(bridge_here, effector, side)


def test_a_bridge_in_this_arms_namespace_is_reached_relatively():
    assert _topic("true", "omnihand", "left") == RELATIVE


def test_an_externally_owned_bridge_is_named_absolutely_per_side():
    assert _topic("false", "omnihand", "left") == "/left_hand/" + RELATIVE
    assert _topic("false", "omnihand", "right") == "/right_hand/" + RELATIVE


def test_an_arm_without_a_hand_keeps_the_default():
    # The driver does not subscribe at all unless effector_type is omnihand, so
    # inventing a hand namespace here would only be misleading.
    assert _topic("false", "none", "") == RELATIVE
    assert _topic("false", "agx_gripper", "left") == RELATIVE


def test_an_unnamed_side_falls_back_rather_than_guessing():
    assert _topic("false", "omnihand", "") == RELATIVE
