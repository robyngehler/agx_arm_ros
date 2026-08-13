"""The arm's fault field has to actually carry a fault.

`AgxArmStatus.err_status` was declared and never assigned: it published 0 for
every arm in every state. The MIT controller gated on it — `arm_fault_active =
int(msg.err_status) != 0` — so it held a fault check that could not fire, and
looked like coverage while providing none.
"""

from agx_arm_msgs.msg import AgxArmStatus

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode, FeedbackSnapshot


class _ErrStatus:
    def __init__(self, angle_limits=(), comm_faults=()):
        for index in range(1, 8):
            setattr(self, f"joint_{index}_angle_limit", index in angle_limits)
            setattr(
                self, f"communication_status_joint_{index}", index in comm_faults
            )


class _StatusMsg:
    def __init__(self, err_code=0, angle_limits=(), comm_faults=()):
        self.ctrl_mode = 1
        self.arm_status = 0
        self.mode_feedback = 0
        self.teach_status = 0
        self.motion_status = 0
        self.trajectory_num = 0
        self.err_code = err_code
        self.err_status = _ErrStatus(angle_limits, comm_faults)


class _Status:
    def __init__(self, msg):
        self.msg = msg


class _Arm:
    def __init__(self, status):
        self._status = status

    def get_arm_status(self):
        return self._status


def _snapshot(status):
    """The batch is acquired once and shared; publishers only read it."""
    return FeedbackSnapshot(
        joint_angles=None,
        motor_states=(),
        flange_pose=None,
        tcp_pose=None,
        arm_status=status,
        leader_joint_angles=None,
        is_ok=True,
        send_error_count=-1,
        acquired_at=0.0,
    )


def _publish(err_code=0, angle_limits=(), comm_faults=()):
    published = []
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.arm_joint_count = 7
    node.arm_status_pub = type("P", (), {"publish": lambda _s, m: published.append(m)})()
    status = _Status(_StatusMsg(err_code, angle_limits, comm_faults))
    node._publish_arm_status(_snapshot(status))
    return published[0]


def test_a_healthy_arm_reports_no_fault():
    msg = _publish()
    assert msg.fault_code == 0
    assert msg.any_fault is False


def test_the_raw_vendor_code_reaches_the_topic():
    msg = _publish(err_code=0x0042)
    assert msg.fault_code == 0x0042
    assert msg.any_fault is True


def test_a_joint_angle_limit_is_a_fault():
    msg = _publish(angle_limits=(3,))
    assert msg.any_fault is True
    assert list(msg.joint_angle_limit)[2] is True


def test_a_joint_communication_fault_is_a_fault():
    msg = _publish(comm_faults=(6,))
    assert msg.any_fault is True
    assert list(msg.communication_status_joint)[5] is True


def test_the_code_is_masked_to_the_sixteen_bits_the_vendor_defines():
    msg = _publish(err_code=0x1FFFF)
    assert msg.fault_code == 0xFFFF


def test_a_missing_vendor_field_does_not_break_publication():
    """Older tiers may not expose err_code; the flags still have to publish."""
    published = []
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.arm_joint_count = 7
    status = _StatusMsg(angle_limits=(1,))
    del status.err_code
    node.arm_status_pub = type("P", (), {"publish": lambda _s, m: published.append(m)})()
    node._publish_arm_status(_snapshot(_Status(status)))

    assert published[0].fault_code == 0
    assert published[0].any_fault is True


def test_the_message_no_longer_carries_the_dead_field():
    assert not hasattr(AgxArmStatus(), "err_status")
    assert hasattr(AgxArmStatus(), "any_fault")
