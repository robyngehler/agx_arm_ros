"""``~/payload_attached``: swap the active gravity model, or refuse.

The service must never answer "applied" when it did not apply: attaching with no
loaded model would leave the arm compensating unloaded while the coordinator
believes the teapot is accounted for. The node needs ROS to construct, so tests
build a bare instance via ``__new__``.
"""

import threading

import pytest
from std_srvs.srv import SetBool

from agx_arm_mit_controller.gravity_launch_utils import derive_fixed_payload_urdf
from agx_arm_mit_controller.mit_controller_node import NeroMitControllerNode

_ARM_URDF = """<?xml version="1.0"?>
<robot name="stub_arm">
  <link name="base_link"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="left_arm_nero_tool0"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="10" velocity="3"/>
  </joint>
  <joint name="left_arm_nero_tool0_joint" type="fixed">
    <parent link="link1"/><child link="left_arm_nero_tool0"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


class _RecordingLogger:
    def __init__(self):
        self.errors: list[str] = []

    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, msg, *_a, **_k):
        self.errors.append(str(msg))


class _StubModel:
    def __init__(self, urdf_path, joint_names=("joint1",)):
        self.urdf_path = urdf_path
        self.joint_names = list(joint_names)


def _node(base=None, loaded=None, gravity_enabled=True):
    node = NeroMitControllerNode.__new__(NeroMitControllerNode)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node.state_lock = threading.RLock()
    node.gravity_compensation_enabled = gravity_enabled
    node.gravity_model_base = base
    node.gravity_model_loaded = loaded
    node.gravity_model = base
    node.payload_attached = False
    return node


def _call(node, value):
    request = SetBool.Request()
    request.data = value
    return node._payload_attached_callback(request, SetBool.Response())


# --- the swap ----------------------------------------------------------------

def test_attach_points_the_control_loop_at_the_loaded_model():
    base, loaded = _StubModel("/tmp/base.urdf"), _StubModel("/tmp/loaded.urdf")
    node = _node(base, loaded)

    response = _call(node, True)

    assert response.success
    assert node.gravity_model is loaded
    assert node.payload_attached is True


def test_detach_points_it_back_at_the_base_model():
    base, loaded = _StubModel("/tmp/base.urdf"), _StubModel("/tmp/loaded.urdf")
    node = _node(base, loaded)
    _call(node, True)

    response = _call(node, False)

    assert response.success
    assert node.gravity_model is base
    assert node.payload_attached is False


@pytest.mark.parametrize("value", [True, False])
def test_the_swap_is_idempotent(value):
    base, loaded = _StubModel("/tmp/base.urdf"), _StubModel("/tmp/loaded.urdf")
    node = _node(base, loaded)

    first = _call(node, value)
    model_after_first = node.gravity_model
    second = _call(node, value)

    assert first.success and second.success
    assert node.gravity_model is model_after_first


# --- refusals ----------------------------------------------------------------

def test_attach_is_refused_when_no_payload_model_was_built():
    base = _StubModel("/tmp/base.urdf")
    node = _node(base, loaded=None)

    response = _call(node, True)

    assert not response.success
    assert "no payload gravity model" in response.message
    # The arm keeps compensating with the model it was actually built for.
    assert node.gravity_model is base
    assert node.payload_attached is False


def test_detach_is_still_refused_without_gravity_compensation():
    node = _node(base=None, loaded=None, gravity_enabled=False)

    response = _call(node, False)

    assert not response.success
    assert "gravity compensation is not active" in response.message


def test_a_refused_attach_leaves_no_trace_of_having_been_requested():
    node = _node(_StubModel("/tmp/base.urdf"), loaded=None)
    _call(node, True)

    # A later detach must not report a state change that never happened, and a
    # later successful attach must still be possible.
    assert node.payload_attached is False


# --- construction ------------------------------------------------------------

def _build_node(tmp_path, **overrides):
    pytest.importorskip("pinocchio")
    from agx_arm_mit_controller.gravity_model import PinocchioGravityModel

    base_path = tmp_path / "base_gravity.urdf"
    base_path.write_text(_ARM_URDF, encoding="utf-8")

    node = _node(PinocchioGravityModel.from_urdf(str(base_path)))
    node.payload_mass_kg = 1.0
    node.payload_com_xyz = [0.15, 0.0, 0.0]
    node.payload_cylinder_radius_m = 0.06
    node.payload_cylinder_height_m = 0.15
    node.payload_parent_link = ""
    node.input_joint_prefix = ""
    node.gravity_backend = "pinocchio"
    node.gravity_mounting_rpy = [0.0, 0.0, 0.0]
    for name, value in overrides.items():
        setattr(node, name, value)
    return node


def test_a_configured_payload_builds_a_loaded_model(tmp_path):
    node = _build_node(tmp_path)
    node._init_loaded_gravity_model()

    assert node.gravity_model_loaded is not None
    assert node.gravity_model_loaded.joint_names == node.gravity_model_base.joint_names
    assert _call(node, True).success


def test_zero_mass_builds_no_loaded_model(tmp_path):
    node = _build_node(tmp_path, payload_mass_kg=0.0)
    node._init_loaded_gravity_model()

    assert node.gravity_model_loaded is None
    assert not _call(node, True).success


def test_an_unresolvable_parent_link_is_logged_and_leaves_attach_refused(tmp_path):
    node = _build_node(tmp_path, payload_parent_link="not_a_link")
    node._init_loaded_gravity_model()

    assert node.gravity_model_loaded is None
    assert any("Payload gravity model unavailable" in e for e in node._logger.errors)
    assert not _call(node, True).success


def test_a_payload_model_with_a_different_joint_set_is_refused(tmp_path):
    """A model swap may change the mass, never which joints are compensated."""
    node = _build_node(tmp_path)
    real_derive = derive_fixed_payload_urdf

    def _derive_with_an_extra_joint(base_path, parent_link, mass, com, inertia=None):
        path = real_derive(base_path, parent_link, mass, com, inertia)
        text = open(path, encoding="utf-8").read().replace(
            "</robot>",
            '<link name="rogue"/>'
            '<joint name="rogue_joint" type="revolute">'
            '<parent link="left_arm_nero_tool0"/><child link="rogue"/>'
            '<origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>'
            '<limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>',
        )
        open(path, "w", encoding="utf-8").write(text)
        return path

    import agx_arm_mit_controller.mit_controller_node as module

    original = module.derive_fixed_payload_urdf
    module.derive_fixed_payload_urdf = _derive_with_an_extra_joint
    try:
        node._init_loaded_gravity_model()
    finally:
        module.derive_fixed_payload_urdf = original

    assert node.gravity_model_loaded is None
    assert any("changed the joint set" in e for e in node._logger.errors)
