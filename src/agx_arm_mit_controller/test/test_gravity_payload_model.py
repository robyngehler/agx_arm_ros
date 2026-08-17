"""Payload gravity model: URDF derivation, and what the derived model must preserve.

The loaded model is the base model plus one fixed link. It must add mass at a
lever without adding a DoF, and detaching must return the base model's torque
exactly, because the two models are the only thing a payload transition swaps.
"""

import math
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

from agx_arm_mit_controller.gravity_launch_utils import (
    PAYLOAD_LINK_NAME,
    derive_fixed_payload_urdf,
    resolve_payload_parent_link,
    solid_cylinder_inertia,
)

# A two-joint arm along +x with a tool0 flange, small enough to reason about by
# hand: joint1 rotates about z at the origin, joint2 about y at x=0.5.
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
  <link name="link2">
    <inertial>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="left_arm_nero_tool0"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="10" velocity="3"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="10" velocity="3"/>
  </joint>
  <joint name="left_arm_nero_tool0_joint" type="fixed">
    <parent link="link2"/><child link="left_arm_nero_tool0"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


@pytest.fixture
def base_urdf(tmp_path) -> str:
    path = tmp_path / "base_gravity.urdf"
    path.write_text(_ARM_URDF, encoding="utf-8")
    return str(path)


def _model(urdf_path: str):
    pytest.importorskip("pinocchio")
    from agx_arm_mit_controller.gravity_model import PinocchioGravityModel

    return PinocchioGravityModel.from_urdf(urdf_path)


# --- URDF derivation ---------------------------------------------------------

def test_derived_urdf_appends_one_fixed_payload_link(base_urdf):
    derived = ET.parse(
        derive_fixed_payload_urdf(base_urdf, "left_arm_nero_tool0", 1.0, [0.15, 0.0, 0.0])
    ).getroot()

    payload_links = [
        link for link in derived.findall("link")
        if link.attrib["name"] == PAYLOAD_LINK_NAME
    ]
    assert len(payload_links) == 1
    joint = next(
        j for j in derived.findall("joint")
        if j.find("child").attrib["link"] == PAYLOAD_LINK_NAME
    )
    assert joint.attrib["type"] == "fixed"
    assert joint.find("parent").attrib["link"] == "left_arm_nero_tool0"
    assert joint.find("origin").attrib["xyz"] == "0.15 0.0 0.0"
    assert float(payload_links[0].find("inertial").find("mass").attrib["value"]) == 1.0


def test_derived_urdf_leaves_the_base_file_untouched(base_urdf):
    before = Path(base_urdf).read_text(encoding="utf-8")
    derive_fixed_payload_urdf(base_urdf, "left_arm_nero_tool0", 1.0, [0.15, 0.0, 0.0])
    assert Path(base_urdf).read_text(encoding="utf-8") == before


def test_derived_urdf_rejects_an_unknown_parent_link(base_urdf):
    with pytest.raises(ValueError) as exc:
        derive_fixed_payload_urdf(base_urdf, "no_such_flange", 1.0, [0.15, 0.0, 0.0])
    # The message has to name the alternatives; a wrong flange is a typo, not a
    # design decision.
    assert "left_arm_nero_tool0" in str(exc.value)


def test_derived_urdf_refuses_to_stack_a_second_payload(base_urdf):
    once = derive_fixed_payload_urdf(base_urdf, "left_arm_nero_tool0", 1.0, [0.15, 0.0, 0.0])
    with pytest.raises(ValueError) as exc:
        derive_fixed_payload_urdf(once, "left_arm_nero_tool0", 1.0, [0.15, 0.0, 0.0])
    assert PAYLOAD_LINK_NAME in str(exc.value)


@pytest.mark.parametrize(
    "mass, com, inertia",
    [
        (0.0, [0.15, 0.0, 0.0], None),
        (-1.0, [0.15, 0.0, 0.0], None),
        (float("nan"), [0.15, 0.0, 0.0], None),
        (1.0, [0.15, float("inf"), 0.0], None),
        (1.0, [0.15, 0.0], None),
        (1.0, [0.15, 0.0, 0.0], [-1.0, 0.0, 0.0]),
    ],
)
def test_derived_urdf_rejects_unphysical_payloads(base_urdf, mass, com, inertia):
    with pytest.raises(ValueError):
        derive_fixed_payload_urdf(base_urdf, "left_arm_nero_tool0", mass, com, inertia)


def test_solid_cylinder_inertia_matches_the_closed_form():
    ixx, iyy, izz = solid_cylinder_inertia(1.0, 0.06, 0.15)
    assert ixx == pytest.approx(1.0 * (3 * 0.06**2 + 0.15**2) / 12.0)
    assert ixx == iyy
    assert izz == pytest.approx(1.0 * 0.06**2 / 2.0)


# --- parent-link resolution --------------------------------------------------

def test_parent_link_resolves_from_the_urdf(base_urdf):
    assert resolve_payload_parent_link(base_urdf) == "left_arm_nero_tool0"


def test_parent_link_prefers_an_explicit_name(base_urdf):
    assert resolve_payload_parent_link(base_urdf, explicit_parent_link="link2") == "link2"


def test_parent_link_is_narrowed_by_the_joint_prefix(tmp_path):
    two_arms = tmp_path / "two_arms.urdf"
    two_arms.write_text(
        '<robot name="duo"><link name="left_arm_nero_tool0"/>'
        '<link name="right_arm_nero_tool0"/></robot>',
        encoding="utf-8",
    )
    assert resolve_payload_parent_link(str(two_arms), "right_arm_") == "right_arm_nero_tool0"


def test_parent_link_refuses_to_guess_between_two_arms(tmp_path):
    two_arms = tmp_path / "two_arms.urdf"
    two_arms.write_text(
        '<robot name="duo"><link name="left_arm_nero_tool0"/>'
        '<link name="right_arm_nero_tool0"/></robot>',
        encoding="utf-8",
    )
    with pytest.raises(ValueError) as exc:
        resolve_payload_parent_link(str(two_arms))
    assert "ambiguous" in str(exc.value)


# --- what the derived model must preserve ------------------------------------

def test_payload_adds_no_dof_and_keeps_the_joint_set(base_urdf):
    base = _model(base_urdf)
    loaded = _model(
        derive_fixed_payload_urdf(base_urdf, "left_arm_nero_tool0", 1.0, [0.15, 0.0, 0.0])
    )

    assert loaded.model_dofs == base.model_dofs
    assert loaded.joint_names == base.joint_names
    assert loaded.joint_q_index == base.joint_q_index


def test_payload_increases_the_gravity_torque_it_loads(base_urdf):
    base = _model(base_urdf)
    loaded = _model(
        derive_fixed_payload_urdf(base_urdf, "left_arm_nero_tool0", 1.0, [0.15, 0.0, 0.0])
    )

    # Arm horizontal along +x: joint2 carries the whole outboard weight, so a
    # mass added past the flange must raise its magnitude.
    q = [0.0, 0.0]
    base_tau = base.compute_gravity(q)
    loaded_tau = loaded.compute_gravity(q)

    assert len(loaded_tau) == len(base_tau) == 2
    assert abs(loaded_tau[1]) > abs(base_tau[1])

    # 1 kg at 0.65 m from joint2's axis (0.5 m flange offset + 0.15 m payload
    # lever), so the difference is m*g*lever.
    delta = abs(loaded_tau[1] - base_tau[1])
    assert delta == pytest.approx(1.0 * 9.81 * 0.65, rel=1e-3)


def test_detach_returns_the_base_models_torque_exactly(base_urdf):
    base = _model(base_urdf)
    loaded = _model(
        derive_fixed_payload_urdf(base_urdf, "left_arm_nero_tool0", 1.0, [0.15, 0.0, 0.0])
    )

    # Detaching swaps the reference back to `base`, so "returns exactly" means
    # the base object still answers identically after the loaded one was used.
    for q in ([0.0, 0.0], [0.3, -0.7], [-1.2, 0.4]):
        before = base.compute_gravity(q)
        loaded.compute_gravity(q)
        after = base.compute_gravity(q)
        assert after == before
        assert all(math.isfinite(value) for value in after)


def test_payload_at_zero_lever_still_loads_the_shoulder(base_urdf):
    """A payload with no lever in its own frame is not a payload with no lever."""
    base = _model(base_urdf)
    loaded = _model(
        derive_fixed_payload_urdf(base_urdf, "left_arm_nero_tool0", 1.0, [0.0, 0.0, 0.0])
    )

    # Still 0.5 m out from joint2's axis: the flange itself is the lever.
    q = [0.0, 0.0]
    delta = abs(loaded.compute_gravity(q)[1] - base.compute_gravity(q)[1])
    assert delta == pytest.approx(1.0 * 9.81 * 0.5, rel=1e-3)
