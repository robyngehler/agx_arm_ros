import sys
from types import SimpleNamespace
from pathlib import Path

from agx_arm_mit_controller.gravity_launch_utils import parse_xacro_mappings, resolve_gravity_urdf_path


def _install_fake_xacro(monkeypatch):
  class FakeXacroResult:
    def __init__(self, xml_text: str):
      self._xml_text = xml_text

    def toprettyxml(self, indent: str = "  ") -> str:
      del indent
      return self._xml_text

  def fake_process_file(path: str, mappings: dict[str, str]):
    del path
    use_left_arm = mappings.get("use_left_arm", "true") == "true"
    use_left_hand = mappings.get("use_left_hand", "false") == "true"
    use_right_arm = mappings.get("use_right_arm", "true") == "true"
    use_right_hand = mappings.get("use_right_hand", "false") == "true"

    lines = ['<robot name="duo_nero_system">', '  <link name="body_base_link"/>']
    if use_left_arm:
      lines.extend(
        [
          '  <link name="left_arm_base_link"/>',
          '  <joint name="left_arm_joint1" type="revolute">',
          '    <parent link="body_base_link"/>',
          '    <child link="left_arm_base_link"/>',
          '    <origin xyz="0 0 0" rpy="0 0 0"/>',
          '    <axis xyz="0 0 1"/>',
          '    <limit lower="-1" upper="1" effort="1" velocity="1"/>',
          '  </joint>',
        ]
      )
    if use_right_arm:
      lines.extend(
        [
          '  <link name="right_arm_base_link"/>',
          '  <joint name="right_arm_joint1" type="revolute">',
          '    <parent link="body_base_link"/>',
          '    <child link="right_arm_base_link"/>',
          '    <origin xyz="0 0 0" rpy="0 0 0"/>',
          '    <axis xyz="0 0 1"/>',
          '    <limit lower="-1" upper="1" effort="1" velocity="1"/>',
          '  </joint>',
        ]
      )
      if use_right_hand:
        lines.extend(
          [
            '  <link name="right_base_link"/>',
            '  <joint name="right_arm_right_hand_base_joint" type="fixed">',
            '    <parent link="right_arm_base_link"/>',
            '    <child link="right_base_link"/>',
            '  </joint>',
            '  <joint name="right_thumb_roll_joint" type="revolute">',
            '    <parent link="right_base_link"/>',
            '    <child link="right_thumb_roll"/>',
            '    <origin xyz="0 0 0" rpy="0 0 0"/>',
            '    <axis xyz="0 0 1"/>',
            '    <limit lower="-1" upper="1" effort="1" velocity="1"/>',
            '  </joint>',
            '  <link name="right_thumb_roll"/>',
          ]
        )
    if use_left_hand:
      lines.extend(
        [
          '  <link name="left_base_link"/>',
          '  <joint name="left_arm_left_hand_base_joint" type="fixed">',
          '    <parent link="left_arm_base_link"/>',
          '    <child link="left_base_link"/>',
          '  </joint>',
        ]
      )

    lines.append('</robot>')
    return FakeXacroResult("\n".join(lines))

  monkeypatch.setitem(sys.modules, "xacro", SimpleNamespace(process_file=fake_process_file))


def test_parse_xacro_mappings_reads_key_value_pairs():
    mappings = parse_xacro_mappings("use_left_arm:=false use_right_arm:=true body_mesh_xyz:='0 0 0'")

    assert mappings == {
        "use_left_arm": "false",
        "use_right_arm": "true",
        "body_mesh_xyz": "0 0 0",
    }


def test_resolve_gravity_urdf_path_builds_single_arm_duo_slice(tmp_path, monkeypatch):
    _install_fake_xacro(monkeypatch)
    custom_model = tmp_path / "duo_system.urdf.xacro"
    custom_model.write_text(
        """<?xml version=\"1.0\"?>
<robot name=\"duo_nero_system\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">
  <xacro:arg name=\"use_left_arm\" default=\"true\"/>
  <xacro:arg name=\"use_left_hand\" default=\"false\"/>
  <xacro:arg name=\"use_right_arm\" default=\"true\"/>
  <xacro:arg name=\"use_right_hand\" default=\"false\"/>
  <link name=\"body_base_link\"/>
  <xacro:if value=\"$(arg use_left_arm)\">
    <link name=\"left_arm_base_link\"/>
    <joint name=\"left_arm_joint1\" type=\"revolute\">
      <parent link=\"body_base_link\"/>
      <child link=\"left_arm_base_link\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <axis xyz=\"0 0 1\"/>
      <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>
    </joint>
  </xacro:if>
  <xacro:if value=\"$(arg use_right_arm)\">
    <link name=\"right_arm_base_link\"/>
    <joint name=\"right_arm_joint1\" type=\"revolute\">
      <parent link=\"body_base_link\"/>
      <child link=\"right_arm_base_link\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <axis xyz=\"0 0 1\"/>
      <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>
    </joint>
  </xacro:if>
</robot>
""",
        encoding="utf-8",
    )

    resolved_path = Path(
        resolve_gravity_urdf_path(
            custom_model=str(custom_model),
            custom_model_xacro_args="use_left_arm:=true use_right_arm:=true",
            input_joint_prefix="right_arm_",
            effector_type="none",
        )
    )
    generated_urdf = resolved_path.read_text(encoding="utf-8")

    assert resolved_path.exists()
    assert "right_arm_joint1" in generated_urdf
    assert "left_arm_joint1" not in generated_urdf


def test_resolve_gravity_urdf_path_duo_side_overrides_prefix(tmp_path, monkeypatch):
    # The prefix-free teach loop selects the arm slice via duo_side directly.
    _install_fake_xacro(monkeypatch)
    custom_model = tmp_path / "duo_system.urdf.xacro"
    custom_model.write_text(
        """<?xml version=\"1.0\"?>
<robot name=\"duo_nero_system\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">
  <xacro:arg name=\"use_left_arm\" default=\"true\"/>
  <xacro:arg name=\"use_left_hand\" default=\"false\"/>
  <xacro:arg name=\"use_right_arm\" default=\"true\"/>
  <xacro:arg name=\"use_right_hand\" default=\"false\"/>
  <link name=\"body_base_link\"/>
  <xacro:if value=\"$(arg use_left_arm)\">
    <link name=\"left_arm_base_link\"/>
    <joint name=\"left_arm_joint1\" type=\"revolute\">
      <parent link=\"body_base_link\"/>
      <child link=\"left_arm_base_link\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <axis xyz=\"0 0 1\"/>
      <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>
    </joint>
  </xacro:if>
  <xacro:if value=\"$(arg use_right_arm)\">
    <link name=\"right_arm_base_link\"/>
    <joint name=\"right_arm_joint1\" type=\"revolute\">
      <parent link=\"body_base_link\"/>
      <child link=\"right_arm_base_link\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <axis xyz=\"0 0 1\"/>
      <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>
    </joint>
  </xacro:if>
</robot>
""",
        encoding="utf-8",
    )

    generated_urdf = Path(
        resolve_gravity_urdf_path(
            custom_model=str(custom_model),
            custom_model_xacro_args="use_left_arm:=true use_right_arm:=true",
            input_joint_prefix="",   # teach loop drops the prefix
            duo_side="right",
        )
    ).read_text(encoding="utf-8")

    assert "right_arm_joint1" in generated_urdf
    assert "left_arm_joint1" not in generated_urdf


def test_resolve_gravity_urdf_path_freezes_duo_omnihand_as_static_payload(tmp_path, monkeypatch):
    _install_fake_xacro(monkeypatch)
    custom_model = tmp_path / "duo_system.urdf.xacro"
    custom_model.write_text(
        """<?xml version=\"1.0\"?>
<robot name=\"duo_nero_system\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">
  <xacro:arg name=\"use_left_arm\" default=\"true\"/>
  <xacro:arg name=\"use_left_hand\" default=\"false\"/>
  <xacro:arg name=\"use_right_arm\" default=\"true\"/>
  <xacro:arg name=\"use_right_hand\" default=\"false\"/>
  <link name=\"body_base_link\"/>
  <xacro:if value=\"$(arg use_right_arm)\">
    <link name=\"right_arm_base_link\"/>
    <joint name=\"right_arm_joint1\" type=\"revolute\">
      <parent link=\"body_base_link\"/>
      <child link=\"right_arm_base_link\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <axis xyz=\"0 0 1\"/>
      <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>
    </joint>
    <xacro:if value=\"$(arg use_right_hand)\">
      <link name=\"right_base_link\"/>
      <joint name=\"right_arm_right_hand_base_joint\" type=\"fixed\">
        <parent link=\"right_arm_base_link\"/>
        <child link=\"right_base_link\"/>
      </joint>
      <joint name=\"right_thumb_roll_joint\" type=\"revolute\">
        <parent link=\"right_base_link\"/>
        <child link=\"right_thumb_roll\"/>
        <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
        <axis xyz=\"0 0 1\"/>
        <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>
      </joint>
      <link name=\"right_thumb_roll\"/>
    </xacro:if>
  </xacro:if>
</robot>
""",
        encoding="utf-8",
    )

    resolved_path = Path(
        resolve_gravity_urdf_path(
            custom_model=str(custom_model),
            custom_model_xacro_args="use_right_arm:=true use_right_hand:=true",
            input_joint_prefix="right_arm_",
            effector_type="omnihand",
        )
    )
    generated_urdf = resolved_path.read_text(encoding="utf-8")

    assert resolved_path.exists()
    assert "right_arm_joint1" in generated_urdf
    assert 'joint name="right_arm_right_hand_base_joint" type="fixed"' in generated_urdf
    assert 'joint name="right_thumb_roll_joint" type="fixed"' in generated_urdf
    assert 'joint name="right_thumb_roll_joint" type="revolute"' not in generated_urdf


def test_resolve_gravity_urdf_path_articulated_keeps_hand_joints_movable(tmp_path, monkeypatch):
    _install_fake_xacro(monkeypatch)
    custom_model = tmp_path / "duo_system.urdf.xacro"
    custom_model.write_text("<robot/>", encoding="utf-8")

    generated_urdf = Path(
        resolve_gravity_urdf_path(
            custom_model=str(custom_model),
            custom_model_xacro_args="use_right_arm:=true use_right_hand:=true",
            input_joint_prefix="right_arm_",
            effector_type="omnihand",
            hand_payload_mode="articulated",
        )
    ).read_text(encoding="utf-8")

    assert 'joint name="right_thumb_roll_joint" type="revolute"' in generated_urdf


def test_resolve_gravity_urdf_path_rejects_unknown_hand_payload_mode(tmp_path):
    custom_model = tmp_path / "duo_system.urdf.xacro"
    custom_model.write_text("<robot/>", encoding="utf-8")

    import pytest

    with pytest.raises(ValueError):
        resolve_gravity_urdf_path(
            custom_model=str(custom_model),
            effector_type="omnihand",
            hand_payload_mode="floppy",
        )