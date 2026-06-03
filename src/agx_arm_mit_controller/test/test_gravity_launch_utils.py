from pathlib import Path

from agx_arm_mit_controller.gravity_launch_utils import parse_xacro_mappings, resolve_gravity_urdf_path


def test_parse_xacro_mappings_reads_key_value_pairs():
    mappings = parse_xacro_mappings("use_left_arm:=false use_right_arm:=true body_mesh_xyz:='0 0 0'")

    assert mappings == {
        "use_left_arm": "false",
        "use_right_arm": "true",
        "body_mesh_xyz": "0 0 0",
    }


def test_resolve_gravity_urdf_path_builds_single_arm_duo_slice(tmp_path):
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


def test_resolve_gravity_urdf_path_keeps_duo_omnihand_slice_arm_only(tmp_path):
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
      <link name=\"right_hand_base_link\"/>
      <joint name=\"right_arm_right_hand_base_joint\" type=\"fixed\">
        <parent link=\"right_arm_base_link\"/>
        <child link=\"right_hand_base_link\"/>
      </joint>
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
    assert "right_arm_right_hand_base_joint" not in generated_urdf