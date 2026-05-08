from agx_arm_mit_controller.model_metadata import (
    default_nero_calibration_path,
    default_nero_urdf_path,
    extract_urdf_model_metadata,
)


def test_extract_urdf_model_metadata_reads_nero_description():
    urdf_path = default_nero_urdf_path()
    metadata = extract_urdf_model_metadata(urdf_path)

    assert urdf_path.name == "nero_description.urdf"
    assert metadata["robot_name"] == "nero"
    assert metadata["revolute_joint_count"] >= 7
    assert metadata["total_mass"] > 0.0
    assert any(joint["name"] == "joint1" for joint in metadata["joints"])
    assert any(link["name"] == "base_link" for link in metadata["inertial_links"])


def test_default_nero_calibration_path_finds_repo_config():
    calibration_path = default_nero_calibration_path()

    assert calibration_path.name == "nero_gravity_calibration.json"
    assert calibration_path.exists()