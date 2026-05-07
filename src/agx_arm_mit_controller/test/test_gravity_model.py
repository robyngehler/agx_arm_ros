import pytest

from agx_arm_mit_controller.gravity_model import GravityModelError, create_gravity_model


def test_create_gravity_model_without_backend_reports_clear_error():
    with pytest.raises(GravityModelError) as exc:
        create_gravity_model("pinocchio")

    assert "Pinocchio is not installed" in str(exc.value)