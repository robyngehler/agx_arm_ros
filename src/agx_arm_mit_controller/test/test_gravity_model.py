from types import SimpleNamespace

import pytest

from agx_arm_mit_controller.gravity_model import GravityModelError, PinocchioGravityModel, create_gravity_model


def test_create_gravity_model_without_backend_reports_clear_error():
    with pytest.raises(GravityModelError) as exc:
        create_gravity_model("pinocchio")

    assert "Pinocchio is not installed" in str(exc.value)


def test_pinocchio_gravity_model_returns_actuator_compensation_sign():
    class FakeTau:
        def __init__(self, values):
            self._values = values

        def __getitem__(self, index):
            return self._values[index]

    class FakePin:
        class utils:
            @staticmethod
            def zero(size):
                return [0.0] * size

        @staticmethod
        def computeGeneralizedGravity(model, data, q):
            del model, data, q
            return FakeTau([1.5, -2.0, 0.25])

    model = PinocchioGravityModel(
        urdf_path="/tmp/fake.urdf",
        joint_names=["joint1", "joint2", "joint3"],
        _pin=FakePin(),
        _model=SimpleNamespace(nq=3),
        _data=object(),
    )

    assert model.compute_gravity([0.0, 0.0, 0.0]) == [-1.5, 2.0, -0.25]