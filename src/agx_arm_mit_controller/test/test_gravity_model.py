from types import SimpleNamespace

import pytest

from agx_arm_mit_controller.gravity_model import GravityModelError, PinocchioGravityModel, create_gravity_model


def test_create_gravity_model_rejects_unknown_backend():
    with pytest.raises(GravityModelError) as exc:
        create_gravity_model("not-a-backend")

    assert "Unsupported gravity backend" in str(exc.value)


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


def test_pinocchio_gravity_model_accepts_prefixed_flange_frames():
    class FakePlacement:
        def __init__(self):
            self.translation = [0.1, 0.2, 0.3]
            self.rotation = object()

    class FakePin:
        class utils:
            @staticmethod
            def zero(size):
                return [0.0] * size

        class rpy:
            @staticmethod
            def matrixToRpy(rotation):
                del rotation
                return [0.4, 0.5, 0.6]

        @staticmethod
        def forwardKinematics(model, data, q):
            del model, data, q

        @staticmethod
        def updateFramePlacements(model, data):
            del model, data

    frames = [SimpleNamespace(name="right_arm_nero_tool0")]
    model = PinocchioGravityModel(
        urdf_path="/tmp/fake_prefixed.urdf",
        joint_names=["joint1", "joint2", "joint3"],
        _pin=FakePin(),
        _model=SimpleNamespace(
            nq=3,
            frames=frames,
            existFrame=lambda name: name == "right_arm_nero_tool0",
            getFrameId=lambda name: 0,
        ),
        _data=SimpleNamespace(oMf=[FakePlacement()]),
    )

    assert model.compute_flange_pose([0.0, 0.0, 0.0]) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]