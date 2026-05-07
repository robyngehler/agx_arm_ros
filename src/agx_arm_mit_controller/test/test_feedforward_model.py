import json

import pytest

from agx_arm_mit_controller.feedforward_model import CalibrationModel
from agx_arm_mit_controller.feedforward_model import load_calibration_model


def test_calibration_model_applies_scale_and_bias():
    model = CalibrationModel(
        joint_names=["joint1", "joint2"],
        scale=[2.0, 0.5],
        bias=[1.0, -1.0],
    )

    assert model.apply([3.0, 8.0]) == [7.0, 3.0]


def test_load_calibration_model_rejects_unsafe_values(tmp_path):
    path = tmp_path / "bad_calibration.json"
    path.write_text(
        json.dumps(
            {
                "joint_names": ["joint1", "joint2"],
                "scale": [1.0, 400.0],
                "bias": [0.0, 0.0],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError) as exc:
        load_calibration_model(path)

    assert "scale exceeds safety bound" in str(exc.value)