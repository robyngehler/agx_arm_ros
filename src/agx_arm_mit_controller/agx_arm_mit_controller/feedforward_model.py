from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CalibrationModel:
    joint_names: list[str]
    scale: list[float]
    bias: list[float]
    source_log: str = ""
    note: str = ""

    def apply(self, torques: list[float]) -> list[float]:
        if len(torques) != len(self.joint_names):
            raise ValueError(
                f"torque vector length mismatch: expected {len(self.joint_names)}, got {len(torques)}"
            )
        return [
            self.scale[index] * torques[index] + self.bias[index]
            for index in range(len(self.joint_names))
        ]

    def to_dict(self) -> dict[str, Any]:
        return {
            "joint_names": list(self.joint_names),
            "scale": list(self.scale),
            "bias": list(self.bias),
            "source_log": self.source_log,
            "note": self.note,
        }


def _validate_numeric_vector(values: list[float], name: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"calibration model contains a non-finite {name} value")


def validate_calibration_model(
    model: CalibrationModel,
    *,
    max_abs_scale: float = 10.0,
    max_abs_bias: float = 16.0,
) -> CalibrationModel:
    _validate_numeric_vector(model.scale, "scale")
    _validate_numeric_vector(model.bias, "bias")

    if any(abs(value) > max_abs_scale for value in model.scale):
        raise ValueError(
            f"calibration model scale exceeds safety bound of {max_abs_scale}; regenerate from a better log"
        )
    if any(abs(value) > max_abs_bias for value in model.bias):
        raise ValueError(
            f"calibration model bias exceeds safety bound of {max_abs_bias} N*m; regenerate from a better log"
        )
    return model


def load_calibration_model(file_path: str | Path, expected_joint_names: list[str] | None = None) -> CalibrationModel:
    path = Path(file_path).expanduser().resolve()
    payload = json.loads(path.read_text(encoding="utf-8"))
    model = CalibrationModel(
        joint_names=list(payload["joint_names"]),
        scale=[float(value) for value in payload["scale"]],
        bias=[float(value) for value in payload["bias"]],
        source_log=str(payload.get("source_log", "")),
        note=str(payload.get("note", "")),
    )
    if len(model.scale) != len(model.joint_names) or len(model.bias) != len(model.joint_names):
        raise ValueError("calibration model scale/bias length mismatch")
    if expected_joint_names is not None and list(expected_joint_names) != model.joint_names:
        raise ValueError(
            f"calibration model joints mismatch: expected {expected_joint_names}, got {model.joint_names}"
        )
    return validate_calibration_model(model)


def save_calibration_model(model: CalibrationModel, file_path: str | Path) -> Path:
    path = Path(file_path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(model.to_dict(), indent=2, sort_keys=True), encoding="utf-8")
    return path