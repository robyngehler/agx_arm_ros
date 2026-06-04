from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from .model_metadata import default_nero_urdf_path


class GravityModelError(RuntimeError):
    pass


class GravityModel(Protocol):
    joint_names: list[str]
    urdf_path: str

    def compute_gravity(self, joint_positions: list[float]) -> list[float]:
        """Return actuator torque needed to compensate gravity at `joint_positions`."""
        ...

    def compute_flange_pose(self, joint_positions: list[float]) -> list[float]:
        ...


@dataclass
class PinocchioGravityModel:
    urdf_path: str
    joint_names: list[str]
    _pin: object
    _model: object
    _data: object

    @classmethod
    def from_urdf(cls, urdf_path: str | Path) -> "PinocchioGravityModel":
        try:
            import pinocchio as pin
        except Exception as exc:
            raise GravityModelError(
                "Pinocchio is not installed. Install python3-pinocchio or a pip-compatible package first."
            ) from exc

        path = str(Path(urdf_path).expanduser().resolve())
        model = pin.buildModelFromUrdf(path)
        data = model.createData()
        joint_names = [name for name in model.names if name not in ("universe",)]
        return cls(path, joint_names, pin, model, data)

    def compute_gravity(self, joint_positions: list[float]) -> list[float]:
        if len(joint_positions) != self.model_dofs:
            raise ValueError(f"expected {self.model_dofs} joint positions, got {len(joint_positions)}")
        q = self._pin.utils.zero(self._model.nq)
        for index, value in enumerate(joint_positions):
            q[index] = value
        tau = self._pin.computeGeneralizedGravity(self._model, self._data, q)
        # Pinocchio returns the gravity term from the dynamics equation. The MIT
        # controller and motor feedback use actuator torque sign, which is the
        # opposite direction for static gravity compensation.
        return [-float(tau[index]) for index in range(self.model_dofs)]

    def compute_flange_pose(self, joint_positions: list[float]) -> list[float]:
        if len(joint_positions) != self.model_dofs:
            raise ValueError(f"expected {self.model_dofs} joint positions, got {len(joint_positions)}")
        q = self._pin.utils.zero(self._model.nq)
        for index, value in enumerate(joint_positions):
            q[index] = value
        self._pin.forwardKinematics(self._model, self._data, q)
        self._pin.updateFramePlacements(self._model, self._data)

        frame_candidates = ["nero_tool0", "link7", "gripper_flange", "tool0", "flange"]
        for frame_name in self._preferred_frame_names(frame_candidates):
            frame_id = self._model.getFrameId(frame_name)
            placement = self._data.oMf[frame_id]
            roll, pitch, yaw = self._pin.rpy.matrixToRpy(placement.rotation)
            return [
                float(placement.translation[0]),
                float(placement.translation[1]),
                float(placement.translation[2]),
                float(roll),
                float(pitch),
                float(yaw),
            ]
        raise GravityModelError("No suitable flange frame found in URDF model")

    def _preferred_frame_names(self, frame_candidates: list[str]) -> list[str]:
        preferred_names: list[str] = []
        for frame_name in frame_candidates:
            if self._model.existFrame(frame_name):
                preferred_names.append(frame_name)

        frames = getattr(self._model, "frames", [])
        for frame in frames:
            resolved_name = getattr(frame, "name", "")
            if not resolved_name:
                continue
            if any(
                resolved_name.endswith(candidate)
                for candidate in frame_candidates
            ) and resolved_name not in preferred_names:
                preferred_names.append(resolved_name)
        return preferred_names

    @property
    def model_dofs(self) -> int:
        return int(self._model.nq)


def create_gravity_model(
    backend: str = "pinocchio",
    urdf_path: str | Path | None = None,
) -> GravityModel:
    resolved_path = default_nero_urdf_path() if urdf_path is None else Path(urdf_path)
    if backend == "pinocchio":
        return PinocchioGravityModel.from_urdf(resolved_path)
    raise GravityModelError(f"Unsupported gravity backend: {backend}")