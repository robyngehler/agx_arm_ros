from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from .model_metadata import default_nero_urdf_path


class GravityModelError(RuntimeError):
    pass


def _resolve_mounting_rpy(mounting_rpy: "list[float] | tuple[float, float, float] | None") -> tuple[float, float, float]:
    if mounting_rpy is None:
        return (0.0, 0.0, 0.0)
    values = list(mounting_rpy)
    if len(values) != 3:
        raise GravityModelError(f"mounting_rpy must have 3 elements, got {len(values)}")
    return (float(values[0]), float(values[1]), float(values[2]))


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
    mounting_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def from_urdf(
        cls,
        urdf_path: str | Path,
        mounting_rpy: "list[float] | tuple[float, float, float] | None" = None,
    ) -> "PinocchioGravityModel":
        try:
            import pinocchio as pin
        except Exception as exc:
            raise GravityModelError(
                "Pinocchio is not installed. Install python3-pinocchio or a pip-compatible package first."
            ) from exc

        path = str(Path(urdf_path).expanduser().resolve())
        model = pin.buildModelFromUrdf(path)

        rpy = _resolve_mounting_rpy(mounting_rpy)
        if any(abs(angle) > 1e-9 for angle in rpy):
            # The URDF is authored with an upright base, so pinocchio's default
            # gravity ([0, 0, -g] in the base frame) only holds for a table
            # mount. On the Duo body the arm base is tilted, so express world
            # gravity in the (rotated) base frame: g_base = R_base_in_world^T @ g.
            # rpy is the base-frame orientation in world (XYZ extrinsic), matching
            # the convention used elsewhere in this stack.
            import numpy as np

            r_base_in_world = pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
            g_world = np.array(model.gravity.linear, dtype=float)
            model.gravity.linear = r_base_in_world.T @ g_world

        data = model.createData()
        joint_names = [name for name in model.names if name not in ("universe",)]
        return cls(path, joint_names, pin, model, data, rpy)

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
    mounting_rpy: "list[float] | tuple[float, float, float] | None" = None,
) -> GravityModel:
    resolved_path = default_nero_urdf_path() if urdf_path is None else Path(urdf_path)
    if backend == "pinocchio":
        return PinocchioGravityModel.from_urdf(resolved_path, mounting_rpy)
    raise GravityModelError(f"Unsupported gravity backend: {backend}")