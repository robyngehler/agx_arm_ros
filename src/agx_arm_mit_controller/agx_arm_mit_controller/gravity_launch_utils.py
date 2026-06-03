from __future__ import annotations

import shlex
import shutil
import subprocess
import tempfile
from pathlib import Path


def parse_xacro_mappings(raw_args: str) -> dict[str, str]:
    mappings: dict[str, str] = {}
    for token in shlex.split(raw_args):
        if ":=" not in token:
            raise ValueError(
                f"custom_model_xacro_args token '{token}' is invalid; expected key:=value"
            )
        key, value = token.split(":=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"custom_model_xacro_args token '{token}' has an empty key")
        mappings[key] = value
    return mappings


def _duo_side_for_prefix(input_joint_prefix: str) -> str:
    normalized_prefix = input_joint_prefix.strip()
    if normalized_prefix.startswith("left_arm_"):
        return "left"
    if normalized_prefix.startswith("right_arm_"):
        return "right"
    return ""


def _apply_duo_slice_mappings(
    model_path: Path,
    mappings: dict[str, str],
    input_joint_prefix: str,
    effector_type: str,
) -> dict[str, str]:
    if model_path.name != "duo_system.urdf.xacro":
        return mappings

    side = _duo_side_for_prefix(input_joint_prefix)
    if not side:
        return mappings

    # Keep the gravity model arm-only for the current MIT controller contract.
    # The controller owns exactly joint1..joint7 and does not execute separate
    # post-Pinocchio hand compensation today.
    side_hand_enabled = "false"
    resolved = dict(mappings)
    if side == "left":
        resolved.update(
            {
                "use_left_arm": "true",
                "use_left_hand": side_hand_enabled,
                "use_right_arm": "false",
                "use_right_hand": "false",
            }
        )
    else:
        resolved.update(
            {
                "use_left_arm": "false",
                "use_left_hand": "false",
                "use_right_arm": "true",
                "use_right_hand": side_hand_enabled,
            }
        )
    return resolved


def resolve_gravity_urdf_path(
    *,
    custom_model: str,
    custom_model_xacro_args: str = "",
    input_joint_prefix: str = "",
    effector_type: str = "none",
    explicit_gravity_urdf_path: str = "",
) -> str:
    if explicit_gravity_urdf_path.strip():
        return str(Path(explicit_gravity_urdf_path).expanduser().resolve())

    if not custom_model.strip():
        return ""

    model_path = Path(custom_model).expanduser().resolve()
    if model_path.suffix != ".xacro":
        return str(model_path)

    mappings = parse_xacro_mappings(custom_model_xacro_args)
    mappings = _apply_duo_slice_mappings(
        model_path,
        mappings,
        input_joint_prefix,
        effector_type,
    )
    temp_urdf = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".urdf",
        prefix=f"{model_path.stem}_gravity_",
        delete=False,
    )

    try:
        import xacro  # type: ignore

        generated_urdf = xacro.process_file(str(model_path), mappings=mappings)
        temp_urdf.write(generated_urdf.toprettyxml(indent="  "))
    except ModuleNotFoundError:
        xacro_command = shutil.which("xacro")
        if xacro_command is None:
            raise RuntimeError(
                "xacro is required to derive a gravity URDF from custom_model, but neither the Python module nor the CLI is available"
            )

        command = [xacro_command, str(model_path)]
        for key, value in mappings.items():
            command.append(f"{key}:={value}")

        try:
            result = subprocess.run(command, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as exc:
            stderr = exc.stderr.strip() if exc.stderr else ""
            stdout = exc.stdout.strip() if exc.stdout else ""
            details = stderr or stdout or str(exc)
            raise RuntimeError(
                f"xacro failed while deriving gravity URDF from {model_path}: {details}"
            ) from exc
        temp_urdf.write(result.stdout)

    temp_urdf.close()
    return temp_urdf.name