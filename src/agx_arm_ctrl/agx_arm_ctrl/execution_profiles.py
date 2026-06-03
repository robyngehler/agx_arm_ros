from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


PROFILE_CONFIG_RELATIVE_PATH = Path("config") / "execution_profiles.yaml"
DUO_MODEL_RELATIVE_PATH = Path("urdf") / "duo_system.urdf.xacro"

SCALAR_PROFILE_KEYS = (
    "robot_name",
    "moveit_profile",
    "custom_model",
    "effector_type",
    "revo2_type",
    "omnihand_type",
    "launch_omnihand_bridge",
    "input_joint_prefix",
    "feedback_joint_prefix",
    "arm_base_frame",
    "arm_tip_frame",
    "tcp_parent_frame",
)


def _workspace_source_path(package_name: str, relative_path: Path) -> Path | None:
    package_share_dir = Path(get_package_share_directory(package_name)).resolve()

    try:
        workspace_root = package_share_dir.parents[3]
    except IndexError:
        return None

    source_path = workspace_root / "src" / package_name / relative_path
    if source_path.is_file():
        return source_path
    return None


def _source_or_installed_path(package_name: str, relative_path: Path) -> Path:
    source_path = _workspace_source_path(package_name, relative_path)
    if source_path is not None:
        return source_path

    return Path(get_package_share_directory(package_name)).resolve() / relative_path


def default_profile_config_path() -> Path:
    return _source_or_installed_path("agx_arm_ctrl", PROFILE_CONFIG_RELATIVE_PATH)


def default_duo_model_path() -> Path:
    return _source_or_installed_path("duo_body_description", DUO_MODEL_RELATIVE_PATH)


def _normalize_bool_text(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value).strip().lower()


def _format_xacro_args(mappings: dict[str, Any]) -> str:
    parts: list[str] = []
    for key, value in mappings.items():
        text = _normalize_bool_text(value) if isinstance(value, bool) else str(value).strip()
        parts.append(f"{key}:={text}")
    return " ".join(parts)


def _format_arm_instances(arm_instances: list[dict[str, Any]]) -> str:
    return yaml.safe_dump(
        arm_instances,
        default_flow_style=True,
        sort_keys=False,
    ).strip()


def load_execution_profiles(config_path: str | Path | None = None) -> dict[str, dict[str, Any]]:
    path = Path(config_path) if config_path is not None else default_profile_config_path()
    content = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    profiles = content.get("profiles", {})
    if not isinstance(profiles, dict):
        raise ValueError(f"Execution profile config at '{path}' must define a 'profiles' mapping")
    return profiles


def available_execution_profiles(config_path: str | Path | None = None) -> list[str]:
    return list(load_execution_profiles(config_path).keys())


def resolve_execution_profile(
    execution_profile: str,
    *,
    config_path: str | Path | None = None,
    duo_model_path: str | Path | None = None,
    allow_multi_arm: bool = True,
) -> dict[str, str]:
    profile_name = execution_profile.strip()
    if not profile_name or profile_name == "manual":
        return {}

    profiles = load_execution_profiles(config_path)
    raw_profile = profiles.get(profile_name)
    if raw_profile is None:
        choices = ", ".join(sorted(profiles))
        raise ValueError(
            f"Unsupported execution_profile '{profile_name}'. Available profiles: {choices}"
        )

    if raw_profile.get("moveit_profile") == "both_arms" and not allow_multi_arm:
        raise ValueError(
            f"execution_profile '{profile_name}' is multi-arm and cannot be used with this launch surface"
        )

    resolved: dict[str, str] = {}

    for key in SCALAR_PROFILE_KEYS:
        value = raw_profile.get(key)
        if value is None:
            continue
        if isinstance(value, bool):
            resolved[key] = _normalize_bool_text(value)
            continue
        text = str(value).strip()
        if text:
            resolved[key] = text

    if raw_profile.get("use_duo_model"):
        resolved["custom_model"] = str(Path(duo_model_path) if duo_model_path is not None else default_duo_model_path())

    xacro_args = raw_profile.get("custom_model_xacro_args")
    if isinstance(xacro_args, dict) and xacro_args:
        resolved["custom_model_xacro_args"] = _format_xacro_args(xacro_args)

    arm_instances = raw_profile.get("arm_instances")
    if isinstance(arm_instances, list) and arm_instances:
        resolved["arm_instances"] = _format_arm_instances(arm_instances)

    return resolved