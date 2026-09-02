"""Minimal reader for the duo motion registry (single source of truth).

The registry lives in agx_arm_description/config/duo_motion_registry.yaml. The
mit_tools bridges/adapters only need the canonical arm joint order from it, so
this reader is intentionally tiny; the full registry surface is used by
agx_arm_ctrl.motion_registry and agx_arm_moveit._multi_arm_runtime.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

_REGISTRY_RELATIVE_PATH = Path("config") / "duo_motion_registry.yaml"
_SOURCE_RELATIVE_PATH = (
    Path("src") / "agx_arm_sim" / "agx_arm_description" / _REGISTRY_RELATIVE_PATH
)


def _find_motion_registry() -> Path:
    try:
        share_dir = Path(get_package_share_directory("agx_arm_description"))
        candidate = share_dir / _REGISTRY_RELATIVE_PATH
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    for parent in Path(__file__).resolve().parents:
        candidate = parent / _SOURCE_RELATIVE_PATH
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "duo_motion_registry.yaml not found in agx_arm_description (share or source tree)"
    )


@lru_cache(maxsize=1)
def canonical_arm_joints() -> list[str]:
    """Canonical (unprefixed) Nero arm joint order from the registry."""
    registry = yaml.safe_load(_find_motion_registry().read_text(encoding="utf-8")) or {}
    return [str(name) for name in registry.get("arm", {}).get("canonical_joints", [])]


def canonical_gripper_joints() -> list[str]:
    """Canonical (unprefixed) AGX gripper finger joints from the registry.

    They live in the arm's name space, so whatever prefixes the arm joints must
    prefix these too.
    """
    registry = yaml.safe_load(_find_motion_registry().read_text(encoding="utf-8")) or {}
    return [str(name) for name in registry.get("gripper", {}).get("canonical_joints", [])]


__all__ = ["canonical_arm_joints", "canonical_gripper_joints"]
