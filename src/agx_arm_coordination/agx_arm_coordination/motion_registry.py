"""Minimal reader for the duo motion registry (single source of truth).

The registry lives in agx_arm_description/config/duo_motion_registry.yaml. The
coordinator's arm executor only needs the per-group joint names from it (canonical
Nero joints side-prefixed per the group's motion profile), so this reader derives
exactly that and nothing else; the full registry surface lives in
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
def load_motion_registry() -> dict:
    return yaml.safe_load(_find_motion_registry().read_text(encoding="utf-8")) or {}


def group_joint_names(group_name: str) -> tuple[str, ...]:
    """Joint names for an arm group, side-prefixed per its motion profile.

    ``both_arms`` -> left_arm_jointN then right_arm_jointN; ``left_arm`` /
    ``right_arm`` -> that side only; an unknown / sideless profile -> the canonical
    unprefixed joints.
    """
    registry = load_motion_registry()
    arm = registry.get("arm", {})
    canonical = [str(joint) for joint in arm.get("canonical_joints", [])]
    sides_cfg = arm.get("sides", {})
    profile = registry.get("motion_profiles", {}).get(group_name, {})
    profile_sides = profile.get("sides", [])
    if not profile_sides:
        return tuple(canonical)
    names: list[str] = []
    for side in profile_sides:
        prefix = str(sides_cfg.get(side, {}).get("prefix", f"{side}_arm_"))
        names.extend(f"{prefix}{joint}" for joint in canonical)
    return tuple(names)


__all__ = ["load_motion_registry", "group_joint_names"]
