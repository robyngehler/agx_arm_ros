"""Shared reader for the duo motion registry (the single source of truth).

The registry lives in agx_arm_description/config/duo_motion_registry.yaml and is
the one place that declares the Duo motion stack: arm canonical joints + side
prefixes/namespaces/CAN ports/frames, the MoveIt motion-profile geometry
(group/prefix/frames + runtime restrictions), and the OmniHand active-joint
sets/limits/mirror/tactile. Every agx_arm_ctrl consumer (models.py,
duo_runtime_contract.py, execution_profiles.py) reads it through this module so
the loader logic and path resolution live in exactly one place per package.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
import yaml

_REGISTRY_RELATIVE_PATH = Path("config") / "duo_motion_registry.yaml"
_SOURCE_RELATIVE_PATH = (
    Path("src") / "agx_arm_sim" / "agx_arm_description" / _REGISTRY_RELATIVE_PATH
)


def find_motion_registry() -> Path:
    """Locate duo_motion_registry.yaml in agx_arm_description (share, then source)."""
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
def load_motion_registry() -> dict[str, Any]:
    """Load and cache the motion registry as a plain dict."""
    return yaml.safe_load(find_motion_registry().read_text(encoding="utf-8")) or {}


def motion_profile(name: str) -> dict[str, Any]:
    """Return one motion_profiles entry (raises KeyError if unknown)."""
    profiles = load_motion_registry().get("motion_profiles", {})
    if name not in profiles:
        raise KeyError(f"unknown motion profile '{name}'; known: {sorted(profiles)}")
    return profiles[name]


def arm_sides() -> dict[str, Any]:
    """Return the arm.sides mapping (left/right -> prefix/namespace/can_port/frames)."""
    return load_motion_registry().get("arm", {}).get("sides", {})


def omnihand_model(name: str) -> dict[str, Any]:
    """Return one omnihand model entry (active_joints, limits, mirror, tactile)."""
    models = load_motion_registry().get("omnihand", {})
    if name not in models:
        raise KeyError(f"unknown omnihand model '{name}'; known: {sorted(models)}")
    return models[name]


__all__ = [
    "find_motion_registry",
    "load_motion_registry",
    "motion_profile",
    "arm_sides",
    "omnihand_model",
]
