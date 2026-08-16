"""Minimal reader for the duo motion registry (single source of truth).

The registry lives in agx_arm_description/config/duo_motion_registry.yaml. The
coordinator needs two things from it: the per-group joint names (canonical Nero
joints side-prefixed per the group's motion profile) and the declared CAN
topology, which decides whether same-side arm and hand are one schedulable
resource or two. The full registry surface lives in agx_arm_ctrl.motion_registry
and agx_arm_moveit._multi_arm_runtime.
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


def moveit_group(group_name: str) -> str:
    """Return the MoveIt planning-group name for an arm group (registry moveit_group)."""
    profile = load_motion_registry().get("motion_profiles", {}).get(group_name, {})
    return str(profile.get("moveit_group", group_name))


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


def bus_topology() -> str:
    """Return the declared CAN topology (C7).

    Defaults to the degraded reading when absent. Assuming parallel operation on
    an undeclared topology is the unsafe direction: it would schedule a hand
    action while the arm still holds the same bus.
    """
    return str(load_motion_registry().get("bus_topology", "shared_per_side")).strip()


def handshake_required() -> bool:
    """True when arm and hand share a bus, so hand motion needs the arm quiesced.

    Derived from the one declared topology rather than configured separately, so
    a stack cannot come up with the handoff enabled on hardware that does not
    need it — or, worse, disabled on hardware that does.
    """
    return bus_topology() != "dedicated_per_device"


def assert_matches_topology(parameter_name: str, configured: bool) -> bool:
    """Fail startup when a handshake parameter contradicts the declared topology.

    The topology is one declared fact (C7). These parameters survive only as
    compatibility inputs, so an override that disagrees with the registry is not
    a preference — it is two truths about one wiring loom, and the failure it
    produces shows up as motion that serializes for no reason or a hand
    commanding a bus the arm still holds. Neither is visible until it matters,
    so the contradiction is refused here instead.
    """
    required = handshake_required()
    if configured != required:
        raise ValueError(
            f"{parameter_name}={configured} contradicts bus_topology="
            f"'{bus_topology()}', which requires {parameter_name}={required}. "
            "The topology is the single source of truth: change bus_topology in "
            "duo_motion_registry.yaml rather than overriding this parameter."
        )
    return required


__all__ = [
    "load_motion_registry",
    "moveit_group",
    "group_joint_names",
    "bus_topology",
    "handshake_required",
    "assert_matches_topology",
]
