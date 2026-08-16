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


def hand_sides() -> dict[str, Any]:
    """Return the omnihand.sides mapping (left/right -> namespace/can_port/ids).

    A hand's interface is its own declared fact. It used to be read off
    ``arm.sides.<side>.can_port``, which pointed both bridges at the arm buses:
    they timed out continuously while transmitting nothing, and the failure was
    invisible until a full-stack bring-up was measured.
    """
    return load_motion_registry().get("omnihand", {}).get("sides", {})


def bus_topology() -> str:
    """Return the declared CAN topology (C7).

    One fact, from which both the hands' interfaces and whether the arm<->hand
    window handshake runs are derived. Defaults to the degraded reading when
    absent, because assuming parallel operation on an undeclared topology is the
    unsafe direction: it would command a hand while the arm holds the same bus.
    """
    return str(load_motion_registry().get("bus_topology", "shared_per_side")).strip()


def handshake_required() -> bool:
    """True when arm and hand share a bus and hand motion needs the window.

    Derived from the one declared topology rather than configured separately.
    The two used to be independent: a launch defaulting to the shared-bus
    handshake while the hardware had four buses meant the arm was quiesced for
    every hand motion that did not need it — and the reverse combination would
    command a hand while the arm holds the same bus.
    """
    return bus_topology() != "dedicated_per_device"


def omnihand_model(name: str) -> dict[str, Any]:
    """Return one omnihand model entry (active_joints, limits, mirror, tactile)."""
    section = load_motion_registry().get("omnihand", {})
    # `sides` is runtime binding, not a hand model; listing it as a known model
    # would send a caller looking for a joint set that does not exist.
    models = {k: v for k, v in section.items() if k != "sides"}
    if name not in models:
        raise KeyError(f"unknown omnihand model '{name}'; known: {sorted(models)}")
    return models[name]


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
    "find_motion_registry",
    "load_motion_registry",
    "motion_profile",
    "arm_sides",
    "omnihand_model",
    "hand_sides",
    "bus_topology",
    "handshake_required",
    "assert_matches_topology",
]
