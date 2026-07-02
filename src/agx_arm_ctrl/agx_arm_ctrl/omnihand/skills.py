"""ROS-free logic for the OmniHand skill controller.

This module keeps the parts of the skill layer that do not need rclpy, so they
can be unit-tested without a running ROS graph: the semantic skill catalogue
(``skill_name`` -> backend motion + target preset), the tactile contact scoring
(raw vendor tactile vector -> per-object ``contact_score``), and the small
helpers the node's state machine leans on.

Design contract (``hand_skill_backend_mapping.md``):

- the public layer only ever sees the semantic ``skill_name``; the
  ``skill_name -> {motion, target preset}`` mapping lives here / in
  ``config/omnihand_skills.yaml`` and can be recalibrated without touching any
  activity graph.
- behaviour is data, not vendor commands: ``completion_policy`` /
  ``fallback_policy`` are interpreted by the node, never carried to the SDK.
- the only calibrated O12 presets today are ``zero`` and ``fist_vendor_demo``
  (see ``config/omnihand_pro_gestures.yaml``). The MVP therefore opens to
  ``zero`` and closes toward ``fist_vendor_demo`` until tactile contact; named
  calibrated ``open`` / glass / bottle grasp poses replace these once measured
  on hardware, with no change to the skill names.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


# --- Skill catalogue ---------------------------------------------------------

# Motion classes the controller knows how to run. Public skills map onto one of
# these; the target preset (a named pose in the model's gesture config) and the
# tactile parameters come from the action metadata / skill config.
MOTION_OPEN = "open"
MOTION_CLOSE_UNTIL_CONTACT = "close_until_contact"
MOTION_FREEZE = "freeze"
VALID_MOTIONS = (MOTION_OPEN, MOTION_CLOSE_UNTIL_CONTACT, MOTION_FREEZE)

# Controller state-machine states (mirrors hand_skill_backend_mapping.md §4).
STATE_IDLE = "IDLE"
STATE_OPENING = "OPENING"
STATE_CLOSING_UNTIL_CONTACT = "CLOSING_UNTIL_CONTACT"
STATE_GRASP_HOLDING = "GRASP_HOLDING"
STATE_RELEASING = "RELEASING"
STATE_FAILED = "FAILED"

# Aggregation strategies for combining per-sensor contact into one score.
AGG_MEAN = "mean"
AGG_MAX = "max"
AGG_MIN = "min"
VALID_AGGREGATIONS = (AGG_MEAN, AGG_MAX, AGG_MIN)

# Aliases so action metadata can name a finger ("ring") even though the bridge
# tactile layout uses tip names ("little_tip") for the same physical finger.
_SENSOR_ALIASES = {
    "pinky": "little",
    "little": "little",
    "ring": "ring",
    "thumb": "thumb",
    "index": "index",
    "middle": "middle",
}


@dataclass(frozen=True)
class SkillDefinition:
    """One semantic skill resolved to a backend motion + target preset."""

    skill_name: str
    motion: str
    target_preset: str | None  # None only for freeze

    def __post_init__(self) -> None:
        if self.motion not in VALID_MOTIONS:
            raise ValueError(
                f"skill '{self.skill_name}': unknown motion '{self.motion}'; "
                f"expected one of {VALID_MOTIONS}"
            )
        if self.motion != MOTION_FREEZE and not self.target_preset:
            raise ValueError(
                f"skill '{self.skill_name}': motion '{self.motion}' needs a target_preset"
            )


@dataclass(frozen=True)
class SkillControllerDefaults:
    """Tunable defaults for the closing/opening loops and slip monitoring.

    All of these are placeholders to calibrate on hardware (the close/open step
    sizes, the slip factors, the tactile block layout). The contact threshold,
    stable-sample count, and timeout are per-object and come from the action
    metadata instead.
    """

    control_rate_hz: float = 20.0
    close_step_rad: float = 0.05
    open_step_rad: float = 0.08
    open_tolerance_rad: float = 0.06
    open_settle_timeout_sec: float = 3.0
    contact_aggregation: str = AGG_MEAN
    normal_force_offset: int = 1  # per-finger tactile block offset of normal force
    tactile_stale_sec: float = 1.0
    slip_warn_factor: float = 0.5
    slip_critical_factor: float = 0.2

    def __post_init__(self) -> None:
        if self.contact_aggregation not in VALID_AGGREGATIONS:
            raise ValueError(
                f"contact_aggregation '{self.contact_aggregation}' invalid; "
                f"expected one of {VALID_AGGREGATIONS}"
            )


@dataclass(frozen=True)
class SkillCatalogue:
    skills: dict[str, SkillDefinition]
    defaults: SkillControllerDefaults

    def resolve(self, skill_name: str) -> SkillDefinition:
        try:
            return self.skills[skill_name]
        except KeyError:
            raise ValueError(
                f"unknown skill_name '{skill_name}'; configured skills: "
                f"{sorted(self.skills)}"
            ) from None


def parse_skill_catalogue(data: dict[str, Any] | None) -> SkillCatalogue:
    """Build a SkillCatalogue from already-parsed YAML.

    Kept separate from file IO so it can be unit-tested with plain dicts. The
    skill_name -> motion/preset mapping has a single source of truth, the
    installed config/omnihand_skills.yaml (resolved by load_skill_catalogue);
    there is no hardcoded duplicate here. Empty data yields an empty catalogue.
    """
    data = data or {}
    raw_skills = data.get("omnihand_skills") or {}
    skills: dict[str, SkillDefinition] = {}
    for name, spec in raw_skills.items():
        spec = spec or {}
        skills[str(name)] = SkillDefinition(
            skill_name=str(name),
            motion=str(spec.get("motion", "")),
            target_preset=(
                str(spec["target_preset"])
                if spec.get("target_preset") is not None
                else None
            ),
        )

    raw_defaults = data.get("defaults") or {}
    known = SkillControllerDefaults().__dict__
    filtered = {key: raw_defaults[key] for key in known if key in raw_defaults}
    defaults = SkillControllerDefaults(**filtered)
    return SkillCatalogue(skills=skills, defaults=defaults)


def load_skill_catalogue(config_path: str | Path | None) -> SkillCatalogue:
    """Load the skill catalogue from a YAML file.

    config/omnihand_skills.yaml is the single source of truth for the
    skill_name -> backend motion/preset mapping (resolved to the package share by
    the skill controller). A missing/unreadable file yields an empty catalogue, so
    a broken install fails loudly on the first skill rather than silently using a
    hardcoded duplicate mapping.
    """
    if config_path is None:
        return parse_skill_catalogue(None)
    try:
        data = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    except OSError:
        return parse_skill_catalogue(None)
    return parse_skill_catalogue(data)


# --- Tactile contact scoring -------------------------------------------------

@dataclass(frozen=True)
class TactileReading:
    """A parsed tactile snapshot from feedback/omnihand/tactile_raw."""

    finger_names: list[str]            # ordered, e.g. ["thumb_tip", "index_tip", ...]
    per_finger_normal: dict[str, float]  # finger_name -> normal-force magnitude


def _normalize_sensor_name(name: str) -> str:
    base = str(name).strip().lower()
    base = base.split("_")[0] if base else base
    return _SENSOR_ALIASES.get(base, base)


def parse_tactile(
    layout_name: str,
    values: list[float],
    normal_force_offset: int = 1,
) -> TactileReading:
    """Split a flat tactile vector into a per-finger normal-force reading.

    The bridge publishes ``layout_name`` as a comma-joined finger list
    (e.g. ``thumb_tip,index_tip,middle_tip,ring_tip,little_tip``) and ``values``
    as that many equal-size blocks concatenated. On the live Pro each block is
    ``[online_state, normal_force, tangent_force, ...]`` (see sprint6
    errors_and_fixes 2026-06-25), so the contact magnitude is the
    ``normal_force_offset`` element of each block. The mock backend uses
    ``flat_array``; with no finger names we cannot attribute contact, so we
    return an empty mapping (callers treat that as "no usable tactile").
    """
    finger_names = [n.strip() for n in str(layout_name).split(",") if n.strip()]
    per_finger: dict[str, float] = {}
    if not finger_names or finger_names == ["flat_array"]:
        return TactileReading(finger_names=finger_names, per_finger_normal=per_finger)

    block_size = len(values) // len(finger_names) if finger_names else 0
    if block_size <= 0:
        return TactileReading(finger_names=finger_names, per_finger_normal=per_finger)

    offset = normal_force_offset if normal_force_offset < block_size else 0
    for index, finger in enumerate(finger_names):
        start = index * block_size
        per_finger[finger] = abs(float(values[start + offset]))
    return TactileReading(finger_names=finger_names, per_finger_normal=per_finger)


def matched_finger_values(
    reading: TactileReading, contact_sensors: list[str]
) -> list[float]:
    """Return the normal-force values for the requested contact sensors.

    ``contact_sensors`` are semantic finger names from the action metadata
    (``thumb``, ``index``, ``ring``, ...). They are matched against the tactile
    layout finger names by base name (``ring`` -> ``ring_tip``, ``pinky`` ->
    ``little_tip``). Sensors with no matching tactile channel are skipped; the
    caller decides whether the remaining set is enough.
    """
    if not contact_sensors:
        wanted = None
    else:
        wanted = {_normalize_sensor_name(s) for s in contact_sensors}

    out: list[float] = []
    for finger, value in reading.per_finger_normal.items():
        if wanted is None or _normalize_sensor_name(finger) in wanted:
            out.append(value)
    return out


def aggregate_contact(values: list[float], aggregation: str) -> float:
    """Combine per-sensor contact magnitudes into a single contact_score."""
    if not values:
        return 0.0
    if aggregation == AGG_MAX:
        return max(values)
    if aggregation == AGG_MIN:
        return min(values)
    return sum(values) / len(values)


def contact_score(
    reading: TactileReading,
    contact_sensors: list[str],
    aggregation: str,
) -> float:
    """Aggregated contact score over the requested sensors, 0.0 if none match."""
    return aggregate_contact(matched_finger_values(reading, contact_sensors), aggregation)


# --- Motion helpers ----------------------------------------------------------

def step_toward(
    current: list[float], target: list[float], max_step: float
) -> list[float]:
    """One bounded step of every joint from ``current`` toward ``target``.

    The per-joint move is clamped to ``max_step`` so the closing/opening motion
    is gradual (a safety property for a contact-seeking grasp). Returns a new
    list; inputs are not mutated.
    """
    if len(current) != len(target):
        raise ValueError(
            f"current/target length mismatch: {len(current)} vs {len(target)}"
        )
    stepped: list[float] = []
    for now, goal in zip(current, target, strict=True):
        delta = goal - now
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        stepped.append(now + delta)
    return stepped


def within_tolerance(
    current: list[float], target: list[float], tolerance: float
) -> bool:
    """True when every joint is within ``tolerance`` of the target."""
    if len(current) != len(target):
        return False
    return all(abs(now - goal) <= tolerance for now, goal in zip(current, target))
