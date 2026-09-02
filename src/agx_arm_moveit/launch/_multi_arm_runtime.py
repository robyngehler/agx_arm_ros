from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml
from launch.substitutions import LaunchConfiguration


DEFAULT_MOVEIT_GROUP = "nero_arm"
DUAL_ARM_MOVEIT_GROUP = "both_arms"
DEFAULT_PREFIXED_FEEDBACK_TOPIC = "feedback/prefixed_joint_states"

# Arm + OmniHand joint sets come from the single source of truth,
# agx_arm_description/config/duo_motion_registry.yaml — no second hand-synced copy
# here (this module previously duplicated them with a "must match" comment).
_REGISTRY_RELATIVE_PATH = Path("config") / "duo_motion_registry.yaml"


def _find_motion_registry() -> Path:
    try:
        share_dir = Path(get_package_share_directory("agx_arm_description"))
        candidate = share_dir / _REGISTRY_RELATIVE_PATH
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    source_rel = Path("src") / "agx_arm_sim" / "agx_arm_description" / _REGISTRY_RELATIVE_PATH
    for parent in Path(__file__).resolve().parents:
        candidate = parent / source_rel
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "duo_motion_registry.yaml not found in agx_arm_description (share or source tree)"
    )


_MOTION_REGISTRY = yaml.safe_load(_find_motion_registry().read_text(encoding="utf-8")) or {}

CANONICAL_ARM_JOINTS = list(_MOTION_REGISTRY["arm"]["canonical_joints"])
# AGX gripper fingers, prefixed with the arm they are mounted on.
CANONICAL_GRIPPER_JOINTS = list(
    _MOTION_REGISTRY.get("gripper", {}).get("canonical_joints", [])
)
# OmniHand Pro O12 active (controllable) joints, side-prefixed at use.
OMNIHAND_O12_ACTIVE_JOINT_SUFFIXES = [
    str(joint["suffix"]) for joint in _MOTION_REGISTRY["omnihand"]["o12_pro"]["active_joints"]
]
# Profile geometry + arm side conventions are registry-derived (no second copy).
# MOTION_PROFILES is public so _moveit_config_builder builds its profile defaults
# (group/prefix/base/tip frames) from the same registry entries.
_MOTION_PROFILES = _MOTION_REGISTRY.get("motion_profiles", {})
MOTION_PROFILES = _MOTION_PROFILES
_ARM = _MOTION_REGISTRY.get("arm", {})
ARM_SIDES = _ARM.get("sides", {})
ARM_CONTROLLER_NAME = str(_ARM.get("controller_name", "arm_controller"))

# profile -> input joint prefix (was a hardcoded dict; now from the registry).
PROFILE_PREFIX_DEFAULTS = {
    name: str(profile.get("input_joint_prefix", ""))
    for name, profile in _MOTION_PROFILES.items()
}


def _trim_string(value: object) -> str:
    return str(value).strip() if value is not None else ""


def _mapping_value(raw_mapping: Mapping[str, object], key: str) -> object:
    if key in raw_mapping:
        return raw_mapping[key]
    return ""


def normalize_relative_namespace(value: object) -> str:
    return _trim_string(value).strip("/")


def join_relative_namespaces(*parts: object) -> str:
    normalized_parts = [normalize_relative_namespace(part) for part in parts]
    return "/".join(part for part in normalized_parts if part)


def absolute_topic_for_namespace(namespace: str, relative_topic: str) -> str:
    normalized_namespace = normalize_relative_namespace(namespace)
    normalized_topic = str(relative_topic).strip().lstrip("/")
    if normalized_namespace:
        return f"/{normalized_namespace}/{normalized_topic}"
    return f"/{normalized_topic}"


def controller_joint_names(joint_prefix: str) -> list[str]:
    if not joint_prefix:
        return list(CANONICAL_ARM_JOINTS)
    return [f"{joint_prefix}{joint_name}" for joint_name in CANONICAL_ARM_JOINTS]


def controller_path(instance: Mapping[str, object]) -> str:
    namespace = normalize_relative_namespace(instance.get("namespace", ""))
    controller_name = _trim_string(instance.get("controller_name") or "arm_controller") or "arm_controller"
    return join_relative_namespaces(namespace, controller_name)


def omnihand_controller_joint_names(side: str) -> list[str]:
    """Side-prefixed O12 active hand joints (e.g. right_thumb_roll_joint)."""
    return [f"{side}_{suffix}" for suffix in OMNIHAND_O12_ACTIVE_JOINT_SUFFIXES]


def omnihand_controller_path(instance: Mapping[str, object]) -> str:
    """`<ns>/<side>_omnihand_controller` for an instance carrying an OmniHand.

    Empty when the instance has no ``omnihand_type``. Namespaced like the arm
    controller so MoveIt's controller list matches the hand FJT action server.
    """
    side = _trim_string(instance.get("omnihand_type"))
    if not side:
        return ""
    namespace = normalize_relative_namespace(instance.get("namespace", ""))
    return join_relative_namespaces(namespace, f"{side}_omnihand_controller")


def gripper_controller_path(instance: Mapping[str, object]) -> str:
    """`<ns>/gripper_controller` for an instance carrying an AGX gripper.

    Empty unless the instance declares ``effector_type: agx_gripper``. Unlike
    the hand's, the name carries no side: the gripper takes the side of the arm
    whose namespace it already sits in.
    """
    if _trim_string(instance.get("effector_type")) != "agx_gripper":
        return ""
    namespace = normalize_relative_namespace(instance.get("namespace", ""))
    return join_relative_namespaces(namespace, "gripper_controller")


def gripper_controller_joint_names(joint_prefix: str) -> list[str]:
    return [f"{joint_prefix}{joint_name}" for joint_name in CANONICAL_GRIPPER_JOINTS]


def parse_arm_instances(raw_value: object) -> list[Mapping[str, object]]:
    text = _trim_string(raw_value)
    if not text:
        return []

    parsed = yaml.safe_load(text)
    if parsed is None:
        return []
    if not isinstance(parsed, list):
        raise ValueError("arm_instances must evaluate to a list of mappings")
    return parsed


def _planning_instance_for_side(side: str) -> dict[str, str]:
    """Planning-only arm instance for a registry side (no driver/CAN bring-up)."""
    side_cfg = ARM_SIDES.get(side, {})
    prefix = str(side_cfg.get("prefix", f"{side}_arm_"))
    namespace = str(side_cfg.get("namespace", side))
    return {
        "name": namespace,
        "namespace": namespace,
        "joint_prefix": prefix,
        "feedback_joint_prefix": prefix,
        "controller_name": ARM_CONTROLLER_NAME,
        "can_port": "",
        "arm_type": "",
        "effector_type": "",
        "omnihand_type": "",
        "revo2_type": "",
        "tcp_offset": "",
        "launch_driver": "",
    }


def _default_arm_instances(
    moveit_profile: str,
    explicit_joint_prefix: str = "",
    explicit_feedback_joint_prefix: str = "",
) -> list[dict[str, str]]:
    if moveit_profile == DUAL_ARM_MOVEIT_GROUP:
        sides = _MOTION_PROFILES.get(DUAL_ARM_MOVEIT_GROUP, {}).get("sides", ["left", "right"])
        return [_planning_instance_for_side(side) for side in sides]

    default_prefix = explicit_joint_prefix or PROFILE_PREFIX_DEFAULTS.get(moveit_profile, "")
    default_feedback_prefix = explicit_feedback_joint_prefix or default_prefix
    default_name = "arm" if moveit_profile == DEFAULT_MOVEIT_GROUP else moveit_profile
    return [
        {
            "name": default_name,
            "namespace": "",
            "joint_prefix": default_prefix,
            "feedback_joint_prefix": default_feedback_prefix,
            "controller_name": ARM_CONTROLLER_NAME,
            "can_port": "",
            "arm_type": "",
            "effector_type": "",
            "omnihand_type": "",
            "revo2_type": "",
            "tcp_offset": "",
            "launch_driver": "",
        }
    ]


def resolve_arm_instances(
    moveit_profile: str,
    arm_instances_raw: object,
    explicit_joint_prefix: str = "",
    explicit_feedback_joint_prefix: str = "",
) -> list[dict[str, str]]:
    parsed_instances = parse_arm_instances(arm_instances_raw)
    if not parsed_instances:
        instances = _default_arm_instances(
            moveit_profile,
            explicit_joint_prefix,
            explicit_feedback_joint_prefix,
        )
    else:
        instances = []
        profile_default_prefix = PROFILE_PREFIX_DEFAULTS.get(moveit_profile, "")
        for index, raw_instance in enumerate(parsed_instances):
            if not isinstance(raw_instance, Mapping):
                raise ValueError("arm_instances entries must be mappings")

            joint_prefix = _trim_string(
                _mapping_value(raw_instance, "joint_prefix")
                or _mapping_value(raw_instance, "input_joint_prefix")
                or explicit_joint_prefix
                or profile_default_prefix
            )
            if moveit_profile == DUAL_ARM_MOVEIT_GROUP and not joint_prefix:
                raise ValueError(
                    f"arm_instances[{index}] must define joint_prefix for moveit_profile 'both_arms'"
                )

            namespace = normalize_relative_namespace(raw_instance.get("namespace", ""))
            feedback_joint_prefix = _trim_string(
                _mapping_value(raw_instance, "feedback_joint_prefix")
                or explicit_feedback_joint_prefix
                or joint_prefix
            )
            name = _trim_string(
                raw_instance.get("name")
                or namespace
                or joint_prefix.removesuffix("_")
                or f"arm_{index + 1}"
            )
            controller_name = _trim_string(raw_instance.get("controller_name") or "arm_controller") or "arm_controller"

            instances.append(
                {
                    "name": name,
                    "namespace": namespace,
                    "joint_prefix": joint_prefix,
                    "feedback_joint_prefix": feedback_joint_prefix,
                    "controller_name": controller_name,
                    "can_port": _trim_string(_mapping_value(raw_instance, "can_port")),
                    "arm_type": _trim_string(_mapping_value(raw_instance, "arm_type")),
                    "effector_type": _trim_string(_mapping_value(raw_instance, "effector_type")),
                    "omnihand_type": _trim_string(_mapping_value(raw_instance, "omnihand_type")),
                    "revo2_type": _trim_string(_mapping_value(raw_instance, "revo2_type")),
                    "tcp_offset": _trim_string(_mapping_value(raw_instance, "tcp_offset")),
                    "launch_driver": _trim_string(_mapping_value(raw_instance, "launch_driver")),
                }
            )

    if moveit_profile == DUAL_ARM_MOVEIT_GROUP and len(instances) != 2:
        raise ValueError("moveit_profile 'both_arms' requires exactly two arm_instances")
    if moveit_profile != DUAL_ARM_MOVEIT_GROUP and len(instances) != 1:
        raise ValueError(
            f"moveit_profile '{moveit_profile}' accepts exactly one arm instance"
        )

    seen_prefixes: set[str] = set()
    seen_controllers: set[str] = set()
    for instance in instances:
        joint_prefix = instance["joint_prefix"]
        if joint_prefix in seen_prefixes:
            raise ValueError(f"Duplicate arm joint_prefix '{joint_prefix}' in arm_instances")
        seen_prefixes.add(joint_prefix)

        path = controller_path(instance)
        if path in seen_controllers:
            raise ValueError(f"Duplicate controller path '{path}' in arm_instances")
        seen_controllers.add(path)

    return instances


def requires_prefixed_feedback(instances: list[Mapping[str, object]]) -> bool:
    if len(instances) > 1:
        return True
    return any(_trim_string(instance.get("feedback_joint_prefix")) for instance in instances)


def resolve_follow_joint_states_topic(context) -> str:
    follow = LaunchConfiguration("follow").perform(context)
    if follow != "true":
        return "control/joint_states"

    explicit_topic = LaunchConfiguration("follow_joint_states_topic").perform(context).strip()
    if explicit_topic and explicit_topic != "feedback/joint_states":
        return explicit_topic

    moveit_profile = LaunchConfiguration("moveit_profile").perform(context).strip()
    explicit_joint_prefix = LaunchConfiguration("input_joint_prefix").perform(context).strip()
    explicit_feedback_joint_prefix = context.launch_configurations.get("feedback_joint_prefix", "")
    arm_instances_raw = context.launch_configurations.get("arm_instances", "")
    instances = resolve_arm_instances(
        moveit_profile,
        arm_instances_raw,
        explicit_joint_prefix,
        explicit_feedback_joint_prefix,
    )
    if requires_prefixed_feedback(instances):
        return DEFAULT_PREFIXED_FEEDBACK_TOPIC
    return "feedback/joint_states"
