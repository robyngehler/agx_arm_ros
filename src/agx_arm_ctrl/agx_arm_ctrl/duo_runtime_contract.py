from __future__ import annotations

from collections.abc import Mapping


def _trim_string(value: object) -> str:
    return str(value).strip() if value is not None else ""


def validate_duo_both_arms_contract(
    moveit_profile: str,
    effector_type: str,
    arm_instances: list[Mapping[str, object]],
    *,
    follow: str,
    use_mit_controller: bool,
) -> None:
    if moveit_profile != "both_arms":
        return

    if _trim_string(effector_type) not in ("", "none"):
        raise ValueError(
            "moveit_profile 'both_arms' currently supports only effector_type 'none'; "
            "hand-aware dual-arm variants are still open work"
        )

    for index, instance in enumerate(arm_instances):
        instance_effector_type = _trim_string(instance.get("effector_type"))
        if instance_effector_type and instance_effector_type != "none":
            raise ValueError(
                f"arm_instances[{index}] sets effector_type '{instance_effector_type}', "
                "but moveit_profile 'both_arms' is currently arm-only"
            )

    if use_mit_controller and follow != "true":
        raise ValueError(
            "moveit_profile 'both_arms' with use_mit_controller:=true requires "
            "follow:=true so MoveIt and RViz consume merged per-arm feedback"
        )
