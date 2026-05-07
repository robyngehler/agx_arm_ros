from __future__ import annotations

from pathlib import Path
from typing import Any, Optional
from xml.etree import ElementTree as ET


def candidate_nero_urdf_paths() -> list[Path]:
    src_root = Path(__file__).resolve().parents[2]
    return [
        src_root
        / "agx_arm_sim"
        / "agx_arm_description"
        / "agx_arm_urdf"
        / "nero"
        / "urdf"
        / "nero_description.urdf",
        src_root
        / "agx_arm_sim"
        / "agx_arm_description"
        / "urdf"
        / "nero_gripper_d435.urdf",
    ]


def default_nero_urdf_path() -> Path:
    for candidate in candidate_nero_urdf_paths():
        if candidate.exists():
            return candidate
    return candidate_nero_urdf_paths()[0]


def _parse_xyz(value: Optional[str]) -> list[float]:
    if not value:
        return [0.0, 0.0, 0.0]
    return [float(part) for part in value.split()]


def _parse_inertia(element: Optional[ET.Element]) -> dict[str, float]:
    if element is None:
        return {}
    return {
        key: float(element.attrib.get(key, 0.0))
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    }


def extract_urdf_model_metadata(urdf_path: str | Path) -> dict[str, Any]:
    path = Path(urdf_path).expanduser().resolve()
    tree = ET.parse(path)
    root = tree.getroot()

    inertial_links = []
    total_mass = 0.0
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        mass_value = float(mass.attrib.get("value", 0.0)) if mass is not None else 0.0
        total_mass += mass_value
        inertial_links.append(
            {
                "name": link.attrib.get("name", ""),
                "mass": mass_value,
                "com_xyz": _parse_xyz(None if origin is None else origin.attrib.get("xyz")),
                "inertia": _parse_inertia(inertia),
            }
        )

    joints = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        origin = joint.find("origin")
        axis = joint.find("axis")
        joints.append(
            {
                "name": joint.attrib.get("name", ""),
                "origin_xyz": _parse_xyz(None if origin is None else origin.attrib.get("xyz")),
                "axis_xyz": _parse_xyz(None if axis is None else axis.attrib.get("xyz")),
                "lower": None if limit is None else float(limit.attrib.get("lower", 0.0)),
                "upper": None if limit is None else float(limit.attrib.get("upper", 0.0)),
                "velocity": None if limit is None else float(limit.attrib.get("velocity", 0.0)),
                "effort": None if limit is None else float(limit.attrib.get("effort", 0.0)),
            }
        )

    return {
        "urdf_path": str(path),
        "robot_name": root.attrib.get("name", ""),
        "link_count": len(root.findall("link")),
        "revolute_joint_count": len(joints),
        "total_mass": total_mass,
        "inertial_links": inertial_links,
        "joints": joints,
    }


def extract_mdh_metadata(robot: str = "nero") -> dict[str, Any]:
    try:
        from pyAgxArm.utiles.mdh_kinematics import get_mdh
    except Exception as exc:
        return {"available": False, "reason": str(exc)}

    mdh = get_mdh(robot)
    return {
        "available": True,
        "robot": robot,
        "link_count": len(mdh),
        "parameters": [list(link) for link in mdh],
    }


def compute_flange_pose_from_mdh(joint_positions: list[float], robot: str = "nero") -> Optional[list[float]]:
    try:
        from pyAgxArm.utiles.mdh_kinematics import fk_from_mdh, get_mdh
    except Exception:
        return None

    try:
        pose = fk_from_mdh(list(get_mdh(robot)), joint_positions)
    except Exception:
        return None
    return [float(value) for value in pose]


def summarize_efforts(effort_samples: list[list[float]], joint_names: list[str]) -> dict[str, Any]:
    if not effort_samples:
        return {"available": False}

    max_abs = []
    mean_abs = []
    for joint_index, joint_name in enumerate(joint_names):
        values = [sample[joint_index] for sample in effort_samples]
        max_abs.append({"joint": joint_name, "value": max(abs(value) for value in values)})
        mean_abs.append(
            {
                "joint": joint_name,
                "value": sum(abs(value) for value in values) / max(1, len(values)),
            }
        )

    return {
        "available": True,
        "note": (
            "Effort samples were captured from feedback during leader mode. "
            "Proposal notes indicate these values may be stale or not equal to physical external torque."
        ),
        "max_abs_effort": max_abs,
        "mean_abs_effort": mean_abs,
    }


def build_gravity_context(urdf_path: Optional[str | Path] = None) -> dict[str, Any]:
    resolved_path = default_nero_urdf_path() if urdf_path is None else Path(urdf_path)
    context = {
        "urdf": None,
        "mdh": extract_mdh_metadata("nero"),
        "gravity_note": (
            "This package records URDF inertial parameters and observed effort samples, "
            "but does not yet solve inverse dynamics or gravity torque."
        ),
        "urdf_source_preference": [str(path) for path in candidate_nero_urdf_paths()],
    }
    if resolved_path.exists():
        context["urdf"] = extract_urdf_model_metadata(resolved_path)
    else:
        context["urdf"] = {"available": False, "reason": f"URDF not found: {resolved_path}"}
    return context