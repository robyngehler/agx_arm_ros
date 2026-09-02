from __future__ import annotations

import math
import shlex
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Sequence
from xml.etree import ElementTree as ET

# Name of the link the payload derivation appends. Fixed so a second call on an
# already-derived URDF is rejected instead of stacking payloads.
PAYLOAD_LINK_NAME = "gravity_payload"


def solid_cylinder_inertia(
    mass_kg: float, radius_m: float, height_m: float
) -> tuple[float, float, float]:
    """Principal inertia (ixx, iyy, izz) of a solid cylinder about its own CoM.

    The cylinder axis is local z. Gravity does not read the tensor, but a
    physically complete payload description keeps the URDF usable for the later
    dynamics feedforward.
    """
    if mass_kg < 0.0 or radius_m < 0.0 or height_m < 0.0:
        raise ValueError("cylinder inertia needs non-negative mass, radius and height")
    ixx = mass_kg * (3.0 * radius_m**2 + height_m**2) / 12.0
    izz = mass_kg * radius_m**2 / 2.0
    return (ixx, ixx, izz)


def _link_names(root: ET.Element) -> list[str]:
    return [link.attrib.get("name", "") for link in root.findall("link")]


def resolve_payload_parent_link(
    base_gravity_urdf_path: str | Path,
    input_joint_prefix: str = "",
    explicit_parent_link: str = "",
    flange_suffix: str = "nero_tool0",
) -> str:
    """Pick the link a payload attaches to, from the URDF rather than by guess.

    An explicit name wins. Otherwise the arm's flange link is taken from the
    URDF: the joint prefix narrows it on a two-arm model, and an ambiguous or
    absent match raises instead of silently loading the wrong arm.
    """
    if explicit_parent_link.strip():
        return explicit_parent_link.strip()

    root = ET.parse(str(Path(base_gravity_urdf_path).expanduser().resolve())).getroot()
    candidates = [name for name in _link_names(root) if name.endswith(flange_suffix)]
    prefix = input_joint_prefix.strip()
    if prefix:
        prefixed = [name for name in candidates if name.startswith(prefix)]
        if prefixed:
            candidates = prefixed
    if not candidates:
        raise ValueError(
            f"no '*{flange_suffix}' link in {base_gravity_urdf_path}; "
            "set payload_parent_link explicitly"
        )
    if len(candidates) > 1:
        raise ValueError(
            f"payload parent link is ambiguous between {sorted(candidates)}; "
            "set payload_parent_link explicitly"
        )
    return candidates[0]


def derive_fixed_payload_urdf(
    base_gravity_urdf_path: str | Path,
    parent_link: str,
    mass_kg: float,
    com_xyz: Sequence[float],
    inertia: "Sequence[float] | None" = None,
) -> str:
    """Write a copy of the gravity URDF with one fixed payload link appended.

    The payload rides `parent_link` through a fixed joint, so it adds mass at a
    lever without adding a DoF: the resulting model has the same joints, the same
    q layout, and the same articulated-payload joint names as the base.

    `com_xyz` is the payload centre of mass in the `parent_link` frame — the
    frame's real axes, not an assumed tool direction. `inertia` is
    (ixx, iyy, izz) about the CoM; omitted means a point mass.
    """
    base_path = Path(base_gravity_urdf_path).expanduser().resolve()
    if not base_path.is_file():
        raise ValueError(f"base gravity URDF does not exist: {base_path}")
    if not parent_link.strip():
        raise ValueError("payload parent_link must be set")
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError(f"payload mass must be finite and > 0, got {mass_kg}")
    com = [float(value) for value in com_xyz]
    if len(com) != 3 or not all(math.isfinite(value) for value in com):
        raise ValueError(f"payload com_xyz must be three finite numbers, got {com_xyz}")

    tensor = [0.0, 0.0, 0.0] if inertia is None else [float(value) for value in inertia]
    if len(tensor) != 3 or not all(math.isfinite(v) and v >= 0.0 for v in tensor):
        raise ValueError(
            f"payload inertia must be three finite, non-negative numbers (ixx, iyy, izz), got {inertia}"
        )

    root = ET.parse(str(base_path)).getroot()
    names = _link_names(root)
    if parent_link not in names:
        raise ValueError(
            f"payload parent_link '{parent_link}' is not a link in {base_path}. "
            f"Candidates: {sorted(n for n in names if n)}"
        )
    if PAYLOAD_LINK_NAME in names:
        raise ValueError(
            f"'{base_path}' already carries a '{PAYLOAD_LINK_NAME}' link; "
            "derive the loaded model from the unloaded gravity URDF"
        )

    link = ET.SubElement(root, "link", {"name": PAYLOAD_LINK_NAME})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": repr(float(mass_kg))})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": repr(tensor[0]), "ixy": "0.0", "ixz": "0.0",
            "iyy": repr(tensor[1]), "iyz": "0.0",
            "izz": repr(tensor[2]),
        },
    )
    joint = ET.SubElement(
        root, "joint", {"name": f"{PAYLOAD_LINK_NAME}_joint", "type": "fixed"}
    )
    ET.SubElement(joint, "parent", {"link": parent_link})
    ET.SubElement(joint, "child", {"link": PAYLOAD_LINK_NAME})
    ET.SubElement(
        joint,
        "origin",
        {"xyz": f"{com[0]} {com[1]} {com[2]}", "rpy": "0 0 0"},
    )

    temp_urdf = tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", prefix=f"{base_path.stem}_payload_", delete=False
    )
    try:
        temp_urdf.write(ET.tostring(root, encoding="unicode"))
    finally:
        temp_urdf.close()
    return temp_urdf.name


def parse_xacro_mappings(raw_args: str) -> dict[str, str]:
    mappings: dict[str, str] = {}
    for token in shlex.split(raw_args):
        if ":=" not in token:
            raise ValueError(
                f"custom_model_xacro_args token '{token}' is invalid; expected key:=value"
            )
        key, value = token.split(":=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"custom_model_xacro_args token '{token}' has an empty key")
        mappings[key] = value
    return mappings


def _duo_side_for_prefix(input_joint_prefix: str) -> str:
    normalized_prefix = input_joint_prefix.strip()
    if normalized_prefix.startswith("left_arm_"):
        return "left"
    if normalized_prefix.startswith("right_arm_"):
        return "right"
    return ""


def _apply_duo_slice_mappings(
    model_path: Path,
    mappings: dict[str, str],
    side: str,
    effector_type: str,
) -> dict[str, str]:
    if model_path.name != "duo_system.urdf.xacro":
        return mappings

    if side not in ("left", "right"):
        return mappings

    # Keep only the active arm slice. The mounted end effector stays in the
    # generated URDF so its inertial load is folded into the arm gravity model:
    # 1.06 kg for the OmniHand, 0.548 kg for the AGX gripper. An effector left
    # out is an uncompensated load at the longest lever arm of the chain.
    side_hand_enabled = "true" if effector_type == "omnihand" else "false"
    side_gripper_enabled = "true" if effector_type == "agx_gripper" else "false"
    resolved = dict(mappings)
    if side == "left":
        resolved.update(
            {
                "use_left_arm": "true",
                "use_left_hand": side_hand_enabled,
                "use_left_gripper": side_gripper_enabled,
                "use_right_arm": "false",
                "use_right_hand": "false",
                "use_right_gripper": "false",
            }
        )
    else:
        resolved.update(
            {
                "use_left_arm": "false",
                "use_left_hand": "false",
                "use_left_gripper": "false",
                "use_right_arm": "true",
                "use_right_hand": side_hand_enabled,
                "use_right_gripper": side_gripper_enabled,
            }
        )
    return resolved


def _freeze_subtree_joints(urdf_text: str, root_link: str) -> str:
    root = ET.fromstring(urdf_text)
    joints = root.findall("joint")

    child_joints: dict[str, list[ET.Element]] = {}
    for joint in joints:
        parent = joint.find("parent")
        parent_link = "" if parent is None else parent.attrib.get("link", "")
        if not parent_link:
            continue
        child_joints.setdefault(parent_link, []).append(joint)

    pending_links = [root_link]
    seen_links: set[str] = set()
    subtree_joints: list[ET.Element] = []
    while pending_links:
        link_name = pending_links.pop()
        if link_name in seen_links:
            continue
        seen_links.add(link_name)
        for joint in child_joints.get(link_name, []):
            subtree_joints.append(joint)
            child = joint.find("child")
            child_link = "" if child is None else child.attrib.get("link", "")
            if child_link:
                pending_links.append(child_link)

    for joint in subtree_joints:
        if joint.attrib.get("type") == "fixed":
            continue
        joint.attrib["type"] = "fixed"
        for tag_name in ("axis", "limit", "mimic", "dynamics"):
            element = joint.find(tag_name)
            if element is not None:
                joint.remove(element)

    return ET.tostring(root, encoding="unicode")


def _apply_omnihand_payload(
    model_path: Path,
    urdf_text: str,
    side: str,
    effector_type: str,
    hand_payload_mode: str,
) -> str:
    if model_path.name != "duo_system.urdf.xacro" or effector_type != "omnihand":
        return urdf_text

    if side not in ("left", "right"):
        return urdf_text
    if hand_payload_mode == "articulated":
        # Keep the hand joints movable so the gravity model can track the live
        # finger pose (the MIT controller feeds hand joint states by name and
        # resolves the URDF mimic coupling). Unfed joints stay at zero, which
        # matches the frozen static payload exactly.
        return urdf_text
    return _freeze_subtree_joints(urdf_text, f"{side}_base_link")


def _apply_gripper_payload(
    model_path: Path,
    urdf_text: str,
    side: str,
    effector_type: str,
) -> str:
    """Freeze the gripper fingers so the gravity model needs no finger feedback.

    The two prismatic fingers are 0.025 kg each against 0.498 kg of flange and
    base, so their pose does not move the arm's gravity torque measurably. There
    is no articulated variant for the same reason.
    """
    if model_path.name != "duo_system.urdf.xacro" or effector_type != "agx_gripper":
        return urdf_text
    if side not in ("left", "right"):
        return urdf_text
    return _freeze_subtree_joints(urdf_text, f"{side}_arm_gripper_base")


def resolve_gravity_urdf_path(
    *,
    custom_model: str,
    custom_model_xacro_args: str = "",
    input_joint_prefix: str = "",
    effector_type: str = "none",
    explicit_gravity_urdf_path: str = "",
    duo_side: str = "",
    hand_payload_mode: str = "static",
) -> str:
    if hand_payload_mode not in ("static", "articulated"):
        raise ValueError(
            f"hand_payload_mode must be 'static' or 'articulated', got '{hand_payload_mode}'"
        )
    if explicit_gravity_urdf_path.strip():
        return str(Path(explicit_gravity_urdf_path).expanduser().resolve())

    if not custom_model.strip():
        return ""

    model_path = Path(custom_model).expanduser().resolve()
    if model_path.suffix != ".xacro":
        return str(model_path)

    # An explicit duo_side wins over deriving the side from the joint prefix, so
    # the (prefix-free) teach loop can still bake the body mount into the gravity
    # URDF by naming the arm side directly.
    side = duo_side.strip() or _duo_side_for_prefix(input_joint_prefix)

    mappings = parse_xacro_mappings(custom_model_xacro_args)
    mappings = _apply_duo_slice_mappings(
        model_path,
        mappings,
        side,
        effector_type,
    )
    temp_urdf = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".urdf",
        prefix=f"{model_path.stem}_gravity_",
        delete=False,
    )

    try:
        import xacro  # type: ignore

        generated_urdf = xacro.process_file(str(model_path), mappings=mappings)
        generated_urdf_text = generated_urdf.toprettyxml(indent="  ")
    except ModuleNotFoundError:
        xacro_command = shutil.which("xacro")
        if xacro_command is None:
            raise RuntimeError(
                "xacro is required to derive a gravity URDF from custom_model, but neither the Python module nor the CLI is available"
            )

        command = [xacro_command, str(model_path)]
        for key, value in mappings.items():
            command.append(f"{key}:={value}")

        try:
            result = subprocess.run(command, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as exc:
            stderr = exc.stderr.strip() if exc.stderr else ""
            stdout = exc.stdout.strip() if exc.stdout else ""
            details = stderr or stdout or str(exc)
            raise RuntimeError(
                f"xacro failed while deriving gravity URDF from {model_path}: {details}"
            ) from exc

        generated_urdf_text = result.stdout

    generated_urdf_text = _apply_omnihand_payload(
        model_path,
        generated_urdf_text,
        side,
        effector_type,
        hand_payload_mode,
    )
    generated_urdf_text = _apply_gripper_payload(
        model_path,
        generated_urdf_text,
        side,
        effector_type,
    )
    temp_urdf.write(generated_urdf_text)

    temp_urdf.close()
    return temp_urdf.name