from __future__ import annotations

import shlex
import shutil
import subprocess
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET


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
    input_joint_prefix: str,
    effector_type: str,
) -> dict[str, str]:
    if model_path.name != "duo_system.urdf.xacro":
        return mappings

    side = _duo_side_for_prefix(input_joint_prefix)
    if not side:
        return mappings

    # Keep only the active arm slice. When the runtime profile uses OmniHand,
    # keep that hand in the generated URDF so its fixed-pose inertial load can
    # be folded into the arm gravity model.
    side_hand_enabled = "true" if effector_type == "omnihand" else "false"
    resolved = dict(mappings)
    if side == "left":
        resolved.update(
            {
                "use_left_arm": "true",
                "use_left_hand": side_hand_enabled,
                "use_right_arm": "false",
                "use_right_hand": "false",
            }
        )
    else:
        resolved.update(
            {
                "use_left_arm": "false",
                "use_left_hand": "false",
                "use_right_arm": "true",
                "use_right_hand": side_hand_enabled,
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


def _apply_static_omnihand_payload(
    model_path: Path,
    urdf_text: str,
    input_joint_prefix: str,
    effector_type: str,
) -> str:
    if model_path.name != "duo_system.urdf.xacro" or effector_type != "omnihand":
        return urdf_text

    side = _duo_side_for_prefix(input_joint_prefix)
    if side not in ("left", "right"):
        return urdf_text
    return _freeze_subtree_joints(urdf_text, f"{side}_base_link")


def resolve_gravity_urdf_path(
    *,
    custom_model: str,
    custom_model_xacro_args: str = "",
    input_joint_prefix: str = "",
    effector_type: str = "none",
    explicit_gravity_urdf_path: str = "",
) -> str:
    if explicit_gravity_urdf_path.strip():
        return str(Path(explicit_gravity_urdf_path).expanduser().resolve())

    if not custom_model.strip():
        return ""

    model_path = Path(custom_model).expanduser().resolve()
    if model_path.suffix != ".xacro":
        return str(model_path)

    mappings = parse_xacro_mappings(custom_model_xacro_args)
    mappings = _apply_duo_slice_mappings(
        model_path,
        mappings,
        input_joint_prefix,
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

    generated_urdf_text = _apply_static_omnihand_payload(
        model_path,
        generated_urdf_text,
        input_joint_prefix,
        effector_type,
    )
    temp_urdf.write(generated_urdf_text)

    temp_urdf.close()
    return temp_urdf.name