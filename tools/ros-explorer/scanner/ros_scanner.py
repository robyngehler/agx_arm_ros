#!/usr/bin/env python3
"""
ros_scanner.py – Scan a ROS2 workspace and emit a WorkspaceData JSON blob.

Usage:
    python3 ros_scanner.py /path/to/ros_ws          # print JSON to stdout
    python3 ros_scanner.py /path/to/ros_ws -o out.json
    python3 ros_scanner.py /path/to/ros_ws --serve  # HTTP server on :7357

The scanner works entirely via static analysis (AST + regex), so no ROS2
installation is required on the machine running the scanner.
"""

import argparse
import ast
import json
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:
    yaml = None


_UNRESOLVED = object()


class _LoopContinue(Exception):
    pass


class _LoopBreak(Exception):
    pass

_UI_BACKEND_ROBOT_METADATA: dict[str, dict[str, Any]] = {
    "ur_1": {
        "display_name": "Universal Robot UR10e",
        "capabilities": ["freedrive", "stop", "gripper", "trajectory"],
        "coordinator_relevant": True,
        "lifecycle_node": "ur_adapter",
        "gripper_type": "RG6",
    },
    "portal": {
        "display_name": "Portal XY System",
        "capabilities": ["power", "jog", "stop", "goto", "home"],
        "coordinator_relevant": True,
        "lifecycle_node": "portal_adapter",
    },
    "panda_1": {
        "display_name": "Franka Emika Panda",
        "capabilities": ["freedrive", "stop", "gripper", "trajectory"],
        "coordinator_relevant": True,
        "lifecycle_node": "panda_adapter",
        "gripper_type": "parallel_jaw",
    },
}


# ── helpers ───────────────────────────────────────────────────────────────────

def _read(path: Path) -> str:
    try:
        return path.read_text(errors="replace")
    except OSError:
        return ""


def _find_files(root: Path, *exts: str) -> list[Path]:
    result: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(root):
        # skip build/install/hidden directories
        dirnames[:] = [
            d for d in dirnames
            if d not in ("build", "install", ".git", "__pycache__", "node_modules")
            and not d.startswith(".")
        ]
        for f in filenames:
            if any(f.endswith(ext) for ext in exts):
                result.append(Path(dirpath) / f)
    return result


def _flatten_params(value: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    flattened: dict[str, Any] = {}
    for key, item in value.items():
        full_key = f"{prefix}.{key}" if prefix else key
        if isinstance(item, dict):
            flattened.update(_flatten_params(item, full_key))
        else:
            flattened[full_key] = item
    return flattened


def _find_ros_ws_root(path: Path) -> Path | None:
    for parent in (path.resolve(), *path.resolve().parents):
        if parent.name == "ros_ws" and (parent / "config").exists():
            return parent
        candidate = parent / "ros_ws"
        if candidate.exists() and (candidate / "config").exists():
            return candidate
    return None


def _resolve_workspace_config_path(current_path: Path, raw_path: str) -> Path | None:
    candidate = Path(raw_path)
    if candidate.is_absolute():
        return candidate if candidate.exists() else None
    ws_root = _find_ros_ws_root(current_path)
    if ws_root is None:
        return None
    if raw_path.startswith("ros_ws/"):
        candidate = ws_root.parent / raw_path
    else:
        candidate = ws_root / raw_path
    return candidate if candidate.exists() else None


def _load_ui_backend_namespace_map(current_path: Path, config_path: str) -> dict[str, str]:
    if yaml is None:
        return {}
    resolved_path = _resolve_workspace_config_path(current_path, config_path)
    if resolved_path is None:
        return {}
    try:
        payload = yaml.safe_load(_read(resolved_path)) or {}
    except Exception:
        return {}

    namespaces = payload.get("namespaces", {}) if isinstance(payload, dict) else {}
    result: dict[str, str] = {}
    for entry in namespaces.get("ur_robots", []) or []:
        if isinstance(entry, dict) and entry.get("name"):
            name = str(entry["name"]).strip()
            if name:
                result[name] = f"/{name}"
    portal = namespaces.get("portal")
    if isinstance(portal, dict) and portal.get("name"):
        name = str(portal["name"]).strip()
        if name:
            result[name] = f"/{name}"
    for entry in namespaces.get("panda_robots", []) or []:
        if isinstance(entry, dict) and entry.get("name"):
            name = str(entry["name"]).strip()
            if name:
                result[name] = f"/{name}"
    return result


def _build_ui_backend_robot_registry(namespace_map: dict[str, str]) -> dict[str, dict[str, Any]]:
    registry: dict[str, dict[str, Any]] = {}
    for robot_id, metadata in _UI_BACKEND_ROBOT_METADATA.items():
        namespace = namespace_map.get(robot_id)
        if namespace is None:
            continue
        registry[robot_id] = {
            **metadata,
            "namespace": namespace,
        }
    for robot_id, namespace in namespace_map.items():
        if robot_id in registry:
            continue
        registry[robot_id] = {
            "namespace": namespace,
            "display_name": robot_id,
            "capabilities": ["stop"],
            "coordinator_relevant": True,
            "lifecycle_node": None,
        }
    return registry


def _qualify_namespace_path(namespace: str, resource: str) -> str:
    normalized_namespace = str(namespace or "").rstrip("/")
    normalized_resource = str(resource or "").lstrip("/")
    if not normalized_namespace:
        return f"/{normalized_resource}"
    return f"{normalized_namespace}/{normalized_resource}"


def _build_ui_backend_managed_targets(
    robot_registry: dict[str, dict[str, Any]],
    db_bridge_namespace: str,
    performer_helper_namespace: str,
    coordinator_namespace: str,
) -> list[dict[str, Any]]:
    ur_robot_id = next((robot_id for robot_id in robot_registry if robot_id.startswith("ur_")), "ur_1")
    ur_namespace = str(robot_registry.get(ur_robot_id, {}).get("namespace") or f"/{ur_robot_id}")
    panda_robot_id = next((robot_id for robot_id in robot_registry if robot_id.startswith("panda_")), "panda_1")
    coord_action_name = _qualify_namespace_path(coordinator_namespace, "execute_activity")
    return [
        {
            "node_name": "db_bridge",
            "namespace": db_bridge_namespace,
            "kind": "service",
            "probe": _qualify_namespace_path(db_bridge_namespace, "list_actions"),
            "lifecycle": _qualify_namespace_path(db_bridge_namespace, "get_state"),
        },
        {
            "node_name": "performer_helper",
            "namespace": performer_helper_namespace,
            "kind": "action",
            "probe": _qualify_namespace_path(performer_helper_namespace, "perform"),
            "lifecycle": _qualify_namespace_path(performer_helper_namespace, "get_state"),
        },
        {
            "node_name": "coordination",
            "namespace": coordinator_namespace,
            "kind": "action",
            "probe": coord_action_name,
            "lifecycle": _qualify_namespace_path(coordinator_namespace, "get_state"),
        },
        {
            "node_name": "portal_adapter",
            "namespace": "/",
            "kind": "service",
            "probe": "/portal_adapter/get_state",
            "lifecycle": "/portal_adapter/get_state",
            "notes": ["Robot adapter for portal"],
        },
        {
            "node_name": "ur_adapter",
            "namespace": ur_namespace,
            "kind": "service",
            "probe": _qualify_namespace_path(ur_namespace, "ur_adapter/get_state"),
            "lifecycle": _qualify_namespace_path(ur_namespace, "ur_adapter/get_state"),
            "notes": [f"Robot adapter for {ur_robot_id}"],
        },
        {
            "node_name": "panda_adapter",
            "namespace": "/",
            "kind": "service",
            "probe": "/panda_adapter/get_state",
            "lifecycle": "/panda_adapter/get_state",
            "notes": [f"Robot adapter for {panda_robot_id}"],
        },
    ]


# ── package.xml parser ───────────────────────────────────────────────────────

def _parse_package_xml(path: Path) -> dict[str, Any] | None:
    text = _read(path)
    name_m = re.search(r"<name>\s*(.*?)\s*</name>", text)
    if not name_m:
        return None
    name = name_m.group(1)

    build_type = "unknown"
    bt_m = re.search(r"<build_type>\s*(.*?)\s*</build_type>", text)
    if bt_m:
        raw = bt_m.group(1)
        if "ament_python" in raw:
            build_type = "ament_python"
        elif "ament_cmake" in raw:
            build_type = "ament_cmake"
        elif "cmake" in raw:
            build_type = "cmake"

    deps: list[str] = []
    for tag in ("depend", "exec_depend", "build_depend"):
        for m in re.finditer(rf"<{tag}>\s*(.*?)\s*</{tag}>", text):
            deps.append(m.group(1))

    return {
        "name": name,
        "path": str(path.parent.relative_to(path.parent.parent.parent)),
        "deps": list(dict.fromkeys(deps)),
        "buildType": build_type,
    }


# ── msg / srv / action parser ─────────────────────────────────────────────────

def _parse_msg_file(path: Path, pkg: str) -> dict[str, Any]:
    lines = _read(path).splitlines()
    fields = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line == "---":
            break
        parts = line.split()
        if len(parts) >= 2:
            fields.append({"type": parts[0], "name": parts[1]})

    kind_map = {".msg": "msg", ".srv": "srv", ".action": "action"}
    return {
        "name": path.stem,
        "package": pkg,
        "kind": kind_map.get(path.suffix, "msg"),
        "fields": fields,
        "filePath": str(path),
    }


# ── Python node parser ────────────────────────────────────────────────────────

class _NodeVisitor(ast.NodeVisitor):
    """Extract ROS2 pub/sub/service/action/param calls from a Python node."""

    def __init__(
        self,
        initial_locals: dict[str, Any] | None = None,
        initial_self_attrs: dict[str, Any] | None = None,
        initial_param_defaults: dict[str, Any] | None = None,
    ) -> None:
        self.node_name: str | None = None
        self.topics: list[dict] = []
        self.services: list[dict] = []
        self.actions: list[dict] = []
        self.parameters: list[dict] = []
        self.lifecycle = False
        self._class_bases: list[str] = []
        # Variable resolution: declared parameter string defaults, self.attrs, local vars
        self._param_defaults: dict[str, Any] = dict(initial_param_defaults or {})
        self._self_attrs: dict[str, Any] = dict(initial_self_attrs or {})
        self._locals: dict[str, Any] = dict(initial_locals or {})
        self._resolved_loop_depth = 0

    def _bind_target_value(self, target: ast.expr, value: Any) -> bool:
        if isinstance(target, ast.Name):
            self._locals[target.id] = value
            return True
        if isinstance(target, (ast.Tuple, ast.List)) and isinstance(value, (tuple, list)):
            if len(target.elts) != len(value):
                return False
            return all(self._bind_target_value(subtarget, subvalue) for subtarget, subvalue in zip(target.elts, value))
        return False

    def _resolve_special_self_call(self, node: ast.Call) -> Any:
        if not (isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "self"):
            return _UNRESOLVED

        method = node.func.attr
        if method == "_get_robot_registry":
            return self._self_attrs.get("_robot_registry", _UNRESOLVED)
        if method == "_build_managed_node_targets":
            return self._self_attrs.get("_managed_node_targets", _UNRESOLVED)
        if method == "_namespace_for" and node.args:
            robot_id = self._resolve_literal_value(node.args[0])
            registry = self._self_attrs.get("_robot_registry", {})
            if isinstance(robot_id, str) and isinstance(registry, dict):
                namespace = str(registry.get(robot_id, {}).get("namespace") or "").strip()
                return namespace or f"/{robot_id}"
        if method in {"get_joint_states_topic", "get_position_topic"} and node.args:
            robot_id = self._resolve_literal_value(node.args[0])
            namespace_map = self._self_attrs.get("_robot_namespaces", {})
            if isinstance(robot_id, str) and isinstance(namespace_map, dict):
                namespace = namespace_map.get(robot_id)
                if not isinstance(namespace, str):
                    return _UNRESOLVED
                suffix = "state/joint_states" if method == "get_joint_states_topic" else "state/position"
                return f"{namespace}/{suffix}"
        if method == "_qualify_path" and len(node.args) >= 2:
            namespace = self._resolve_literal_value(node.args[0])
            resource = self._resolve_literal_value(node.args[1])
            if isinstance(namespace, str) and isinstance(resource, str):
                return _qualify_namespace_path(namespace, resource)
        return _UNRESOLVED

    def _resolve_iterable_value(self, node: ast.expr | None) -> Any:
        if node is None:
            return _UNRESOLVED
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            container = self._resolve_literal_value(node.func.value)
            if node.func.attr == "items" and isinstance(container, dict):
                return list(container.items())
            if node.func.attr == "values" and isinstance(container, dict):
                return list(container.values())
            if node.func.attr == "keys" and isinstance(container, dict):
                return list(container.keys())
        value = self._resolve_literal_value(node)
        if isinstance(value, (list, tuple)):
            return value
        if isinstance(value, dict):
            return list(value)
        return _UNRESOLVED

    def _resolve_condition_value(self, node: ast.expr | None) -> bool | None:
        if node is None:
            return None
        if isinstance(node, ast.Constant) and isinstance(node.value, bool):
            return node.value
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.Not):
            operand = self._resolve_condition_value(node.operand)
            return None if operand is None else not operand
        if isinstance(node, ast.BoolOp):
            values = [self._resolve_condition_value(item) for item in node.values]
            if any(value is None for value in values):
                return None
            if isinstance(node.op, ast.And):
                return all(values)
            if isinstance(node.op, ast.Or):
                return any(values)
        if isinstance(node, ast.Compare) and len(node.ops) == 1 and len(node.comparators) == 1:
            left = self._resolve_literal_value(node.left)
            right = self._resolve_literal_value(node.comparators[0])
            if left is _UNRESOLVED or right is _UNRESOLVED:
                return None
            op = node.ops[0]
            if isinstance(op, ast.Eq):
                return left == right
            if isinstance(op, ast.NotEq):
                return left != right
            if isinstance(op, ast.In):
                try:
                    return left in right
                except TypeError:
                    return None
            if isinstance(op, ast.NotIn):
                try:
                    return left not in right
                except TypeError:
                    return None
        return None

    def _resolve_literal_value(self, node: ast.expr | None) -> Any:
        """Resolve a concrete literal-ish expression without falling back to source text."""
        if node is None:
            return _UNRESOLVED
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Dict):
            resolved: dict[Any, Any] = {}
            for key_node, value_node in zip(node.keys, node.values):
                key = self._resolve_literal_value(key_node)
                value = self._resolve_literal_value(value_node)
                if key is _UNRESOLVED or value is _UNRESOLVED:
                    return _UNRESOLVED
                resolved[key] = value
            return resolved
        if isinstance(node, ast.List):
            resolved_items: list[Any] = []
            for item in node.elts:
                value = self._resolve_literal_value(item)
                if value is _UNRESOLVED:
                    return _UNRESOLVED
                resolved_items.append(value)
            return resolved_items
        if isinstance(node, ast.Tuple):
            resolved_items: list[Any] = []
            for item in node.elts:
                value = self._resolve_literal_value(item)
                if value is _UNRESOLVED:
                    return _UNRESOLVED
                resolved_items.append(value)
            return tuple(resolved_items)
        if isinstance(node, ast.JoinedStr):
            resolved = self._resolve_joined_str(node)
            return resolved if resolved else _UNRESOLVED
        if isinstance(node, ast.BoolOp):
            resolved = self._resolve_bool_op(node)
            return resolved if resolved else _UNRESOLVED
        if isinstance(node, ast.Name):
            return self._locals.get(node.id, _UNRESOLVED)
        if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                and node.value.id == "self"):
            return self._self_attrs.get(node.attr, _UNRESOLVED)
        if isinstance(node, ast.Subscript):
            container = self._resolve_literal_value(node.value)
            if container is _UNRESOLVED:
                return _UNRESOLVED
            key = self._resolve_literal_value(node.slice)
            if key is _UNRESOLVED:
                return _UNRESOLVED
            try:
                return container[key]
            except (KeyError, IndexError, TypeError):
                return _UNRESOLVED
        if isinstance(node, ast.Call):
            special_self_call = self._resolve_special_self_call(node)
            if special_self_call is not _UNRESOLVED:
                return special_self_call
            if isinstance(node.func, ast.Name) and node.func.id == "str" and node.args:
                value = self._resolve_literal_value(node.args[0])
                return _UNRESOLVED if value is _UNRESOLVED else str(value)
            if isinstance(node.func, ast.Attribute):
                if node.func.attr in {"items", "values", "keys"}:
                    container = self._resolve_literal_value(node.func.value)
                    if isinstance(container, dict):
                        if node.func.attr == "items":
                            return list(container.items())
                        if node.func.attr == "values":
                            return list(container.values())
                        return list(container.keys())
                    return _UNRESOLVED
                if node.func.attr == "get":
                    container = self._resolve_literal_value(node.func.value)
                    if isinstance(container, dict) and node.args:
                        key = self._resolve_literal_value(node.args[0])
                        if key is _UNRESOLVED:
                            return _UNRESOLVED
                        default = _UNRESOLVED
                        if len(node.args) > 1:
                            default = self._resolve_literal_value(node.args[1])
                        return container.get(key, default if default is not _UNRESOLVED else None)
                if node.func.attr == "get_parameter" and isinstance(node.func.value, ast.Attribute):
                    value = node.func.value
                    if value.attr == "value" and node.args:
                        pname_node = node.args[0]
                        pname = self._resolve_literal_value(pname_node)
                        if isinstance(pname, str):
                            return self._param_defaults.get(pname, _UNRESOLVED)
        if (isinstance(node, ast.Attribute) and node.attr == "value"
                and isinstance(node.value, ast.Call)):
            call = node.value
            if (isinstance(call.func, ast.Attribute)
                    and call.func.attr == "get_parameter" and call.args):
                pname = self._resolve_literal_value(call.args[0])
                if isinstance(pname, str):
                    return self._param_defaults.get(pname, _UNRESOLVED)
        return _UNRESOLVED

    def _resolve_literal_string(self, node: ast.expr | None) -> str:
        """Resolve a concrete string expression without falling back to source text."""
        value = self._resolve_literal_value(node)
        return value if isinstance(value, str) else ""

    def _resolve_joined_str(self, node: ast.JoinedStr) -> str:
        """Resolve an f-string when all interpolated values are known."""
        parts: list[str] = []
        for value in node.values:
            if isinstance(value, ast.Constant):
                parts.append(str(value.value))
                continue
            if isinstance(value, ast.FormattedValue):
                resolved = self._resolve_literal_string(value.value)
                if not resolved:
                    return ""
                parts.append(resolved)
        return "".join(parts)

    def _resolve_bool_op(self, node: ast.BoolOp) -> str:
        """Resolve `a or b` by returning the first resolvable non-empty branch."""
        if not isinstance(node.op, ast.Or):
            return ""
        for value in node.values:
            resolved = self._resolve_literal_string(value)
            if resolved:
                return resolved
        return ""

    def _bind_function_defaults(self, node: ast.FunctionDef | ast.AsyncFunctionDef) -> None:
        """Bind simple function argument defaults into the local scope while visiting."""
        positional_args = list(node.args.posonlyargs) + list(node.args.args)
        defaults = list(node.args.defaults)
        if defaults:
            default_targets = positional_args[-len(defaults):]
            for arg_node, default_node in zip(default_targets, defaults):
                default_value = self._resolve_literal_value(default_node)
                if default_value is not _UNRESOLVED:
                    self._locals[arg_node.arg] = default_value

    # ── variable resolution helpers ───────────────────────────────────────────

    def _resolve(self, n: ast.expr | None) -> str:
        """Resolve an AST expression to a string using known variable bindings."""
        if n is None:
            return ""
        if isinstance(n, ast.Constant):
            return str(n.value)
        if isinstance(n, ast.JoinedStr):
            return self._resolve_joined_str(n)
        if isinstance(n, ast.BoolOp):
            return self._resolve_bool_op(n)
        # self.attr → look up in tracked self-attributes
        if (isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name)
                and n.value.id == "self"):
            val = self._self_attrs.get(n.attr, "")
            if val:
                return val
        # local variable → look up in tracked locals
        if isinstance(n, ast.Name):
            val = self._locals.get(n.id, "")
            if val:
                return val
        # fallback: return source representation
        if hasattr(ast, "unparse"):
            return ast.unparse(n)
        return ""

    def _extract_string_value(self, node: ast.expr) -> str:
        """Extract a plain string from an assignment RHS.

        Handles: "literal", str(expr), self.get_parameter("name").value
        """
        return self._resolve_literal_string(node)

    def _dedupe_connections(self, connections: list[dict[str, Any]]) -> list[dict[str, Any]]:
        seen: set[tuple[tuple[str, str], ...]] = set()
        deduped: list[dict[str, Any]] = []
        for connection in connections:
            key = tuple(sorted((str(k), str(v)) for k, v in connection.items()))
            if key in seen:
                continue
            seen.add(key)
            deduped.append(connection)
        return deduped

    # ── assignment tracking ───────────────────────────────────────────────────

    def visit_Assign(self, node: ast.Assign) -> None:
        """Track self.attr = <string> and local_var = <string> assignments."""
        val = self._resolve_literal_value(node.value)
        if val is not _UNRESOLVED:
            for target in node.targets:
                if isinstance(target, ast.Name):
                    self._locals[target.id] = val
                elif (isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id == "self"):
                    self._self_attrs[target.attr] = val
        self.generic_visit(node)

    def visit_For(self, node: ast.For) -> None:
        iterable = self._resolve_iterable_value(node.iter)
        if iterable is _UNRESOLVED:
            self.generic_visit(node)
            return

        previous_locals = dict(self._locals)
        try:
            for item in iterable:
                self._locals = dict(previous_locals)
                if not self._bind_target_value(node.target, item):
                    continue
                try:
                    self._resolved_loop_depth += 1
                    for statement in node.body:
                        self.visit(statement)
                except _LoopContinue:
                    continue
                except _LoopBreak:
                    break
                finally:
                    self._resolved_loop_depth -= 1
                for statement in node.orelse:
                    self.visit(statement)
        finally:
            self._locals = previous_locals

    def visit_Continue(self, node: ast.Continue) -> None:
        if self._resolved_loop_depth <= 0:
            return
        raise _LoopContinue()

    def visit_Break(self, node: ast.Break) -> None:
        if self._resolved_loop_depth <= 0:
            return
        raise _LoopBreak()

    def visit_If(self, node: ast.If) -> None:
        condition = self._resolve_condition_value(node.test)
        if condition is True:
            for statement in node.body:
                self.visit(statement)
            return
        if condition is False:
            for statement in node.orelse:
                self.visit(statement)
            return
        self.generic_visit(node)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        previous_locals = dict(self._locals)
        self._bind_function_defaults(node)
        self.generic_visit(node)
        self._locals = previous_locals

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        previous_locals = dict(self._locals)
        self._bind_function_defaults(node)
        self.generic_visit(node)
        self._locals = previous_locals

    # ── class definitions ─────────────────────────────────────────────────────

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        for base in node.bases:
            name = ast.unparse(base) if hasattr(ast, "unparse") else ""
            if "LifecycleNode" in name:
                self.lifecycle = True
            if "Node" in name:
                self._class_bases.append(name)
        self.generic_visit(node)

    # ── ROS2 API call extraction ──────────────────────────────────────────────

    def visit_Call(self, node: ast.Call) -> None:  # noqa: C901
        func = node.func
        fname = ""
        if isinstance(func, ast.Attribute):
            fname = func.attr
        elif isinstance(func, ast.Name):
            fname = func.id

        # super().__init__("node_name")
        if fname == "__init__" and node.args:
            val = self._resolve_literal_string(node.args[0])
            if val:
                self.node_name = val

        # create_publisher(MsgType, "topic", qos)
        # create_lifecycle_publisher(MsgType, "topic", qos)
        if fname in {"create_publisher", "create_lifecycle_publisher"} and len(node.args) >= 2:
            msg_type = self._resolve(node.args[0])
            topic = self._resolve_literal_string(node.args[1])
            if topic:
                self.topics.append({
                    "topic": topic,
                    "msgType": msg_type.replace(".", "/"),
                    "direction": "pub",
                })

        # create_subscription(MsgType, "topic", cb, qos)
        if fname == "create_subscription" and len(node.args) >= 2:
            msg_type = self._resolve(node.args[0])
            topic = self._resolve_literal_string(node.args[1])
            if topic:
                self.topics.append({
                    "topic": topic,
                    "msgType": msg_type.replace(".", "/"),
                    "direction": "sub",
                })

        # create_service(SrvType, "name", cb)
        if fname == "create_service" and len(node.args) >= 2:
            srv_type = self._resolve(node.args[0])
            svc = self._resolve_literal_string(node.args[1])
            if svc:
                self.services.append({
                    "service": svc,
                    "srvType": srv_type.replace(".", "/"),
                    "role": "server",
                })

        # create_client(SrvType, "name")
        if fname == "create_client" and len(node.args) >= 2:
            srv_type = self._resolve(node.args[0])
            svc = self._resolve_literal_string(node.args[1])
            if svc:
                self.services.append({
                    "service": svc,
                    "srvType": srv_type.replace(".", "/"),
                    "role": "client",
                })

        # ActionServer(node_ref, ActionType, "action_name", execute_callback=...)
        if fname == "ActionServer" and len(node.args) >= 3:
            action_type = self._resolve(node.args[1])
            action = self._resolve_literal_string(node.args[2])
            if action:
                self.actions.append({
                    "action": action,
                    "actionType": action_type.replace(".", "/"),
                    "role": "server",
                })

        # ActionClient(node_ref, ActionType, "action_name")
        if fname == "ActionClient" and len(node.args) >= 3:
            action_type = self._resolve(node.args[1])
            action = self._resolve_literal_string(node.args[2])
            if action:
                self.actions.append({
                    "action": action,
                    "actionType": action_type.replace(".", "/"),
                    "role": "client",
                })

        # declare_parameter("name", default, ParameterDescriptor(...))
        if fname == "declare_parameter" and node.args:
            pname = self._resolve_literal_string(node.args[0])
            default: str | None = None
            if len(node.args) > 1:
                default_node = node.args[1]
                if (isinstance(default_node, ast.Constant)
                        and isinstance(default_node.value, str)):
                    default = default_node.value
                    # Cache string defaults for later variable resolution
                    self._param_defaults[pname] = default
            desc = ""
            for kw in node.keywords:
                if kw.arg == "description":
                    desc = self._resolve(kw.value)
            if pname:
                self.parameters.append({
                    "name": pname,
                    "type": "unknown",
                    "default": default,
                    "description": desc,
                })

        self.generic_visit(node)


def _load_package_params(pkg_dir: Path, pkg_name: str) -> dict[str, Any]:
    if yaml is None:
        return {}

    params_path = pkg_dir / "config" / f"{pkg_name}_params.yaml"
    if not params_path.exists():
        return {}

    try:
        payload = yaml.safe_load(_read(params_path)) or {}
    except Exception:
        return {}

    for section_name in (pkg_name, f"{pkg_name}_node"):
        node_section = payload.get(section_name, {})
        ros_params = node_section.get("ros__parameters", {})
        if isinstance(ros_params, dict) and ros_params:
            return ros_params
    return {}


def _seed_scan_context_for_python_file(path: Path, pkg_name: str, package_params: dict[str, Any]) -> dict[str, Any]:
    context: dict[str, Any] = {
        "locals": None,
        "self_attrs": None,
        "param_defaults": _flatten_params(package_params) if package_params else None,
    }
    stem = path.stem
    if pkg_name == "performer_helper" and stem.endswith("_performer_helper"):
        helper_key = stem[: -len("_performer_helper")]
        device_helpers = package_params.get("device_helpers", {})
        helper_config = device_helpers.get(helper_key)
        if isinstance(helper_config, dict):
            context["locals"] = {"config": helper_config}
        return context

    if pkg_name == "ui_backend" and path.parent.name == "facades":
        flat_params = context["param_defaults"] or {}
        namespace_config = str(flat_params.get("namespace_config", "ros_ws/config/namespaces.yaml"))
        namespace_map = _load_ui_backend_namespace_map(path, namespace_config)
        registry = _build_ui_backend_robot_registry(namespace_map)
        self_attrs: dict[str, Any] = {}

        if stem == "telemetry_hub":
            self_attrs["_robot_namespaces"] = namespace_map
            portal_ns = namespace_map.get("portal", "/portal")
            self_attrs["_portal_position_topic"] = f"{portal_ns}/state/position"
        elif stem == "robot_ops_facade":
            self_attrs["_robot_registry"] = registry
        elif stem == "system_inspector":
            db_bridge_namespace = str(flat_params.get("db_bridge.namespace", "/db_bridge"))
            performer_helper_namespace = str(flat_params.get("performer_helper.namespace", "/performer_helper"))
            coordinator_namespace = str(flat_params.get("coordinator.namespace", "/coord"))
            self_attrs["_robot_registry"] = registry
            self_attrs["_db_bridge_namespace"] = db_bridge_namespace
            self_attrs["_performer_helper_namespace"] = performer_helper_namespace
            self_attrs["_coordinator_namespace"] = coordinator_namespace
            self_attrs["_coord_action_name"] = _qualify_namespace_path(coordinator_namespace, "execute_activity")
            self_attrs["_core_services"] = {
                "db_bridge": _qualify_namespace_path(db_bridge_namespace, "list_actions"),
                "performer_helper": _qualify_namespace_path(performer_helper_namespace, "perform"),
                "coordination": _qualify_namespace_path(coordinator_namespace, "execute_activity"),
            }
            self_attrs["_managed_node_targets"] = _build_ui_backend_managed_targets(
                registry,
                db_bridge_namespace,
                performer_helper_namespace,
                coordinator_namespace,
            )

        if self_attrs:
            context["self_attrs"] = self_attrs

    return context


def _parse_python_node(
    path: Path,
    pkg: str,
    initial_locals: dict[str, Any] | None = None,
    initial_self_attrs: dict[str, Any] | None = None,
    initial_param_defaults: dict[str, Any] | None = None,
) -> dict[str, Any] | None:
    text = _read(path)
    if "rclpy" not in text and "Node" not in text:
        return None
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return None

    visitor = _NodeVisitor(
        initial_locals=initial_locals,
        initial_self_attrs=initial_self_attrs,
        initial_param_defaults=initial_param_defaults,
    )
    visitor.visit(tree)

    if not visitor.topics and not visitor.services and not visitor.actions and not visitor.node_name:
        return None

    node_name = visitor.node_name or path.stem
    node_id = f"{pkg}/{node_name}"

    return {
        "id": node_id,
        "nodeName": node_name,
        "package": pkg,
        "filePath": str(path),
        "topics": visitor._dedupe_connections(visitor.topics),
        "services": visitor._dedupe_connections(visitor.services),
        "actions": visitor._dedupe_connections(visitor.actions),
        "parameters": visitor._dedupe_connections(visitor.parameters),
        "lifecycleNode": visitor.lifecycle,
        "lifecycleStates": [],
    }


def _merge_node_data(base_node: dict[str, Any], extra_node: dict[str, Any]) -> None:
    for key in ("topics", "services", "actions", "parameters"):
        combined = list(base_node.get(key, [])) + list(extra_node.get(key, []))
        deduped: list[dict[str, Any]] = []
        seen: set[tuple[tuple[str, str], ...]] = set()
        for entry in combined:
            signature = tuple(sorted((str(k), str(v)) for k, v in entry.items()))
            if signature in seen:
                continue
            seen.add(signature)
            deduped.append(entry)
        base_node[key] = deduped


def _build_performer_helper_fallback_nodes(
    pkg_dir: Path,
    package_params: dict[str, Any],
) -> list[dict[str, Any]]:
    """Build helper nodes from performer_helper config as a discovery fallback."""
    fallback_nodes: list[dict[str, Any]] = []
    device_helpers = package_params.get("device_helpers", {})
    if not isinstance(device_helpers, dict):
        return fallback_nodes

    for helper_name, helper_cfg in device_helpers.items():
        if not isinstance(helper_name, str) or not isinstance(helper_cfg, dict):
            continue

        node_name = f"{helper_name}_performer_helper"
        file_path = str(pkg_dir / "performer_helper" / f"{node_name}.py")

        topics: list[dict[str, Any]] = []
        for topic_name in (helper_cfg.get("subscribers", {}) or {}).values():
            if isinstance(topic_name, str) and topic_name:
                topics.append({"topic": topic_name, "msgType": "unknown", "direction": "sub"})
        for topic_name in (helper_cfg.get("state_topics", {}) or {}).values():
            if isinstance(topic_name, str) and topic_name:
                topics.append({"topic": topic_name, "msgType": "unknown", "direction": "sub"})

        services: list[dict[str, Any]] = []
        for service_name in (helper_cfg.get("service_clients", {}) or {}).values():
            if isinstance(service_name, str) and service_name:
                services.append({"service": service_name, "srvType": "unknown", "role": "client"})

        actions: list[dict[str, Any]] = []
        for action_name in (helper_cfg.get("action_clients", {}) or {}).values():
            if isinstance(action_name, str) and action_name:
                actions.append({"action": action_name, "actionType": "unknown", "role": "client"})

        fallback_nodes.append({
            "id": f"performer_helper/{node_name}",
            "nodeName": node_name,
            "package": "performer_helper",
            "filePath": file_path,
            "topics": topics,
            "services": services,
            "actions": actions,
            "parameters": [],
            "lifecycleNode": False,
            "lifecycleStates": [],
        })

    return fallback_nodes


def _assign_unique_node_ids(nodes: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for node in nodes:
        grouped.setdefault(node["id"], []).append(node)

    for group in grouped.values():
        if len(group) == 1:
            continue
        used_ids: set[str] = set()
        for index, node in enumerate(group, start=1):
            stem = Path(node["filePath"]).stem
            candidate = f"{node['id']}@{stem}"
            if candidate in used_ids:
                candidate = f"{candidate}-{index}"
            used_ids.add(candidate)
            node["id"] = candidate
    return nodes


# ── launch file parser ────────────────────────────────────────────────────────

class _LaunchVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.args: list[dict] = []
        self.nodes: list[dict] = []
        self.includes: list[dict] = []

    def _str(self, n: ast.expr | None) -> str:
        if n is None:
            return ""
        if isinstance(n, ast.Constant):
            return str(n.value)
        if hasattr(ast, "unparse"):
            return ast.unparse(n)
        return ""

    def _extract_launch_filename(self, n: ast.expr) -> str:
        """Best-effort extraction of a launch filename from a path expression.

        Handles common patterns:
          os.path.join(get_package_share_directory('pkg'), 'launch', 'file.launch.py')
          PathJoinSubstitution([FindPackageShare('pkg'), 'launch', 'file.launch.py'])
        Falls back to ast.unparse for dynamic/variable expressions.
        """
        # Direct string constant
        if isinstance(n, ast.Constant) and isinstance(n.value, str):
            return n.value
        # os.path.join(..., 'file.launch.py') — last positional arg is the filename
        if isinstance(n, ast.Call):
            func = n.func
            is_join = isinstance(func, ast.Attribute) and func.attr == "join"
            if is_join and n.args:
                last = n.args[-1]
                if isinstance(last, ast.Constant) and isinstance(last.value, str):
                    return last.value
        # PathJoinSubstitution([..., 'file.launch.py']) — list/tuple, last element
        if isinstance(n, (ast.List, ast.Tuple)) and n.elts:
            last = n.elts[-1]
            if isinstance(last, ast.Constant) and isinstance(last.value, str):
                return last.value
        # Walk all constants and return the last one ending in .launch.py / .launch
        for child in ast.walk(n):
            if isinstance(child, ast.Constant) and isinstance(child.value, str):
                if child.value.endswith(".launch.py") or child.value.endswith(".launch"):
                    return child.value
        # Give up — return the full unparse so the frontend can still display something
        if hasattr(ast, "unparse"):
            return ast.unparse(n)
        return ""

    def _kw(self, call: ast.Call, key: str) -> str:
        for kw in call.keywords:
            if kw.arg == key:
                return self._str(kw.value)
        return ""

    def visit_Call(self, node: ast.Call) -> None:  # noqa: C901
        func = node.func
        fname = ""
        if isinstance(func, ast.Attribute):
            fname = func.attr
        elif isinstance(func, ast.Name):
            fname = func.id

        # DeclareLaunchArgument(name, default_value=..., description=..., choices=...)
        if fname == "DeclareLaunchArgument" and node.args:
            name = self._str(node.args[0])
            default = self._kw(node, "default_value")
            desc = self._kw(node, "description")
            choices_node = next(
                (kw.value for kw in node.keywords if kw.arg == "choices"), None
            )
            choices: list[str] | None = None
            if isinstance(choices_node, (ast.List, ast.Tuple)):
                choices = [self._str(e) for e in choices_node.elts]
            if name:
                self.args.append({
                    "name": name,
                    "default": default or None,
                    "description": desc or None,
                    "choices": choices,
                })

        # Node(package=..., executable=..., name=..., namespace=...)
        if fname == "Node":
            pkg = self._kw(node, "package")
            exe = self._kw(node, "executable")
            nm = self._kw(node, "name") or None
            ns = self._kw(node, "namespace") or None
            cond = self._kw(node, "condition") or None
            if pkg and exe:
                self.nodes.append({
                    "package": pkg,
                    "executable": exe,
                    "name": nm,
                    "namespace": ns,
                    "condition": cond,
                })

        # IncludeLaunchDescription(PythonLaunchDescriptionSource(...))
        if fname == "IncludeLaunchDescription" and node.args:
            src = node.args[0]
            file_str = ""
            if isinstance(src, ast.Call) and src.args:
                # src = PythonLaunchDescriptionSource(path_expr)
                # src.args[0] is the path expression — extract filename from it
                file_str = self._extract_launch_filename(src.args[0])
            if not file_str:
                file_str = self._str(src)
            cond = self._kw(node, "condition") or None
            if file_str:
                self.includes.append({"file": file_str, "condition": cond})

        self.generic_visit(node)


def _parse_launch_file(path: Path, pkg: str, rel: Path) -> dict[str, Any] | None:
    text = _read(path)
    if "generate_launch_description" not in text:
        return None
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return None

    visitor = _LaunchVisitor()
    visitor.visit(tree)

    launch_id = f"{pkg}/launch/{path.stem}"
    return {
        "id": launch_id,
        "filePath": str(rel),
        "package": pkg,
        "args": visitor.args,
        "nodes": visitor.nodes,
        "includes": visitor.includes,
    }


# ── setup.py entry-point scanner ─────────────────────────────────────────────

_CONSOLE_SCRIPT_RE = re.compile(
    r"""['"]([\w\-]+)\s*=\s*([\w\.]+):([\w]+)['"]"""
)


def _parse_setup_entry_points(setup_py: Path, pkg_name: str) -> list[dict[str, Any]]:
    """Extract console_scripts from setup.py as ros2-run entry points."""
    text = _read(setup_py)
    # Only look inside the console_scripts block
    cs_match = re.search(r"console_scripts\s*[:=]\s*\[([^\]]*)\]", text, re.DOTALL)
    if not cs_match:
        return []
    block = cs_match.group(1)
    results = []
    for m in _CONSOLE_SCRIPT_RE.finditer(block):
        name, module, fn = m.group(1), m.group(2), m.group(3)
        results.append({
            "name": name,
            "module": f"{module}:{fn}",
            "package": pkg_name,
        })
    return results


# ── workspace scanner ─────────────────────────────────────────────────────────

def scan_workspace(root: str) -> dict[str, Any]:
    ws = Path(root).resolve()
    src = ws / "src"
    if not src.exists():
        src = ws  # flat workspace

    packages: list[dict] = []
    nodes: list[dict] = []
    launches: list[dict] = []
    messages: list[dict] = []
    entry_points: list[dict] = []

    for pkg_xml in _find_files(src, "package.xml"):
        pkg_info = _parse_package_xml(pkg_xml)
        if not pkg_info:
            continue
        packages.append(pkg_info)
        pkg_name = pkg_info["name"]
        pkg_dir = pkg_xml.parent
        package_params = _load_package_params(pkg_dir, pkg_name)
        ui_backend_facade_nodes: list[dict[str, Any]] = []
        ui_backend_main_node: dict[str, Any] | None = None
        package_nodes: list[dict[str, Any]] = []

        # messages
        for ext in (".msg", ".srv", ".action"):
            for f in _find_files(pkg_dir, ext):
                messages.append(_parse_msg_file(f, pkg_name))

        # python nodes
        for f in _find_files(pkg_dir, ".py"):
            if "launch" in str(f):
                continue
            if f.name.startswith("test_") or f.name.endswith("_test.py"):
                continue
            scan_context = _seed_scan_context_for_python_file(f, pkg_name, package_params)
            result = _parse_python_node(
                f,
                pkg_name,
                initial_locals=scan_context.get("locals"),
                initial_self_attrs=scan_context.get("self_attrs"),
                initial_param_defaults=scan_context.get("param_defaults"),
            )
            if result:
                if pkg_name == "ui_backend" and f.parent.name == "facades":
                    ui_backend_facade_nodes.append(result)
                    continue
                if pkg_name == "ui_backend" and f.stem == "ui_backend_node":
                    ui_backend_main_node = result
                package_nodes.append(result)

        if pkg_name == "performer_helper":
            fallback_helpers = _build_performer_helper_fallback_nodes(pkg_dir, package_params)
            package_nodes_by_id = {node["id"]: node for node in package_nodes}
            for helper_node in fallback_helpers:
                existing = package_nodes_by_id.get(helper_node["id"])
                if existing is not None:
                    _merge_node_data(existing, helper_node)
                else:
                    package_nodes.append(helper_node)
                    package_nodes_by_id[helper_node["id"]] = helper_node

        if pkg_name == "ui_backend" and ui_backend_main_node is not None:
            for facade_node in ui_backend_facade_nodes:
                _merge_node_data(ui_backend_main_node, facade_node)

        nodes.extend(package_nodes)

        # launch files
        for f in _find_files(pkg_dir, ".launch.py"):
            rel = f.relative_to(ws) if f.is_relative_to(ws) else f
            result = _parse_launch_file(f, pkg_name, rel)
            if result:
                launches.append(result)

        # ros2 run entry points from setup.py console_scripts
        setup_py = pkg_dir / "setup.py"
        if setup_py.exists():
            for ep in _parse_setup_entry_points(setup_py, pkg_name):
                entry_points.append(ep)

    nodes = _assign_unique_node_ids(nodes)

    return {
        "root": str(ws),
        "scannedAt": datetime.now(timezone.utc).isoformat(),
        "packages": packages,
        "nodes": nodes,
        "launches": launches,
        "messages": messages,
        "entryPoints": entry_points,
    }


# ── HTTP server mode ──────────────────────────────────────────────────────────

def _serve(workspace: str, port: int = 7357) -> None:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from urllib.parse import parse_qs, urlparse
    import socket

    frontend_dist = Path(__file__).parent.parent / "dist"

    class _ReusableHTTPServer(HTTPServer):
        allow_reuse_address = True
        def server_bind(self):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            super().server_bind()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            pass  # suppress default access log

        def _send_json(self, data: Any, status: int = 200) -> None:
            body = json.dumps(data).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_file(self, path: Path) -> None:
            ext_map = {
                ".html": "text/html",
                ".js": "application/javascript",
                ".css": "text/css",
                ".svg": "image/svg+xml",
                ".ico": "image/x-icon",
                ".json": "application/json",
                ".woff2": "font/woff2",
            }
            ct = ext_map.get(path.suffix, "application/octet-stream")
            data = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", ct)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            path = parsed.path
            qs = parse_qs(parsed.query)

            if path == "/api/scan":
                ws_path = qs.get("path", [workspace])[0]
                try:
                    data = scan_workspace(ws_path)
                    self._send_json(data)
                except Exception as e:
                    self._send_json({"error": str(e)}, 500)
                return

            # Static file serving from dist/
            if frontend_dist.exists():
                static_path = (frontend_dist / path.lstrip("/")).resolve()
                # Security: ensure path stays within dist
                if not str(static_path).startswith(str(frontend_dist)):
                    self.send_response(403)
                    self.end_headers()
                    return
                if static_path.is_file():
                    self._send_file(static_path)
                    return
                # SPA fallback
                index = frontend_dist / "index.html"
                if index.exists():
                    self._send_file(index)
                    return

            self.send_response(404)
            self.end_headers()

    print(f"ROS Explorer scanner serving on http://localhost:{port}")
    print(f"  API: http://localhost:{port}/api/scan?path={workspace}")
    if frontend_dist.exists():
        print(f"  UI:  http://localhost:{port}/")
    _ReusableHTTPServer(("", port), Handler).serve_forever()


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 workspace static scanner")
    parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS2 workspace root")
    parser.add_argument("-o", "--output", help="Write JSON to file instead of stdout")
    parser.add_argument("--serve", action="store_true", help="Start HTTP server (API + UI)")
    parser.add_argument("--port", type=int, default=7357, help="HTTP server port (default: 7357)")
    args = parser.parse_args()

    if args.serve:
        _serve(args.workspace, args.port)
        return

    data = scan_workspace(args.workspace)
    out = json.dumps(data, indent=2, ensure_ascii=False)
    if args.output:
        Path(args.output).write_text(out)
        print(f"Written to {args.output}", file=sys.stderr)
    else:
        print(out)


if __name__ == "__main__":
    main()
